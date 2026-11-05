/****************************************************************************
 * arch/arm/src/armv7-m/arm_busfault.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <debug.h>

#include <arch/irq.h>

#include "nvic.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DEBUG_BUSFAULT
#  define bfalert(format, ...) _alert(format, ##__VA_ARGS__)
#else
#  define bfalert(x...)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_busfault_fixup
 *
 * Description:
 *   Optional, board/driver-supplied hook that is given the chance to
 *   recover from a precise data bus fault before the kernel panics.  The
 *   default implementation (weak) does nothing and returns false, so the
 *   normal panic path runs.
 *
 *   A strong override (e.g. the iMX95 NETC VSI driver) may inspect BFAR /
 *   CFSR, and if the faulting access targets a known, tolerable window
 *   (such as a NETC SI register region that the PSI/A55 has torn down),
 *   advance the stacked return PC past the faulting instruction and
 *   return true to resume execution instead of panicking.
 *
 *   Called with 'regs' pointing at the exception register save area
 *   (indexable with REG_PC etc.) and 'cfsr' holding the latched
 *   Configurable Fault Status Register value.
 *
 * Returned Value:
 *   true  - fault handled, resume from the (adjusted) stacked context
 *   false - not handled, proceed to panic
 *
 ****************************************************************************/

bool weak_function arm_busfault_fixup(uint32_t *regs, uint32_t cfsr)
{
  UNUSED(regs);
  UNUSED(cfsr);
  return false;
}

/****************************************************************************
 * Name: arm_busfault
 *
 * Description:
 *   This is Bus Fault exception handler.  It also catches SVC call
 *   exceptions that are performed in bad contexts.
 *
 ****************************************************************************/

int arm_busfault(int irq, void *context, void *arg)
{
  uint32_t cfsr = getreg32(NVIC_CFAULTS);

  /* Give a driver-supplied hook the first chance to recover from a precise
   * data bus fault (e.g. an access to a NETC SI window the PSI tore down).
   * If it fixes up the stacked PC and returns true, clear the latched
   * fault status and resume instead of panicking.
   */

  if (arm_busfault_fixup((uint32_t *)context, cfsr))
    {
      /* Clear the entire latched BusFault status byte (CFSR bits 8..15)
       * so a stale status/address does not leak into the next fault.
       */

      putreg32(cfsr & NVIC_CFAULTS_BUSFAULTSR_MASK, NVIC_CFAULTS);
      return OK;
    }

  bfalert("PANIC!!! Bus Fault:\n");
  bfalert("\tIRQ: %d regs: %p\n", irq, context);
  bfalert("\tBASEPRI: %08x PRIMASK: %08x IPSR: %08"
          PRIx32 " CONTROL: %08" PRIx32 "\n",
          getbasepri(), getprimask(), getipsr(), getcontrol());
  bfalert("\tCFSR: %08" PRIx32 " HFSR: %08" PRIx32 " DFSR: %08"
          PRIx32 " BFAR: %08" PRIx32 " AFSR: %08" PRIx32 "\n",
          cfsr, getreg32(NVIC_HFAULTS), getreg32(NVIC_DFAULTS),
          getreg32(NVIC_BFAULT_ADDR), getreg32(NVIC_AFAULTS));

  bfalert("Bus Fault Reason:\n");
  if (cfsr & NVIC_CFAULTS_IBUSERR)
    {
      bfalert("\tInstruction bus error\n");
    }

  if (cfsr & NVIC_CFAULTS_PRECISERR)
    {
      bfalert("\tPrecise data bus error\n");
    }

  if (cfsr & NVIC_CFAULTS_IMPRECISERR)
    {
      bfalert("\tImprecise data bus error\n");
    }

  if (cfsr & NVIC_CFAULTS_UNSTKERR)
    {
      bfalert("\tBusFault on unstacking for a return from exception\n");
    }

  if (cfsr & NVIC_CFAULTS_STKERR)
    {
      bfalert("\tBusFault on stacking for exception entry\n");
    }

  if (cfsr & NVIC_CFAULTS_LSPERR)
    {
      bfalert("\tFloating-point lazy state preservation error\n");
    }

  up_irq_save();
  PANIC_WITH_REGS("panic", context);
  return OK;
}
