/*
 * Copyright (c) 2009-2021 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/*
 * MIMX9594_cm7_COMMON.h - stub for NuttX NETC HAL build.
 *
 * The real COMMON.h pulls in core_cm7.h (CMSIS) and hundreds of PERI headers.
 * For NuttX we already have CMSIS/ARM definitions from the NuttX toolchain.
 * This stub satisfies the include guard used by the PERI_*.h files without
 * re-defining the types that NuttX already provides.
 */

#ifndef MIMX9594_CM7_COMMON_H_
#define MIMX9594_CM7_COMMON_H_

/*
 * The PERI_*.h files only need the base address macros and type aliases
 * defined in MIMX9594_cm7_netc.h which is already included earlier in the
 * include chain via fsl_device_registers.h -> MIMX9594_cm7_netc.h.
 * Nothing else is needed here.
 */

/* ARM Cortex-M7 barrier intrinsics used by the SDK HAL files. */
#if !defined(__DSB)
#  define __DSB()  __asm__ volatile ("dsb 0xF":::"memory")
#endif
#if !defined(__ISB)
#  define __ISB()  __asm__ volatile ("isb 0xF":::"memory")
#endif
#if !defined(__DMB)
#  define __DMB()  __asm__ volatile ("dmb 0xF":::"memory")
#endif

#endif /* MIMX9594_CM7_COMMON_H_ */
