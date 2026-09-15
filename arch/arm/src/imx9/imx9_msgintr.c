/****************************************************************************
 *
 * Copyright 2022-2023, 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/* NuttX MSGINTR driver for iMX95 M7.
 *
 * Ported from NXP MCUX SDK fsl_msgintr.c (BSD-3-Clause) to NuttX idioms:
 *   - EnableIRQ/DisableIRQ  -> irq_attach / up_enable_irq / up_disable_irq
 *   - SDK assert()          -> DEBUGASSERT()
 *   - Static base/IRQ arrays remain; opaque arg added to callback.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "arm_internal.h"
#include "imx9_msgintr.h"
#include "hardware/imx95/imx95_msgintr.h"
#include "hardware/imx95/imx95_memorymap.h"
#include "arch/irq.h"
#include "nvic.h"
#ifdef CONFIG_ARCH_RAMVECTORS
#  include "ram_vectors.h"
#endif

/* IRQSTEER (WAKEUPMIX) routes peripheral IRQs to M7 NVIC outputs.
 * iMX95 M7 NVIC only implements extints 0-223 (ICTR shows nintlines=7,
 * i.e. 7x32=224 lines).  MSGINTR1/2 are system IRQs 246/247 which land
 * in NVIC extints 246/247 - outside the physical range.
 * The IRQSTEER fan-in aggregates these inputs onto lower NVIC lines:
 *
 *   IRQSTEER base       : 0x44680000
 *   nxp,irq-offset      : 234  (first system IRQ managed by IRQSTEER)
 *   nxp,num-irqs        : 640  (total inputs)
 *   MSGINTR1 input index: 246 - 234 = 12  (master0, bit 12)
 *   MSGINTR2 input index: 247 - 234 = 13  (master0, bit 13)
 *   master0 NVIC output : extint 224 = IMX9_IRQ_IRQSTEER_0
 *
 * CHANMASK register index for input N:
 *   reg_idx = (num_irqs - 1 - N) / 32
 *   For N=12: (640 - 1 - 12) / 32 = 19  -> CHnMASK[19] at offset 0x04+4*19=0x50
 *   Bit within register = N % 32 = 12
 *
 * Both MSGINTR1 and MSGINTR2 map to master0, so a single attach to
 * IMX9_IRQ_IRQSTEER_0 serves both instances.
 */

#ifndef IMX9_IRQSTEER_BASE
#  define IMX9_IRQSTEER_BASE        (0x44680000u)
#endif
#define IMX9_IRQSTEER_NUM_IRQS      (640u)
#define IMX9_IRQSTEER_IRQ_OFFSET    (234u)
#define IMX9_IRQSTEER_CHANMASK_BASE (IMX9_IRQSTEER_BASE + 0x04u)

/* Compute CHANMASK register address and bit for a given system IRQ number */

#define IRQSTEER_REG_IDX(sysirq) \
  (((IMX9_IRQSTEER_NUM_IRQS) - 1u - ((sysirq) - IMX9_IRQSTEER_IRQ_OFFSET)) / 32u)

#define IRQSTEER_REG_ADDR(sysirq) \
  (IMX9_IRQSTEER_CHANMASK_BASE + 4u * IRQSTEER_REG_IDX(sysirq))

#define IRQSTEER_REG_BIT(sysirq) \
  (((sysirq) - IMX9_IRQSTEER_IRQ_OFFSET) % 32u)

/* IRQSTEER register layout (from Zephyr intc_nxp_irqsteer.c):
 *
 *   CTRL_STRIDE_OFF(t, r) = t * 4 * r   (t = reg_num = num_irqs/32 = 20)
 *   CHAN_MASK(n, t)        = t*4*0 + 4*n + 4  = 4*n + 4
 *   CHAN_STATUS(n, t)      = t*4*2 + 4*n + 4  = t*8 + 4*n + 4
 *
 * reg_num = IMX9_IRQSTEER_NUM_IRQS / 32 = 20
 * For reg_index n=19 (input 12):
 *   CHAN_MASK   offset = 4*19 + 4 = 0x50   (matches CHANMASK_BASE + 4*19)
 *   CHAN_STATUS offset = 20*8 + 4*19 + 4 = 0xF0
 */

#define IMX9_IRQSTEER_REG_NUM       (IMX9_IRQSTEER_NUM_IRQS / 32u)  /* 20 */

/* CHAN_STATUS(n, t) = t*8 + 4*n + 4 */

#define IRQSTEER_STATUS_ADDR(sysirq) \
  (IMX9_IRQSTEER_BASE + (IMX9_IRQSTEER_REG_NUM * 8u) + \
   4u * IRQSTEER_REG_IDX(sysirq) + 4u)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of MSGINTR instances on iMX95 (index 0 unused; 1 and 2 are valid) */

#define IMX9_MSGINTR_NUM_INSTANCES  3u  /* indices 0..2; 0 is a placeholder */

/* System IRQ numbers for MSGINTR1 and MSGINTR2 as raw offsets from EXTINT.
 * Used only for IRQSTEER channel-mask programming; NOT used as NVIC IRQs.
 */

#define IMX9_MSGINTR1_SYSIRQ  (246u)   /* MSGINTR1: system IRQ = EXTINT+246 */
#define IMX9_MSGINTR2_SYSIRQ  (247u)   /* MSGINTR2: system IRQ = EXTINT+247 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imx9_msgintr_inst_s
{
  uintptr_t            base;      /* peripheral base address */
  int                  irq;       /* NVIC IRQ number */
  imx9_msgintr_cb_t    callback;  /* user callback (NULL when not init'd) */
  void                *arg;       /* opaque argument for callback */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Instance table indexed by hardware instance number (1 or 2).
 * Index 0 is a sentinel so we can use 1-based instance numbers directly.
 */

static struct imx9_msgintr_inst_s g_msgintr_inst[IMX9_MSGINTR_NUM_INSTANCES] =
{
  /* [0] - unused placeholder */
  {
    .base     = 0,
    .irq      = 0,
    .callback = NULL,
    .arg      = NULL,
  },
  /* [1] - MSGINTR1
   * IRQ field holds the IRQSTEER fan-in NVIC line (IMX9_IRQ_IRQSTEER_0),
   * NOT the raw system IRQ 246 which lies outside the M7 NVIC's range.
   * CHANMASK for input 12 is enabled separately in imx9_msgintr_init().
   */
  {
    .base     = IMX9_MSGINTR1_BASE,
    .irq      = IMX9_IRQ_IRQSTEER_0,
    .callback = NULL,
    .arg      = NULL,
  },
  /* [2] - MSGINTR2 (also via IRQSTEER master0, input 13 -> IMX9_IRQ_IRQSTEER_0) */
  {
    .base     = IMX9_MSGINTR2_BASE,
    .irq      = IMX9_IRQ_IRQSTEER_0,
    .callback = NULL,
    .arg      = NULL,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_msgintr_isr
 *
 * Description:
 *   Common ISR for all MSGINTR instances.  Reads MSIR for each channel,
 *   calls the registered callback for any channel with pending bits, then
 *   returns.  Reading MSIR clears the pending bits in hardware.
 *
 ****************************************************************************/

static int imx9_msgintr_isr(int irq, void *context, void *arg)
{
  struct imx9_msgintr_inst_s *inst = (struct imx9_msgintr_inst_s *)arg;
  struct imx95_msgintr_regs_s *regs =
    (struct imx95_msgintr_regs_s *)inst->base;
  uint8_t  ch;
  uint32_t pending;
  uint32_t sysirq;
  uint32_t stat_bit;

  /* Determine instance from base address since both use the same NVIC IRQ. */

  int instance = (inst->base == IMX9_MSGINTR1_BASE) ? 1 : 2;

  sysirq   = (instance == IMX9_MSGINTR_INSTANCE1) ?
              IMX9_MSGINTR1_SYSIRQ : IMX9_MSGINTR2_SYSIRQ;
  stat_bit = 1u << IRQSTEER_REG_BIT(sysirq);

  /* Guard: IMX9_IRQ_IRQSTEER_0 (master0) is a level signal that stays
   * asserted while ANY enabled input in master0 is active.  If another
   * peripheral in master0 caused this interrupt and our input is not
   * pending, return immediately.
   */

  if ((getreg32(IRQSTEER_STATUS_ADDR(sysirq)) & stat_bit) == 0u)
    {
      return OK;
    }

  for (ch = 0; ch < IMX9_MSGINTR_CHANNEL_NUM; ch++)
    {
      pending = regs->msi[ch].msir;  /* read clears the pending bits */

      if (pending != 0u)
        {
#ifdef ENETC_VSI_DEBUG_INTR
          syslog(LOG_ERR,
                 "MSGINTR ISR: irq=%d inst=%d ch=%u pending=0x%08x\n",
                 irq, instance, (unsigned)ch, (unsigned)pending);
#endif
          if (inst->callback != NULL)
            {
              inst->callback(instance, ch, pending, inst->arg);
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_msgintr_init
 ****************************************************************************/

int imx9_msgintr_init(int instance, imx9_msgintr_cb_t callback, void *arg)
{
  struct imx9_msgintr_inst_s *inst;
  uint32_t sysirq;
  uintptr_t regaddr;
  uint32_t  bit;
  uint32_t  val;

#ifdef ENETC_VSI_DEBUG_INTR
  syslog(LOG_ERR, "imx9_msgintr_init: instance=%d\n", instance);
#endif

  DEBUGASSERT(instance == IMX9_MSGINTR_INSTANCE1 ||
              instance == IMX9_MSGINTR_INSTANCE2);
  DEBUGASSERT(callback != NULL);

  inst = &g_msgintr_inst[instance];

  inst->callback = callback;
  inst->arg      = arg;

  /* Route the MSGINTR peripheral IRQ through IRQSTEER to a reachable NVIC line.
   *
   * The M7 NVIC physically implements only extints 0-223 (ICTR nintlines=7).
   * MSGINTR1 (sysirq 246) and MSGINTR2 (sysirq 247) both live above that limit.
   * The WAKEUPMIX IRQSTEER fans these inputs into master0 whose NVIC output is
   * IMX9_IRQ_IRQSTEER_0 (extint 224 - within range on this silicon variant).
   *
   * Step 1: enable the IRQSTEER CHANMASK bit for this instance's input so the
   *         IRQSTEER hardware forwards the peripheral assertion to master0.
   * Step 2: attach the ISR to IMX9_IRQ_IRQSTEER_0 and enable it in the NVIC.
   *         Both instances share the same NVIC line; the ISR reads MSIR to
   *         identify the source.
   */

  sysirq  = (instance == IMX9_MSGINTR_INSTANCE1) ?
              IMX9_MSGINTR1_SYSIRQ : IMX9_MSGINTR2_SYSIRQ;
  regaddr = IRQSTEER_REG_ADDR(sysirq);
  bit     = 1u << IRQSTEER_REG_BIT(sysirq);

  val = getreg32(regaddr);
  putreg32(val | bit, regaddr);
  val = getreg32(regaddr);   /* readback */

#ifdef ENETC_VSI_DEBUG_INTR
  syslog(LOG_ERR,
         "MSGINTR%d: IRQSTEER CHANMASK sysirq=%u regaddr=0x%08x bit=0x%08x readback=0x%08x\n",
         instance, sysirq, (unsigned)regaddr, bit, val);

  syslog(LOG_ERR,
         "MSGINTR%d: attaching ISR to IMX9_IRQ_IRQSTEER_0=%d (extint=%d)\n",
         instance, IMX9_IRQ_IRQSTEER_0,
         (int)(IMX9_IRQ_IRQSTEER_0 - IMX9_IRQ_EXTINT));
#endif

  /* When CONFIG_ARCH_RAMVECTORS is enabled, the hardware VECTAB points to
   * g_ram_vectors[].  irq_attach() only records the handler in the NuttX
   * software dispatch table (g_irqvector[]) and does NOT update the RAM
   * vector table.  The hardware fires vector 240 (extint 224) and reads
   * from g_ram_vectors[240]; if that slot was never populated with a valid
   * Thumb function pointer the CPU takes a UsageFault/INVSTATE.
   *
   * arm_ramvec_attach() writes exception_common into g_ram_vectors[irq] so
   * the hardware has a valid entry and exception_common dispatches through
   * g_irqvector[] to our actual ISR.
   */
#ifdef CONFIG_ARCH_RAMVECTORS
  arm_ramvec_attach(inst->irq, exception_common);
#endif

  irq_attach(inst->irq, imx9_msgintr_isr, inst);
  up_enable_irq(inst->irq);

#ifdef ENETC_VSI_DEBUG_INTR
  /* Verify the IRQSTEER_0 NVIC enable bit was actually set. */
  {
    uint32_t extint   = (uint32_t)(inst->irq - IMX9_IRQ_EXTINT);
    uint32_t iser_idx = extint / 32u;
    uint32_t iser_bit = 1u << (extint % 32u);
    uint32_t iser_val = getreg32(0xe000e100u + iser_idx * 4u);
    syslog(LOG_ERR,
           "MSGINTR%d: NVIC ISER[%u]=0x%08x bit=0x%08x set=%d\n",
           instance, iser_idx, iser_val, iser_bit,
           (iser_val & iser_bit) ? 1 : 0);
  }
#endif

  return OK;
}

/****************************************************************************
 * Name: imx9_msgintr_deinit
 ****************************************************************************/

void imx9_msgintr_deinit(int instance)
{
  struct imx9_msgintr_inst_s   *inst;
  struct imx95_msgintr_regs_s  *regs;
  uint8_t ch;

  DEBUGASSERT(instance == IMX9_MSGINTR_INSTANCE1 ||
              instance == IMX9_MSGINTR_INSTANCE2);

  inst = &g_msgintr_inst[instance];
  regs = (struct imx95_msgintr_regs_s *)inst->base;

  up_disable_irq(inst->irq);
  irq_detach(inst->irq);

  inst->callback = NULL;
  inst->arg      = NULL;

  /* Drain any stale pending bits so no spurious interrupt fires on re-init */
  for (ch = 0; ch < IMX9_MSGINTR_CHANNEL_NUM; ch++)
    {
      (void)regs->msi[ch].msir;
    }
}

/****************************************************************************
 * Name: imx9_msgintr_get_addr
 ****************************************************************************/

uintptr_t imx9_msgintr_get_addr(int instance, uint8_t channel)
{
  DEBUGASSERT(instance == IMX9_MSGINTR_INSTANCE1 ||
              instance == IMX9_MSGINTR_INSTANCE2);
  DEBUGASSERT(channel < IMX9_MSGINTR_CHANNEL_NUM);

  return g_msgintr_inst[instance].base +
         (channel * IMX95_MSGINTR_MSI_STRIDE) +
         IMX95_MSGINTR_MSIIR_OFFSET;
}

/****************************************************************************
 * Name: imx9_msgintr_rearm
 ****************************************************************************/

void imx9_msgintr_rearm(int instance)
{
  uint32_t  sysirq;
  uintptr_t regaddr;
  uint32_t  bit;

  DEBUGASSERT(instance == IMX9_MSGINTR_INSTANCE1 ||
              instance == IMX9_MSGINTR_INSTANCE2);

  sysirq  = (instance == IMX9_MSGINTR_INSTANCE1) ?
              IMX9_MSGINTR1_SYSIRQ : IMX9_MSGINTR2_SYSIRQ;
  regaddr = IRQSTEER_REG_ADDR(sysirq);
  bit     = 1u << IRQSTEER_REG_BIT(sysirq);

  putreg32(getreg32(regaddr) | bit, regaddr);
}
