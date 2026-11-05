/****************************************************************************
 *
 * Copyright 2026 NXP
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

/* NXP NETC Virtual Station Interface (VSI) Ethernet driver for iMX95 M7.
 *
 * The PSI owner (typically Linux on the A55) manages the physical MAC and
 * enables SR-IOV.  This driver uses only VSI-specific SI registers, which
 * become accessible only after the PSI has enabled SR-IOV.
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/mutex.h>
#include <nuttx/kthread.h>
#include <nuttx/wdog.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>
#include <nuttx/net/net.h>
#include <nuttx/net/ip.h>
#include <net/ethernet.h>

#include "arm_internal.h"
#include "nvic.h"
#include "imx9_msgintr.h"

#include "imx9_netc_priv.h"
#include "imx9_netc_msg.h"

/* NXP MCUX SDK NETC HAL */

#include "fsl_netc_endpoint.h"
#include "fsl_netc_mdio.h"

/* Board-level configuration (SI index, MSGINTR instance, MAC address) */

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Stack size for the VSI service thread */

#ifndef CONFIG_IMX9_NETC_VSI_THREAD_STACKSIZE
#  define CONFIG_IMX9_NETC_VSI_THREAD_STACKSIZE  4096
#endif

/* Priority for the VSI service thread */

#ifndef CONFIG_IMX9_NETC_VSI_THREAD_PRIORITY
#  define CONFIG_IMX9_NETC_VSI_THREAD_PRIORITY  100
#endif

/* Polling interval while waiting for SR-IOV to be enabled (ms) */

#ifndef CONFIG_IMX9_NETC_VSI_READY_POLL_MS
#  define CONFIG_IMX9_NETC_VSI_READY_POLL_MS  500
#endif

/* Maximum RX frames processed per wakeup before yielding */

#ifndef CONFIG_IMX9_NETC_VSI_RX_BUDGET
#  define CONFIG_IMX9_NETC_VSI_RX_BUDGET  16
#endif

/* Shorthand for the Ethernet header at the start of d_buf */

#define BUF ((struct ether_header *)(priv->dev.d_buf))

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MSGINTR callback (forward declaration needed by imx9_netc_vsi_bring_up) */

static void imx9_netc_vsi_msgintr_cb(int instance, uint8_t channel,
                                      uint32_t pending, void *arg);

/* netdev ops */

static int  imx9_netc_vsi_ifup(struct net_driver_s *dev);
static int  imx9_netc_vsi_ifdown(struct net_driver_s *dev);
static int  imx9_netc_vsi_txavail(struct net_driver_s *dev);
static int  imx9_netc_vsi_transmit(struct net_driver_s *dev);
#ifdef CONFIG_NET_MCASTGROUP
static int  imx9_netc_vsi_addmac(struct net_driver_s *dev,
                                  const uint8_t *mac);
static int  imx9_netc_vsi_rmmac(struct net_driver_s *dev,
                                 const uint8_t *mac);
#endif
#ifdef CONFIG_NETDEV_IOCTL
static int  imx9_netc_vsi_ioctl(struct net_driver_s *dev, int cmd,
                                 unsigned long arg);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NETC_VSI_DEFINE_BUFFERS(n) -- declare all DMA-accessible buffers and
 * BD arrays for VSI instance n.  Placed in .netc_dma (non-cacheable DDR).
 * The semicolon after the last declaration is supplied by the call site.
 */

#define NETC_VSI_DEFINE_BUFFERS(n)                                            \
  uint8_t g_vsi##n##_tx_buff                                         \
    [CONFIG_IMX9_NETC_VSI_TX_RING_LEN][CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE]    \
    __attribute__((section(".netc_dma"), aligned(NETC_BUFF_ALIGN)));          \
  uint8_t g_vsi##n##_msg_buff[NETC_MSG_VSI_BUF_SIZE]                \
    __attribute__((section(".netc_dma"), aligned(NETC_BUFF_ALIGN)));          \
  netc_tx_bd_t                                                        \
    g_vsi##n##_txbd[CONFIG_IMX9_NETC_VSI_TX_RING_LEN]                        \
    __attribute__((section(".netc_dma"), aligned(NETC_BD_ALIGN)));            \
  netc_tx_frame_info_t                                                \
    g_vsi##n##_txdirty[CONFIG_IMX9_NETC_VSI_TX_RING_LEN];                    \
  netc_rx_buf_t                                                        \
    g_vsi##n##_rx_buff[CONFIG_IMX9_NETC_VSI_RX_RING_LEN]                     \
    __attribute__((section(".netc_dma"), aligned(NETC_BUFF_ALIGN)));          \
  uint64_t                                                             \
    g_vsi##n##_rx_buff_addr[CONFIG_IMX9_NETC_VSI_RX_RING_LEN];               \
  uint8_t g_vsi##n##_rx_frame[NETC_RX_FRAME_BUF_SIZE]               \
    __attribute__((section(".netc_dma"), aligned(NETC_BUFF_ALIGN)));          \
  uint8_t g_vsi##n##_tx_frame[NETC_TX_FRAME_BUF_SIZE]               \
    __attribute__((section(".netc_dma"), aligned(NETC_BUFF_ALIGN)));          \
  netc_rx_bd_t                                                        \
    g_vsi##n##_rxbd[CONFIG_IMX9_NETC_VSI_RX_RING_LEN]                        \
    __attribute__((section(".netc_dma"), aligned(NETC_BD_ALIGN)))

/* NETC_VSI_DEFINE_INSTANCE(n, _si, _msgintr, _mac_idx) -- define the BD
 * ring initializer function, the compile-time config struct, and the
 * per-instance driver state struct for VSI instance n.
 * The terminating semicolon is supplied by the call site.
 */

#define NETC_VSI_DEFINE_INSTANCE(n, _si, _msgintr, _mac_idx)                 \
  static void imx9_netc_vsi##n##_bdr_init(netc_bdr_config_t    *bdr,         \
                                           netc_rx_bdr_config_t *rx,          \
                                           netc_tx_bdr_config_t *tx)          \
  {                                                                             \
    uint16_t _i;                                                                \
    for (_i = 0; _i < CONFIG_IMX9_NETC_VSI_RX_RING_LEN; _i++)               \
      {                                                                         \
        g_vsi##n##_rx_buff_addr[_i] =                                         \
          (uint64_t)(uintptr_t)&g_vsi##n##_rx_buff[_i];                      \
      }                                                                         \
    bdr->rxBdrConfig    = rx;                                                   \
    bdr->txBdrConfig    = tx;                                                   \
    rx[0].bdArray       = (netc_rx_bd_t *)g_vsi##n##_rxbd;                   \
    rx[0].len           = CONFIG_IMX9_NETC_VSI_RX_RING_LEN;                  \
    rx[0].buffAddrArray = (uint64_t *)g_vsi##n##_rx_buff_addr;               \
    rx[0].buffSize      = NETC_RX_FRAME_BUF_SIZE;                             \
    rx[0].msixEntryIdx  = NETC_RX_MSIX_ENTRY_IDX;                             \
    rx[0].extendDescEn  = false;                                                \
    rx[0].enThresIntr   = true;                                                 \
    /* coalescing interrupt (RXCOIE) is the working RX interrupt source */     \
    rx[0].enCoalIntr    = true;                                                 \
    rx[0].intrThreshold = 1;                                                    \
    tx[0].bdArray       = (netc_tx_bd_t *)g_vsi##n##_txbd;                   \
    tx[0].len           = CONFIG_IMX9_NETC_VSI_TX_RING_LEN;                  \
    tx[0].dirtyArray    = (netc_tx_frame_info_t *)g_vsi##n##_txdirty;        \
    tx[0].msixEntryIdx  = NETC_TX_MSIX_ENTRY_IDX;                             \
    tx[0].enIntr        = false;                                                \
    tx[0].enThresIntr   = true;                                                 \
  }                                                                             \
  static const struct imx9_netc_vsi_config_s g_vsi##n##_config =             \
  {                                                                             \
    .si_idx            = (_si),                                                 \
    .msgintr_instance  = (_msgintr),                                            \
    .mac_default_index = (_mac_idx),                                            \
    .bdr_init          = imx9_netc_vsi##n##_bdr_init,                         \
  };                                                                            \
  static struct imx9_netc_vsi_s g_vsi##n

/* Instantiate buffers and driver state for each enabled VSI instance */

#ifdef CONFIG_IMX9_NETC_VSI0
NETC_VSI_DEFINE_BUFFERS(0);
NETC_VSI_DEFINE_INSTANCE(0, IMX9_NETC_VSI0_SI_IDX,
                          IMX9_NETC_VSI0_MSGINTR_INSTANCE, 0);
#endif /* CONFIG_IMX9_NETC_VSI0 */

#ifdef CONFIG_IMX9_NETC_VSI1
NETC_VSI_DEFINE_BUFFERS(1);
NETC_VSI_DEFINE_INSTANCE(1, IMX9_NETC_VSI1_SI_IDX,
                          IMX9_NETC_VSI1_MSGINTR_INSTANCE, 1);
#endif /* CONFIG_IMX9_NETC_VSI1 */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_netc_vsi_probe_sriov
 *
 * Description:
 *   Read the PSI's PCIe SR-IOV control register in a bus-fault-safe way.
 *
 *   When the PSI firewall (NETCMIX TRDC) is closed, or the PSI owner has
 *   disappeared (FLR, SR-IOV disable, A55/Linux reboot), a load from the
 *   PF config space either returns all-ones (0xFFFF) or raises an imprecise
 *   BusFault (CFSR IMPRECISERR) that would panic the M7. To survive this
 *   the probe sets the Configuration Control Register (CCR) BFHFNMIGN bit,
 *   which makes bus faults in privileged mode non-fatal: the faulting load
 *   is ignored and execution continues, the destination register taking an
 *   UNKNOWN value.  DSB/ISB barriers bracket the single read so the
 *   imprecise fault retires inside the window before CCR is restored. This
 *   mirrors the Zephyr NXP NETC VSI driver's bus-fault suppression during
 *   SR-IOV readiness polling.
 *
 *   Returns true only when the read is a plausible value (not the 0xFFFF
 *   firewall/absent sentinel) and both the VF-enable and memory-space-enable
 *   bits are set.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_netc_vsi_safe_read32
 *
 * Description:
 *   Read a 32-bit MMIO location with ARMv7-M bus faults suppressed, so a
 *   read of a firewalled / absent NETC window returns a value instead of
 *   panicking the M7.  The caller detects loss via an all-ones (0xFFFFFFFF)
 *   result.  See imx9_netc_vsi_probe_sriov() for the BFHFNMIGN rationale.
 *
 ****************************************************************************/

static uint32_t imx9_netc_vsi_safe_read32(volatile const uint32_t *addr)
{
  uint32_t ccr_save;
  uint32_t val;

  ccr_save = getreg32(NVIC_CFGCON);
  putreg32(ccr_save | NVIC_CFGCON_BFHFNMIGN, NVIC_CFGCON); /* ignore bus faults */
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");

  val = *addr;                                  /* may fault -> ignored */
  __asm__ volatile ("dsb" ::: "memory");

  putreg32(ccr_save, NVIC_CFGCON);              /* restore fault behavior */
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");

  return val;
}

/****************************************************************************
 * Name: imx9_netc_vsi_busfault_ignore / _restore
 *
 * Description:
 *   Enter/leave a window in which ARMv7-M data bus faults are non-fatal
 *   (CCR.BFHFNMIGN set).  Used to wrap multi-access SI sequences (the
 *   PSI mailbox / command-BDR path inside program_psi) that cannot be
 *   made race-free with a pre-check: Linux can tear the VF SI window down
 *   asynchronously *in the middle* of the sequence, so a probe that passed
 *   moments earlier is no guarantee.  Within the ignore window a faulting
 *   load returns UNKNOWN and a faulting store is dropped; the sequence then
 *   fails its own status/timeout checks and the caller retries once the PSI
 *   settles, instead of panicking the M7 on a precise BusFault.
 *
 *   Returns the saved CCR to pass to _restore().  Nesting is not supported;
 *   callers must pair begin/restore on the same path.
 *
 ****************************************************************************/

static uint32_t imx9_netc_vsi_busfault_ignore(void)
{
  uint32_t ccr_save = getreg32(NVIC_CFGCON);
  putreg32(ccr_save | NVIC_CFGCON_BFHFNMIGN, NVIC_CFGCON);
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");
  return ccr_save;
}

static void imx9_netc_vsi_busfault_restore(uint32_t ccr_save)
{
  __asm__ volatile ("dsb" ::: "memory");
  putreg32(ccr_save, NVIC_CFGCON);
  __asm__ volatile ("dsb" ::: "memory");
  __asm__ volatile ("isb" ::: "memory");
}

/****************************************************************************
 * Name: arm_busfault_fixup
 *
 * Description:
 *   Strong override of the weak hook in armv7-m/arm_busfault.c. The ARMv7-M
 *   CCR.BFHFNMIGN bit only suppresses bus faults while the core already runs
 *   at priority -1/-2 (HardFault/NMI); it does NOT help in thread or normal
 *   IRQ context, where an access to a NETC SI window that the PSI/A55 has
 *   torn down (Linux reboot / SR-IOV disable) escalates to a precise
 *   BusFault and panics the M7.
 *
 *   This handler is the mechanism the Zephyr NXP NETC driver uses: when the
 *   faulting data address (BFAR) lies inside the NETC SI register aperture,
 *   treat the access as benign - advance the stacked return PC past the
 *   faulting load/store (decoding Thumb 16- vs 32-bit instruction length)
 *   and resume. The in-flight SI sequence then observes an UNKNOWN read /
 *   dropped write, fails its own status/timeout check, and the VSI service
 *   thread detects the loss on its next vf_ready() poll and tears the SI
 *   down cleanly. Any fault outside the NETC aperture is left to panic.
 *
 *   NETC SI aperture on iMX95: the per-SI windows observed in the crash
 *   logs (0x4ca0_0000, 0x4cd3_0000, 0x4cd7_0000, ...) all fall within the
 *   NETC block 0x4C000000..0x4CFFFFFF.
 *
 ****************************************************************************/

#define IMX9_NETC_SI_BASE   0x4c000000ul
#define IMX9_NETC_SI_LIMIT  0x4cfffffful

bool arm_busfault_fixup(uint32_t *regs, uint32_t cfsr)
{
  uint32_t bfar;
  uint32_t pc;
  uint16_t hw;

  /* Imprecise data bus error (CFSR IMPRECISERR, BFAR not valid).
   *
   * This is a buffered store that already retired; the fault surfaces on
   * a later, unrelated instruction, so BFAR is meaningless and the stacked
   * PC does NOT point at the offending access.  We therefore must NOT
   * advance the PC - the current instruction is innocent and should run.
   *
   * On this M7 the only Device-memory writes that can asynchronously fault
   * are to the NETC SI window being torn down by the PSI/A55 (the driver's
   * SI register stores), so an imprecise bus error here is always a NETC
   * teardown artifact.  Barrier to drain any further buffered writes, then
   * acknowledge and resume at the (innocent) stacked PC.
   */

  if ((cfsr & NVIC_CFAULTS_IMPRECISERR) != 0)
    {
      __asm__ volatile ("dsb" ::: "memory");
      __asm__ volatile ("isb" ::: "memory");
      return true;
    }

  /* Precise error: the stacked PC points at the faulting load/store and
   * BFAR holds its target address.  Only act when that target is inside
   * the NETC SI aperture.
   *
   * Do NOT require CFSR.BFARVALID here.  A precise fault taken while
   * CCR.BFHFNMIGN is set (the program_psi bus-fault-ignore window) can be
   * delivered with the PRECISERR/BFARVALID status bits clear (observed as
   * CFSR=0x00000000) while BFAR still latches the real faulting NETC SI
   * address.  Gating on BFARVALID would miss that case and panic.  BFAR is
   * only meaningless for an imprecise error, which we already returned for
   * above, so reading it unconditionally here is safe.
   */

  bfar = getreg32(NVIC_BFAULT_ADDR);
  if (bfar < IMX9_NETC_SI_BASE || bfar > IMX9_NETC_SI_LIMIT)
    {
      return false;
    }

  /* The faulting instruction is at the stacked return PC.  Decode its
   * length: a 16-bit Thumb instruction has its first halfword top 5 bits
   * in 0b11100 or lower (0x0000-0xE7FF); values 0xE800-0xFFFF are the
   * first halfword of a 32-bit Thumb-2 instruction.  Advance PC by 2 or 4
   * so execution resumes at the next instruction instead of re-faulting.
   */

  pc = regs[REG_PC];
  hw = *(volatile const uint16_t *)(uintptr_t)pc;

  if (hw >= 0xe800u)
    {
      regs[REG_PC] = pc + 4u;   /* 32-bit Thumb-2 */
    }
  else
    {
      regs[REG_PC] = pc + 2u;   /* 16-bit Thumb */
    }

  return true;
}

static bool imx9_netc_vsi_probe_sriov(ENETC_PCI_TYPE0_Type *pf,
                                      uint16_t mask)
{
  uint16_t sriov_ctl;

  sriov_ctl = (uint16_t)imx9_netc_vsi_safe_read32(
                  (volatile const uint32_t *)&pf->PCIE_CFC_SRIOV_CTL);

  /* 0xFFFF == firewall closed or PF gone; require both ready bits set. */

  return (sriov_ctl != 0xffffu) && ((sriov_ctl & mask) == mask);
}

/****************************************************************************
 * Name: imx9_netc_vsi_vf_ready
 *
 * Description:
 *   Check whether the PSI has enabled SR-IOV for our ENETC instance.
 *   Reads the PCI VF-enable and memory-space-enable bits from the PSI's
 *   PCIe SR-IOV control register.
 *
 *   This always performs the (bus-fault-safe) probe rather than short-
 *   circuiting on priv->vf_confirmed, so that a PSI that disappears after
 *   initial bring-up is detected live.  vf_confirmed is used only to gate
 *   the one-shot readiness wait in imx9_netc_vsi_wait_ready(), never as an
 *   access guard.
 *
 ****************************************************************************/

static bool imx9_netc_vsi_vf_ready(struct imx9_netc_vsi_s *priv)
{
  const struct imx9_netc_vsi_config_s *cfg = priv->config;
  ENETC_PCI_TYPE0_Type *pf;
  ENETC_SI_Type        *si;
  uint16_t vf_ready;

  /* PF PCIe config base addresses (one per ENETC MAC instance).
   * Instance 0 -> index 1 in ENETC_PCI_TYPE0_BASE_ADDRS (index 0 is VSI).
   */

  static const uintptr_t pf_pci_base[] = ENETC_PCI_TYPE0_BASE_ADDRS;

  pf = (ENETC_PCI_TYPE0_Type *)
       pf_pci_base[1u + getSiInstance((netc_hw_si_idx_t)cfg->si_idx)];

  vf_ready = ENETC_PCI_TYPE0_PCIE_CFC_SRIOV_CTL_VF_ENABLE_MASK |
             ENETC_PCI_TYPE0_PCIE_CFC_SRIOV_CTL_VF_MSE_MASK;

  if (!imx9_netc_vsi_probe_sriov(pf, vf_ready))
    {
      return false;
    }

  /* The PF SR-IOV control bit can still read "enabled" for a short window
   * after the PSI/A55 has torn the VF down (reboot / FLR): the PF config
   * space and the VF SI data window are behind different firewall regions,
   * and the SI window traps (precise BusFault) before the PF bit clears.
   * So the PF bit alone is not a sufficient liveness signal.  Additionally
   * probe the actual VSI SI register window - SICAPR0 is a read-only
   * capability register at offset 0x900 that reads back a fixed non-zero
   * value when the window is live and all-ones (or faults -> all-ones via
   * the safe reader) once the window is firewalled.  Only report ready
   * when BOTH the PF bit and the live SI window agree.
   */

  si = priv->handle.hw.si;
  if (si == NULL)
    {
      return false;
    }

  {
    uint32_t cap = imx9_netc_vsi_safe_read32(
                       (volatile const uint32_t *)&si->SICAPR0);
    if (cap == 0xffffffffu || cap == 0u)
      {
        return false;
      }
  }

  return true;
}

/****************************************************************************
 * Name: imx9_netc_vsi_wait_ready
 *
 * Description:
 *   Block until the SR-IOV VF-enable bit is set by the PSI.
 *   Called from the service thread (not from init).
 *
 ****************************************************************************/

static void imx9_netc_vsi_wait_ready(struct imx9_netc_vsi_s *priv)
{
  /* If PSI firewall was already opened during a previous bring_up (e.g.
   * after ifdown/ifup), skip the blocking wait entirely -- the firewall
   * remains open and no new MU messages will arrive to unblock us.
   */
  if (priv->vf_confirmed)
    {
#ifdef ENETC_VSI_DEBUG_INTR
      syslog(LOG_ERR,
             "NETC VSI: PSI already confirmed, skipping wait\n");
#endif
      return;
    }

#ifdef ENETC_VSI_DEBUG_INTR
  syslog(LOG_ERR, "NETC VSI: waiting for PSI ready signal (A55 boot)...\n");
#endif

  /* Poll the (bus-fault-safe) SR-IOV readiness probe until the PSI/A55
   * NETC PF driver has enabled our VF and opened the NETCMIX TRDC firewall.
   *
   * This replaces the previous dependency on the rptun MU7 first kick plus
   * a fixed CONFIG_IMX9_NETC_VSI_RPTUN_READY_DELAY_MS settle delay.  The
   * rptun kick only indicated that Linux remoteproc/rpmsg had initialized;
   * it was an indirect proxy for "PF driver loaded and firewall open" and
   * forced a long worst-case wait.  imx9_netc_vsi_vf_ready() checks the
   * actual hardware state (PF SR-IOV VF-enable bit AND a live read of the
   * VSI SI capability window), so we can detect readiness directly without
   * the MU dependency and without over-waiting.
   */
  syslog(LOG_ERR, "NETC VSI: polling for SR-IOV VF enable...\n");
  while (!imx9_netc_vsi_vf_ready(priv))
    {
      nxsig_usleep(CONFIG_IMX9_NETC_VSI_READY_POLL_MS * 1000u);
    }
  syslog(LOG_ERR, "NETC VSI: SR-IOV VF ready, proceeding to SI bringup\n");

  priv->vf_confirmed = true;
}

/****************************************************************************
 * Name: imx9_vsi_config_tx_bdr
 *
 * Description:
 *   Configure one TX BD ring on the SI.  Delegates to the SDK function
 *   NETC_SIConfigTxBDR which programs SIMSITRVR, TBBAR0/1, TBPIR, TBCIR,
 *   TBLENR, TBIER, and TBMR.  These registers live in the VSI SI space
 *   (hw.si = ENETC_VSI0 base) and are accessible from M7 after the PSI
 *   has enabled SR-IOV.
 *
 ****************************************************************************/

static int imx9_vsi_config_tx_bdr(ENETC_SI_Type *base, uint8_t ring,
                                    const netc_tx_bdr_config_t *cfg)
{
  if (NETC_SIConfigTxBDR(base, ring, cfg) != kStatus_Success)
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: imx9_vsi_config_rx_bdr
 *
 * Description:
 *   Configure one RX BD ring on the SI. Delegates to the SDK function
 *   NETC_SIConfigRxBDR which programs SIMSIRRVR, RBBAR0/1, RBBSR, RBPIR,
 *   RBCIR, RBLENR, RBIER, RBICR0/1, and RBMR.  These registers live in
 *   the VSI SI space and are accessible from M7 after SR-IOV is enabled.
 *
 ****************************************************************************/

static int imx9_vsi_config_rx_bdr(ENETC_SI_Type *base, uint8_t ring,
                                    const netc_rx_bdr_config_t *cfg)
{
  if (NETC_SIConfigRxBDR(base, ring, cfg) != kStatus_Success)
    {
      return -EINVAL;
    }

  return 0;
}

/****************************************************************************
 * Name: imx9_netc_tx_coalesce_init
 *
 * Description:
 *   Program the SI Tx interrupt-coalescing threshold registers.
 *
 ****************************************************************************/

void imx9_netc_tx_coalesce_init(struct imx9_netc_vsi_s *priv)
{
  netc_tx_bdr_t *ring = &priv->handle.txBdRing[0];

  /* NETC_TxRingSetCoalesce is not present in this SDK version.
   * Interrupt coalescing can be configured directly if needed. */
  (void)ring;
}

/****************************************************************************
 * Name: imx9_netc_vsi_si_bringup
 *
 * Description:
 *   Configure and enable the SI BD rings, set up MSI-X vectors, and
 *   enable the SI.  Called from the service thread after SR-IOV is ready.
 *
 ****************************************************************************/

static int imx9_netc_vsi_si_bringup(struct imx9_netc_vsi_s *priv)
{
  const struct imx9_netc_vsi_config_s *cfg = priv->config;
  ep_handle_t       *handle = &priv->handle;
  netc_rx_bdr_config_t rx_cfg[1];
  netc_tx_bdr_config_t tx_cfg[1];
  netc_bdr_config_t    bdr_cfg;
  uint8_t              ring;

  /* Map the SI registers now that SR-IOV is enabled by the PSI */

  NETC_SocGetBaseResource(&handle->hw, (netc_hw_si_idx_t)cfg->si_idx);

  syslog(LOG_ERR,
         "VSI si_bringup: si_idx=0x%04x hw.base=%p hw.si=%p hw.msixTable=%p\n",
         cfg->si_idx,
         handle->hw.base,
         handle->hw.si,
         handle->hw.msixTable);

  memset(rx_cfg, 0, sizeof(rx_cfg));
  memset(tx_cfg, 0, sizeof(tx_cfg));
  memset(&bdr_cfg, 0, sizeof(bdr_cfg));

  cfg->bdr_init(&bdr_cfg, rx_cfg, tx_cfg);

  /* Populate capability.vsiNum without accessing the PSI-privileged
   * ENETC management block (owned by the PSI/A55).  See
   * NETC_VSI_ENETC0_VSI_NUM in imx9_netc_priv.h for the rationale.
   */
  handle->capability.vsiNum = NETC_VSI_ENETC0_VSI_NUM;

  /* Set RX BD ring grouping: 1 group of 1 ring. */
  NETC_SISetRxBDRGroup(handle->hw.si, 0, 1);
  NETC_SISetDefaultRxBDRGroup(handle->hw.si, kNETC_SiBDRGroupOne);

  /* Configure TX BD rings */
  for (ring = 0; ring < handle->cfg.txRingUse; ring++)
    {
      if (imx9_vsi_config_tx_bdr(handle->hw.si, ring,
                                  &bdr_cfg.txBdrConfig[ring]) != 0)
        {
          nerr("VSI Tx BD ring %u config failed\n", ring);
          return -ENOBUFS;
        }

      handle->txBdRing[ring].bdBase       = bdr_cfg.txBdrConfig[ring].bdArray;
      handle->txBdRing[ring].dirtyBase    = bdr_cfg.txBdrConfig[ring].dirtyArray;
      handle->txBdRing[ring].len          = bdr_cfg.txBdrConfig[ring].len;
      handle->txBdRing[ring].producerIndex = 0;
      handle->txBdRing[ring].cleanIndex   = 0;
    }

  imx9_netc_tx_coalesce_init(priv);

  /* Configure RX BD rings */
  for (ring = 0; ring < handle->cfg.rxRingUse; ring++)
    {
      const netc_rx_bdr_config_t *rc = &bdr_cfg.rxBdrConfig[ring];
      netc_rx_bd_t *rx_desc;
      uint16_t      idx;

      if (imx9_vsi_config_rx_bdr(handle->hw.si, ring, rc) != 0)
        {
          nerr("VSI Rx BD ring %u config failed\n", ring);
          return -ENOBUFS;
        }

      handle->rxBdRing[ring].extendDesc = rc->extendDescEn;
      handle->rxBdRing[ring].bdBase     = rc->bdArray;
      handle->rxBdRing[ring].len        = rc->len;
      handle->rxBdRing[ring].buffSize   = rc->buffSize;
      handle->rxBdRing[ring].index      = 0;

      /* Attach RX buffers to each descriptor */
      rx_desc = &rc->bdArray[0];
      handle->rxBdRing[ring].buffArray = rc->buffAddrArray;

      for (idx = 0; idx < rc->len; idx++)
        {
          memset(rx_desc, 0, sizeof(netc_rx_bd_t));
          rx_desc->standard.addr =
            (uint64_t)MEMORY_ConvertMemoryMapAddress(
              (uintptr_t)handle->rxBdRing[ring].buffArray[idx],
              kMEMORY_Local2DMA);
          rx_desc++;
        }



      /* Enable the RX ring (sets RBMR.EN in the VSI SI space). */
      NETC_SIRxRingEnable(handle->hw.si, ring, true);
    }

  NETC_SIEnableVlanToIpv(handle->hw.si, false);
  NETC_SIEnable(handle->hw.si, true);
  ninfo("VSI si_bringup: SI enabled\n");

  /* Program the MSI-X table AFTER SI enable.
   * NETC_SIEnable() may reset the MSI-X table on some silicon revisions;
   * writing the table before SIEnable causes the entries to be cleared.
   * Reference implementation writes the table inside EP_Init() which is
   * called after all ring configuration but the SI may not yet be fully
   * enabled at that point.  Writing here, after SIEnable, guarantees the
   * table persists.
   *
   * Each entry: [addr_lo, addr_hi, data (IBS), control (0=unmasked)]
   */
  {
    uint32_t  *tbl   = (uint32_t *)(uintptr_t)handle->hw.msixTable;
    uintptr_t  msiir = imx9_msgintr_get_addr(cfg->msgintr_instance,
                                              NETC_MSGINTR_CHANNEL);

    tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 0u] = (uint32_t)(msiir & 0xffffffffu);
    tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 1u] = 0u;
    tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 2u] = NETC_TX_MSGINTR_IBS;
    tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 3u] = 0u;

    tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 0u] = (uint32_t)(msiir & 0xffffffffu);
    tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 1u] = 0u;
    tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 2u] = NETC_RX_MSGINTR_IBS;
    tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 3u] = 0u;

    tbl[NETC_MSG_MSIX_ENTRY_IDX * 4u + 0u] = (uint32_t)(msiir & 0xffffffffu);
    tbl[NETC_MSG_MSIX_ENTRY_IDX * 4u + 1u] = 0u;
    tbl[NETC_MSG_MSIX_ENTRY_IDX * 4u + 2u] = NETC_MSG_MSGINTR_IBS;
    tbl[NETC_MSG_MSIX_ENTRY_IDX * 4u + 3u] = 0u;

    /* DSB to ensure all table writes hit SRAM before RBIER doorbell */
    __asm__ volatile("dsb" ::: "memory");

#ifdef ENETC_VSI_DEBUG_INTR
    /* Readback to verify table was not cleared */
    syslog(LOG_ERR,
           "VSI MSIX tbl@%p: "
           "TX[addr=0x%08x data=%u ctl=%u] "
           "RX[addr=0x%08x data=%u ctl=%u] "
           "msiir=0x%08x\n",
           tbl,
           tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 0u],
           tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 2u],
           tbl[NETC_TX_MSIX_ENTRY_IDX * 4u + 3u],
           tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 0u],
           tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 2u],
           tbl[NETC_RX_MSIX_ENTRY_IDX * 4u + 3u],
           (unsigned)msiir);
#endif
  }

  /* Arm the RX threshold interrupt immediately so the very first incoming
   * frame fires MSI-X channel 1 without waiting for the service thread to
   * complete a full drain-then-rearm cycle.  Without this, frames that
   * arrive within the first 500 ms poll window (e.g. ARP replies) land in
   * the ring silently because RBIER is still 0 and no MSI-X is generated. */
  handle->hw.si->BDR[0].RBIER = ENETC_SI_RBIER_RXTIE(1U);

#ifdef ENETC_VSI_DEBUG_INTR
  /* Readback RBIER to verify it was not cleared by a later HAL call. */
  syslog(LOG_ERR, "VSI RBIER readback=0x%08x (expected 0x1)\n",
         (unsigned)handle->hw.si->BDR[0].RBIER);
  ninfo("VSI si_bringup: RBIER armed\n");
#endif

  /* Point the PSI->VSI message interrupt at the MSG MSI-X entry */
  handle->hw.si->SIMSIVR =
    NETC_MSG_MSIX_ENTRY_IDX & ENETC_SI_SIMSIVR_VECTOR_MASK;

  /* Drain any PSI->VSI message that arrived before this bringup.
   *
   * The Linux PF driver polls PSIMSGSR (100 ms timeout) waiting for
   * the VSI to acknowledge a sent link-status message by reading
   * VSIMSGRR.  Hardware clears PSIMSGSR[MS(i)] only when the VSI reads
   * VSIMSGRR - NOT when VSIIDR.MR is cleared via EP_VsiClearStatus.
   *
   * If the PF sent a link-status notification before this bringup (e.g.
   * during the VSI init delay), PSIMSGSR[MS(0)] is already set.
   * Without reading VSIMSGRR here, EP_VsiClearStatus only silences the
   * local VSIIDR.MR flag while leaving PSIMSGSR set.  The PF then times
   * out and logs "PF notifies link status failed".
   *
   * Read VSIMSGRR unconditionally: if a message is pending it clears
   * PSIMSGSR; if not, the read is harmless (hardware returns 0).
   * Discard the value - the message will be re-notified via the
   * link-notify subscription set up after bringup.
   *
   * After draining, clear VSIIDR.MR/TX to suppress the spurious MSI-X
   * edge that would otherwise fire as soon as VSIIER is enabled and
   * cause a premature link_notify / double program_psi sequence.
   */

  EP_VsiReceiveMsg(handle, &(uint16_t){0});

  EP_VsiClearStatus(handle,
                    (uint32_t)kNETC_VsiMsgRxFlag |
                    (uint32_t)kNETC_VsiMsgTxFlag);
  EP_VsiEnableInterrupt(handle, kNETC_VsiMsgRxFlag, true);

  return 0;
}

/****************************************************************************
 * Name: imx9_netc_vsi_program_mac
 *
 * Description:
 *   Ask the PSI to install our primary unicast MAC filter, retrying briefly
 *   while the PSI is still starting up.
 *
 ****************************************************************************/

static void imx9_netc_vsi_program_mac(struct imx9_netc_vsi_s *priv)
{
  int attempt;
  int ret;

  syslog(LOG_ERR, "VSI: programming MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
         priv->mac_addr[0], priv->mac_addr[1], priv->mac_addr[2],
         priv->mac_addr[3], priv->mac_addr[4], priv->mac_addr[5]);

  for (attempt = 0; attempt < NETC_VSI_MAC_SET_RETRIES; attempt++)
    {
      ret = imx9_netc_vsi_msg_set_primary_mac(priv);
      syslog(LOG_ERR, "VSI: MAC filter attempt %d ret=%d\n", attempt, ret);
      if (ret == 0)
        {
          syslog(LOG_ERR, "VSI: MAC filter accepted by PSI\n");
          return;
        }

      nxsig_usleep(NETC_VSI_MAC_SET_RETRY_MS * 1000);
    }

  syslog(LOG_ERR, "VSI: MAC filter REJECTED by PSI - no unicast RX!\n");
}

/****************************************************************************
 * Name: imx9_netc_vsi_program_psi
 *
 * Description:
 *   Push MAC address, multicast hash, and VLAN hash to the PSI.
 *   Marks si_ready = true under msg_lock so no join/leave is missed.
 *
 ****************************************************************************/

static void imx9_netc_vsi_program_psi(struct imx9_netc_vsi_s *priv)
{
  uint32_t ccr_save;

  /* program_psi reaches the SI mailbox / command-BDR register path through
   * the msg helpers (set_primary_mac / set_mac_hash / set_vlan_hash).  The
   * PSI/A55 can tear the VF SI window down asynchronously *in the middle*
   * of this multi-register sequence (Linux reboot), so a pre-check with
   * vf_ready() is racy - the window may pass the probe and then trap a few
   * instructions later.  Run the whole sequence with data bus faults
   * suppressed: a faulting access degrades to an UNKNOWN read / dropped
   * write, the msg helper then fails its own status/timeout check and
   * returns an error, and the service thread's next loop detects the loss
   * via vf_ready() and tears the SI down cleanly.  This converts the
   * precise BusFault panic into a recoverable soft failure.
   */

  ccr_save = imx9_netc_vsi_busfault_ignore();

  imx9_netc_vsi_program_mac(priv);

  nxrmutex_lock(&priv->msg_lock);
  priv->si_ready = true;

  if (imx9_netc_vsi_msg_set_mac_hash(priv) != 0)
    {
      nwarn("VSI MC hash not programmed\n");
    }

  if (imx9_netc_vsi_msg_set_vlan_hash(priv) != 0)
    {
      nwarn("VSI VLAN hash not programmed\n");
    }

  nxrmutex_unlock(&priv->msg_lock);

  imx9_netc_vsi_busfault_restore(ccr_save);
}


/****************************************************************************
 * Name: imx9_netc_vsi_bring_up
 *
 * Description:
 *   Full SI bring-up sequence: wait for SR-IOV, configure BD rings, push
 *   steering to PSI, subscribe to link notifications, raise carrier.
 *
 ****************************************************************************/

static void imx9_netc_vsi_bring_up(struct imx9_netc_vsi_s *priv)
{
  int ret;

  imx9_netc_vsi_wait_ready(priv);

  do
    {
      ret = imx9_netc_vsi_si_bringup(priv);
      if (ret != 0)
        {
          nerr("VSI SI bring-up failed (%d), retrying\n", ret);
          nxsig_usleep(CONFIG_IMX9_NETC_VSI_READY_POLL_MS * 1000);
        }
    }
  while (ret != 0);

  imx9_netc_vsi_program_psi(priv);

  /* Raise carrier immediately after bringup.  The physical link is up at
   * this point (PSI has enabled SR-IOV, MAC filter is programmed, SI is
   * running).  If the PSI supports link-status notifications it will send
   * a link-down message later should the link drop; if not, we stay up.
   */
  netdev_carrier_on(&priv->dev);

  if (imx9_netc_vsi_msg_enable_link_notify(priv) == 0)
    {
      syslog(LOG_ERR, "VSI %s: carrier up, link-notify enabled\n",
             priv->dev.d_ifname);
    }
  else
    {
      syslog(LOG_ERR, "VSI %s: carrier up (link-notify unavailable)\n",
             priv->dev.d_ifname);
    }
}

/****************************************************************************
 * Name: imx9_netc_vsi_rx
 *
 * Description:
 *   Drain one Rx frame from the SI ring and deliver it to the network stack.
 *
 * Returned Value:
 *   0 if a frame was received, -ENOBUFS when the ring is empty.
 *
 ****************************************************************************/

int imx9_netc_vsi_rx(struct imx9_netc_vsi_s *priv)
{
  ep_handle_t      *handle = &priv->handle;
  netc_frame_attr_t attr;
  status_t          status;
  status_t          rx_result;
  uint32_t          length;

#ifdef ENETC_VSI_DEBUG_TRX
  syslog(LOG_ERR, "\n\nVSI %s: RX entering\n", priv->dev.d_ifname);
#endif

  /* Check rx frame size; any result other than success or empty requires
   * dropping the current BD to recycle it back to hardware.  Calling
   * EP_ReceiveFrameCopy with a NULL buffer triggers EP_DropFrame internally,
   * which advances rxBdRing->index and writes RBCIR, preventing ring stall.
   *
   * The coalescing interrupt (RXCOIE) fires when the NETC fabric counts a
   * received frame, which can happen slightly before the DMA writeback of
   * the BD to DDR is visible to the M7 before the isReady check.
   * Retry up to ~10 us to let the NETC DMA writeback complete
   * complete before concluding the ring is empty. */

  {
    int _retry = 2;  /* 2 x 5 us = 10 us max */
    do
      {
        rx_result = EP_GetRxFrameSize(handle, 0, &length);
        if (rx_result != kStatus_NETC_RxFrameEmpty)
          {
            break;
          }

        up_udelay(5);
      }
    while (--_retry > 0);
  }

  if (rx_result == kStatus_NETC_RxFrameEmpty)
    {
      return -ENOBUFS;
    }

  /* Drop frame immediately if interface is not administratively up.
   * Do this before EP_ReceiveFrameCopy and any debug syslog prints to
   * avoid stalling the drain loop with slow serial output during traffic.
   */

  if ((priv->dev.d_flags & IFF_UP) == 0)
    {
      EP_ReceiveFrameCopy(handle, 0, NULL, 0, NULL);
      return 0;
    }

#ifdef ENETC_VSI_DEBUG_TRX
  /* Log raw BD fields to diagnose non-regular frames. */
  {
    volatile netc_rx_bd_t *bd =
        &handle->rxBdRing[0].bdBase[handle->rxBdRing[0].index];
    syslog(LOG_ERR,
           "VSI %s: RX rx_result=%d length=%u "
           "bd[%u]: isReady=%u isFinal=%u error=%u hr=%u bufLen=%u\n",
           priv->dev.d_ifname, (int)rx_result, (unsigned)length,
           (unsigned)handle->rxBdRing[0].index,
           (unsigned)bd->writeback.isReady,
           (unsigned)bd->writeback.isFinal,
           (unsigned)bd->writeback.error,
           (unsigned)bd->writeback.hr,
           (unsigned)bd->writeback.bufLen);
  }
#endif

  if (rx_result != kStatus_Success)
    {
      /* Non-regular or error frame: log and drop.
       *
       * EP_DropFrame loops until isFinal=1.  If the PSI delivers a
       * management/admin frame with isFinal=0 (zero-length notification),
       * EP_DropFrame would consume extra BDs from the ring and stall valid
       * frames.  Guard: force isFinal=1 on the current BD before dropping
       * so EP_DropFrame consumes exactly one descriptor. */
      volatile netc_rx_bd_t *bd =
          &handle->rxBdRing[0].bdBase[handle->rxBdRing[0].index];

      if (bd->writeback.isFinal == 0U)
        {
          syslog(LOG_WARNING,
                 "VSI RX: non-regular frame hr=%u has isFinal=0 at bd[%u],"
                 " forcing isFinal=1 to prevent ring over-consume\n",
                 (unsigned)bd->writeback.hr,
                 (unsigned)handle->rxBdRing[0].index);
          bd->writeback.isFinal = 1U;
        }

      syslog(LOG_ERR, "VSI %s: RX dropping frame, EP_GetRxFrameSize=%d\n",
             priv->dev.d_ifname, (int)rx_result);
      EP_ReceiveFrameCopy(handle, 0, NULL, 0, NULL);
      return -EIO;
    }

  status = EP_ReceiveFrameCopy(handle, 0, priv->rx_frame, length, &attr);
#ifdef ENETC_VSI_DEBUG_TRX
  syslog(LOG_ERR, "VSI %s: RX status=%d len=%u\n", priv->dev.d_ifname, (int)status, (unsigned)length);
#endif
  if (status == kStatus_NETC_RxFrameEmpty)
    {
      return -ENOBUFS;
    }

  if (status != kStatus_Success || length == 0)
    {
      syslog(LOG_ERR, "VSI %s: RX error status=%d len=%u\n",
             priv->dev.d_ifname, (int)status, (unsigned)length);
      return -EIO;
    }

#ifdef ENETC_VSI_DEBUG_TRX
  {
    const uint8_t *f = priv->rx_frame;
    static uint32_t rx_frames;
    rx_frames++;
    syslog(LOG_ERR,
           "VSI %s: RX#%u len=%u "
           "dst=%02x:%02x:%02x:%02x:%02x:%02x "
           "src=%02x:%02x:%02x:%02x:%02x:%02x "
           "type=%02x%02x\n",
           priv->dev.d_ifname, rx_frames, (unsigned)length,
           f[0],  f[1],  f[2],  f[3],  f[4],  f[5],
           f[6],  f[7],  f[8],  f[9],  f[10], f[11],
           f[12], f[13]);
  }
#endif

  /* Hand the frame to the NuttX network stack */

  net_lock();

  priv->dev.d_buf = priv->rx_frame;
  priv->dev.d_len = (uint16_t)length;

#ifdef CONFIG_NET_IPv4
  if (BUF->ether_type == HTONS(ETHTYPE_IP))
    {
      size_t reply_len;
#ifdef CONFIG_NET_ARP
      arp_ipin(&priv->dev);
#endif
      ipv4_input(&priv->dev);
      reply_len = priv->dev.d_len;
      priv->dev.d_len = 0;
      net_unlock();
      if (reply_len > 0)
        {
          imx9_netc_vsi_tx(priv, priv->rx_frame, reply_len);
        }

      return 0;
    }
#ifdef CONFIG_NET_ARP
  else if (BUF->ether_type == HTONS(ETHTYPE_ARP))
    {
      size_t reply_len;
      arp_input(&priv->dev);
      reply_len = priv->dev.d_len;
      priv->dev.d_len = 0;
      net_unlock();
      if (reply_len > 0)
        {
          imx9_netc_vsi_tx(priv, priv->rx_frame, reply_len);
        }

      return 0;
    }
#endif
#endif

#ifdef CONFIG_NET_IPv6
  if (BUF->ether_type == HTONS(ETHTYPE_IP6))
    {
      size_t reply_len;
      ipv6_input(&priv->dev);
      reply_len = priv->dev.d_len;
      priv->dev.d_len = 0;
      net_unlock();
      if (reply_len > 0)
        {
          imx9_netc_vsi_tx(priv, priv->rx_frame, reply_len);
        }

      return 0;
    }
#endif

  net_unlock();

  return 0;
}

/****************************************************************************
 * Name: imx9_netc_vsi_tx
 *
 * Description:
 *   Transmit one Ethernet frame via the SI TX BD ring.
 *
 ****************************************************************************/

int imx9_netc_vsi_tx(struct imx9_netc_vsi_s *priv,
                     const uint8_t *buf, size_t len)
{
  ep_handle_t       *handle  = &priv->handle;
  uint8_t           *ddr_buf;
  status_t           status;
  unsigned int       retry;
  netc_buffer_struct_t pkt;
  netc_frame_struct_t frame  =
    {
      .buffArray = &pkt,
      .length    = 1,
    };

  /* Bounded retry when the TX BD ring has no idle descriptor left.
   * NETC_TX_SEND_RETRIES short waits of NETC_TX_SEND_RETRY_US microseconds
   * give the NETC DMA time to advance its TX consumer index so BDs can be
   * reclaimed. */

  const unsigned int NETC_TX_SEND_RETRIES  = 8u;
  const unsigned int NETC_TX_SEND_RETRY_US = 100u;

  if (len > NETC_TX_FRAME_BUF_SIZE)
    {
      return -EMSGSIZE;
    }


  /* The source buffer may be in M7 SRAM (not accessible by NETC DMA).
   * Always copy the frame into a DDR (.netc_dma) TX buffer first so the
   * A55 NETC DMA engine can reach it.  Use a round-robin index across the
   * TX ring slots so concurrent sends don't alias. */

  ddr_buf = priv->tx_buff[priv->tx_idx];
  priv->tx_idx = (priv->tx_idx + 1u) %
                 (uint8_t)CONFIG_IMX9_NETC_VSI_TX_RING_LEN;

  memcpy(ddr_buf, buf, len);

#ifdef ENETC_VSI_DEBUG_TRX
  {
    static uint32_t tx_dbg;
    tx_dbg++;
    syslog(LOG_ERR,
           "VSI %s: TX#%u len=%u "
           "dst=%02x:%02x:%02x:%02x:%02x:%02x "
           "src=%02x:%02x:%02x:%02x:%02x:%02x "
           "type=%02x%02x\n",
           priv->dev.d_ifname, tx_dbg, (unsigned)len,
           ddr_buf[0],  ddr_buf[1],  ddr_buf[2],
           ddr_buf[3],  ddr_buf[4],  ddr_buf[5],
           ddr_buf[6],  ddr_buf[7],  ddr_buf[8],
           ddr_buf[9],  ddr_buf[10], ddr_buf[11],
           ddr_buf[12], ddr_buf[13]);
  }
#endif

  pkt.buffer = ddr_buf;
  pkt.length = (uint32_t)len;

  EP_ReclaimTxDescriptor(handle, 0);

  /* DSB: drain the M7 write buffer so the TX BD fields written by the
   * previous memcpy and EP_SendFrame's own descriptor writes are all
   * visible to the NETC DMA in DDR *before* EP_SendFrame writes the
   * TBPIR producer-index doorbell register.  Without this, the DMA sees
   * the new TBPIR value but reads stale BD content (Normal non-cacheable
   * DDR writes can sit in the M7 store buffer while Device-memory writes
   * to SI registers proceed; ARMv7-M does not order them automatically). */

  __asm__ volatile ("dsb" ::: "memory");

  /* EP_SendFrame returns kStatus_Busy when there is no idle TX BD left.
   * The hardware TX consumer index advances asynchronously as the NETC
   * DMA drains the ring, so re-run reclaim and retry a bounded number of
   * times to let in-flight frames complete and free BDs.  Only report
   * back-pressure if the ring is still full after the retries. */

  status = EP_SendFrame(handle, 0, &frame, NULL, NULL);

  for (retry = 0; status == kStatus_Busy && retry < NETC_TX_SEND_RETRIES;
       retry++)
    {
      up_udelay(NETC_TX_SEND_RETRY_US);

      EP_ReclaimTxDescriptor(handle, 0);
      __asm__ volatile ("dsb" ::: "memory");

      status = EP_SendFrame(handle, 0, &frame, NULL, NULL);
    }

#ifdef ENETC_VSI_DEBUG_TRX
  /* TX ring diagnostic.  Compare VSI0 (ENETC2 / switch, works) against
   * VSI1 (ENETC0 / Realtek, stalls):
   *   - producerIndex climbing while TBCIR (consumer) stays frozen means
   *     the ENETC VF TX egress is not completing (PF/SR-IOV VF-TX enable
   *     or port TX readiness issue), NOT a ring-software bug.
   *   - TBCIR advancing but completion IRQ never firing points at the
   *     MSGINTR-instance / MSI-X entry aliasing between the two VSIs. */

  {
    uint16_t tbcir = NETC_SIGetTxConsumer(handle->hw.si, 0);
    syslog(LOG_ERR,
           "VSI %s: TX ring prod=%u clean=%u tbcir=%u retry=%u status=%d\n",
           priv->dev.d_ifname,
           (unsigned)handle->txBdRing[0].producerIndex,
           (unsigned)handle->txBdRing[0].cleanIndex,
           (unsigned)tbcir, retry, (int)status);
  }
#endif

  if (status == kStatus_Busy)
    {
      /* Ring still full: tell the stack to retry later instead of
       * treating this as a hard error. */

      return -EAGAIN;
    }


  if (status != kStatus_Success)
    {
      return -EIO;
    }

  return 0;
}


/****************************************************************************
 * Name: imx9_netc_vsi_msgintr_cb
 *
 * Description:
 *   MSGINTR callback - called from the MSGINTR ISR when any of the three
 *   MSI channels fire.  Dispatches to Tx reclaim, Rx wake, or PSI message.
 *
 ****************************************************************************/

static void imx9_netc_vsi_msgintr_cb(int instance, uint8_t channel,
                                      uint32_t pending, void *arg)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)arg;
  static uint32_t tx_count  __attribute__((unused));
  static uint32_t rx_count  __attribute__((unused));
  static uint32_t msg_count __attribute__((unused));

  (void)instance;
  (void)channel;

#ifdef ENETC_VSI_DEBUG_INTR
  syslog(LOG_ERR, "\nimx9_netc_vsi_msgintr_cb: instance = %d, channel = %d, pending = 0x%08x\n", instance, channel, pending);
#endif
  /* All three MSI-X events share NETC_MSGINTR_CHANNEL (0).
   * The IBS value written to MSIIR sets a unique bit in MSIR;
   * dispatch by checking the corresponding bit in pending. */

  if (pending & NETC_TX_MSGINTR_BIT)
    {
      tx_count++;
      EP_CleanTxIntrFlags(&priv->handle, 1U, 1U);
      nxsem_post(&priv->tx_sem);
    }

  if (pending & NETC_RX_MSGINTR_BIT)
    {
#ifdef ENETC_VSI_DEBUG_INTR
      rx_count++;
      syslog(LOG_ERR, "VSI %s: MSGINTR RX fired count=%u pending=0x%08x\n",
             priv->dev.d_ifname, (unsigned)rx_count, (unsigned)pending);
#endif
      /* When the interface is administratively down, just clear the
       * interrupt flag and return without waking the service thread.
       * This prevents the thread from spin-draining (and starving the
       * UART/console) while IFF_UP is not set.
       */
      if ((priv->dev.d_flags & IFF_UP) == 0)
        {
          EP_CleanRxIntrFlags(&priv->handle, 1U);
          return;
        }

      EP_CleanRxIntrFlags(&priv->handle, 1U);
      nxsem_post(&priv->rx_sem);
    }

  if (pending & NETC_MSG_MSGINTR_BIT)
    {
#ifdef ENETC_VSI_DEBUG_INTR
      msg_count++;
      syslog(LOG_ERR, "VSI %s: MSGINTR MSG fired count=%u pending=0x%08x\n",
             priv->dev.d_ifname, (unsigned)msg_count, (unsigned)pending);
#endif
      /* If si_bringup has not yet completed, the service thread is still
       * polling vf_ready() for SR-IOV enable; this MSG interrupt is
       * informational only.  After vf_confirmed is set (si_bringup done),
       * dispatch normally.
       */
      if (!priv->vf_confirmed)
        {
          /* The vf_ready() poll in the service thread drives the
           * firewall-open gate; a MSG interrupt before si_bringup is
           * informational only.
           */
          syslog(LOG_ERR,
                 "NETC VSI: MSG interrupt before si_bringup (vf_ready poll gates)\n");
        }
      else
        {
          imx9_netc_vsi_msg_isr(priv);
        }
    }
}

/****************************************************************************
 * Name: imx9_netc_vsi_msg_isr
 *
 * Description:
 *   Handle a PSI->VSI message interrupt.  Polls the link state and either
 *   wakes the service thread (link-up) or drops the carrier (link-down).
 *
 ****************************************************************************/

void imx9_netc_vsi_msg_isr(struct imx9_netc_vsi_s *priv)
{
  if (priv->dev.d_ifup == NULL)
    {
      return;
    }

  /* Guard against a PSI that has disappeared (FLR / SR-IOV disable /
   * A55 reboot).  The PSI may deliver a stale MSG interrupt whose MSIR
   * bit is set while the SI register window has already become invalid.
   * imx9_netc_vsi_msg_poll_link() reads SI registers (VSIMSGRR/VSIIDR);
   * doing so after the window dies raises a precise BusFault that panics
   * the M7 in ISR context.  Re-validate SR-IOV with the bus-fault-safe
   * probe first; if the PSI is gone, flag a deferred link-down and let
   * the service thread tear the SI down from thread context instead of
   * touching the trapped mapping here.
   */

  if (!imx9_netc_vsi_vf_ready(priv))
    {
      /* PSI gone.  Detach the MSGINTR from within the ISR itself so the
       * PSI's interrupt storm (it keeps asserting link-up MSG interrupts
       * as the A55 reboots) stops immediately.  Otherwise each stale MSG
       * interrupt re-enters this ISR and runs vf_ready()'s fault-safe SI
       * reads (which fault and are fixed up) before the service thread
       * ever gets scheduled to detach it - a feedback loop that starves
       * NSH and the PX4 sensor threads for the whole teardown window.
       * The service thread re-attaches the MSGINTR on SR-IOV restore.
       */

      imx9_msgintr_deinit(priv->config->msgintr_instance);

      priv->si_ready          = false;
      priv->vf_confirmed      = false;
      priv->link_down_pending = true;
      nxsem_post(&priv->rx_sem);
      return;
    }


  switch (imx9_netc_vsi_msg_poll_link(priv))
    {
      case IMX9_NETC_MSG_LINK_UP:

        /* Ignore spurious LINK_UP before si_ready: bring_up calls
         * program_psi itself; a stale interrupt here would cause a
         * double-message that corrupts the PSI state machine.
         */
        syslog(LOG_ERR, "VSI %s: link UP\n", priv->dev.d_ifname);
        priv->link_notify = true;
        priv->si_ready = true;
        break;

      case IMX9_NETC_MSG_LINK_DOWN:
        syslog(LOG_ERR, "VSI %s: link DOWN\n", priv->dev.d_ifname);
        priv->si_ready = false;
        /* Do not call netdev_carrier_off() from ISR context.
         * It triggers devif_dev_event(NETDEV_DOWN) which invokes TCP
         * close callbacks that may try to acquire net_lock, causing a
         * deadlock if net_lock is already held by another task (e.g. NSH).
         * Set a flag and let the service thread call netdev_carrier_off
         * from thread context instead.
         */
        priv->link_down_pending = true;
        nxsem_post(&priv->rx_sem);
        break;

      default:
        syslog(LOG_ERR, "VSI %s: unknown link msg\n", priv->dev.d_ifname);
        break;
    }
}

/****************************************************************************
 * Name: imx9_netc_vsi_thread
 *
 * Description:
 *   VSI service thread.  Performs the initial bring-up, then loops:
 *   - detects SR-IOV loss/restore and re-brings-up the SI
 *   - handles link-up notifications by reprogramming the PSI
 *   - drains the RX ring up to RX_BUDGET frames per wakeup
 *
 ****************************************************************************/

static int imx9_netc_vsi_thread(int argc, char *argv[])
{
  struct imx9_netc_vsi_s *priv =
    (struct imx9_netc_vsi_s *)(uintptr_t)strtoul(argv[1], NULL, 16);
  bool up = true;
  int  work;
  static uint32_t loop_count __attribute__((unused));

  /* Use printf so output reaches the console regardless of syslog routing. */

  imx9_netc_vsi_bring_up(priv);
#ifdef ENETC_VSI_DEBUG
  ninfo("VSI thread: bring_up done, entering loop\n");
#endif

  bool ready = imx9_netc_vsi_vf_ready(priv);
  for (;;)
    {
#ifdef ENETC_VSI_DEBUG_THREAD
     syslog(LOG_ERR, "VSI thread: loop count=%u ready=%d up=%d, priv->si_ready=%d, priv->link_notify=%d\n",
        (unsigned)loop_count, ready, up, priv->si_ready, priv->link_notify);
#endif
      loop_count++;

      /* While SR-IOV is down (up == false) the MSGINTR is detached, so no
       * semaphore post will ever arrive to wake a blocking rx_sem wait -
       * recovery depends entirely on this thread polling vf_ready().  Do
       * NOT block on the semaphore in that state; sleep a fixed poll
       * interval and fall through so the SR-IOV re-check below runs.  A
       * blocking tickwait here would stall the thread until the next
       * reboot happens to post rx_sem, which is exactly the "counter
       * stuck, advances one step per reboot" symptom.
       */

      if (!up)
        {
          nxsig_usleep(CONFIG_IMX9_NETC_VSI_READY_POLL_MS * 1000u);
        }
      else

      /* Wait for RX wake, txavail notify, link change, or poll timeout.
       * Use a relative-tick wait to avoid depending on CLOCK_REALTIME
       * being set (NuttX boots with tv_sec=0 if no RTC; an absolute
       * timeout of epoch+500ms is always in the past, causing an instant
       * return and a 100% CPU busy-spin).
       */

      if (0 == nxsem_tickwait(&priv->rx_sem, MSEC2TICK(CONFIG_IMX9_NETC_VSI_READY_POLL_MS)))

      {
        /* Check needs_bringup before draining: if ifdown was called, the
         * SI BD rings and MSI-X table are in an undefined state; skip rx
         * drain and let the bringup path below restore everything.
         */
        if (!priv->needs_bringup && priv->si_ready)
          {
            int drained = 0;

            work = 0;
            while (imx9_netc_vsi_rx(priv) == 0)
              {
                /* Hard cap the drain.  If the PSI tears the SI window down
                 * mid-drain, the SI reads inside imx9_netc_vsi_rx() fault
                 * and are fixed up to garbage, which EP_GetRxFrameSize can
                 * misread as an endless stream of "valid" frames.  Without
                 * a cap this spins forever, each iteration taking fixup
                 * faults in high-priority context and starving NSH and the
                 * PX4 sensor threads.  Bound the burst and re-check SR-IOV;
                 * bail out of the drain if the window is gone so the loss
                 * path below can run.
                 */

                if (++drained >= (4 * CONFIG_IMX9_NETC_VSI_RX_BUDGET))
                  {
                    if (!imx9_netc_vsi_vf_ready(priv))
                      {
                        break;
                      }

                    drained = 0;
                  }

                if (++work == CONFIG_IMX9_NETC_VSI_RX_BUDGET)
                  {
                    work = 0;
                    sched_yield();
                  }
              }

          }
      }


      /* Drain any extra posts the ISR accumulated while we were working. */

      while (nxsem_trywait(&priv->rx_sem) == 0)
      {
          /* consumed */
      }

      /* Refresh SR-IOV readiness every loop with the bus-fault-safe probe
       * so a PSI that disappears (FLR / SR-IOV disable / A55 reboot) is
       * detected here before any SI-register access below (RX drain,
       * program_psi, TX poll).  This must be a live probe, not a one-shot
       * cached value, or the loss path never fires and the thread drives
       * program_psi into the dead SI window (precise BusFault).
       */

      ready = imx9_netc_vsi_vf_ready(priv);

      if (up && !ready)

        {
          nxrmutex_lock(&priv->msg_lock);
          priv->si_ready = false;
          nxrmutex_unlock(&priv->msg_lock);
          netdev_carrier_off(&priv->dev);

          /* Silence the MSGINTR for this instance while SR-IOV is down.
           * Otherwise the PSI (as the A55 reboots) keeps firing link-up
           * MSG interrupts; each one dispatches into SI register reads
           * that now fault and are single-stepped by the busfault fixup.
           * That fault storm runs in high-priority exception context and
           * starves NSH and the PX4 sensor threads (observed console lag
           * and sensor TIMEOUTs).  Detaching the MSGINTR stops the storm;
           * the service thread's periodic vf_ready() poll (tickwait) is
           * what drives recovery, so no interrupt is needed while down.
           */

          imx9_msgintr_deinit(priv->config->msgintr_instance);
          priv->vf_confirmed = false;

          syslog(LOG_ERR, "VSI %s: link DOWN (SR-IOV lost)\n",
                 priv->dev.d_ifname);
          up = false;
        }
      else if (!up && ready)
        {
          /* Re-attach the MSGINTR before bring-up so PSI->VSI link
           * messages are delivered again once the SI is live.
           */

          imx9_msgintr_init(priv->config->msgintr_instance,
                            imx9_netc_vsi_msgintr_cb, priv);
          imx9_netc_vsi_bring_up(priv);
          up = true;
          priv->si_ready = true;
        }


      /* Handle ifdown/ifup cycle: si_bringup must be re-run because
       * ifdown masked the MSI-X table and stopped the SI.  The 'up'
       * flag stays true (no SR-IOV loss), so the !up&&ready path above
       * does not fire.  needs_bringup fills this gap.
       */
      if (priv->needs_bringup && priv->needs_ifup)
        {
          priv->needs_bringup = false;
          priv->needs_ifup    = false;
          syslog(LOG_ERR, "VSI %s: re-bringup after ifdown/ifup\n",
                 priv->dev.d_ifname);
          imx9_netc_vsi_bring_up(priv);

          /* Drain immediately after re-bringup.  Frames may have arrived
           * after RBIER was armed but before the thread returned to
           * tickwait; the nxsem_trywait loop above consumed those ISR
           * posts before bring_up ran, leaving frames stranded in the
           * ring.  An explicit poll here catches them.
           */
          if (priv->si_ready)
            {
              work = 0;
              while (imx9_netc_vsi_rx(priv) == 0)
                {
                  if (++work == CONFIG_IMX9_NETC_VSI_RX_BUDGET)
                    {
                      work = 0;
                      sched_yield();
                    }
                }
            }
        }

      if (!up)
        {
          continue;
        }

      if (priv->link_down_pending)
        {
          priv->link_down_pending = false;
          netdev_carrier_off(&priv->dev);
          syslog(LOG_ERR, "VSI %s: link DOWN (deferred from ISR)\n",
                 priv->dev.d_ifname);
        }

      if (priv->link_notify)
        {
          priv->link_notify = false;

          /* Re-validate the SI window immediately before touching it.
           * On PSI re-enable (Linux boot) the PSI emits a storm of
           * link-up MSG interrupts; each sets link_notify.  program_psi()
           * reaches the SI command-BDR register path (via the msg
           * helpers), which can still fault during the re-enable
           * transient even though the loop-top probe briefly passed.
           * Skip reprogramming unless the window is confirmed live; a
           * later interrupt (or the next loop) retries once it settles.
           */

          if (ready && imx9_netc_vsi_vf_ready(priv))
            {
              imx9_netc_vsi_program_psi(priv);
              netdev_carrier_on(&priv->dev);
              syslog(LOG_ERR, "VSI %s: link UP; PSI reprogrammed\n",
                     priv->dev.d_ifname);
            }
          else
            {
              syslog(LOG_ERR,
                     "VSI %s: link UP ignored, SI not live yet\n",
                     priv->dev.d_ifname);
            }
        }


      if (!priv->si_ready)
        {
          continue;
        }

      /* If txavail() requested a TX poll, run devif_poll now so the
       * network stack can hand us any queued outbound frames.
       */

      if (priv->tx_pending)
        {
          priv->tx_pending = false;
          net_lock();
          priv->dev.d_buf = priv->tx_frame;
          devif_poll(&priv->dev, imx9_netc_vsi_transmit);
          priv->dev.d_buf = NULL;
          net_unlock();
        }
    }

  return 0;  /* not reached */
}

/****************************************************************************
 * netdev operations
 ****************************************************************************/

#ifdef CONFIG_NET_MCASTGROUP
static uint8_t imx9_netc_mac_hash_idx(const uint8_t *addr)
{
  uint64_t ether = 0;
  uint64_t fold;
  uint64_t mask  = 0;
  uint8_t  res   = 0;
  int      i;

  for (i = 0; i < 6; i++)
    {
      ether = (ether << 8) | addr[i];
    }

  fold = __builtin_bswap64(ether) >> 16;

  for (i = 0; i < 8; i++)
    {
      mask |= (uint64_t)1u << (i * 6);
    }

  for (i = 0; i < 6; i++)
    {
      res |= (uint8_t)(__builtin_popcountll(fold & (mask << i)) & 0x1u) << i;
    }

  return res;
}
#endif /* CONFIG_NET_MCASTGROUP */

static int imx9_netc_vsi_transmit(struct net_driver_s *dev)
{
  struct imx9_netc_vsi_s *priv =
    (struct imx9_netc_vsi_s *)dev;
#ifdef ENETC_VSI_DEBUG_TRX
  static uint32_t tx_frames;
#endif
  int ret;

  if (!priv->si_ready)
    {
      syslog(LOG_ERR, "VSI %s: TX si_ready=false, dropping\n", priv->dev.d_ifname);
      return -ENETDOWN;
    }

  if (!imx9_netc_vsi_vf_ready(priv))
    {
      nxsem_post(&priv->rx_sem);
      syslog(LOG_ERR, "VSI %s: TX vf not ready, dropping\n", priv->dev.d_ifname);
      return -ENETDOWN;
    }

  ret = imx9_netc_vsi_tx(priv, dev->d_buf, dev->d_len);
#ifdef ENETC_VSI_DEBUG_TRX
  tx_frames++;
  syslog(LOG_ERR, "VSI %s: TX len=%u ret=%d total=%u\n",
         priv->dev.d_ifname, (unsigned)dev->d_len, ret, tx_frames);
#endif
  return ret;
}

static int imx9_netc_vsi_txavail(struct net_driver_s *dev)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;

#ifdef ENETC_VSI_DEBUG_TRX
  syslog(LOG_ERR, "VSI %s: txavail entered\n", priv->dev.d_ifname);
#endif
  if (!priv->si_ready)
    {
      return OK;
    }

  /* txavail is called by the NuttX network stack with net_lock held.
   * We must poll (and if needed transmit) here and now, not deferred
   * to the VSI service thread.  If we defer, ARP requests formatted by
   * arp_send() in d_buf will never be sent before arp_wait() times out
   * (returning ENETUNREACH to the caller).
   *
   * Pattern follows imxrt_enet.c / stm32_eth.c:
   *   - if d_len > 0: a packet is already formatted in d_buf, send it
   *   - always call devif_poll to pick up any protocol-queued frames
   */

  if (dev->d_len > 0)
    {
      imx9_netc_vsi_transmit(dev);
    }

  /* Use the dedicated TX scratch buffer for devif_poll (symmetric with
   * the RX path, which uses rx_frame).  tx_frame is in the .netc_dma
   * region and large enough for a max frame.  imx9_netc_vsi_tx copies
   * from it into a separate tx_buff[] slot, so there is no overlapping
   * memcpy hazard. */

  dev->d_buf = priv->tx_frame;
  devif_poll(dev, imx9_netc_vsi_transmit);
  dev->d_buf = NULL;

  /* Also wake the service thread so it can drain any frames that may have
   * arrived during/after our transmit (e.g. ARP reply, ICMP echo reply).
   * Cap the semaphore at 1 to avoid accumulation. */

  priv->tx_pending = false;
  {
    int semval = 0;
    nxsem_get_value(&priv->rx_sem, &semval);
    if (semval <= 0)
      {
        nxsem_post(&priv->rx_sem);
      }
  }

  return OK;
}

static int imx9_netc_vsi_ifup(struct net_driver_s *dev)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;
  const struct imx9_netc_vsi_config_s *cfg = priv->config;

  /* Re-register MSGINTR so MSI-X interrupts (RX, TX, MSG) can arrive.
   * MSI-X entries are NOT unmasked here: the SI address space (msixTable)
   * may not be accessible yet at boot (TRDC firewall is closed until the
   * PSI/A55 enables SR-IOV).  The service thread calls si_bringup() after
   * confirming the firewall is open, and si_bringup() programs the entire
   * MSI-X table with control=0 (unmasked), which is the authoritative
   * unmask point.
   * ifdown() called imx9_msgintr_deinit() which disabled the IRQSTEER
   * channel and detached the ISR; re-init restores both.
   */
  imx9_msgintr_init(cfg->msgintr_instance,
                    imx9_netc_vsi_msgintr_cb, priv);

  /* Signal the service thread that ifup() has completed.  The thread
   * checks for needs_bringup && needs_ifup together so it only re-runs
   * si_bringup after BOTH ifdown and ifup have been called -- preventing
   * a race where needs_bringup is seen while IFF_UP is still set during
   * the ifdown call (before NuttX clears IFF_UP).
   */
  priv->needs_ifup = true;
  nxsem_post(&priv->rx_sem);

  return OK;
}

static int imx9_netc_vsi_ifdown(struct net_driver_s *dev)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;
  uint32_t *tbl = (uint32_t *)(uintptr_t)priv->handle.hw.msixTable;

  /* Mask all MSI-X table entries before deiniting MSGINTR.
   * After deinit the MSGINTR callback is NULL and the NVIC line may still
   * be enabled (shared with VSI0).  If a packet arrives and triggers a
   * write to MSIIR, the IRQ asserts and the ISR finds no callback -- but
   * MSIR is never cleared, so the NVIC keeps re-entering the ISR in an
   * infinite loop that starves all tasks (including NSH).
   * Setting the MSI-X entry control word bit 0 (Mask Bit) prevents the
   * SI from writing to MSIIR while the interface is down.
   */
  if (tbl != NULL)
    {
      tbl[NETC_TX_MSIX_ENTRY_IDX  * 4u + 3u] = 1u; /* masked */
      tbl[NETC_RX_MSIX_ENTRY_IDX  * 4u + 3u] = 1u;
      tbl[NETC_MSG_MSIX_ENTRY_IDX * 4u + 3u] = 1u;
      __asm__ volatile("dsb" ::: "memory");
    }

  imx9_msgintr_deinit(priv->config->msgintr_instance);
  priv->si_ready = false;
  /* Signal the service thread that si_bringup() must be re-run.
   * needs_ifup is cleared here and set by ifup() so that the service
   * thread only runs bringup when BOTH flags are set, avoiding a race
   * where needs_bringup is seen while IFF_UP is still set during ifdown.
   */
  priv->needs_ifup   = false;
  priv->needs_bringup = true;
  return OK;
}

#ifdef CONFIG_NET_MCASTGROUP
static int imx9_netc_vsi_addmac(struct net_driver_s *dev,
                                  const uint8_t *mac)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;
  uint8_t idx = imx9_netc_mac_hash_idx(mac);
  int     ret = 0;

  nxrmutex_lock(&priv->msg_lock);

  if (priv->mc_hash_refcnt[idx] < UINT8_MAX)
    {
      priv->mc_hash_refcnt[idx]++;
    }

  priv->mc_hash |= (uint64_t)1u << idx;

  if (priv->si_ready)
    {
      ret = imx9_netc_vsi_msg_set_mac_hash(priv);
    }

  nxrmutex_unlock(&priv->msg_lock);
  return ret;
}

static int imx9_netc_vsi_rmmac(struct net_driver_s *dev,
                                const uint8_t *mac)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;
  uint8_t idx = imx9_netc_mac_hash_idx(mac);
  int     ret = 0;

  nxrmutex_lock(&priv->msg_lock);

  if (priv->mc_hash_refcnt[idx] > 0u)
    {
      priv->mc_hash_refcnt[idx]--;
    }

  if (priv->mc_hash_refcnt[idx] == 0u)
    {
      priv->mc_hash &= ~((uint64_t)1u << idx);
    }

  if (priv->si_ready)
    {
      ret = imx9_netc_vsi_msg_set_mac_hash(priv);
    }

  nxrmutex_unlock(&priv->msg_lock);
  return ret;
}
#endif /* CONFIG_NET_MCASTGROUP */

#ifdef CONFIG_NETDEV_IOCTL
static int imx9_netc_vsi_ioctl(struct net_driver_s *dev, int cmd,
                                unsigned long arg)
{
  return -ENOTTY;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_netc_vsi_do_initialize
 *
 * Description:
 *   Common initialization body shared by all VSI instances.
 *   Wires up the driver state, registers the netdev, and spawns the
 *   service kthread.  The caller supplies the instance-specific state,
 *   config, and DMA buffer pointers.
 *
 * Parameters:
 *   priv      - per-instance driver state (pre-allocated, will be zeroed)
 *   cfg       - compile-time config (si_idx, msgintr_instance, bdr_init)
 *   tx_buff   - pointer to the instance TX buffer array in .netc_dma
 *   msg_buff  - pointer to the instance VSI->PSI message buffer
 *   rx_frame  - pointer to the instance RX reassembly buffer
 *   mac_override - optional hard-coded MAC from board.h (NULL = use default)
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

static int imx9_netc_vsi_do_initialize(
    struct imx9_netc_vsi_s              *priv,
    const struct imx9_netc_vsi_config_s *cfg,
    uint8_t (*tx_buff)[NETC_TX_FRAME_BUF_SIZE],
    uint8_t                             *msg_buff,
    uint8_t                             *rx_frame,
    uint8_t                             *tx_frame,
    const uint8_t                       *mac_override)
{
  ep_handle_t *handle = &priv->handle;
  char         arg1[32];
  char        *argv[2];
  int          ret;

  memset(priv, 0, sizeof(*priv));
  priv->config    = cfg;
  priv->tx_buff   = (uint8_t (*)[NETC_TX_FRAME_BUF_SIZE])tx_buff;
  priv->msg_buff  = (uint8_t *)msg_buff;
  priv->rx_frame  = (uint8_t *)rx_frame;
  priv->tx_frame  = (uint8_t *)tx_frame;

  NETC_SocGetBaseResource(&handle->hw, (netc_hw_si_idx_t)cfg->si_idx);

  /* Fill SDK handle cfg (no register access here) */

  handle->cfg.si              = (netc_hw_si_idx_t)cfg->si_idx;
  handle->cfg.rxRingUse       = 1;
  handle->cfg.txRingUse       = 1;
  handle->cfg.rxBdrGroupNum   = 0;
  handle->cfg.ringPerBdrGroup = 1;
  handle->cfg.rxCacheMaintain = false;
  handle->cfg.txCacheMaintain = false;
  handle->cfg.rxZeroCopy      = false;
  handle->cfg.entryNum        = NETC_MSIX_VSI_EVENTS_COUNT;
  handle->cfg.userData        = priv;
  handle->cfg.reclaimCallback = NULL;

  /* Determine MAC address: use board override when provided, else derive
   * a deterministic fallback from the instance index. */

  if (mac_override != NULL)
    {
      memcpy(priv->mac_addr, mac_override, IFHWADDRLEN);
    }
  else
    {
      imx9_netc_default_mac(priv->mac_addr, cfg->mac_default_index);
    }

  priv->si_ready    = false;
  priv->link_notify = false;

  nxrmutex_init(&priv->msg_lock);
  nxsem_init(&priv->tx_sem, 0, 0);
  nxsem_init(&priv->rx_sem, 0, 0);

  /* Register the MSGINTR callback early.  The PSI sends a MSG interrupt
   * only after it has opened the NETC TRDC/XRDC firewall.  MSGINTR lives
   * at 0x4B230000 and is NOT firewalled, so this call is safe even when
   * the VSI SI register space is still locked.
   */

  imx9_msgintr_init(cfg->msgintr_instance,
                    imx9_netc_vsi_msgintr_cb, priv);

  /* Configure the netdev ops */

  priv->dev.d_ifup    = imx9_netc_vsi_ifup;
  priv->dev.d_ifdown  = imx9_netc_vsi_ifdown;
  priv->dev.d_txavail = imx9_netc_vsi_txavail;
#ifdef CONFIG_NET_MCASTGROUP
  priv->dev.d_addmac  = imx9_netc_vsi_addmac;
  priv->dev.d_rmmac   = imx9_netc_vsi_rmmac;
#endif
#ifdef CONFIG_NETDEV_IOCTL
  priv->dev.d_ioctl   = imx9_netc_vsi_ioctl;
#endif
  priv->dev.d_private = priv;

#ifdef CONFIG_NET_ETHERNET
  memcpy(priv->dev.d_mac.ether.ether_addr_octet,
         priv->mac_addr, IFHWADDRLEN);
#endif

  /* Register with the NuttX network layer */

  ret = netdev_register(&priv->dev, NET_LL_ETHERNET);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: netdev_register: %d\n", ret);
      nerr("netdev_register failed: %d\n", ret);
      return ret;
    }

  /* Spawn the service kthread that will drive the SI */

  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  priv->rx_thread_pid = kthread_create(priv->dev.d_ifname,
                                       CONFIG_IMX9_NETC_VSI_THREAD_PRIORITY,
                                       CONFIG_IMX9_NETC_VSI_THREAD_STACKSIZE,
                                       imx9_netc_vsi_thread,
                                       argv);
  if (priv->rx_thread_pid < 0)
    {
      nerr("kthread_create failed: %d\n", priv->rx_thread_pid);
      netdev_unregister(&priv->dev);
      return (int)priv->rx_thread_pid;
    }

  ninfo("ENETC VSI%u registered (SI idx 0x%04x, "
        "MAC %02x:%02x:%02x:%02x:%02x:%02x)\n",
        (unsigned)cfg->mac_default_index,
        cfg->si_idx,
        priv->mac_addr[0], priv->mac_addr[1], priv->mac_addr[2],
        priv->mac_addr[3], priv->mac_addr[4], priv->mac_addr[5]);

  return OK;
}

/****************************************************************************
 * Name: imx9_netc_vsi_initialize
 *
 * Description:
 *   Initialize and register the ENETC VSI Ethernet driver for the given
 *   instance index.  Called from the board bringup code (imx95_bringup.c).
 *
 * Parameters:
 *   instance - VSI instance to initialize (0, 1, ...)
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_netc_vsi_initialize(int instance)
{
  switch (instance)
    {
#ifdef CONFIG_IMX9_NETC_VSI0
      case 0:
        {
#ifdef IMX9_NETC_VSI0_MAC
          static const uint8_t mac0[] = IMX9_NETC_VSI0_MAC;
          return imx9_netc_vsi_do_initialize(
              &g_vsi0, &g_vsi0_config,
              (uint8_t (*)[CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE])g_vsi0_tx_buff,
              (uint8_t *)g_vsi0_msg_buff,
              (uint8_t *)g_vsi0_rx_frame,
	      (uint8_t *)g_vsi0_tx_frame,
              mac0);
#else
          return imx9_netc_vsi_do_initialize(
              &g_vsi0, &g_vsi0_config,
              (uint8_t (*)[CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE])g_vsi0_tx_buff,
              (uint8_t *)g_vsi0_msg_buff,
              (uint8_t *)g_vsi0_rx_frame,
	      (uint8_t *)g_vsi0_tx_frame,
              NULL);
#endif
        }
#endif /* CONFIG_IMX9_NETC_VSI0 */

#ifdef CONFIG_IMX9_NETC_VSI1
      case 1:
        {
#ifdef IMX9_NETC_VSI1_MAC
          static const uint8_t mac1[] = IMX9_NETC_VSI1_MAC;
          return imx9_netc_vsi_do_initialize(
              &g_vsi1, &g_vsi1_config,
              (uint8_t (*)[CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE])g_vsi1_tx_buff,
              (uint8_t *)g_vsi1_msg_buff,
              (uint8_t *)g_vsi1_rx_frame,
	      (uint8_t *)g_vsi1_tx_frame,
              mac1);
#else
          return imx9_netc_vsi_do_initialize(
              &g_vsi1, &g_vsi1_config,
              (uint8_t (*)[CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE])g_vsi1_tx_buff,
              (uint8_t *)g_vsi1_msg_buff,
              (uint8_t *)g_vsi1_rx_frame,
	      (uint8_t *)g_vsi1_tx_frame,
              NULL);
#endif
        }
#endif /* CONFIG_IMX9_NETC_VSI1 */

      default:
        nerr("Unknown VSI instance %d\n", instance);
        return -EINVAL;
    }
}

