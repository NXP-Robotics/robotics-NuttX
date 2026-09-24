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
#include "imx9_msgintr.h"
#include "imx9_netc_priv.h"
#include "imx9_rptun.h"
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

/* Quiet window (ms) after the rptun first kick before accessing NETC SI regs.
 * The rptun MU7 kick arrives at Linux remoteproc connect; the NETC PF driver
 * that opens the TRDC/XRDC firewall loads later.  This delay bridges the gap.
 */

#ifndef CONFIG_IMX9_NETC_VSI_RPTUN_READY_DELAY_MS
#  define CONFIG_IMX9_NETC_VSI_RPTUN_READY_DELAY_MS  20000
#endif

/* Maximum RX frames processed per wakeup before yielding */

#ifndef CONFIG_IMX9_NETC_VSI_RX_BUDGET
#  define CONFIG_IMX9_NETC_VSI_RX_BUDGET  16
#endif

/* Shorthand for the Ethernet header at the start of d_buf */

#define BUF ((struct ether_header *)(priv->dev.d_buf /*+ CONFIG_NET_GUARDSIZE*/))

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

/* DMA-accessible buffers for VSI instance 0.
 * Placed in .noncachable which lands in DDR (NOLOAD, remoteproc-loadable).
 * buffers reside in non-cacheable OCRAM (.netc_dma section),
 * around DMA operations (see imx9_netc_msg.c).
 */

volatile uint8_t g_vsi0_tx_buff[CONFIG_IMX9_NETC_VSI_TX_RING_LEN]
                              [CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE]
  __attribute__((section(".netc_dma"),
                 aligned(NETC_BUFF_ALIGN)));

volatile uint8_t g_vsi0_msg_buff[NETC_MSG_VSI_BUF_SIZE]
  __attribute__((section(".netc_dma"),
                 aligned(NETC_BUFF_ALIGN)));

volatile netc_tx_bd_t g_vsi0_txbd[CONFIG_IMX9_NETC_VSI_TX_RING_LEN]
  __attribute__((section(".netc_dma"),
                 aligned(NETC_BD_ALIGN)));

volatile netc_tx_frame_info_t
  g_vsi0_txdirty[CONFIG_IMX9_NETC_VSI_TX_RING_LEN];

volatile netc_rx_buf_t g_vsi0_rx_buff[CONFIG_IMX9_NETC_VSI_RX_RING_LEN]
  __attribute__((section(".netc_dma"),
                 aligned(NETC_BUFF_ALIGN)));

volatile uint64_t
  g_vsi0_rx_buff_addr[CONFIG_IMX9_NETC_VSI_RX_RING_LEN];

volatile uint8_t g_vsi0_rx_frame[NETC_RX_FRAME_BUF_SIZE]
  __attribute__((section(".netc_dma"),
                 aligned(NETC_BUFF_ALIGN)));

volatile netc_rx_bd_t g_vsi0_rxbd[CONFIG_IMX9_NETC_VSI_RX_RING_LEN]
  __attribute__((section(".netc_dma"),
                 aligned(NETC_BD_ALIGN)));

/* BD ring initializer for VSI instance 0 */

static void imx9_netc_vsi0_bdr_init(netc_bdr_config_t    *bdr,
                                     netc_rx_bdr_config_t *rx,
                                     netc_tx_bdr_config_t *tx)
{
  uint16_t i;

  for (i = 0; i < CONFIG_IMX9_NETC_VSI_RX_RING_LEN; i++)
    {
      g_vsi0_rx_buff_addr[i] = (uint64_t)(uintptr_t)&g_vsi0_rx_buff[i];
    }

  bdr->rxBdrConfig = rx;
  bdr->txBdrConfig = tx;

  rx[0].bdArray       = (netc_rx_bd_t *)g_vsi0_rxbd;
  rx[0].len           = CONFIG_IMX9_NETC_VSI_RX_RING_LEN;
  rx[0].buffAddrArray = (uint64_t *)g_vsi0_rx_buff_addr;
  rx[0].buffSize      = NETC_RX_FRAME_BUF_SIZE;
  rx[0].msixEntryIdx  = NETC_RX_MSIX_ENTRY_IDX;
  rx[0].extendDescEn  = false;
  rx[0].enThresIntr   = true;
  rx[0].enCoalIntr    = true;  /* coalescing interrupt (RXCOIE) is the working RX interrupt source on iMX95 */
  rx[0].intrThreshold = 1;

  tx[0].bdArray      = (netc_tx_bd_t *)g_vsi0_txbd;
  tx[0].len          = CONFIG_IMX9_NETC_VSI_TX_RING_LEN;
  tx[0].dirtyArray   = (netc_tx_frame_info_t *)g_vsi0_txdirty;
  tx[0].msixEntryIdx = NETC_TX_MSIX_ENTRY_IDX;
  tx[0].enIntr       = false;
  tx[0].enThresIntr  = true;
}

/* Per-instance config (compile-time) */

static const struct imx9_netc_vsi_config_s g_vsi0_config =
{
  .si_idx           = IMX9_NETC_VSI0_SI_IDX,
  .msgintr_instance = IMX9_NETC_VSI0_MSGINTR_INSTANCE,
  .mac_default_index = 0,
  .bdr_init         = imx9_netc_vsi0_bdr_init,
};

/* Per-instance state */

static struct imx9_netc_vsi_s g_vsi0;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_netc_vsi_vf_ready
 *
 * Description:
 *   Check whether the PSI has enabled SR-IOV for our ENETC instance.
 *   Reads the PCI VF-enable and memory-space-enable bits from the PSI's
 *   PCIe SR-IOV control register.
 *
 ****************************************************************************/

static bool imx9_netc_vsi_vf_ready(struct imx9_netc_vsi_s *priv)
{
  const struct imx9_netc_vsi_config_s *cfg = priv->config;
  ENETC_PCI_TYPE0_Type *pf;
  uint16_t vf_ready;

  if (priv->vf_confirmed)
    {
      return true;
    }

  /* PF PCIe config base addresses (one per ENETC MAC instance).
   * Instance 0 -> index 1 in ENETC_PCI_TYPE0_BASE_ADDRS (index 0 is VSI).
   */

  static const uintptr_t pf_pci_base[] = ENETC_PCI_TYPE0_BASE_ADDRS;

  pf = (ENETC_PCI_TYPE0_Type *)
       pf_pci_base[1u + getSiInstance((netc_hw_si_idx_t)cfg->si_idx)];

  vf_ready = ENETC_PCI_TYPE0_PCIE_CFC_SRIOV_CTL_VF_ENABLE_MASK |
             ENETC_PCI_TYPE0_PCIE_CFC_SRIOV_CTL_VF_MSE_MASK;

  {
    uint16_t sriov_ctl = pf->PCIE_CFC_SRIOV_CTL;
    syslog(LOG_DEBUG, "NETC pf=%p PCIE_CFC_SRIOV_CTL=0x%04x mask=0x%04x\n",
           pf, sriov_ctl, vf_ready);
    return (sriov_ctl & vf_ready) == vf_ready;
  }
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

  /* Block until the A55 rptun first kick has been received.
   * The rptun MU7 kick arrives when Linux remoteproc/rpmsg initializes.
   * The NETCMIX TRDC firewall is opened by the Linux NETC PF driver which
   * loads AFTER the remoteproc connection.  Wait an additional quiet window
   * after the kick to ensure the firewall is open before accessing SI regs.
   */
  syslog(LOG_ERR, "NETC VSI: waiting for rptun ready (A55 MU kick)...\n");
  imx9_rptun_wait_ready();
  syslog(LOG_ERR,
         "NETC VSI: rptun kick received, waiting %u ms for TRDC open...\n",
         (unsigned)CONFIG_IMX9_NETC_VSI_RPTUN_READY_DELAY_MS);
  nxsig_usleep((uint32_t)CONFIG_IMX9_NETC_VSI_RPTUN_READY_DELAY_MS * 1000u);
  syslog(LOG_ERR, "NETC VSI: TRDC open, proceeding to SI bringup\n");

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

  /* Populate capability.vsiNum without accessing the PSI-privileged ENETC
   * management block at hw.base (ENETC0_BASE = 0x4CC10000).  That register
   * block is owned by the PSI/A55 and M7 reads to it can disrupt the Linux
   * PSI driver on the NETC interconnect.
   */
  handle->capability.vsiNum = 4U;

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
      syslog(LOG_ERR, "VSI ready; carrier up, link-notify enabled\n");
    }
  else
    {
      syslog(LOG_ERR, "VSI ready; carrier up (link-notify unavailable)\n");
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

#ifdef ENETC_VSI_DEBUG_TRX
  syslog(LOG_ERR, "\n\nVSI RX: entering!\n");
#endif
	status_t rx_result;
	uint32_t length;

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

#ifdef ENETC_VSI_DEBUG_TRX
  /* Log raw BD fields to diagnose non-regular frames. */
  {
    volatile netc_rx_bd_t *bd =
        &handle->rxBdRing[0].bdBase[handle->rxBdRing[0].index];
    syslog(LOG_ERR,
           "VSI RX: rx_result=%d length=%u "
           "bd[%u]: isReady=%u isFinal=%u error=%u hr=%u bufLen=%u\n",
           (int)rx_result, (unsigned)length,
           (unsigned)handle->rxBdRing[0].index,
           (unsigned)bd->writeback.isReady,
           (unsigned)bd->writeback.isFinal,
           (unsigned)bd->writeback.error,
           (unsigned)bd->writeback.hr,
           (unsigned)bd->writeback.bufLen);
  }
#endif

  if (rx_result == kStatus_NETC_RxFrameEmpty)
    {
      return -ENOBUFS;
    }

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

      syslog(LOG_ERR, "VSI RX: dropping frame, EP_GetRxFrameSize=%d\n",
             (int)rx_result);
      EP_ReceiveFrameCopy(handle, 0, NULL, 0, NULL);
      return -EIO;
    }

  status = EP_ReceiveFrameCopy(handle, 0, priv->rx_frame, length, &attr);
#ifdef ENETC_VSI_DEBUG_TRX
  syslog(LOG_ERR, "VSI RX: status=%d len=%u\n", (int)status, (unsigned)length);
#endif
  if (status == kStatus_NETC_RxFrameEmpty)
    {
      return -ENOBUFS;
    }

  if (status != kStatus_Success || length == 0)
    {
      syslog(LOG_ERR, "VSI RX: error status=%d len=%u\n",
             (int)status, (unsigned)length);
      return -EIO;
    }

#ifdef ENETC_VSI_DEBUG_TRX
  {
    const uint8_t *f = priv->rx_frame;
    static uint32_t rx_frames;
    rx_frames++;
    syslog(LOG_ERR,
           "VSI RX#%u len=%u "
           "dst=%02x:%02x:%02x:%02x:%02x:%02x "
           "src=%02x:%02x:%02x:%02x:%02x:%02x "
           "type=%02x%02x\n",
           rx_frames, (unsigned)length,
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
#ifdef CONFIG_NET_ARP
      arp_ipin(&priv->dev);
#endif
      ipv4_input(&priv->dev);

      if (priv->dev.d_len > 0)
        {
          imx9_netc_vsi_transmit(&priv->dev);
        }
    }
#ifdef CONFIG_NET_ARP
  else if (BUF->ether_type == HTONS(ETHTYPE_ARP))
    {
      arp_input(&priv->dev);

      if (priv->dev.d_len > 0)
        {
          imx9_netc_vsi_transmit(&priv->dev);
        }
    }
#endif
#endif

#ifdef CONFIG_NET_IPv6
  if (BUF->ether_type == HTONS(ETHTYPE_IP6))
    {
      ipv6_input(&priv->dev);

      if (priv->dev.d_len > 0)
        {
          imx9_netc_vsi_transmit(&priv->dev);
        }
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
  netc_buffer_struct_t pkt;
  netc_frame_struct_t frame  =
    {
      .buffArray = &pkt,
      .length    = 1,
    };

  if (len > CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE)
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
           "VSI TX#%u len=%u "
           "dst=%02x:%02x:%02x:%02x:%02x:%02x "
           "src=%02x:%02x:%02x:%02x:%02x:%02x "
           "type=%02x%02x\n",
           tx_dbg, (unsigned)len,
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

  if (EP_SendFrame(handle, 0, &frame, NULL, NULL) != kStatus_Success)
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
      syslog(LOG_ERR, "VSI MSGINTR: RX fired count=%u pending=0x%08x\n",
             (unsigned)rx_count, (unsigned)pending);
#endif
      /* Disable RX threshold interrupt to prevent ISR flood.
       * The SI asserts MSI-X while SIRXIDR0 is set AND RBIER.RXTIE=1.
       * Clearing RBIER.RXTIE stops re-assertion without losing pending
       * frame notifications.  The service thread re-enables RXTIE after
       * draining the ring (NAPI-style: disable in ISR, re-arm after drain).
       */
      EP_CleanRxIntrFlags(&priv->handle, 1U);
      nxsem_post(&priv->rx_sem);
    }

  if (pending & NETC_MSG_MSGINTR_BIT)
    {
#ifdef ENETC_VSI_DEBUG_INTR
      msg_count++;
      syslog(LOG_ERR, "VSI MSGINTR: MSG fired count=%u pending=0x%08x\n",
             (unsigned)msg_count, (unsigned)pending);
#endif
      /* If si_bringup has not yet completed, the rptun semaphore gates
       * the service thread; this MSG interrupt is informational only.
       * After vf_confirmed is set (si_bringup done), dispatch normally.
       */
      if (!priv->vf_confirmed)
        {
          /* rptun semaphore handles the firewall-open signal; MSG interrupt
           * before si_bringup is informational only.
           */
          syslog(LOG_ERR,
                 "NETC VSI: MSG interrupt before si_bringup (rptun handles gate)\n");
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

  switch (imx9_netc_vsi_msg_poll_link(priv))
    {
      case IMX9_NETC_MSG_LINK_UP:
        /* Ignore spurious LINK_UP before si_ready: bring_up calls
         * program_psi itself; a stale interrupt here would cause a
         * double-message that corrupts the PSI state machine.
         */
        syslog(LOG_ERR, "imx9_netc_vsi_msg_poll_link: LINK_UP!\n");
        priv->link_notify = true;
        priv->si_ready = true;
        break;

      case IMX9_NETC_MSG_LINK_DOWN:
      syslog(LOG_ERR, "imx9_netc_vsi_msg_poll_link: LINK_DOWN!\n");
        priv->si_ready = false;
        netdev_carrier_off(&priv->dev);
        break;

      default:
        syslog(LOG_ERR, "imx9_netc_vsi_msg_poll_link: unknown msg !!!\n");
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
      /* Wait for RX wake, txavail notify, link change, or poll timeout.
       * Use a relative-tick wait to avoid depending on CLOCK_REALTIME
       * being set (NuttX boots with tv_sec=0 if no RTC; an absolute
       * timeout of epoch+500ms is always in the past, causing an instant
       * return and a 100% CPU busy-spin).
       */

      if (0 == nxsem_tickwait(&priv->rx_sem, MSEC2TICK(CONFIG_IMX9_NETC_VSI_READY_POLL_MS)))
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

#if 0 /* Not needed, kept for debugging/experimental purpouse. */
        /* Ring drained: re-arm the RX threshold interrupt.
         * The ISR callback cleared SIRXIDR0 (W1C) and disabled RBIER.RXTIE
         * to prevent flooding.  Re-enable RXTIE here so the next frame
         * that arrives fires MSI-X -> MSIR -> ISR again (NAPI re-arm). */
        priv->handle.hw.si->BDR[0].RBIER = ENETC_SI_RBIER_RXTIE(1U);
#endif
      }

      /* Drain any extra posts the ISR accumulated while we were working. */

      while (nxsem_trywait(&priv->rx_sem) == 0)
      {
          /* consumed */
      }

      /* ready = imx9_netc_vsi_vf_ready(priv); */

      if (up && !ready)
        {
          nxrmutex_lock(&priv->msg_lock);
          priv->si_ready = false;
          nxrmutex_unlock(&priv->msg_lock);
          netdev_carrier_off(&priv->dev);
          ninfo("VSI down; awaiting SR-IOV re-enable\n");
          up = false;
        }
      else if (!up && ready)
        {
          imx9_netc_vsi_bring_up(priv);
          up = true;
          priv->si_ready = true;
        }

      if (!up)
        {
          continue;
        }

      if (priv->link_notify)
        {
          priv->link_notify = false;
          imx9_netc_vsi_program_psi(priv);
          netdev_carrier_on(&priv->dev);
          ninfo("VSI link-up; PSI reprogrammed\n");
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
          priv->dev.d_buf = priv->rx_frame;
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
      syslog(LOG_ERR, "VSI TX: si_ready=false, dropping\n");
      return -ENETDOWN;
    }

  if (!imx9_netc_vsi_vf_ready(priv))
    {
      nxsem_post(&priv->rx_sem);
      syslog(LOG_ERR, "VSI TX: vf not ready, dropping\n");
      return -ENETDOWN;
    }

  ret = imx9_netc_vsi_tx(priv, dev->d_buf, dev->d_len);
#ifdef ENETC_VSI_DEBUG_TRX
  tx_frames++;
  syslog(LOG_ERR, "VSI TX: len=%u ret=%d total=%u\n",
         (unsigned)dev->d_len, ret, tx_frames);
#endif
  return ret;
}

static int imx9_netc_vsi_txavail(struct net_driver_s *dev)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;

#ifdef ENETC_VSI_DEBUG_TRX
  syslog(LOG_ERR, "VSI txavail: entered !!!\n");
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

  /* Use rx_frame as a scratch buffer for devif_poll (same as the service
   * thread does).  rx_frame is in DDR and large enough for a max frame.
   * imx9_netc_vsi_tx will copy from it into a separate tx_buff[] slot, so
   * there is no overlapping-memcpy hazard. */

  dev->d_buf = priv->rx_frame;
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
   * ifdown() called imx9_msgintr_deinit() which disabled the IRQSTEER
   * channel and detached the ISR; re-init restores both.
   */
  imx9_msgintr_init(cfg->msgintr_instance,
                    imx9_netc_vsi_msgintr_cb, priv);

  /* Wake the service thread so it runs bring_up() again.
   * The thread is parked in nxsem_tickwait on rx_sem after ifdown.
   * Posting rx_sem causes it to loop, see !up && ready, and call
   * imx9_netc_vsi_bring_up() which re-initializes the SI BD rings
   * and re-programs the PSI.
   * Note: vf_confirmed is already true (SI firewall still open), so
   * wait_ready() returns immediately on the second bring_up.
   */
  nxsem_post(&priv->rx_sem);

  return OK;
}

static int imx9_netc_vsi_ifdown(struct net_driver_s *dev)
{
  struct imx9_netc_vsi_s *priv = (struct imx9_netc_vsi_s *)dev;

  imx9_msgintr_deinit(priv->config->msgintr_instance);
  priv->si_ready = false;
  netdev_carrier_off(dev);
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
 * Name: imx9_netc_vsi_initialize
 *
 * Description:
 *   Initialize and register the ENETC VSI Ethernet driver.
 *   Called once from the board bringup code (imx95_bringup.c).
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_netc_vsi_initialize(void)
{
  struct imx9_netc_vsi_s    *priv   = &g_vsi0;
  const struct imx9_netc_vsi_config_s *cfg = &g_vsi0_config;
  ep_handle_t               *handle = &priv->handle;
  char                       arg1[32];
  char                      *argv[2];
  int                        ret;

  memset(priv, 0, sizeof(*priv));
  priv->config    = cfg;
  priv->tx_buff   = (uint8_t (*)[CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE])g_vsi0_tx_buff;
  priv->msg_buff  = (uint8_t *)g_vsi0_msg_buff;
  priv->rx_frame  = (uint8_t *)g_vsi0_rx_frame;

  NETC_SocGetBaseResource(&handle->hw, (netc_hw_si_idx_t)cfg->si_idx);

  /* Fill SDK handle cfg (no register access here) */

  handle->cfg.si             = (netc_hw_si_idx_t)cfg->si_idx;
  handle->cfg.rxRingUse      = 1;
  handle->cfg.txRingUse      = 1;
  handle->cfg.rxBdrGroupNum  = 0;
  handle->cfg.ringPerBdrGroup = 1;
  handle->cfg.rxCacheMaintain = false;
  handle->cfg.txCacheMaintain = false;
  handle->cfg.rxZeroCopy      = false;
  handle->cfg.entryNum        = NETC_MSIX_VSI_EVENTS_COUNT;
  handle->cfg.userData        = priv;
  handle->cfg.reclaimCallback = NULL;

  /* Determine MAC address */

#ifdef IMX9_NETC_VSI0_MAC
  {
    static const uint8_t board_mac[] = IMX9_NETC_VSI0_MAC;
    memcpy(priv->mac_addr, board_mac, IFHWADDRLEN);
  }
#else
  imx9_netc_default_mac(priv->mac_addr, cfg->mac_default_index);
#endif

  priv->si_ready    = false;
  priv->link_notify = false;

  nxrmutex_init(&priv->msg_lock);
  nxsem_init(&priv->tx_sem, 0, 0);
  nxsem_init(&priv->rx_sem, 0, 0);

  /* Register the MSGINTR callback early.  The PSI sends a MSG interrupt
   * only after it has opened the NETC TRDC/XRDC firewall.  MSGINTR lives
   * at 0x4B230000 and is NOT firewalled, so this call is safe even when
   * the VSI SI register space (0x4CD30000) is still locked.
   */

  imx9_msgintr_init(cfg->msgintr_instance,
                    imx9_netc_vsi_msgintr_cb, priv);

  /* Configure the netdev */

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

  /* Spawn the service thread that will drive the SI */
  snprintf(arg1, sizeof(arg1), "%p", priv);
  argv[0] = arg1;
  argv[1] = NULL;

  priv->rx_thread_pid = kthread_create("netc_vsi",
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

  ninfo("ENETC VSI0 registered (SI idx 0x%04x, MAC %02x:%02x:%02x:%02x:%02x:%02x)\n",
        cfg->si_idx,
        priv->mac_addr[0], priv->mac_addr[1], priv->mac_addr[2],
        priv->mac_addr[3], priv->mac_addr[4], priv->mac_addr[5]);

  return OK;
}

