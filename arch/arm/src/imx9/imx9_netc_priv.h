/****************************************************************************
 * arch/arm/src/imx9/imx9_netc_priv.h
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

/* Private types shared between imx9_netc_vsi.c and imx9_netc_msg.c.
 *
 * Ported from Zephyr eth_nxp_imx_netc_priv.h / eth_nxp_imx_netc_vsi.c.
 * OS primitives replaced:
 *   struct k_mutex  -> mutex_t  (nuttx/mutex.h)
 *   struct k_sem    -> sem_t    (semaphore.h)
 *   struct k_thread -> pthread_t + pthread_attr_t
 */

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_NETC_PRIV_H
#define __ARCH_ARM_SRC_IMX9_IMX9_NETC_PRIV_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/mutex.h>
#include <nuttx/net/netdev.h>
#include <nuttx/net/ethernet.h>

/* NXP MCUX SDK NETC HAL - no OS dependencies, pure register-level */

#include "fsl_netc_endpoint.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Buffer and descriptor alignment requirements (from NETC hardware spec) */

#define NETC_BUFF_ALIGN   128u   /* byte alignment for data buffers */
#define NETC_BD_ALIGN     128u   /* byte alignment for BD ring entries */

/* MSI-X entry indices for a VSI (3 events: TX, RX, PSI->VSI message) */

#define NETC_TX_MSIX_ENTRY_IDX   0u
#define NETC_RX_MSIX_ENTRY_IDX   1u
#define NETC_MSG_MSIX_ENTRY_IDX  2u   /* VSI only */

#define NETC_MSIX_VSI_EVENTS_COUNT  3u

/* MSGINTR channel and IBS (Interrupt Bit Select) assignments.
 *
 * All three VSI MSI-X events (TX done, RX, PSI->VSI message) share a
 * SINGLE MSGINTR channel (channel 0).  Each event uses a different IBS
 * value encoded in the MSI-X msgData word.  The MSGINTR hardware writes
 * the IBS value as a bit position into the MSIR register; the ISR reads
 * MSIR and checks each bit to identify which event fired.
 *
 * This matches the NXP SDK reference (MSGINTR_GetIntrSelectAddr + single
 * channel) and the Zephyr iMX95 PSI driver implementation, both of which
 * funnel all interrupt sources through one MSGINTR channel.  Using a
 * separate channel per event is not supported by the NETC SI hardware.
 */

#define NETC_MSGINTR_CHANNEL      0u   /* single channel used for all events */

#define NETC_TX_MSGINTR_BIT       (1u << 0u)  /* IBS=0, MSIR bit 0 */
#define NETC_RX_MSGINTR_BIT       (1u << 1u)  /* IBS=1, MSIR bit 1 */
#define NETC_MSG_MSGINTR_BIT      (1u << 2u)  /* IBS=2, MSIR bit 2 */

/* IBS data values written into each MSI-X entry's msgData word */

#define NETC_TX_MSGINTR_IBS       0u
#define NETC_RX_MSGINTR_IBS       1u
#define NETC_MSG_MSGINTR_IBS      2u


/* Tx interrupt coalescing: fire after ~half the ring completes or timeout.
 * ICPT is a 4-bit field (max 15); ICTT is in NETC clock cycles.
 */

#define NETC_TX_COALESCE_FRAMES  \
  (((CONFIG_IMX9_NETC_VSI_TX_RING_LEN) / 2u) < 15u ? \
   ((CONFIG_IMX9_NETC_VSI_TX_RING_LEN) / 2u) : 15u)
#define NETC_TX_COALESCE_TICKS   1024u

/* Deterministic fallback MAC (Freescale OUI, locally administered).
 * Used when no MAC is available from the board configuration.
 */

#define NETC_FALLBACK_OUI_B0  0x02u  /* bit 1 set = locally administered */
#define NETC_FALLBACK_OUI_B1  0x04u
#define NETC_FALLBACK_OUI_B2  0x9fu

/* Retry parameters for the initial primary-MAC request to the PSI */

#define NETC_VSI_MAC_SET_RETRIES   10
#define NETC_VSI_MAC_SET_RETRY_MS  200

/* Size of the DMA-visible VSI->PSI message buffer (bytes, 64-byte aligned) */

#define NETC_MSG_VSI_BUF_SIZE  64u

/* RX single-frame reassembly buffer size (aligned to NETC_BUFF_ALIGN) */

#define NETC_RX_FRAME_BUF_SIZE \
  (((CONFIG_IMX9_NETC_VSI_RX_BUF_SIZE) + NETC_BUFF_ALIGN - 1u) & \
   ~(NETC_BUFF_ALIGN - 1u))

/* MAC filter type bits (VSI->PSI mailbox ABI) */

#define NETC_MAC_FILTER_TYPE_UC  (1u << 0)
#define NETC_MAC_FILTER_TYPE_MC  (1u << 1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Per-instance Rx buffer type (non-cacheable, aligned) */

typedef uint8_t netc_rx_buf_t[NETC_RX_FRAME_BUF_SIZE];

/* Driver configuration (compile-time constants from board.h / Kconfig).
 * One instance of this struct is defined as a const in imx9_netc_vsi.c.
 */

struct imx9_netc_vsi_config_s
{
  /* SI index: encodes (mac_instance << 8) | si_index */

  uint16_t si_idx;

  /* MSGINTR instance used for all three MSI-X events (TX, RX, MSG) */

  int msgintr_instance;

  /* Per-instance index for the deterministic fallback MAC address */

  uint8_t mac_default_index;

  /* BD ring initializer callback: fills in bdr_config, rx_bdr_config,
   * tx_bdr_config with per-instance buffer pointers and sizes.
   */

  void (*bdr_init)(netc_bdr_config_t    *bdr_config,
                   netc_rx_bdr_config_t *rx_bdr_config,
                   netc_tx_bdr_config_t *tx_bdr_config);
};

/* Driver state (one instance per VSI) */

struct imx9_netc_vsi_s
{
  /* net_driver_s MUST be the first member so the driver can be cast
   * between struct imx9_netc_vsi_s * and struct net_driver_s *.
   */

  struct net_driver_s dev;

  /* Back-pointer to compile-time config */

  const struct imx9_netc_vsi_config_s *config;

  /* NXP SDK endpoint handle (owns all SI register access) */

  ep_handle_t handle;

  /* MAC address (6 bytes) */

  uint8_t mac_addr[IFHWADDRLEN];

  /* TX path */

  uint8_t (*tx_buff)[CONFIG_IMX9_NETC_VSI_TX_BUF_SIZE];
  uint8_t   tx_idx;    /* round-robin index into tx_buff[] */
  sem_t   tx_sem;

  /* RX path */

  sem_t    rx_sem;
  pid_t    rx_thread_pid;
  uint8_t *rx_frame;   /* non-cacheable single-frame reassembly buffer */

  /* VSI->PSI control message buffer (non-cacheable, 64-byte aligned) */

  uint8_t *msg_buff;

  /* SI readiness: true once BD rings are configured and SI is enabled.
   * Written under msg_lock; read from multiple contexts.
   */

  volatile bool si_ready;

  /* Set to true once SR-IOV VF-enable is confirmed during initial bring-up.
   * After this point, imx9_netc_vsi_vf_ready() skips the read of PSI PCI
   * config space and returns true immediately, avoiding any interference
   * with the PSI/A55 NETC driver on the shared interconnect.
   * Cleared to false only when a link-down is detected so the next
   * bring-up re-waits for SR-IOV.
   */


  volatile bool vf_confirmed;

  /* Link-up notification from the PSI->VSI message ISR.
   * Set in ISR context, cleared and acted on in the service thread.
   */

  volatile bool link_notify;

  /* TX poll request from txavail().
   * Set in txavail() context, cleared and acted on in the service thread.
   */

  volatile bool tx_pending;

  /* Multicast hash filter (64 bits) mirrored to the PSI.
   * refcnt[] keeps a count per hash-bucket so colliding groups
   * do not clear the bit when only one group leaves.
   */

  uint64_t mc_hash;
  uint8_t  mc_hash_refcnt[64];

  /* VLAN hash filter (64 bits), same reference-counting scheme */

  uint64_t vlan_hash;
  uint8_t  vlan_hash_refcnt[64];

  /* Serializes VSI->PSI messages from the service thread and ifioctl().
   * Must be a recursive mutex: addmac/rmmac/program_psi hold this lock
   * while calling msg_set_mac_hash / msg_set_vlan_hash, which internally
   * call msg_send, which also acquires msg_lock.
   */

  rmutex_t msg_lock;
};

/****************************************************************************
 * Public Function Prototypes (shared between vsi.c and msg.c)
 ****************************************************************************/

/* TX/RX primitives (imx9_netc_vsi.c) */

int  imx9_netc_vsi_tx(struct imx9_netc_vsi_s *priv,
                      const uint8_t *buf, size_t len);
int  imx9_netc_vsi_rx(struct imx9_netc_vsi_s *priv);
void imx9_netc_tx_coalesce_init(struct imx9_netc_vsi_s *priv);

/* PSI->VSI message ISR handler, called from the MSGINTR callback */

void imx9_netc_vsi_msg_isr(struct imx9_netc_vsi_s *priv);

/* Default MAC generator */

static inline void imx9_netc_default_mac(uint8_t mac[IFHWADDRLEN],
                                         uint8_t index)
{
  mac[0] = NETC_FALLBACK_OUI_B0;
  mac[1] = NETC_FALLBACK_OUI_B1;
  mac[2] = NETC_FALLBACK_OUI_B2;
  mac[3] = 0x00u;
  mac[4] = 0x11u;
  mac[5] = index;
}

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_NETC_PRIV_H */
