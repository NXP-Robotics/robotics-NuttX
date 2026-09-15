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

/* NXP NETC VSI-to-PSI mailbox messaging API.
 *
 * A VSI cannot write the PSI's shared MAC/filter/link registers directly,
 * so it requests those operations from the PSI owner over the VSI-to-PSI
 * message channel.  This module owns that transport and the per-operation
 * message encoders; imx9_netc_vsi.c calls this API and keeps the
 * interface/lifecycle policy.
 *
 * Ported from Zephyr eth_nxp_imx_netc_msg.h (Apache-2.0).
 */

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_NETC_MSG_H
#define __ARCH_ARM_SRC_IMX9_IMX9_NETC_MSG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/* Forward declaration - full definition in imx9_netc_priv.h */

struct imx9_netc_vsi_s;

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Link state parsed from a pending PSI->VSI message */

enum imx9_netc_msg_link_e
{
  IMX9_NETC_MSG_LINK_NONE = 0,
  IMX9_NETC_MSG_LINK_UP,
  IMX9_NETC_MSG_LINK_DOWN,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* Ask the PSI to route this VSI's primary unicast MAC (priv->mac_addr) */

int imx9_netc_vsi_msg_set_primary_mac(struct imx9_netc_vsi_s *priv);

/* Program this VSI's 64-bit multicast hash table (priv->mc_hash) in PSI */

int imx9_netc_vsi_msg_set_mac_hash(struct imx9_netc_vsi_s *priv);

/* Program this VSI's 64-bit VLAN hash table (priv->vlan_hash) in PSI */

int imx9_netc_vsi_msg_set_vlan_hash(struct imx9_netc_vsi_s *priv);

/* Ask the PSI to set or clear UC/MC promiscuous mode for this VSI */

int imx9_netc_vsi_msg_set_mac_promisc(struct imx9_netc_vsi_s *priv,
                                      uint8_t type_mask, bool enable);

/* Subscribe to PSI link-status notifications for this VSI */

int imx9_netc_vsi_msg_enable_link_notify(struct imx9_netc_vsi_s *priv);

/* Consume a pending PSI->VSI message and report any link-state change */

enum imx9_netc_msg_link_e
imx9_netc_vsi_msg_poll_link(struct imx9_netc_vsi_s *priv);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_NETC_MSG_H */
