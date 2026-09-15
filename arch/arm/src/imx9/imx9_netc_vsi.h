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

/* Public API for the NETC VSI Ethernet driver on iMX95 M7.
 *
 * Call imx9_netc_vsi_initialize() once from the board bringup function
 * (imx95_bringup.c) when CONFIG_IMX9_NETC_VSI is selected.  The function
 * registers the network interface with the NuttX network stack and spawns
 * the service kthread; it does not block waiting for SR-IOV readiness.
 */

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_NETC_VSI_H
#define __ARCH_ARM_SRC_IMX9_IMX9_NETC_VSI_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_IMX9_NETC_VSI

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: imx9_netc_vsi_initialize
 *
 * Description:
 *   Initialize and register the NETC VSI Ethernet driver.  Allocates driver
 *   state, registers the netdev with the NuttX network stack (eth0), and
 *   spawns the "netc_vsi" service kthread.  SR-IOV readiness and PSI
 *   mailbox negotiation happen asynchronously inside the kthread after the
 *   interface is brought up (d_ifup).
 *
 * Returned Value:
 *   0 on success, or a negated errno on failure.
 *
 ****************************************************************************/

int imx9_netc_vsi_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_IMX9_NETC_VSI */
#endif /* __ARCH_ARM_SRC_IMX9_IMX9_NETC_VSI_H */
