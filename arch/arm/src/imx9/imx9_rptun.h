/****************************************************************************
 * arch/arm/src/imx9/imx9_rptun.h
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2024 NXP
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

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_RPTUN_H
#define __ARCH_ARM_SRC_IMX9_IMX9_RPTUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: imx9_rptun_init
 ****************************************************************************/

int imx9_rptun_init(const char *shmemname, const char *cpuname);

/****************************************************************************
 * Name: imx9_rptun_wait_ready
 *
 * Description:
 *   Block until the first MU7 rptun kick from the A55 has been received.
 *   At that point the NETCMIX TRDC firewall has been opened by Linux and
 *   it is safe to access the NETC PCI config space.
 ****************************************************************************/

void imx9_rptun_wait_ready(void);

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_RPTUN_H */
