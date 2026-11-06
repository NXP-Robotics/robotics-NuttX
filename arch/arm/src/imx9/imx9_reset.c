/****************************************************************************
 * arch/arm/src/imx9/imx9_reset.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2026 NXP
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
#include "imx9_reset.h"
#include "imx9_scmi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/
static bool imx9_reset_allowed = true;

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void imx9_reset_setallowed(bool value)
{
    imx9_reset_allowed = value;
}

bool imx9_reset_getallowed(void)
{
    return imx9_reset_allowed;
}

int imx9_reset_perform(void)
{
    return imx9_scmi_syspowerstateset(SM_PLATFORM_A2P, SCMI_SYS_FLAG_FORCEFUL, SCMI_SYS_STATE_COLD_RESET);
}
