/****************************************************************************
 *
 * Copyright 2022, 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/* NuttX driver for the iMX95 MSGINTR (Message-Signaled Interrupt) peripheral.
 *
 * MSGINTR is a write-to-address doorbell mechanism: a PCIe endpoint (e.g.,
 * NETC SI) triggers an interrupt on the M7 by writing a value to a physical
 * address (the MSIIR register of a chosen channel).  The MSGINTR peripheral
 * latches the write and asserts an NVIC IRQ.  The ISR reads MSIR to identify
 * the pending bit(s) and calls the registered callback.
 *
 * Two instances are available on iMX95: MSGINTR1 and MSGINTR2.
 * Each has 3 MSI channels.
 */

#ifndef __ARCH_ARM_SRC_IMX9_IMX9_MSGINTR_H
#define __ARCH_ARM_SRC_IMX9_IMX9_MSGINTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* MSGINTR instance indices (1-based, matching hardware naming) */

#define IMX9_MSGINTR_INSTANCE1  1
#define IMX9_MSGINTR_INSTANCE2  2

/* Number of MSI channels per instance */

#define IMX9_MSGINTR_CHANNEL_NUM  3u

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Callback invoked from the ISR when a channel fires.
 *
 * Parameters:
 *   instance  - MSGINTR instance index (1 or 2)
 *   channel   - MSI channel index (0 to IMX9_MSGINTR_CHANNEL_NUM - 1)
 *   pending   - bitmask of pending interrupt bits read from MSIR
 *   arg       - opaque argument registered with imx9_msgintr_init()
 */

typedef void (*imx9_msgintr_cb_t)(int instance, uint8_t channel,
                                  uint32_t pending, void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: imx9_msgintr_init
 *
 * Description:
 *   Initialize an MSGINTR instance and register the ISR callback.
 *   Attaches the NVIC interrupt and enables it.
 *
 * Input Parameters:
 *   instance - MSGINTR instance (IMX9_MSGINTR_INSTANCE1 or _INSTANCE2)
 *   callback - Function called from the ISR for each pending channel
 *   arg      - Opaque argument passed to the callback
 *
 * Returned Value:
 *   OK on success, negated errno on failure.
 *
 ****************************************************************************/

int imx9_msgintr_init(int instance, imx9_msgintr_cb_t callback, void *arg);

/****************************************************************************
 * Name: imx9_msgintr_deinit
 *
 * Description:
 *   Disable and detach the MSGINTR instance interrupt.
 *   Clears any pending status bits.
 *
 * Input Parameters:
 *   instance - MSGINTR instance (IMX9_MSGINTR_INSTANCE1 or _INSTANCE2)
 *
 ****************************************************************************/

void imx9_msgintr_deinit(int instance);

/****************************************************************************
 * Name: imx9_msgintr_get_addr
 *
 * Description:
 *   Return the physical address that an MSI-X source must write to in order
 *   to trigger the given channel of the given MSGINTR instance.
 *   This address is programmed into the PCIe endpoint's MSI-X table.
 *
 * Input Parameters:
 *   instance - MSGINTR instance (IMX9_MSGINTR_INSTANCE1 or _INSTANCE2)
 *   channel  - MSI channel index (0 to IMX9_MSGINTR_CHANNEL_NUM - 1)
 *
 * Returned Value:
 *   Physical address of the MSIIR register for the given instance/channel.
 *
 ****************************************************************************/

uintptr_t imx9_msgintr_get_addr(int instance, uint8_t channel);

/****************************************************************************
 * Name: imx9_msgintr_rearm
 *
 * Description:
 *   Re-enable the IRQSTEER channel mask for the given MSGINTR instance
 *   after the ISR has been serviced and the consumer has drained its ring
 *   and re-armed its hardware interrupt enable (e.g. RBIER=1 for NETC VSI).
 *
 *   The ISR masks the channel before invoking the callback to prevent the
 *   level-triggered RX threshold from causing an ISR flood.  The consumer
 *   must call this function once it has cleared the condition.
 *
 * Input Parameters:
 *   instance - MSGINTR instance (IMX9_MSGINTR_INSTANCE1 or _INSTANCE2)
 *
 ****************************************************************************/

void imx9_msgintr_rearm(int instance);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_SRC_IMX9_IMX9_MSGINTR_H */
