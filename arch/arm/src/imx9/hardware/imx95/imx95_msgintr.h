/****************************************************************************
 *
 * Copyright 2022, 2026 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 ****************************************************************************/

/* Register layout for the iMX95 MSGINTR peripheral (message-signaled
 * interrupt controller).  Each instance exposes 3 MSI channels.  A PCIe
 * endpoint triggers a channel by writing any value to MSIIR; the M7 reads
 * MSIR to see which bit(s) fired, then clears them by reading.
 *
 * Reference: iMX95 Reference Manual, MSGINTR chapter.
 * Derived from NXP MCUX SDK PERI_MSGINTR.h (BSD-3-Clause).
 */

#ifndef __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_MSGINTR_H
#define __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_MSGINTR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include "hardware/imx95/imx95_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Number of MSI channels per MSGINTR instance */

#define IMX95_MSGINTR_CHANNEL_NUM  3u

/* Per-channel register offsets within the MSI[n] array (stride = 8 bytes) */

#define IMX95_MSGINTR_MSIIR_OFFSET  0x00u  /* write-only: trigger interrupt */
#define IMX95_MSGINTR_MSIR_OFFSET   0x04u  /* read-only:  pending bit field */
#define IMX95_MSGINTR_MSI_STRIDE    0x08u  /* bytes per channel entry */

/* Absolute register addresses for MSGINTR1 */

#define IMX95_MSGINTR1_MSI0_MSIIR  (IMX9_MSGINTR1_BASE + 0x00u)
#define IMX95_MSGINTR1_MSI0_MSIR   (IMX9_MSGINTR1_BASE + 0x04u)
#define IMX95_MSGINTR1_MSI1_MSIIR  (IMX9_MSGINTR1_BASE + 0x08u)
#define IMX95_MSGINTR1_MSI1_MSIR   (IMX9_MSGINTR1_BASE + 0x0cu)
#define IMX95_MSGINTR1_MSI2_MSIIR  (IMX9_MSGINTR1_BASE + 0x10u)
#define IMX95_MSGINTR1_MSI2_MSIR   (IMX9_MSGINTR1_BASE + 0x14u)

/* Absolute register addresses for MSGINTR2 */

#define IMX95_MSGINTR2_MSI0_MSIIR  (IMX9_MSGINTR2_BASE + 0x00u)
#define IMX95_MSGINTR2_MSI0_MSIR   (IMX9_MSGINTR2_BASE + 0x04u)
#define IMX95_MSGINTR2_MSI1_MSIIR  (IMX9_MSGINTR2_BASE + 0x08u)
#define IMX95_MSGINTR2_MSI1_MSIR   (IMX9_MSGINTR2_BASE + 0x0cu)
#define IMX95_MSGINTR2_MSI2_MSIIR  (IMX9_MSGINTR2_BASE + 0x10u)
#define IMX95_MSGINTR2_MSI2_MSIR   (IMX9_MSGINTR2_BASE + 0x14u)

/* MSIIR field: IBS - Interrupt Bit Select (bits [4:0]) */

#define MSGINTR_MSIIR_IBS_MASK   (0x1fu)
#define MSGINTR_MSIIR_IBS_SHIFT  (0u)
#define MSGINTR_MSIIR_IBS(x)     (((uint32_t)(x)) & MSGINTR_MSIIR_IBS_MASK)

/* MSIR field: SHn - pending interrupt bits (bits [31:0], one per IBS value) */

#define MSGINTR_MSIR_SHN_MASK   (0xffffffffu)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Hardware register layout - matches the MSGINTR_Type from the NXP SDK.
 * Use volatile pointers to IMX9_MSGINTRn_BASE to access registers.
 */

struct imx95_msgintr_regs_s
{
  struct
    {
      volatile uint32_t msiir;  /* offset 0x00: write to trigger interrupt */
      volatile uint32_t msir;   /* offset 0x04: read to get pending bits   */
    } msi[IMX95_MSGINTR_CHANNEL_NUM];
};

#endif /* __ARCH_ARM_SRC_IMX9_HARDWARE_IMX95_IMX95_MSGINTR_H */
