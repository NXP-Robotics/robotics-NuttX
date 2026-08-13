/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* fsl_memory.h for iMX95 CM7 NuttX port.
 * Adapted from NXP SDK MIMX9596/drivers/fsl_memory.h.
 * Intentionally does NOT include fsl_common.h to avoid a circular
 * include chain (fsl_netc.h -> fsl_common.h -> fsl_device_registers.h
 * -> MIMX9594_cm7_netc.h -> [this file via fsl_netc.h]).
 * Only stdint.h is needed for uint32_t.
 *
 * Address translation for iMX95 CM7:
 *   - ITCM (0x00000000 - 0x0003FFFF) -> system view: +0x203C0000
 *   - DTCM (0x20000000 - 0x2007FFFF) -> system view: +0x400000
 *   - DDR  (0x90000000 ...) -> NO OFFSET, same address on DMA bus
 */

#ifndef FSL_MEMORY_H_
#define FSL_MEMORY_H_

#include <stdint.h>

/* CM7 local ITCM address range and system-bus offset */
#define FSL_MEM_M7_ITCM_BEGIN   0x00000000U
#define FSL_MEM_M7_ITCM_END     0x0003FFFFU
#define FSL_MEM_M7_ITCM_OFFSET  0x203C0000U

/* CM7 local DTCM address range and system-bus offset */
#define FSL_MEM_M7_DTCM_BEGIN   0x20000000U
#define FSL_MEM_M7_DTCM_END     0x2007FFFFU
#define FSL_MEM_M7_DTCM_OFFSET  0x400000U

typedef enum _mem_direction
{
    kMEMORY_Local2DMA = 0,
    kMEMORY_DMA2Local,
} mem_direction_t;

/* Convert address between CM7-local and DMA (system-bus) views.
 * Only ITCM and DTCM need adjustment.
 * DDR (0x90000000) is visible to the NETC DMA at the same address as the
 * CM7 core on iMX95, so DDR addresses pass through unchanged.
 */
static inline uint32_t MEMORY_ConvertMemoryMapAddress(uint32_t addr,
                                                      mem_direction_t direction)
{
    uint32_t dest;

    switch (direction)
    {
        case kMEMORY_Local2DMA:
            /* ITCM: lower bound is 0 so only check upper bound */
            if (addr <= FSL_MEM_M7_ITCM_END)
            {
                dest = addr + FSL_MEM_M7_ITCM_OFFSET;
            }
            else if ((addr >= FSL_MEM_M7_DTCM_BEGIN) && (addr <= FSL_MEM_M7_DTCM_END))
            {
                dest = addr + FSL_MEM_M7_DTCM_OFFSET;
            }
            else
            {
                dest = addr;
            }
            break;

        case kMEMORY_DMA2Local:
            if ((addr >= (FSL_MEM_M7_ITCM_BEGIN + FSL_MEM_M7_ITCM_OFFSET)) &&
                (addr <= (FSL_MEM_M7_ITCM_END + FSL_MEM_M7_ITCM_OFFSET)))
            {
                dest = addr - FSL_MEM_M7_ITCM_OFFSET;
            }
            else if ((addr >= (FSL_MEM_M7_DTCM_BEGIN + FSL_MEM_M7_DTCM_OFFSET)) &&
                     (addr <= (FSL_MEM_M7_DTCM_END + FSL_MEM_M7_DTCM_OFFSET)))
            {
                dest = addr - FSL_MEM_M7_DTCM_OFFSET;
            }
            else
            {
                dest = addr;
            }
            break;

        default:
            dest = addr;
            break;
    }

    return dest;
}

#endif /* FSL_MEMORY_H_ */
