/*
 * Copyright 2026 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MIMX9594_cm7_COMMON.h - stub for NuttX NETC HAL build.
 *
 * The real COMMON.h pulls in core_cm7.h (CMSIS) and hundreds of PERI headers.
 * For NuttX we already have CMSIS/ARM definitions from the NuttX toolchain.
 * This stub satisfies the include guard used by the PERI_*.h files without
 * re-defining the types that NuttX already provides.
 */

#ifndef MIMX9594_CM7_COMMON_H_
#define MIMX9594_CM7_COMMON_H_

/*
 * The PERI_*.h files only need the base address macros and type aliases
 * defined in MIMX9594_cm7_netc.h which is already included earlier in the
 * include chain via fsl_device_registers.h -> MIMX9594_cm7_netc.h.
 * Nothing else is needed here.
 */

/* ARM Cortex-M7 barrier intrinsics used by the SDK HAL files. */
#if !defined(__DSB)
#  define __DSB()  __asm__ volatile ("dsb 0xF":::"memory")
#endif
#if !defined(__ISB)
#  define __ISB()  __asm__ volatile ("isb 0xF":::"memory")
#endif
#if !defined(__DMB)
#  define __DMB()  __asm__ volatile ("dmb 0xF":::"memory")
#endif

#endif /* MIMX9594_CM7_COMMON_H_ */
