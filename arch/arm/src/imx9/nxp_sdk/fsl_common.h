/*
 * Copyright 2026 NXP
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Minimal fsl_common.h shim for NuttX NETC VSI driver.
 * Provides the subset of the NXP SDK common layer needed by fsl_netc*.
 * Does not depend on fsl_device_registers.h or fsl_clock.h.
 */

#ifndef FSL_COMMON_H_
#define FSL_COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <assert.h>

/* Pull in device-specific peripheral register type definitions */
#include "fsl_device_registers.h"

/* ARM Cortex-M7 barrier intrinsics - define before any SDK HAL headers
 * that may use them without including a CMSIS core header.
 */
#if !defined(__DSB)
#  define __DSB()  __asm__ volatile ("dsb 0xF":::"memory")
#endif
#if !defined(__ISB)
#  define __ISB()  __asm__ volatile ("isb 0xF":::"memory")
#endif
#if !defined(__DMB)
#  define __DMB()  __asm__ volatile ("dmb 0xF":::"memory")
#endif

/* Version macro */
#define MAKE_VERSION(major, minor, bugfix) \
    (((major) << 16) | ((minor) << 8) | (bugfix))

/* Status code builder macro */
#ifndef MAKE_STATUS
#define MAKE_STATUS(group, code)  (((group) * 100) + (code))
#endif

/* Status type used throughout the SDK */
typedef int32_t status_t;

/* Common status codes */
#define kStatus_Success          (0)
#define kStatus_Fail             (1)
#define kStatus_InvalidArgument  (4)
#define kStatus_Timeout          (6)
#define kStatus_NoTransferInProgress (7)
#define kStatus_Busy             (3)
#define kStatus_NoData           (5)
#ifndef kStatus_OutOfRange
#  define kStatus_OutOfRange     (8)
#endif

/* Compiler/attribute helpers */
#if defined(__GNUC__)
#  define AT_NONCACHEABLE_SECTION(var)       var __attribute__((section(".noncacheable")))
#  define AT_NONCACHEABLE_SECTION_ALIGN(var, align) \
       var __attribute__((section(".noncacheable"), aligned(align)))
#  define AT_NONCACHEABLE_SECTION_INIT(var)  var __attribute__((section(".noncacheable.init")))
#  define RAMFUNCTION_SECTION_CODE(func)     func __attribute__((section(".ramfunc")))
#  define SDK_ALIGN(var, align)              var __attribute__((aligned(align)))
#  define SDK_PACKED_BEGIN
#  define SDK_PACKED_END                     __attribute__((packed))
#else
#  define AT_NONCACHEABLE_SECTION(var)       var
#  define AT_NONCACHEABLE_SECTION_ALIGN(var, align) var
#  define AT_NONCACHEABLE_SECTION_INIT(var)  var
#  define SDK_ALIGN(var, align)              var
#  define SDK_PACKED_BEGIN
#  define SDK_PACKED_END
#endif

/* Suppress unused-variable warnings for SDK HAL assertions */
#ifndef NDEBUG
#  define SDK_DEBUGCONSOLE 0
#endif

/* Endian helpers */
#define SWAP2(x) __builtin_bswap16(x)
#define SWAP4(x) __builtin_bswap32(x)

/* Bit manipulation */
#define BIT(n)  (1u << (n))

/* Cache ops - no-op on platforms that do not need them (NuttX handles cache) */
#ifndef FSL_ETH_ENABLE_CACHE_CONTROL
#  define NETC_DCACHE_INVALIDATE(addr, len)  do {} while (0)
#  define NETC_DCACHE_CLEAN(addr, len)       do {} while (0)
#endif

/* Declare the iMX95 M7 CPU so fsl_netc_soc.c compiles */
#ifndef CPU_MIMX9522DVKXM
#  define CPU_MIMX9522DVKXM 1
#endif

/* Memory map address conversion is provided by fsl_memory.h (included via
 * fsl_netc.h when FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET=1).
 * Do not define MEMORY_ConvertMemoryMapAddress here to avoid redefinition.
 */

#endif /* FSL_COMMON_H_ */
