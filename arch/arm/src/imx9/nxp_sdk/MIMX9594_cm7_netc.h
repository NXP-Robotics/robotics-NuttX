/*
 * Copyright 1997-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2024, 2026 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * MIMX9594_cm7_netc.h - Minimal iMX95 M7 device header for NETC HAL compilation.
 *
 * Contains only the peripheral register struct types and base address definitions
 * required by fsl_netc*, fsl_netc_soc, and fsl_netc_hw* sources.
 * No full CMSIS device header is included; only NETC/ENETC peripherals are defined.
 */

#ifndef MIMX9594_CM7_NETC_H_
#define MIMX9594_CM7_NETC_H_

#include <stdint.h>

/* CMSIS-style register access qualifiers */
#ifndef __IO
#define __IO  volatile
#endif
#ifndef __I
#define __I   volatile const
#endif
#ifndef __O
#define __O   volatile
#endif

/* MAKE_STATUS helper used by fsl_common.h MAKE_STATUS macro */
#ifndef MAKE_STATUS
#define MAKE_STATUS(group, code)  (((group) * 100) + (code))
#endif

/* Status group number for NETC */
#define kStatusGroup_NETC  166

/* FSL feature flags for iMX95 NETC (3 SI per endpoint, switch not present on M7) */
#define FSL_FEATURE_NETC_SWITCH_MAX_PORT_NUMBER  5
/* iMX95 CM7: DDR is at 0x90000000 locally but NETC DMA (A55 bus) sees it
 * at 0x80000000.  The SDK uses this flag to enable address conversion in
 * EP_SendFrame / EP_ReceiveFrame and BD ring base address programming.
 * Local2DMA: subtract 0x10000000; DMA2Local: add 0x10000000.
 */
#define FSL_FEATURE_MEMORY_HAS_ADDRESS_OFFSET    1

/* Include the NETC/ENETC peripheral register layout headers (copied from SDK) */
#include "PERI_ENETC_PCI_TYPE0.h"
#include "PERI_ENETC_VF_PCI_TYPE0.h"
#include "PERI_ENETC_SI.h"
#include "PERI_ENETC.h"
#include "PERI_ENETC_GLOBAL.h"
#include "PERI_ENETC_PF_TMR.h"
#include "PERI_NETC_ENETC.h"
#include "PERI_NETC_PORT.h"
#include "PERI_NETC_ETH_LINK.h"
#include "PERI_NETC_SW_ENETC.h"
#include "PERI_NETC_IERB.h"
#include "PERI_NETC_PRIV.h"
#include "PERI_NETC_TCU.h"
#include "PERI_BLK_CTRL_NETCMIX.h"

/* ========== ENETC (ENETC_Type, ENETC0 global) ========== */
#define ENETC0_GLOBAL_BASE    (0x4CC20000u)
#define ENETC0_GLOBAL         ((ENETC_Type *)ENETC0_GLOBAL_BASE)
#define ENETC_BASE_ADDRS      { ENETC0_GLOBAL_BASE }
#define ENETC_BASE_PTRS       { ENETC0_GLOBAL }

/* ========== ENETC_GLOBAL ========== */
#define EMDIO_GLOBAL_BASE     (0x4CCF0000u)
#define EMDIO_GLOBAL          ((ENETC_GLOBAL_Type *)EMDIO_GLOBAL_BASE)
#define ENETC1_GLOBAL_BASE    (0x4CC60000u)
#define ENETC1_GLOBAL         ((ENETC_GLOBAL_Type *)ENETC1_GLOBAL_BASE)
#define ENETC2_GLOBAL_BASE    (0x4CCA0000u)
#define ENETC2_GLOBAL         ((ENETC_GLOBAL_Type *)ENETC2_GLOBAL_BASE)
#define TMR0_GLOBAL_BASE      (0x4CCD0000u)
#define TMR0_GLOBAL           ((ENETC_GLOBAL_Type *)TMR0_GLOBAL_BASE)
#define ENETC_GLOBAL_BASE_ADDRS  { EMDIO_GLOBAL_BASE, ENETC1_GLOBAL_BASE, ENETC2_GLOBAL_BASE, TMR0_GLOBAL_BASE }
#define ENETC_GLOBAL_BASE_PTRS   { EMDIO_GLOBAL, ENETC1_GLOBAL, ENETC2_GLOBAL, TMR0_GLOBAL }

/* ========== ENETC_PCI_TYPE0 ========== */
#define EMDIO0_PCI_HDR_TYPE0_BASE   (0x4CB00000u)
#define EMDIO0_PCI_HDR_TYPE0        ((ENETC_PCI_TYPE0_Type *)EMDIO0_PCI_HDR_TYPE0_BASE)
#define ENETC0_PCI_HDR_TYPE0_BASE   (0x4CA00000u)
#define ENETC0_PCI_HDR_TYPE0        ((ENETC_PCI_TYPE0_Type *)ENETC0_PCI_HDR_TYPE0_BASE)
#define ENETC1_PCI_HDR_TYPE0_BASE   (0x4CA40000u)
#define ENETC1_PCI_HDR_TYPE0        ((ENETC_PCI_TYPE0_Type *)ENETC1_PCI_HDR_TYPE0_BASE)
#define ENETC2_PCI_HDR_TYPE0_BASE   (0x4CA80000u)
#define ENETC2_PCI_HDR_TYPE0        ((ENETC_PCI_TYPE0_Type *)ENETC2_PCI_HDR_TYPE0_BASE)
#define TMR0_PCI_HDR_TYPE0_BASE     (0x4CAC0000u)
#define TMR0_PCI_HDR_TYPE0          ((ENETC_PCI_TYPE0_Type *)TMR0_PCI_HDR_TYPE0_BASE)
#define ENETC_PCI_TYPE0_BASE_ADDRS  { EMDIO0_PCI_HDR_TYPE0_BASE, ENETC0_PCI_HDR_TYPE0_BASE, ENETC1_PCI_HDR_TYPE0_BASE, ENETC2_PCI_HDR_TYPE0_BASE, TMR0_PCI_HDR_TYPE0_BASE }
#define ENETC_PCI_TYPE0_BASE_PTRS   { EMDIO0_PCI_HDR_TYPE0, ENETC0_PCI_HDR_TYPE0, ENETC1_PCI_HDR_TYPE0, ENETC2_PCI_HDR_TYPE0, TMR0_PCI_HDR_TYPE0 }

/* ========== ENETC_PF_EMDIO ========== */
#define EMDIO_BASE_BASE           (0x4CCE0000u)
#define EMDIO_BASE                ((ENETC_PF_EMDIO_Type *)EMDIO_BASE_BASE)
#define ENETC_PF_EMDIO_BASE_ADDRS { EMDIO_BASE_BASE }
#define ENETC_PF_EMDIO_BASE_PTRS  { EMDIO_BASE }

/* ========== ENETC_PF_TMR ========== */
#define TMR0_BASE_BASE            (0x4CCC0000u)
#define TMR0_BASE                 ((ENETC_PF_TMR_Type *)TMR0_BASE_BASE)
#define ENETC_PF_TMR_BASE_ADDRS   { TMR0_BASE_BASE }
#define ENETC_PF_TMR_BASE_PTRS    { TMR0_BASE }

/* ========== ENETC_SI (PSI + VSI) ========== */
#define ENETC0_PSI_BASE     (0x4CC00000u)
#define ENETC0_PSI          ((ENETC_SI_Type *)ENETC0_PSI_BASE)
#define ENETC1_PSI_BASE     (0x4CC40000u)
#define ENETC1_PSI          ((ENETC_SI_Type *)ENETC1_PSI_BASE)
#define ENETC2_PSI_BASE     (0x4CC80000u)
#define ENETC2_PSI          ((ENETC_SI_Type *)ENETC2_PSI_BASE)
#define ENETC_VSI0_BASE     (0x4CD20000u)
#define ENETC_VSI0          ((ENETC_SI_Type *)ENETC_VSI0_BASE)
#define ENETC_VSI1_BASE     (0x4CD30000u)
#define ENETC_VSI1          ((ENETC_SI_Type *)ENETC_VSI1_BASE)
#define ENETC_VSI2_BASE     (0x4CD40000u)
#define ENETC_VSI2          ((ENETC_SI_Type *)ENETC_VSI2_BASE)
#define ENETC_VSI3_BASE     (0x4CD50000u)
#define ENETC_VSI3          ((ENETC_SI_Type *)ENETC_VSI3_BASE)
#define ENETC_VSI4_BASE     (0x4CD60000u)
#define ENETC_VSI4          ((ENETC_SI_Type *)ENETC_VSI4_BASE)
#define ENETC_VSI5_BASE     (0x4CD70000u)
#define ENETC_VSI5          ((ENETC_SI_Type *)ENETC_VSI5_BASE)
#define ENETC_SI_BASE_ADDRS { ENETC0_PSI_BASE, ENETC1_PSI_BASE, ENETC2_PSI_BASE, ENETC_VSI0_BASE, ENETC_VSI1_BASE, ENETC_VSI2_BASE, ENETC_VSI3_BASE, ENETC_VSI4_BASE, ENETC_VSI5_BASE }
#define ENETC_SI_BASE_PTRS  { ENETC0_PSI, ENETC1_PSI, ENETC2_PSI, ENETC_VSI0, ENETC_VSI1, ENETC_VSI2, ENETC_VSI3, ENETC_VSI4, ENETC_VSI5 }

/* ========== ENETC_VF_PCI_TYPE0 ========== */
#define NETC_VF1_PCI_HDR_TYPE0_BASE  (0x4CA10000u)
#define NETC_VF1_PCI_HDR_TYPE0       ((ENETC_VF_PCI_TYPE0_Type *)NETC_VF1_PCI_HDR_TYPE0_BASE)
#define NETC_VF2_PCI_HDR_TYPE0_BASE  (0x4CA20000u)
#define NETC_VF2_PCI_HDR_TYPE0       ((ENETC_VF_PCI_TYPE0_Type *)NETC_VF2_PCI_HDR_TYPE0_BASE)
#define NETC_VF3_PCI_HDR_TYPE0_BASE  (0x4CA50000u)
#define NETC_VF3_PCI_HDR_TYPE0       ((ENETC_VF_PCI_TYPE0_Type *)NETC_VF3_PCI_HDR_TYPE0_BASE)
#define NETC_VF4_PCI_HDR_TYPE0_BASE  (0x4CA60000u)
#define NETC_VF4_PCI_HDR_TYPE0       ((ENETC_VF_PCI_TYPE0_Type *)NETC_VF4_PCI_HDR_TYPE0_BASE)
#define NETC_VF5_PCI_HDR_TYPE0_BASE  (0x4CA90000u)
#define NETC_VF5_PCI_HDR_TYPE0       ((ENETC_VF_PCI_TYPE0_Type *)NETC_VF5_PCI_HDR_TYPE0_BASE)
#define NETC_VF6_PCI_HDR_TYPE0_BASE  (0x4CAA0000u)
#define NETC_VF6_PCI_HDR_TYPE0       ((ENETC_VF_PCI_TYPE0_Type *)NETC_VF6_PCI_HDR_TYPE0_BASE)
#define ENETC_VF_PCI_TYPE0_BASE_ADDRS { NETC_VF1_PCI_HDR_TYPE0_BASE, NETC_VF2_PCI_HDR_TYPE0_BASE, NETC_VF3_PCI_HDR_TYPE0_BASE, NETC_VF4_PCI_HDR_TYPE0_BASE, NETC_VF5_PCI_HDR_TYPE0_BASE, NETC_VF6_PCI_HDR_TYPE0_BASE }
#define ENETC_VF_PCI_TYPE0_BASE_PTRS  { NETC_VF1_PCI_HDR_TYPE0, NETC_VF2_PCI_HDR_TYPE0, NETC_VF3_PCI_HDR_TYPE0, NETC_VF4_PCI_HDR_TYPE0, NETC_VF5_PCI_HDR_TYPE0, NETC_VF6_PCI_HDR_TYPE0 }

/* ========== NETC_ENETC ========== */
#define ENETC0_BASE_BASE      (0x4CC10000u)
#define ENETC0_BASE           ((NETC_ENETC_Type *)ENETC0_BASE_BASE)
#define ENETC1_BASE_BASE      (0x4CC50000u)
#define ENETC1_BASE           ((NETC_ENETC_Type *)ENETC1_BASE_BASE)
#define ENETC2_BASE_BASE      (0x4CC90000u)
#define ENETC2_BASE           ((NETC_ENETC_Type *)ENETC2_BASE_BASE)
#define NETC_ENETC_BASE_ADDRS { ENETC0_BASE_BASE, ENETC1_BASE_BASE, ENETC2_BASE_BASE }
#define NETC_ENETC_BASE_PTRS  { ENETC0_BASE, ENETC1_BASE, ENETC2_BASE }

/* ========== NETC_ETH_LINK ========== */
#define ENETC0_ETH_MAC_PORT_BASE   (0x4CC15000u)
#define ENETC0_ETH_MAC_PORT        ((NETC_ETH_LINK_Type *)ENETC0_ETH_MAC_PORT_BASE)
#define ENETC1_ETH_MAC_PORT_BASE   (0x4CC55000u)
#define ENETC1_ETH_MAC_PORT        ((NETC_ETH_LINK_Type *)ENETC1_ETH_MAC_PORT_BASE)
#define ENETC2_ETH_MAC_PORT_BASE   (0x4CC95000u)
#define ENETC2_ETH_MAC_PORT        ((NETC_ETH_LINK_Type *)ENETC2_ETH_MAC_PORT_BASE)
#define NETC_ETH_LINK_BASE_ADDRS   { ENETC0_ETH_MAC_PORT_BASE, ENETC1_ETH_MAC_PORT_BASE, ENETC2_ETH_MAC_PORT_BASE }
#define NETC_ETH_LINK_BASE_PTRS    { ENETC0_ETH_MAC_PORT, ENETC1_ETH_MAC_PORT, ENETC2_ETH_MAC_PORT }

/* ========== NETC_PORT ========== */
#define ENETC0_PORT_BASE       (0x4CC14000u)
#define ENETC0_PORT            ((NETC_PORT_Type *)ENETC0_PORT_BASE)
#define ENETC1_PORT_BASE       (0x4CC54000u)
#define ENETC1_PORT            ((NETC_PORT_Type *)ENETC1_PORT_BASE)
#define ENETC2_PORT_BASE       (0x4CC94000u)
#define ENETC2_PORT            ((NETC_PORT_Type *)ENETC2_PORT_BASE)
#define NETC_PORT_BASE_ADDRS   { ENETC0_PORT_BASE, ENETC1_PORT_BASE, ENETC2_PORT_BASE }
#define NETC_PORT_BASE_PTRS    { ENETC0_PORT, ENETC1_PORT, ENETC2_PORT }

/* ========== NETC_SW_ENETC ========== */
#define ENETC0_COMMON_BASE        (0x4CC11000u)
#define ENETC0_COMMON             ((NETC_SW_ENETC_Type *)ENETC0_COMMON_BASE)
#define ENETC1_COMMON_BASE        (0x4CC51000u)
#define ENETC1_COMMON             ((NETC_SW_ENETC_Type *)ENETC1_COMMON_BASE)
#define ENETC2_COMMON_BASE        (0x4CC91000u)
#define ENETC2_COMMON             ((NETC_SW_ENETC_Type *)ENETC2_COMMON_BASE)
#define NETC_SW_ENETC_BASE_ADDRS  { ENETC0_COMMON_BASE, ENETC1_COMMON_BASE, ENETC2_COMMON_BASE }
#define NETC_SW_ENETC_BASE_PTRS   { ENETC0_COMMON, ENETC1_COMMON, ENETC2_COMMON }

/* Pseudo-link and switch types - not present on iMX95 standalone ENETC */
typedef NETC_ETH_LINK_Type NETC_PSEUDO_LINK_Type;

/* PPMCR (Pseudo Port MAC Control Register) does not exist on iMX95 rev A0.
 * The fsl_netc_endpoint.c SDK source uses it only when enPseudoMacTxPad is
 * true.  Map it to an unused/harmless field to allow compilation.
 * On iMX95 this path will never be taken at runtime because no VSI
 * configuration sets enPseudoMacTxPad.
 */
#define PPMCR PPSCR
#ifndef NETC_PSEUDO_LINK_PPMCR_TXPAD
#  define NETC_PSEUDO_LINK_PPMCR_TXPAD(x) (0u)
#endif
typedef NETC_SW_ENETC_Type NETC_SW_Type;

#define NETC_PSEUDO_LINK_BASE_PTRS  { NULL }

/* Feature flags used by fsl_netc headers */
#define FSL_FEATURE_NETC_SI_RING_NUM_MAX          (24)
#define FSL_FEATURE_NETC_ENETC_SI_MAX_NUM         (9)

/* Missing status code */
#define kStatus_OutOfRange  (MAKE_STATUS(0, 10))

#endif /* MIMX9594_CM7_NETC_H_ */
