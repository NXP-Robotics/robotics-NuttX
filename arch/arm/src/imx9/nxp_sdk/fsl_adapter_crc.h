/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Minimal software shim for fsl_adapter_crc - NuttX port.
 * Only CRC-16/CCITT-FALSE used by fsl_netc_msg.c is implemented.
 */

#ifndef FSL_ADAPTER_CRC_H
#define FSL_ADAPTER_CRC_H

#include <stdint.h>
#include <stddef.h>

/* Enumerations required by the SDK CRC adapter API */

typedef enum
{
    KHAL_CrcInputNoRef  = 0,
    KHAL_CrcInputByBit  = 1,
    KHAL_CrcInputByByte = 2,
} hal_crc_refin_t;

typedef enum
{
    KHAL_CrcOutputNoRef = 0,
    KHAL_CrcOutputRef   = 1,
} hal_crc_refout_t;

typedef enum
{
    KHAL_CrcMSByteFirst = 0,
    KHAL_CrcLSByteFirst = 1,
} hal_crc_byte_order_t;

typedef enum
{
    KHAL_CrcPolynomial_CRC_16   = 0x8005U,
    KHAL_CrcPolynomial_CRC_CCITT = 0x1021U,
    KHAL_CrcPolynomial_CRC_32   = 0x04C11DB7UL,
} hal_crc_polynomial_t;

typedef struct
{
    uint32_t crcSize;
    uint32_t crcStartByte;
    hal_crc_refin_t  crcRefIn;
    hal_crc_refout_t crcRefOut;
    hal_crc_byte_order_t crcByteOrder;
    uint32_t complementChecksum;
    uint32_t crcSeed;
    uint32_t crcPoly;
    uint32_t crcXorOut;
} hal_crc_config_t;

/*
 * HAL_CrcCompute - software CRC computation.
 * Computes a CRC over data[0..length-1] using parameters in config.
 * Only the 16-bit non-reflected variant is tested here (used by VSI/PSI
 * message verification in fsl_netc_msg.c).
 */
static inline uint32_t HAL_CrcCompute(const hal_crc_config_t *config,
                                       const uint8_t *data,
                                       size_t length)
{
    uint32_t crc  = config->crcSeed;
    uint32_t poly = config->crcPoly;
    uint32_t bits = config->crcSize * 8U;
    uint32_t mask = (bits < 32U) ? ((1U << bits) - 1U) : 0xFFFFFFFFU;
    size_t   i;
    uint8_t  j;

    if (config->crcStartByte >= length)
    {
        return (crc ^ config->crcXorOut) & mask;
    }

    for (i = config->crcStartByte; i < length; i++)
    {
        uint8_t byte = data[i];
        if (config->crcRefIn == KHAL_CrcInputNoRef)
        {
            /* Feed MSB first */
            crc ^= ((uint32_t)byte << (bits - 8U)) & mask;
            for (j = 0U; j < 8U; j++)
            {
                if (crc & (1U << (bits - 1U)))
                {
                    crc = ((crc << 1U) ^ poly) & mask;
                }
                else
                {
                    crc = (crc << 1U) & mask;
                }
            }
        }
        else
        {
            /* Feed LSB first (reflected) */
            crc ^= (uint32_t)byte;
            for (j = 0U; j < 8U; j++)
            {
                if (crc & 1U)
                {
                    crc = (crc >> 1U) ^ poly;
                }
                else
                {
                    crc >>= 1U;
                }
            }
        }
    }

    return (crc ^ config->crcXorOut) & mask;
}

#endif /* FSL_ADAPTER_CRC_H */
