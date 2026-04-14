/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-22     zylx         first version
 */

#ifndef __QSPI_CONFIG_H__
#define __QSPI_CONFIG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_QSPI
#ifndef QSPI_BUS_CONFIG
#define QSPI_BUS_CONFIG                                        \
    {                                                          \
        .Instance = QUADSPI,                                   \
        .Init.FifoThreshold = 32,                               \
        .Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE, \
        .Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE,  \
        .Init.FlashID = QSPI_FLASH_ID_1,                       \
        .Init.DualFlash = QSPI_DUALFLASH_DISABLE,              \
    }
#endif /* QSPI_BUS_CONFIG */
#endif /* BSP_USING_QSPI */

#ifdef BSP_QSPI_USING_DMA
#ifndef QSPI_DMA_CONFIG
#define QSPI_DMA_CONFIG                                     \
    {                                                       \
        .Instance = QSPI_DMA_INSTANCE,                      \
        .Init.Request = MDMA_REQUEST_QUADSPI_FIFO_TH,       \
        .Init.TransferTriggerMode = MDMA_BUFFER_TRANSFER,   \
        .Init.Priority = MDMA_PRIORITY_LOW,                 \
        .Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE, \
        .Init.SourceInc = MDMA_SRC_INC_BYTE,                \
        .Init.DestinationInc = MDMA_DEST_INC_BYTE,          \
        .Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE,      \
        .Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE,       \
        .Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE,    \
        .Init.BufferTransferLength = 128,                     \
        .Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE,       \
        .Init.DestBurst = MDMA_DEST_BURST_SINGLE,           \
        .Init.SourceBlockAddressOffset = 0,                 \
        .Init.DestBlockAddressOffset = 0,                   \
    }
#endif /* QSPI_DMA_CONFIG */
#endif /* BSP_QSPI_USING_DMA */

#define QSPI_IRQn                   QUADSPI_IRQn
#define QSPI_IRQHandler             QUADSPI_IRQHandler

#ifdef __cplusplus
}
#endif

#endif /* __QSPI_CONFIG_H__ */
