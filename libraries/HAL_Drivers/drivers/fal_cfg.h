/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-5      SummerGift   first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include "drv_config.h"

#ifdef BSP_USING_QSPI_FLASH
extern struct fal_flash_dev nor_flash0;
#endif

#ifdef BSP_USING_ON_CHIP_FLASH
#define FLASH_SIZE_GRANULARITY_128K (16 * 128 * 1024)
#define STM32_FLASH_START_ADRESS_128K (0x8000000)
extern const struct fal_flash_dev stm32_onchip_flash_128k;
#endif

/* flash device table */
#ifdef BSP_USING_QSPI_FLASH
    #ifdef BSP_USING_ON_CHIP_FLASH
#define FAL_FLASH_DEV_TABLE       \
    {                             \
        &nor_flash0,              \
        &stm32_onchip_flash_128k, \
    }
#else
#define FAL_FLASH_DEV_TABLE \
    {                       \
        &nor_flash0,        \
    }
#endif
#elif defined(BSP_USING_ON_CHIP_FLASH)
    #define FAL_FLASH_DEV_TABLE   \
    {                               \
        &stm32_onchip_flash_128k, \
    }
#endif


/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG
/* partition table */
#ifdef BSP_USING_QSPI_FLASH
#ifdef BSP_USING_ON_CHIP_FLASH
#define FAL_PART_TABLE                                                                            \
    {                                                                                             \
        {FAL_PART_MAGIC_WROD, "spiflash0", FAL_USING_NOR_FLASH_DEV_NAME, 0, 16 * 1024 * 1024, 0}, \
        {FAL_PART_MAGIC_WROD, "bootloader", "onchip_flash_128k", 0, 4 * 128 * 1024, 0},           \
        {FAL_PART_MAGIC_WROD, "app", "onchip_flash_128k", 4 * 128 * 1024, 12 * 128 * 1024, 0},    \
    }
#else
#define FAL_PART_TABLE \
    {                  \
        {FAL_PART_MAGIC_WROD, "spiflash0", FAL_USING_NOR_FLASH_DEV_NAME, 0, 16 * 1024 * 1024, 0}\
    }
#endif
#elif defined(BSP_USING_ON_CHIP_FLASH)
#define FAL_PART_TABLE                                                                            \
    {                                                                                             \
        {FAL_PART_MAGIC_WROD, "app", "onchip_flash_128k", 0, 8 * 128 * 1024, 0},                  \
        {FAL_PART_MAGIC_WROD, "param", "onchip_flash_128k", 8 * 128 * 1024, 128 * 1024, 0},       \
        {FAL_PART_MAGIC_WROD, "factory", "onchip_flash_128k", 9 * 128 * 1024, 7 * 128 * 1024, 0}, \
    }
#endif
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */
