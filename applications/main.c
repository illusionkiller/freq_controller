/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include "drv_gpio.h"
#define DBG_TAG "MAIN"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
static int last_reset_code = 0;
enum
{
    SYSTEM_RESET_CODE_POR_RESET = 0,
    SYSTEM_RESET_CODE_BOR_RESET,
    SYSTEM_RESET_CODE_PIN_RESET,
    SYSTEM_RESET_CODE_SOFT_RESET,
    SYSTEM_RESET_CODE_WDT_RESET,
    SYSTEM_RESET_CODE_LOWPWR_RESET,
};
static int check_reset_reason(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
        LOG_RAW("por reset happend\r\n");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_POR_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
    {
        LOG_RAW("bor reset happend\r\n");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_BOR_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
        LOG_RAW("pin reset happend\r\n");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_PIN_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
    {
        LOG_RAW("soft reset happend\r\n");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_SOFT_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET)
    {
        LOG_RAW("wdt reset happend\r\n");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_WDT_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST) != RESET)
    {
        LOG_RAW("low power reset happend\r\n");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_LOWPWR_RESET);
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return RT_EOK;
}
INIT_DEVICE_EXPORT(check_reset_reason);
/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(E, 9)

int main(void)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    
    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
