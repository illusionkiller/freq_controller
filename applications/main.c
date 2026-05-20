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
#include "msh.h"
#include "env.h"
#include "dfs_file.h" /* 当需要使用文件操作时，需要包含这个头文件 */

#define DBG_TAG "MAIN"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
static uint8_t last_reset_code = 0;
enum
{
    SYSTEM_RESET_CODE_POR_RESET = 1 << 0,
    SYSTEM_RESET_CODE_BOR_RESET = 1 << 1,
    SYSTEM_RESET_CODE_PIN_RESET = 1 << 2,
    SYSTEM_RESET_CODE_SOFT_RESET = 1 << 3,
    SYSTEM_RESET_CODE_WDT_RESET = 1 << 4,
    SYSTEM_RESET_CODE_LOWPWR_RESET = 1 << 5,
};
static int check_reset_reason(void)
{
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET)
    {
        LOG_D("por reset happend");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_POR_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != RESET)
    {
        LOG_D("bor reset happend");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_BOR_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET)
    {
        LOG_D("pin reset happend");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_PIN_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET)
    {
        LOG_D("soft reset happend");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_SOFT_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != RESET)
    {
        LOG_D("wdt reset happend");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_WDT_RESET);
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST) != RESET)
    {
        LOG_D("low power reset happend");
        SET_BIT(last_reset_code, SYSTEM_RESET_CODE_LOWPWR_RESET);
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return RT_EOK;
}
INIT_DEVICE_EXPORT(check_reset_reason);

uint8_t get_last_reset_code(void)
{
    return last_reset_code;
}
#ifdef RT_USING_WDT
#define WDT_DEVICE_NAME "wdt"
static rt_device_t wdg_dev = RT_NULL;

static void idle_hook(void)
{
    if (wdg_dev)
        rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
}

static int wdt_init(void)
{
    rt_uint32_t timeout = 10;
    wdg_dev = rt_device_find(WDT_DEVICE_NAME);
    if (!wdg_dev)
    {
        LOG_RAW("find %s failed!\r\n", WDT_DEVICE_NAME);
        return RT_ERROR;
    }
    rt_device_init(wdg_dev);
    if (rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout) != RT_EOK)
    {
        return RT_ERROR;
    }

    if (rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_START, RT_NULL) != RT_EOK)
    {
        return -RT_ERROR;
    }

    rt_thread_idle_sethook(idle_hook);
    return RT_EOK;
}
INIT_DEVICE_EXPORT(wdt_init);
#endif
/*
 ******************************************************************************************************
 *    函 数 名: JumpToBootloader
 *    功能说明: 跳转到系统BootLoader   //ref  stm32AN2606
 *    形    参: 无
 *    返 回 值: 无
 ******************************************************************************************************
 */
// static void JumpToBootloader(void)
// {
//     void (*SysMemBootJump)(void);
// #ifdef RT_USING_WDT
//     if (wdg_dev)
//     {
//         rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
//         rt_uint32_t timeout = 32;
//         rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_SET_TIMEOUT, &timeout);
//     }
// #endif
//     HAL_DeInit();
//     HAL_RCC_DeInit();
//     __disable_irq();
//     SysTick->CTRL = 0;
//     SysTick->LOAD = 0;
//     SysTick->VAL = 0;
//     HAL_RCC_DeInit();
//     NVIC->ICER[0] = 0xFFFFFFFF;
//     NVIC->ICPR[0] = 0xFFFFFFFF;
//     __IO uint32_t BootAddr = 0x1FFF0000; /* STM32系统BootLoader地址 */
//     SysMemBootJump = (void (*)(void))(*((uint32_t *)(BootAddr + 4)));
//     __set_MSP(*(uint32_t *)BootAddr);
//     __set_CONTROL(0);
//     SysMemBootJump();
//     while (1)
//         ;
// }
// MSH_CMD_EXPORT_ALIAS(JumpToBootloader, reboot_loader, jump to system memory);

/* defined the LED0 pin: PB1 */
#define LED0_PIN GET_PIN(E, 9)

int main(void)
{
    /* set LED0 pin mode to output */
    LOG_D("firmware version release_v%s", STR2(DTU_FIRMWARE_VERSION));
    // if (wdg_dev)
    //     rt_device_control(wdg_dev, RT_DEVICE_CTRL_WDT_KEEPALIVE, NULL);
    extern void dtu_init_thread(void *parameter);
    dtu_init_thread(RT_NULL);
    msh_exec("tftp -s", rt_strlen("tftp -s")); // 启动tftp服务器
    rt_thread_mdelay(100);
    msh_exec("telnet_server", rt_strlen("telnet_server")); // 启动telnet服务器
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);
    while (1)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
}
