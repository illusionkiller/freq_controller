/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */


#include <drv_usart_v2.h>

#define DBG_TAG "COM.TEST"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// static int _fd = -1;

static rt_device_t com; /* 串口设备句柄 */
static struct rt_completion com_sem;
static struct rt_timer mb_timer_T35;
#define MB_RECV_PACKAT_TIMEOUT_T35 5   // ms
static struct rt_semaphore rx_sem; /* 用于接收消息的信号量 */
// （连续接收两个字符之间的间隔大于字符时间的3倍，则认为接收到一帧数据）

#define MB_RECV_PACKAT_TIMEOUT_T35 5   // ms
#define MB_SLAVE_RESPONSE_TIMEOUT 1000 // ms

// （连续接收两个字符之间的间隔大于字符时间的3倍，则认为接收到一帧数据）
static void timer_timeout_ind(void *para)
{
    struct rt_completion *completion = (struct rt_completion *)para;
    rt_completion_done(completion);
}

static rt_err_t com_cb(rt_device_t dev, rt_size_t size)
{
    rt_sem_release(&rx_sem);
    // // rt_completion_done(&rx_sem);
    // rt_timer_start(&mb_timer_T35);  //启动断帧检测
    return RT_EOK;
}

static int rtt_uart_receive1(rt_device_t serial, uint8_t *buf, int bufsz, int timeout, int bytes_timeout)
{
    int len = 0;
    while (1)
    {
        rt_sem_control(&rx_sem, RT_IPC_CMD_RESET, RT_NULL);
        int rc = rt_device_read(serial, 0, buf + len, bufsz);
        if (rc > 0)
        {
            timeout = bytes_timeout;
            len += rc;
            bufsz -= rc;
            if (bufsz == 0)
            {
                break;
            }
            continue;
        }

        if (rt_sem_take(&rx_sem, rt_tick_from_millisecond(timeout)) != RT_EOK)
        {
            break;
        }
        timeout = bytes_timeout;
    }
    return len;
}
static void com_thread_entry(void *param)
{
    uint8_t ctx_read_buf[256] = {0};
    uint8_t ctx_send_buf[256] = {0};
    while (1)
    {
        int len = rtt_uart_receive1(com, ctx_read_buf, sizeof(ctx_read_buf), MB_SLAVE_RESPONSE_TIMEOUT, 10);
        if (len > 0)
        {
            LOG_I("recv data len: %d", len);
            LOG_D("recv data: %s", ctx_read_buf);
            // rt_device_write(com, 0, &ctx_read_buf, len);
        }
        else
        {
            LOG_W("recv data timeout");
        }

        // rt_completion_wait(&com_sem, RT_WAITING_FOREVER);
        // int len = rt_device_read(com, 0, &ctx_read_buf, sizeof(ctx_read_buf));
        // if(len)
        // {
        //     LOG_I("recv data len: %d", len);
        //     rt_device_write(com, 0, &ctx_read_buf, len);
        // }
    }
}

static int com1_init(void)
{
    int ret = RT_EOK;
    com = rt_device_find(COM4_PORT_NAME);
    if (com == RT_NULL)
        return -RT_ERROR;
    rt_device_close(com);
    rt_device_open(com, RT_SERIAL_RX_NON_BLOCKING | RT_SERIAL_TX_BLOCKING);
    rt_device_set_rx_indicate(com, com_cb);
    rt_completion_init(&com_sem);
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_timer_init(&mb_timer_T35, "mbtimer", timer_timeout_ind, &com_sem, MB_RECV_PACKAT_TIMEOUT_T35, RT_TIMER_FLAG_ONE_SHOT);
    rt_thread_t thread = rt_thread_create("com1", com_thread_entry, com, 2048, 17, 20);

    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = -RT_ERROR;
    }

    return ret;
}
INIT_APP_EXPORT(com1_init);

static int rs485_echo_test(void)
{
    uint8_t ctx_read_buf[] = "hellow_world123";
    rt_device_write(com, 0, &ctx_read_buf, rt_strlen(ctx_read_buf));
    return 0;
}
MSH_CMD_EXPORT_ALIAS(rs485_echo_test, rs485_echo, rs485_echo_test);