/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include "modbus_tcp.h"
#include "modbus_data_trans.h"
#include "netdev.h"
#include "env.h"
#include "Ku_down.h"

#define DBG_TAG "DTU"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#if defined BSP_USING_KU_DOWN

typedef union
{
    struct
    {
        uint16_t net_link : 1; // 网络连接状态 0:未连接 1:已连接
        uint16_t pll_link : 1;
        uint16_t clk1_link : 1;
        uint16_t clk2_link : 1;
        uint16_t clk3_link : 1;
        uint16_t kusuv_link : 1; // 变频器连接状态 0:未连接 1:已连接
        uint16_t RESERVED : 4;
        uint16_t RESET_CODE : 6;
    };
    uint16_t val;
} sys_status_t;

typedef union
{
    struct
    {
        uint16_t RESET : 1;      // 1:系统复位
        uint16_t PLL_UPDATE : 1; // 1:锁相源信息更新
        uint16_t RESERVED : 14;
    };
    uint16_t val;
} sys_ctrl_t;

typedef union
{
    struct
    {
        uint16_t PLL_REF_STA : 2; // 锁相源参考输入信号状态  0:无外部参考 1:参考信号正常 2:参考信号异常
        uint16_t PLL_LOCK : 1;    // 锁相源输出锁相状态  0:未锁相 1:已锁相
        uint16_t CLK1_LOCK : 1;   // 本振1频率锁定状态 0:未锁定 1:已锁定
        uint16_t CLK2_LOCK : 1;   // 本振2频率锁定状态 0:未锁定 1:已锁定
        uint16_t CLK3_LOCK : 1;   // 本振3频率锁定状态 0:未锁定 1:已锁定
    };
    uint16_t val;
} channel_status_t;

typedef union
{
    struct
    {
        uint16_t OUTPUT_EN : 1;   // 输出频率开关 0:关闭 1:打开
        uint16_t RF_PATH_SEL : 2; // 中频输出选择  0:70MHz 1:1200MHz
        uint16_t RESERVED : 13;
    };
    uint16_t val;
} channel_ctrl_t;

enum MB_MODE
{
    MB_MODE_READ = 0,
    MB_MODE_WRITE = 1,
};

#define MAX_HOLDING_REG_NUM (64)
#define SYS_HOLDING_REG_START_ADDR_OFFSET 0x1000
static uint16_t modbus_sys_holding_regs[MAX_HOLDING_REG_NUM] = {0};
static tcp_modbus_device_t tsu = RT_NULL;
static rtu_modbus_device_t rtu = RT_NULL;
static sys_status_t sys_status = {0};
static struct _reent r = {0};
int sys_holding_regs_process(uint16_t *reg_buffer, int index, int reg_count, enum MB_MODE mode)
{
    struct netdev *netdev = netdev_get_by_name("e0");
    RT_ASSERT(netdev != RT_NULL);
    channel_attr_t channel_attr = get_channel_attr();
    for (int addr_index = index; addr_index < index + reg_count;)
    {
        switch (addr_index)
        {
        case 0x00: // ip
        {
            if (mode == MB_MODE_READ)
            {
                modbus_data_uint2regs(LITTLE_ENDIAL, (uint16_t *)&reg_buffer[addr_index], &netdev->ip_addr.addr, 1);
            }
            else
            {
                modbus_data_regs2uint(LITTLE_ENDIAL, &netdev->ip_addr.addr, (uint16_t *)&reg_buffer[addr_index], 2);
                _setenv_r(&r, "IP", inet_ntoa(netdev->ip_addr.addr), 1);
            }
            addr_index += 2;
            break;
        }
        case 0x02: // mask
        {
            if (mode == MB_MODE_READ)
            {
                modbus_data_uint2regs(LITTLE_ENDIAL, (uint16_t *)&reg_buffer[addr_index], &netdev->netmask.addr, 1);
            }
            else
            {
                modbus_data_regs2uint(LITTLE_ENDIAL, &netdev->netmask.addr, (uint16_t *)&reg_buffer[addr_index], 2);
                _setenv_r(&r, "IP_MASK", inet_ntoa(netdev->netmask.addr), 1);
            }
            addr_index += 2;
            break;
        }
        case 0x04: // gateway
        {
            if (mode == MB_MODE_READ)
            {
                modbus_data_uint2regs(LITTLE_ENDIAL, (uint16_t *)&reg_buffer[addr_index], &netdev->gw.addr, 1);
            }
            else
            {
                modbus_data_regs2uint(LITTLE_ENDIAL, &netdev->gw.addr, (uint16_t *)&reg_buffer[addr_index], 2);
                _setenv_r(&r, "GATE_WAY", inet_ntoa(netdev->gw.addr), 1);
            }
            addr_index += 2;
            break;
        }
        case 0x06: // SN
        {
            if (mode == MB_MODE_READ)
            {
                char sn[12] = {0};
                rt_strncpy(sn, getenv("SN"), sizeof(sn));
                modbus_data_bytes2regs(LITTLE_ENDIAL, (uint16_t *)&reg_buffer[addr_index], sn, sizeof(sn));
            }
            addr_index += 6; // 12个字节
            break;
        }
        case 0x0C: // 固件版本
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = DTU_FIRMWARE_VERSION;
            }
            addr_index++;
            break;
        }
        case 0x0E: // 系统控制
        {
            if (mode == MB_MODE_WRITE)
            {
                sys_ctrl_t sys_ctrl = {0};
                sys_ctrl.val = reg_buffer[addr_index];
                if (sys_ctrl.RESET)
                {
                    LOG_D("reboot now");
                    rt_thread_mdelay(100); // 安全操作
                    rt_hw_cpu_reset();
                }
                if (sys_ctrl.PLL_UPDATE)
                {
                    sys_ctrl.PLL_UPDATE = 0;
                    get_pll_info();
                }
            }
            addr_index++;
            break;
        }
        case 0x0F: // 系统状态
        {
            if (mode == MB_MODE_READ)
            {
                sys_status.pll_link = channel_attr.pll_node->link_status;
                sys_status.clk1_link = channel_attr.clk_node[0].link_status;
                sys_status.clk2_link = channel_attr.clk_node[1].link_status;
                sys_status.clk3_link = channel_attr.clk_node[2].link_status;
                sys_status.kusuv_link = channel_attr.frecov_node[0].link_status;
                reg_buffer[addr_index] = sys_status.val;
            }
            addr_index++;
            break;
        }
        case 0x10: // 通道状态
        {
            if (mode == MB_MODE_READ)
            {
                channel_status_t channel_status = {0};
                channel_status.PLL_REF_STA = channel_attr.pll_node->status.INPUT_REF_STA;
                channel_status.PLL_LOCK = channel_attr.pll_node->status.PLL_LOCK;
                channel_status.CLK1_LOCK = channel_attr.clk_node[0].status.CLK_LOCK;
                channel_status.CLK2_LOCK = channel_attr.clk_node[1].status.CLK_LOCK;
                channel_status.CLK3_LOCK = channel_attr.clk_node[2].status.CLK_LOCK;
                reg_buffer[addr_index] = channel_status.val;
            }
            addr_index++;
            break;
        }
        case 0x11: // 通道控制
        {
            channel_ctrl_t channel_ctrl = {0};
            if (mode == MB_MODE_READ)
            {
                channel_ctrl.OUTPUT_EN = channel_attr.output_en;
                channel_ctrl.RF_PATH_SEL = channel_attr.if_idx;
                reg_buffer[addr_index] = channel_ctrl.val;
            }
            else
            {
                channel_ctrl.val = reg_buffer[addr_index];
                config_channel_fre(channel_attr.input_freq_hz, channel_ctrl.RF_PATH_SEL);
                config_channel_output(channel_ctrl.OUTPUT_EN);
            }
            addr_index++;
            break;
        }
        case 0x12: //输入频率配置
        {
            if (mode == MB_MODE_READ)
            {
                uint32_t freq = channel_attr.input_freq_hz / 1000;
                modbus_data_uint2regs(BIG_ENDIAL, &reg_buffer[addr_index], &freq, 1);
            }
            else
            {
                uint32_t input_freq_hz = 0;
                modbus_data_regs2uint(BIG_ENDIAL, &input_freq_hz, (uint16_t *)&reg_buffer[addr_index], 2);
                uint64_t input_freq_hz_64 = (uint64_t)input_freq_hz * 1000;
                config_channel_fre(input_freq_hz_64, channel_attr.if_idx);
            }
            addr_index += 2;
            break;
        }
        case 0x16: // 通道衰减
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = channel_attr.decay_value;
            }
            else
            {
                channel_attr.decay_value = reg_buffer[addr_index];
                config_channel_link_decay(channel_attr.decay_value);
            }
            addr_index++;
            break;
        }
        case 0x17: // 通道PLL温度
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = channel_attr.pll_node->temperature;    
            }
            addr_index++;
            break;
        }
        case 0x18: // 本振1温度
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = channel_attr.clk_node[0].temperature;
            }
            addr_index++;
            break;
        }
        case 0x19: // 本振2温度
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = channel_attr.clk_node[1].temperature;
            }
            addr_index++;
            break;
        }
        case 0x1A: // 本振3温度
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = channel_attr.clk_node[2].temperature;
            }
            addr_index++;
            break;
        }
        case 0x1B: // 变频温度
        {
            if (mode == MB_MODE_READ)
            {
                reg_buffer[addr_index] = channel_attr.frecov_node[0].temperature;
            }
            addr_index++;
            break;
        }
        default:
            addr_index++;
            break;
        }
    }
    env_save();
    return 0;
}

int mb_fun_holding_registers(int addr, unsigned char reg_count, enum MB_MODE mode)
{
    if (reg_count > MAX_HOLDING_REG_NUM)
    {
        return -MODBUS_RT_ERROR;
    }
    int ret = MODBUS_RT_EOK;
    uint16_t *reg_buffer = NULL;
    int start_index = 0;
    int start_addr = addr;
    int end_addr = start_addr + reg_count - 1;
    if (start_addr >= SYS_HOLDING_REG_START_ADDR_OFFSET && end_addr < SYS_HOLDING_REG_START_ADDR_OFFSET + MAX_HOLDING_REG_NUM)
    {
        reg_buffer = modbus_sys_holding_regs;
        start_index = start_addr - SYS_HOLDING_REG_START_ADDR_OFFSET;
        ret = sys_holding_regs_process(reg_buffer, start_index, reg_count, mode);
    }
    else
    {
        return -MODBUS_RT_ERROR;
    }
    return ret;
}

int slave_pre_ans_callback(agile_modbus_t *ctx, int slave, int function, int addr, int quantity)
{
    (void)(ctx);
    // LOG_I("slaveid: %d, function: %d, addr: 0x%04x, quantity: %d.", slave, function, addr, quantity);
    if (ctx)
        if (slave == 1)
        {
            rt_err_t ret = RT_EOK;
            switch (function)
            {
            case AGILE_MODBUS_FC_READ_HOLDING_REGISTERS:
                ret = mb_fun_holding_registers(addr, quantity, MB_MODE_READ);
                break;
            default:
                break;
            }
        }
    return MODBUS_RT_EOK;
}

int slave_done_callback(agile_modbus_t *ctx, int slave, int function, int addr, int quantity)
{
    (void)(ctx);
    if (slave == 1)
    {
        rt_err_t ret = RT_EOK;
        switch (function)
        {
        case AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER:
            ret = mb_fun_holding_registers(addr, 1, MB_MODE_WRITE);
            break;
        case AGILE_MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            ret = mb_fun_holding_registers(addr, quantity, MB_MODE_WRITE);
            break;
        default:
            break;
        }
    }
    return MODBUS_RT_EOK;
}

tcp_modbus_device_t modbus_tcp_slave_for_udp_init(char *ip, uint16_t port)
{
    tcp_modbus_device_t tsu = modbus_tcp(MODBUS_SLAVE);
    RT_ASSERT(tsu != NULL);
    modbus_tcp_set_net(tsu, ip, port, SOCK_DGRAM);
    modbus_tcp_set_strict(tsu, 1);
    modbus_tcp_add_block(tsu, REGISTERS, SYS_HOLDING_REG_START_ADDR_OFFSET, modbus_sys_holding_regs, MAX_HOLDING_REG_NUM);
    modbus_tcp_set_pre_ans_callback(tsu, slave_pre_ans_callback);
    modbus_tcp_set_done_callback(tsu, slave_done_callback);
    modbus_tcp_open(tsu);
    LOG_I("modbus tcp slave for udp open on port %d success", port);
    return tsu;
}

rtu_modbus_device_t modbus_rtu_slave_init(char *port)
{
    rtu_modbus_device_t rs = modbus_rtu(MODBUS_SLAVE);
    RT_ASSERT(rs != NULL);
    modbus_rtu_set_addr(rs, 1);
    modbus_rtu_set_serial(rs, port, 115200, 8, 'N', 1, 0);
    modbus_rtu_add_block(rs, REGISTERS, SYS_HOLDING_REG_START_ADDR_OFFSET, modbus_sys_holding_regs, MAX_HOLDING_REG_NUM);
    modbus_rtu_set_pre_ans_callback(rs, slave_pre_ans_callback);
    modbus_rtu_set_done_callback(rs, slave_done_callback);
    modbus_rtu_open(rs);
    LOG_I("modbus rtu slave open on port %s success", port);
    return rs;
}

static void netdev_status_callback(struct netdev *netdev, enum netdev_cb_type type)
{
    if (type == NETDEV_CB_STATUS_LINK_UP)
    {
        sys_status.net_link = RT_TRUE;
        LOG_I("netdev event(NETDEV_CB_STATUS_LINK_UP) happend");
        // if (tsu == RT_NULL)
        // {
        //     tsu = modbus_tcp_slave_for_udp_init(NULL, 502);
        // }
    }
    else if (type == NETDEV_CB_STATUS_LINK_DOWN)
    {
        sys_status.net_link = RT_FALSE;
        LOG_I("netdev event(NETDEV_CB_STATUS_LINK_DOWN) happend");
    }
    else if (type == NETDEV_CB_STATUS_INTERNET_UP)
    {
        LOG_I("netdev event(NETDEV_CB_STATUS_INTERNET_UP) happend");
    }
    else if (type == NETDEV_CB_STATUS_INTERNET_DOWN)
    {
        LOG_I("netdev event(NETDEV_CB_STATUS_INTERNET_DOWN) happend");
    }
}

static rt_err_t netdev_set_if(struct netdev *netdev, char *ip_addr, char *gw_addr, char *nm_addr)
{
    ip_addr_t addr;

    sys_status.net_link = netdev_is_link_up(netdev) ? RT_TRUE : RT_FALSE;

#ifdef RT_LWIP_DHCP
    netdev_dhcp_enabled(netdev, RT_FALSE);
#endif
    /* set IP address */
    if ((ip_addr != RT_NULL) && inet_aton(ip_addr, &addr))
    {
        netdev_set_ipaddr(netdev, &addr);
    }

    /* set gateway address */
    if ((gw_addr != RT_NULL) && inet_aton(gw_addr, &addr))
    {
        netdev_set_gw(netdev, &addr);
    }

    /* set netmask address */
    if ((nm_addr != RT_NULL) && inet_aton(nm_addr, &addr))
    {
        netdev_set_netmask(netdev, &addr);
    }

#ifdef RT_LWIP_DHCP
    netdev_dhcp_enabled(netdev, RT_TRUE);
#endif
    return RT_EOK;
}

void dtu_init_thread(void *parameter)
{
    extern uint8_t get_last_reset_code(void);
    uint8_t reset_code = get_last_reset_code();
    sys_status.RESET_CODE = reset_code;
    struct netdev *netdev = netdev_get_by_name("e0");
    if (netdev)
    {
        netdev_set_status_callback(netdev, netdev_status_callback);
    }
    rt_thread_mdelay(2000); // 必须等待外设模块（网络、文件系统、模块电源）准备完成
    env_init();
    char *ip = getenv("IP");
    char *ip_mask = getenv("IP_MASK");
    char *gate_way = getenv("GATE_WAY");
    netdev_set_if(netdev, ip, gate_way, ip_mask);
    tsu = modbus_tcp_slave_for_udp_init(NULL, 502);
    RT_ASSERT(tsu != NULL);
    rtu = modbus_rtu_slave_init(COM1_PORT_NAME); // 屏幕
    RT_ASSERT(rtu != NULL);
    controler_init();
}
#endif