/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <board.h>
#include <rtthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <rtdevice.h>
#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include "netdev.h"
#include "pcf8575.h"
#include "flashdb.h"

#define DBG_TAG "DTU"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>
#define DTU_FIRMWARE_VERSION  1
#define DTU_DEVICE_DEFAULT_SN "DTHT-001"   //最长12位

enum
{
    DEVICE_FRE_CONTROL_RANGE_X = 0,
    DEVICE_FRE_CONTROL_RANGE_KA,
    DEVICE_FRE_CONTROL_RANGE_Q
};
typedef union
{
    struct
    {
        uint16_t PLL_REF : 2; //锁相源参考输入信号状态  0:无外部参考 1:参考信号正常 2:参考信号异常
        uint16_t PLL_LOCK : 1; //锁相源输出锁相状态  0:未锁相 1:已锁相
        uint16_t CH1_CLK1_LOCK: 1; //通道1本振1频率锁定状态 0:未锁定 1:已锁定
        uint16_t CH1_CLK2_LOCK: 1; //通道1本振2频率锁定状态 0:未锁定 1:已锁定
        uint16_t CH2_CLK1_LOCK: 1; //通道2本振1频率锁定状态 0:未锁定 1:已锁定
        uint16_t CH2_CLK2_LOCK: 1; //通道2本振2频率锁定状态 0:未锁定 1:已锁定
    };
    uint16_t val;
} reg1010_t;

typedef union
{
    struct
    {
        uint16_t PLL_INPUT_SEL : 2; // 锁相源参考输入信号选择0:使用内部参考 1:使用外部参考10M 2:使用外部参考100M
        uint16_t CH1_OUTPUT_EN : 1; // 通道1输出频率开关 0:关闭 1:打开
        uint16_t CH1_MIDF_SW : 3; // 
        uint16_t CH2_OUTPUT_EN : 1; // 通道2输出频率开关 0:关闭 1:打开
        uint16_t CH2_MIDF_SW : 3; //
    };
    uint16_t val;
} reg1011_t;

typedef union
{
    struct
    {
        uint16_t net_link : 1; // 网络连接状态 0:未连接 1:已连接
        uint16_t com1_link: 1; // 485通道1通信状态 0:未连接 1:已连接
        uint16_t com2_link: 1; // 485通道2通信状态 0:未连接 1:已连接
        uint16_t com3_link: 1; // 485通道3通信状态 0:未连接 1:已连接
        uint16_t com4_link: 1; // 485通道4通信状态 0:未连接 1:已连接
    };
    uint16_t val;
} reg_sysstatus_t;

typedef union
{
    struct
    {
        uint16_t CH1_GAIN : 8; //通道1增益
        uint16_t CH2_GAIN : 8; //通道2增益
    };
    uint16_t val;
} reg1016_t;


typedef struct
{
    uint32_t IP;          // 0x1000  /rw
    uint32_t IP_MASK;     // 0x1002
    uint32_t GATE_WAY;    // 0x1004
    uint8_t SN[12];          // 设备SN,本地IP  //0x1006
    uint16_t version;    //固件版本
    uint16_t LW1010;     
    uint16_t LW1011;        //中频选择
    uint32_t ch1_input_fre; // 通道1输入频率
    uint32_t ch2_input_fre; // 通道2输入频率
    uint16_t LW1016;        //链路增益
} device_reg_t;

#define  DEVICE_FRE_CONTROL_RANGE_X
#define REG_NUM (128)
#define LWREG_START_ADDR 0X1000
static uint16_t modbus_regs[REG_NUM] = {0};

static device_reg_t device_reg = {
    .IP = 0x1e01a8c0,    //192.168.1.30
    .IP_MASK = 0x00ffffff,
    .GATE_WAY = 0x0101a8c0,
    .SN = DTU_DEVICE_DEFAULT_SN,
    .version = DTU_FIRMWARE_VERSION,
    .LW1010 = 0,
    .LW1011 = 0,
    .ch1_input_fre = 0,
    .ch2_input_fre = 0,
    .LW1016 = 0,
};

static rtu_modbus_device_t rtu1 = RT_NULL;
static rtu_modbus_device_t rtu2 = RT_NULL;
static rtu_modbus_device_t rtu3 = RT_NULL;
static rtu_modbus_device_t rtu4 = RT_NULL;
static tcp_modbus_device_t tsu = RT_NULL;

static struct fdb_kvdb kvdb = {0};
static struct rt_mutex lock;
static struct fdb_default_kv_node default_kv_table[] = {
    {"DEVICE_REG", &device_reg, sizeof(device_reg_t)},
};
static void kvdb_lock(fdb_db_t db)
{
    rt_mutex_take(&lock, RT_WAITING_FOREVER);
}

static void kvdb_unlock(fdb_db_t db)
{
    rt_mutex_release(&lock);
}


static rt_err_t set_ch_input_fre(uint8_t ch_id,uint32_t fre)
{
    rt_err_t ret = RT_EOK;
#if defined DEVICE_FRE_CONTROL_RANGE_X
    uint32_t fre_temp = fre + 3100000; // Khz
    if (ch_id == 1)
    {
        LOG_D("set ch1 clk1 frequency %u", fre_temp);
    }
    else if (ch_id == 2)
    {
        LOG_D("set ch2 clk1 frequency %u", fre_temp);
    }
#elif defined DEVICE_FRE_CONTROL_RANGE_KA
    uint32_t fre_temp = (fre + 6000000) / 4; 
    if (ch_id == 1)
    {
        LOG_D("set ch1 clk1 frequency %u", fre_temp);
    }
    else if (ch_id == 2)
    {
        LOG_D("set ch2 clk1 frequency %u", fre_temp);
    }
#elif defined DEVICE_FRE_CONTROL_RANGE_Q
    uint32_t fre_temp = (fre + 6000000); // Khz
    if (ch_id == 1)
    {
        LOG_D("set ch1 clk1 frequency %u", fre_temp);
    }
#endif
    return ret;
}

// gain = -0.5*gain_value +50
static rt_err_t set_ch_link_gain(uint8_t ch_id, uint8_t gain_value)
{
    rt_err_t ret = RT_EOK;
#if defined DEVICE_FRE_CONTROL_RANGE_X
    if (ch_id == 1)
    {
        LOG_D("set ch1 link gain %d", gain_value);
    }
    else if (ch_id == 2)
    {
        LOG_D("set ch2 link gain %d", gain_value);
    }
#elif defined DEVICE_FRE_CONTROL_RANGE_KA
    struct pcf8575c_io_map io_map = {0};
    if (gain_value <= 60) // 衰减没有超过30dB,只使用前级衰减
    {
        io_map.mask = 0x003F; // bit0-bit5    gs5801_1
        io_map.value = gain_value;
    }
    else
    {
        io_map.mask = 0x0FFF; // bit0-bit11   gs5801_1  gs5801_2
        io_map.value = ((gain_value - 60) << 6);
    }
    if (ch_id == 1)
    {
        LOG_D("set ch1 link gain %d", gain_value);
        // rt_device_control(gain_dev[0], PCF8575C_CMD_SET_VALUE, &io_map);
    }
    else if (ch_id == 2)
    {
        LOG_D("set ch2 link gain %d", gain_value);
        // rt_device_control(gain_dev[1], PCF8575C_CMD_SET_VALUE, &io_map);
    }
#elif defined DEVICE_FRE_CONTROL_RANGE_Q
    if (gain_value <= 60)
    {
        io_map.mask = 0x003F; // bit0-bit5    gs5801_1
        io_map.value = gain_value;
    }
    else
    {
        io_map.mask = 0x0FFF; // bit0-bit11   gs5801_1  gs5801_2
        io_map.value = ((gain_value - 60) << 6);
    }
    LOG_D("set ch link gain %d", gain_value);
    // rt_device_control(gain_dev[0], PCF8575C_CMD_SET_VALUE, &io_map);
#endif
    return ret;
}

static rt_err_t switch_medium_frequency(uint8_t ch_id, uint8_t switch_id)
{
    rt_err_t ret = RT_EOK;
    uint32_t fre_temp = 0;
#if defined DEVICE_FRE_CONTROL_RANGE_X
    switch (switch_id)
    {
    case 0:
        fre_temp = 3820000; // Khz
        LOG_D("open RF1");
        break;
    case 1:
        fre_temp = 4300000; // Khz
        LOG_D("open RF2");
        break;
    default:
        break;
    }
    if (ch_id == 1)
    {
        LOG_D("set ch1 clk2 frequency %u", fre_temp);
    }
    else if (ch_id == 2)
    {
        LOG_D("set ch2 clk2 frequency %u", fre_temp);
    }
#elif defined(DEVICE_FRE_CONTROL_RANGE_KA) || defined(DEVICE_FRE_CONTROL_RANGE_Q)
    switch (switch_id)
    {
    case 0:
        fre_temp = 6720000; // Khz
        LOG_D("open RF1");
        break;
    case 1:
        fre_temp = 7200000; // Khz
        LOG_D("open RF2");
        break;
    case 2:
        fre_temp = 7500000; // Khz
        LOG_D("open RF3");
        break;
    case 3:
        fre_temp = 7800000; // Khz
        LOG_D("open RF4");
        break;
    default:
        break;
    }
    if (ch_id == 1)
    {
        LOG_D("set ch1 clk2 frequency %u", fre_temp);//通道1本振2
    }
    else if (ch_id == 2) // Q频段没有通道2
    {
        LOG_D("set ch2 clk2  frequency %u", fre_temp);
    }
#endif
    return ret;
}

rt_err_t mb_fun_read_holding_registers(uint16_t addr, unsigned char reg_count, uint16_t *reg_buffer)
{
    rt_err_t ret = RT_EOK;
    if (addr < LWREG_START_ADDR)
        return -RT_ERROR;
    uint16_t start_addr_index = addr - LWREG_START_ADDR;
    uint16_t end_addr_index = start_addr_index + reg_count;
    if (end_addr_index >= REG_NUM) return -RT_ERROR;
    for (uint16_t addr_index = start_addr_index; addr_index < end_addr_index;)
    {
        switch (addr_index)
        {
        case 0:
            {
                uint32_t big_endian_ip = htonl(device_reg.IP);
                reg_buffer[addr_index] = big_endian_ip >> 16;
                reg_buffer[addr_index + 1] = big_endian_ip & 0xFFFF;
                addr_index += 2;
            }
            break;
        case 2:
            {
                uint32_t big_endian_ip_mask = htonl(device_reg.IP_MASK);
                reg_buffer[addr_index] = big_endian_ip_mask >> 16;
                reg_buffer[addr_index + 1] = big_endian_ip_mask & 0xFFFF;
                addr_index += 2;
            }
            break;
        case 4:
            {
                uint32_t big_endian_gateway = htonl(device_reg.GATE_WAY);
                reg_buffer[addr_index] = big_endian_gateway >> 16;
                reg_buffer[addr_index + 1] = big_endian_gateway & 0xFFFF;
                addr_index += 2;
            }
            break;
        case 6:
            {
                for (int i = 0; i < 6; i++)
                {
                    reg_buffer[addr_index + i] = (device_reg.SN[i * 2] << 8) | device_reg.SN[i * 2 + 1];
                }
                addr_index += 6;
            }
            break;
        case 12:
            reg_buffer[addr_index] = device_reg.version;
            addr_index += 1;
            break;
        case 13:    //系统状态
            reg_buffer[addr_index] = 0;
            addr_index += 1;
            break;
        case 16:
            reg_buffer[addr_index] = device_reg.LW1010;
            addr_index += 1;
            break;
        case 17:
            reg_buffer[addr_index] = device_reg.LW1011;
            addr_index += 1;
            break;
        case 18:
            reg_buffer[addr_index] = device_reg.ch1_input_fre >> 16;
            reg_buffer[addr_index + 1] = device_reg.ch1_input_fre & 0xFFFF;
            addr_index += 2;
            break;
        case 20:
            reg_buffer[addr_index] = device_reg.ch2_input_fre >> 16;
            reg_buffer[addr_index + 1] = device_reg.ch2_input_fre & 0xFFFF;
            addr_index += 2;
            break;
        case 22:
            reg_buffer[addr_index] = device_reg.LW1016;
            addr_index += 1;
            break;
        default:
            addr_index++;
            break;
        }
    }
    return ret;
}

rt_err_t mb_fun_write_registers(uint16_t addr, unsigned char reg_count, uint16_t *reg_buffer)
{
    rt_err_t ret = RT_EOK;
    if (addr < LWREG_START_ADDR)
        return -RT_ERROR;
    uint16_t start_addr_index = addr - LWREG_START_ADDR;
    uint16_t end_addr_index = start_addr_index + reg_count;
    struct fdb_blob blob;
    if (end_addr_index >= REG_NUM)
        return -RT_ERROR;
    for (uint16_t addr_index = start_addr_index; addr_index < end_addr_index;)
    {
        switch (addr_index)
        {
        case 0:
            device_reg.IP = ntohl((reg_buffer[addr_index] << 16) | reg_buffer[addr_index + 1]);
            addr_index += 2;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        case 2:
            device_reg.IP_MASK = ntohl((reg_buffer[addr_index] << 16) | reg_buffer[addr_index + 1]);
            addr_index += 2;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        case 4:
            device_reg.GATE_WAY = ntohl((reg_buffer[addr_index] << 16) | reg_buffer[addr_index + 1]);
            addr_index += 2;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        case 6:
            rt_memcpy(device_reg.SN, (uint8_t *)&reg_buffer[addr_index], sizeof(device_reg.SN));
            addr_index += 1;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        case 14:
            addr_index += 1;
            if (READ_BIT(reg_buffer[addr_index], 1) == 0x01) // bit0系统重启控制位
            {
                LOG_D("reboot now");
                rt_thread_mdelay(100);  //安全操作
                rt_hw_cpu_reset();
            }
            break;
        case 17:
            {
                if (device_reg.LW1011 != reg_buffer[addr_index])
                {
                    reg1011_t *ptr1 = (reg1011_t *)&device_reg.LW1011;
                    reg1011_t *ptr2 = (reg1011_t *)&reg_buffer[addr_index];
                    if(ptr1->CH1_MIDF_SW != ptr2->CH1_MIDF_SW)
                    {
                        ptr1->CH1_MIDF_SW = ptr2->CH1_MIDF_SW;
                    }
                    if(ptr1->CH2_MIDF_SW != ptr2->CH2_MIDF_SW)
                    {
                        ptr1->CH2_MIDF_SW = ptr2->CH2_MIDF_SW;
                    }
                    if(ptr1->CH1_MIDF_SW != ptr2->CH1_MIDF_SW)
                    {
                        ptr1->CH1_MIDF_SW = ptr2->CH1_MIDF_SW;
                    }
                    if(ptr1->CH2_MIDF_SW != ptr2->CH2_MIDF_SW)
                    {
                        ptr1->CH2_MIDF_SW = ptr2->CH2_MIDF_SW;
                    }
                    // if (ptr->CH1_MIDF_SW)
                    //     fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
                }
            }
            addr_index += 1;
            break;
        case 18:
            device_reg.ch1_input_fre = (reg_buffer[addr_index] << 16) | reg_buffer[addr_index + 1];
            addr_index += 2;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        case 20:
            device_reg.ch2_input_fre = (reg_buffer[addr_index] << 16) | reg_buffer[addr_index + 1];
            addr_index += 2;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        case 22:
            device_reg.LW1016 = reg_buffer[addr_index];
            addr_index += 1;
            // fdb_kv_set_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
            break;
        default:
            addr_index++;
            break;
        }
    }
    return ret;
}

int rtu_slave_done_callback(agile_modbus_t *ctx, int slave, int function,int addr, int quantity) {
    (void)(ctx);
    LOG_D("rtu_slave done. addr: %d, function: %d, regaddr: %d, quantity: %d.", slave, function, addr, quantity);
    rt_err_t ret = RT_EOK;
    // modbus_rtu_excuse(rtu1, ) 
    switch (function)
    {
    case AGILE_MODBUS_FC_READ_HOLDING_REGISTERS: 
    case AGILE_MODBUS_FC_READ_INPUT_REGISTERS:
        ret = mb_fun_read_holding_registers(addr, quantity, modbus_regs);
        break;
    case AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER:
        ret = mb_fun_write_registers(addr, 1, modbus_regs);
        break;
    case AGILE_MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
        ret = mb_fun_write_registers(addr, quantity, modbus_regs);
        break;
    default:
        break;
    }
    return MODBUS_RT_EOK;
}

int rtu_master_done_callback(agile_modbus_t *ctx, int slave, int function, int addr, int quantity)
{
    (void)(ctx);
    LOG_I("rtu_master done. addr: %d, function: %d, regaddr: %d, quantity: %d.", slave, function, addr, quantity);
    return MODBUS_RT_EOK;
}

int udp_slave_done_callback(agile_modbus_t *ctx, int slave, int function, int addr, int quantity)
{
    (void)(ctx);
    LOG_I("udp_slave done. addr: %d, function: %d, addr: %d, quantity: %d.", slave, function, addr, quantity);
    if (slave == 1)
    {
        rt_err_t ret = RT_EOK;
        switch (function)
        {
        case AGILE_MODBUS_FC_READ_HOLDING_REGISTERS:
        case AGILE_MODBUS_FC_READ_INPUT_REGISTERS:
            ret = mb_fun_read_holding_registers(addr, quantity, modbus_regs);
            break;

        case AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER:
            ret = mb_fun_write_registers(addr, 1, modbus_regs);
            break;
        case AGILE_MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            ret = mb_fun_write_registers(addr, quantity, modbus_regs);
            break;
        default:
            break;
        }
    }
    return MODBUS_RT_EOK;
}

rtu_modbus_device_t modbus_rtu_slave_init(char *port)
{
    rtu_modbus_device_t rs = modbus_rtu(MODBUS_SLAVE);
    RT_ASSERT(rs != NULL);
    modbus_rtu_set_addr(rs, 1);
    modbus_rtu_set_serial(rs, port, 115200, 8, 'N', 1, 0);
    modbus_rtu_add_block(rs, REGISTERS, LWREG_START_ADDR, modbus_regs, REG_NUM);
    modbus_rtu_set_done_callback(rs, rtu_slave_done_callback);
    modbus_rtu_open(rs);
    LOG_I("modbus rtu slave open on port %s success", port);
    return rs;
}

rtu_modbus_device_t modbus_rtu_master_init(char *port)
{
    rtu_modbus_device_t rm = modbus_rtu(MODBUS_MASTER);
    RT_ASSERT(rm != NULL);
    modbus_rtu_set_serial(rm, port, 115200, 8, 'N', 1, 0);
    modbus_rtu_set_done_callback(rm, rtu_master_done_callback);
    modbus_rtu_open(rm);
    LOG_I("modbus rtu master open on port %s success", port);
    return rm;
}

tcp_modbus_device_t modbus_tcp_slave_for_udp_init(char *ip, uint16_t port)
{
    tcp_modbus_device_t tsu = modbus_tcp(MODBUS_SLAVE);
    RT_ASSERT(tsu != NULL);
    modbus_tcp_set_net(tsu, ip, port, SOCK_DGRAM);
    modbus_tcp_set_strict(tsu, 1);
    modbus_tcp_add_block(tsu, REGISTERS, LWREG_START_ADDR, modbus_regs, REG_NUM);
    modbus_tcp_set_done_callback(tsu, udp_slave_done_callback);
    modbus_tcp_open(tsu);
    LOG_I("modbus tcp slave for udp open on port %d success", port);
    return tsu;
}

static rt_err_t netdev_set_if(char *netdev_name, char *ip_addr, char *gw_addr, char *nm_addr)
{
    struct netdev *netdev = RT_NULL;
    ip_addr_t addr;

    netdev = netdev_get_by_name(netdev_name);
    if (netdev == RT_NULL)
    {
        LOG_E("bad network interface device name(%s).", netdev_name);
        return -RT_ERROR;
    }

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
    return RT_EOK;
}

static void load_kvdb_para(void)
{
    struct fdb_blob blob;
    char ip[IP4ADDR_STRLEN_MAX+1] = {0};
    char ip_mask[IP4ADDR_STRLEN_MAX+1] = {0};
    char gate_way[IP4ADDR_STRLEN_MAX+1] = {0};
    fdb_kv_get_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &device_reg, sizeof(device_reg)));
    inet_ntoa_r(device_reg.IP, ip, IP4ADDR_STRLEN_MAX);
    inet_ntoa_r(device_reg.IP_MASK, ip_mask, IP4ADDR_STRLEN_MAX);
    inet_ntoa_r(device_reg.GATE_WAY, gate_way, IP4ADDR_STRLEN_MAX);
    netdev_set_if("e0", ip, gate_way, ip_mask);
}
static int env_kvdb_init(void)
{
    rt_thread_delay(RT_TICK_PER_SECOND);
// #define dir_path "/spiflash/fdb_kvdb/"
#define dir_path "spiflash1"
    fdb_err_t result;
    bool file_mode = true;
    bool not_formatable = true;
    uint32_t sec_size = 4096;
    uint32_t db_size = sec_size * 4;
    struct fdb_default_kv default_kv;
    default_kv.kvs = default_kv_table;
    default_kv.num = sizeof(default_kv_table) / sizeof(default_kv_table[0]);
#ifdef FDB_KV_AUTO_UPDATE
    kvdb.ver_num = 2;
#endif
    rt_mutex_init(&lock, "env", RT_IPC_FLAG_FIFO);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, kvdb_lock);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, kvdb_unlock);
    /* set the sector and database max size */
    // fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_SEC_SIZE, &sec_size);
    // fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_MAX_SIZE, &db_size);
    /* enable file mode */
    // fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_FILE_MODE, &file_mode);
    // fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_NOT_FORMAT, &not_formatable);
    // mkdir(dir_path, 0777);
    result = fdb_kvdb_init(&kvdb, "env", dir_path, &default_kv, NULL);
    if (result != FDB_NO_ERR)
    {
        return -RT_ERROR;
    }
    return RT_EOK;
}


static int env_print_regs(uint8_t argc, char **argv)
{
    fdb_kv_print(&kvdb);
    struct fdb_blob blob;
    device_reg_t regs = {0};
    fdb_kv_get_blob(&kvdb, "DEVICE_REG", fdb_blob_make(&blob, &regs, sizeof(regs)));
    if (blob.saved.len > 0)
    {
        LOG_D("IP:%d\nMASK:%d\nGW:%d\nVERSION:%d\nCH1_INPUT_FREQ:%d\nCH2_INPUT_FREQ:%d\nSN:%s\nLW1011:%d\nLW1016:%d",
              regs.IP, regs.IP_MASK, regs.GATE_WAY, regs.version,
              regs.ch1_input_fre,
              regs.ch2_input_fre, regs.SN, regs.LW1011, regs.LW1016);
        return 0;
    }
    return -1;
}
MSH_CMD_EXPORT_ALIAS(env_print_regs, env_print, print saved regs saved kvdb);

static void netdev_status_callback(struct netdev *netdev, enum netdev_cb_type type)
{
    if (type == NETDEV_CB_STATUS_LINK_UP)
    {
        LOG_I("netdev event(NETDEV_CB_STATUS_LINK_UP) happend");
        if(tsu == RT_NULL)
        {
            tsu = modbus_tcp_slave_for_udp_init(NULL, 502);
        }
    }
    else if (type == NETDEV_CB_STATUS_LINK_DOWN)
    {
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

static void dtu_thread_entry(void *parameter)
{
    uint16_t regs[5] = {0};
    reg1010_t *reg1010 = RT_NULL;
    reg1010 = (reg1010_t *)&device_reg.LW1010;
    // load_kvdb_para();
    while (1)
    {
#if defined DEVICE_FRE_CONTROL_RANGE_X
        //获取锁相源模块信息
        if(rtu4)
        {
            int ret = modbus_rtu_excuse(rtu4, 0x01, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 3, &regs);
            if (ret == MODBUS_RT_EOK)
            {
                reg1010->PLL_REF = regs[0] & 0x0003;
                reg1010->PLL_LOCK = READ_BIT(regs[0], 3);
                LOG_D("PLL temperature %.1f°", (float)regs[2] / 10);
            }
            else
            {
                LOG_E("get pll info error %d", ret);
            }
        }
        // 获取通道1本振1信息
        if(rtu2)
        {
            int ret = modbus_rtu_excuse(rtu2, 0x01, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
            if (ret == MODBUS_RT_EOK)
            {
                reg1010->CH1_CLK1_LOCK = READ_BIT(regs[0], 1);
                LOG_D("ch1 clk1 temperature %.1f°", (float)regs[2] / 10);
                uint32_t fre = regs[3] << 16 | regs[4];
                LOG_D("ch1 clk1 frequency %.3f MHz", (float)regs[3] / 1000);
            }
            else
            {
                LOG_E("get ch1 local osc1 info error %d", ret);
            }
        }

        // //获取通道1本振2信息
        // ret = modbus_rtu_excuse(rtu2, 0x02, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 local osc2 info error %d",ret);
        // }
        // //获取通道1 变频器1信息
        // ret = modbus_rtu_excuse(rtu2, 0x03, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 frequency converter1 info error %d",ret);
        // }
        // //获取通道1 变频器2信息
        // ret = modbus_rtu_excuse(rtu2, 0x04, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 frequency converter2 info error %d",ret);
        // }
        // //获取通道2本振1信息
        // ret = modbus_rtu_excuse(rtu3, 0x01, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 local osc1 info error %d",ret);
        // }
        // //获取通道2本振2信息
        // ret = modbus_rtu_excuse(rtu3, 0x02, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 local osc2 info error %d",ret);
        // }
        // //获取通道2 变频器1信息
        // ret = modbus_rtu_excuse(rtu3, 0x03, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 frequency converter1 info error %d",ret);
        // }
        // //获取通道2 变频器2信息
        // ret = modbus_rtu_excuse(rtu3, 0x04, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        // if(ret == MODBUS_RT_EOK)
        // {
        //     for(int i = 0; i < 5; i++)
        //     {
        //         LOG_D("reg%d:%d",i, regs[i]);
        //     }
        // }
        // else
        // {
        //     LOG_E("get ch1 frequency converter2 info error %d",ret);
        // }

#elif defined DEVICE_FRE_CONTROL_RANGE_KA
#elif defined DEVICE_FRE_CONTROL_RANGE_Q
#endif
        rt_thread_mdelay(500);
    }
}

static int dtu_init(void)
{
    LOG_I("test float data %.3f", 390.223);
    env_kvdb_init();
    // struct netdev *dev = netdev_get_by_name(NET_DEV_NAME); // lwip设置网络连接callback
    // if (dev && netdev_is_link_up(dev))
    // {
    //     tsu = modbus_tcp_slave_for_udp_init(NULL, 502);
    //     netdev_set_status_callback(dev, netdev_status_callback);
    // }
    // rtu1 = modbus_rtu_slave_init(COM2_PORT_NAME);  // 屏幕
    // // rtu2 = modbus_rtu_master_init(COM2_PORT_NAME); // 通道1  --本振 + 变频
    // // rtu3 = modbus_rtu_master_init(COM3_PORT_NAME); //通道2
    // rtu4 = modbus_rtu_master_init(COM4_PORT_NAME); // 通道3  -- 锁相源
    // rt_thread_t thread = rt_thread_create("dtu", dtu_thread_entry, RT_NULL, 4096, MODBUS_THREAD_PRIO + 1, 20);
    // if (thread != RT_NULL)
    //     rt_thread_startup(thread);
    return 0;
}
INIT_FS_EXPORT(dtu_init);

