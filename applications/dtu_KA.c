/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include "netdev.h"
#include "pcf8575.h"
#include "env.h"

#define DBG_TAG "DTU"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

typedef union
{
    struct
    {
        uint16_t PLL_REF_STA : 2; //锁相源参考输入信号状态  0:无外部参考 1:参考信号正常 2:参考信号异常
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
        uint16_t PLL_INPUT_SEL : 2; // 锁相源参考输入信号选择0:使用内部参考100M 1:使用外部参考10M 2:使用外部参考100M
        uint16_t CH1_OUTPUT_EN : 1; // 通道1输出频率开关 0:关闭 1:打开
        uint16_t CH1_MIDF_SW : 3; // 通道1中频输出选择
        uint16_t CH2_OUTPUT_EN : 1; // 通道2输出频率开关 0:关闭 1:打开
        uint16_t CH2_MIDF_SW : 3; // 通道2中频输出选择
        uint16_t RESERVED : 6;
    };
    uint16_t val;
} reg1011_t;

typedef union
{
    struct
    {
        uint16_t net_link : 1; // 网络连接状态 0:未连接 1:已连接
        uint16_t pll_link: 1;
        uint16_t ch1_clk1_link: 1; 
        uint16_t ch1_clk2_link: 1; 
        uint16_t ch1_x2_link: 1; 
        uint16_t ch2_clk1_link: 1; 
        uint16_t ch2_clk2_link: 1; 
        uint16_t ch2_x2_link: 1;
        uint16_t RESERVED : 2;
        uint16_t RESET_CODE : 6;
    };
    uint16_t val;
} sys_status_t;

typedef union
{
    struct
    {
        uint16_t reset : 1;      // 1:系统复位
        uint16_t RESERVED : 15;
    };
    uint16_t val;
} sys_ctrl_t;
typedef union
{
    struct
    {
        uint16_t CH1_GAIN : 8; //通道1增益
        uint16_t CH2_GAIN : 8; //通道2增益
    };
    uint16_t val;
} reg1016_t;

typedef union
{
    struct
    {
        uint16_t PLL_LOCK : 1; // D0
        uint16_t INPUT_REF_STA : 2;
        uint16_t RESERVED : 5;
        uint16_t RESET_CODE : 8;
    };
    uint16_t val;
} pll_node_status_t;

typedef union
{
    struct
    {
        uint16_t REF_10M_100M_SEL : 2; // D0  10M/100M参考输入选择  锁相源参考输入信号选择0:使用内部参考100M 1:使用外部参考10M 2:使用外部参考100M
    };
    uint16_t val;
} pll_node_ctrl_t;
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    pll_node_status_t status;
    uint16_t ctrl_reg;
    uint16_t temperature;     
    uint16_t version;
    uint16_t input_ref_vol;
    uint64_t freq_set;
} pll_node_attr_t;

typedef union
{
    struct
    {
        uint16_t CLK_LOCK : 1; 
        uint16_t RESET_CODE : 15;
    };
    uint16_t val;
} clk_node_status_t;
// 本振节点属性
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    clk_node_status_t status;
    uint16_t ctrl_reg;
    uint16_t temperature;
    uint16_t version;
    uint16_t spur_param;
    uint64_t freq_set;
} clk_node_attr_t;
typedef union
{
    struct
    {
        uint16_t PA_LDO_EN : 1;
        uint16_t RF_PATH_SEL : 2;
        uint16_t RESERVED : 13;
    };
    uint16_t val;
} frecov_node_ctrl_t;

// 变频模块节点属性
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    uint16_t status;
    uint16_t ctrl_reg;
    uint16_t temperature;
    uint16_t version;
    uint16_t decay_value;
} frecov_node_attr_t;
//二次变频频率组合定义
typedef struct
{
    uint64_t IF_freq; // Hz
    uint64_t f1_2[2];      // Hz
} Combo;
//衰减器io映射定义
typedef union
{
    struct
    {
        uint16_t p00 : 1;
        uint16_t p01_06 : 6; // gs5801_1 ct1-ct6
        uint16_t p07 : 1;
        uint16_t p10 : 1;
        uint16_t p11_p16 : 6; // gs5801_2 ct1-ct6
        uint16_t p17 : 1;
    };
    uint16_t val;
} ct_io_map_t;

#define REG_NUM (128)
#define LWREG_START_ADDR 0X1000
#define LWREG_END_ADDR (LWREG_START_ADDR + REG_NUM)
#define RS485_BUS_IDLE_WAITTIME 100 // ms
#define PFD_OFFSET 100000000        // 100 MHz
#define DEFAULT_PLL_MB_ID 1    // 锁相源模块485通信地址
#define DEFAULT_CH1_CLK1_MB_ID 1
#define DEFAULT_CH1_CLK2_MB_ID 2
#define DEFAULT_CH1_FRECOV_MB_ID 3
#define DEFAULT_CH2_CLK1_MB_ID 1
#define DEFAULT_CH2_CLK2_MB_ID 2
#define DEFAULT_CH2_FRECOV_MB_ID 3

static uint16_t modbus_regs[REG_NUM] = {0};
static rtu_modbus_device_t rtu1 = RT_NULL;
static rtu_modbus_device_t rtu2 = RT_NULL;
static rtu_modbus_device_t rtu3 = RT_NULL;
static rtu_modbus_device_t rtu4 = RT_NULL;
static tcp_modbus_device_t tsu = RT_NULL;
extern sys_reg_t sys_reg;
static sys_status_t sys_status = {0};
static reg1010_t node_status = {0};  //外设模块状态
static sys_ctrl_t sys_ctrl = {0};

static frecov_node_attr_t frecov_X2_node[2] = {
    {
        .mb_id = DEFAULT_CH1_FRECOV_MB_ID,
    },
    {
        .mb_id = DEFAULT_CH2_FRECOV_MB_ID,
    },
};
static pll_node_attr_t pll_node = {
    .mb_id = DEFAULT_PLL_MB_ID,
};
static clk_node_attr_t clk_node[2][2] = {
    {
        {
            .mb_id = DEFAULT_CH1_CLK1_MB_ID,
        },
        {
            .mb_id = DEFAULT_CH1_CLK2_MB_ID,
        },
    },
    {
        {
            .mb_id = DEFAULT_CH2_CLK1_MB_ID,
        },
        {
            .mb_id = DEFAULT_CH2_CLK2_MB_ID,
        },
    }};
extern rt_device_t pcf8575_dev[2];

#define FREQ_MULTI_FACTOR  3
#define INPUT_FREQ_START   17000000000 //17G
#define INPUT_FREQ_END     28000000000   // 17G
#define IF_MIN 5980000000ULL             // 1中频范围 定义
#define IF_FREQ 6000000000ULL      
#define IF_MAX 6020000000ULL     
#define PFD1_MIN 5000000         //Hz,距离PFD1频率的最小距离,优化杂散
#define PFD2_MIN 10000000       
static uint64_t CH1_IF2_fre_table[] = {720000000, 1200000000, 1500000000, 1800000000};
static uint64_t CH2_IF2_fre_table[] = {720000000, 1200000000, 1500000000, 1800000000};

typedef struct 
{
    uint64_t input_freq_hz;
    uint64_t decay_compensation_value;
    uint8_t decay_value;
}channel_attr_t;

static channel_attr_t channel_attr[2] = {0}; // 通道衰减补偿值

// 定义一个衰减补偿表给后级衰减使用
typedef struct
{
    uint64_t freq_hz;
    uint8_t value;
} decay_compensation_t; // 衰减补偿

const decay_compensation_t decay_compensation_table[2][3] = {
    {
        {24000000000, 3},
        {25000000000, 2},
        {25500000000, 1},
    },
    {
        {24000000000, 3},
        {25000000000, 2},
        {25500000000, 1},
    }

};
static bool check_frequency_condition1(uint64_t frequency)
{
    uint64_t remainder = frequency % PFD_OFFSET;
    if (remainder < PFD1_MIN || (PFD_OFFSET - remainder) < PFD1_MIN)
    {
        return false;
    }
    return true;
}

static bool check_frequency_condition2(uint64_t frequency)
{
    uint64_t remainder = frequency % PFD_OFFSET;
    if (remainder < PFD2_MIN || (PFD_OFFSET - remainder) < PFD2_MIN)
    {
        return false;
    }
    return true;
}
// 查找满足条件的组合，写入 combos，返回写入的数量
static size_t find_valid_combinations(uint64_t input_freq, Combo *combos, size_t max_combos, uint64_t IF2_freq)
{
    size_t count = 0;
    for (uint64_t IF1_freq = IF_MIN; IF1_freq <= IF_MAX; IF1_freq += 1000000ULL)
    { // 步进1 MHz
        uint64_t f1, f2;
        f1 = (input_freq + IF1_freq) / FREQ_MULTI_FACTOR;
        f2 = IF1_freq + IF2_freq;
        if (check_frequency_condition1(f1) && check_frequency_condition2(f2))
        {
            if (count < max_combos)
            {
                combos[count].IF_freq = IF1_freq;
                combos[count].f1_2[0] = f1;
                combos[count].f1_2[1] = f2;
                count++;
            }
        }
    }
    return count;
}
// 返回最接近 target 的 IF 的索引 
static size_t find_closest_if_index(const Combo *combos, size_t count, uint64_t target)
{
    if (count == 0)
    {
        return SIZE_MAX;
    }
    size_t closest_idx = 0;
    uint64_t first = combos[0].IF_freq;
    uint64_t best_diff = (first > target) ? (first - target) : (target - first);

    for (size_t i = 1; i < count; ++i)
    {
        uint64_t diff = (combos[i].IF_freq > target)
                       ? (combos[i].IF_freq - target)
                       : (target - combos[i].IF_freq);
        if (diff < best_diff)
        {
            best_diff = diff;
            closest_idx = i;
        }
    }
    return closest_idx;
}
//pll参考输入信号选择
static rt_err_t pll_inputref_select(uint8_t ref_sel)
{
    pll_node_ctrl_t pll_ctrl;
    pll_ctrl.val = pll_node.ctrl_reg;
    if (ref_sel > 2) return -RT_ERROR;
    pll_ctrl.REF_10M_100M_SEL = ref_sel;
    if (pll_node.ctrl_reg != pll_ctrl.val)
    {
        if (modbus_rtu_excuse(pll_node.channel_handle, pll_node.mb_id, AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0001, 1, &pll_ctrl.val) != MODBUS_RT_EOK)
        {
            LOG_E("pll input ref select failed");
            return -RT_ERROR;
        }
        pll_node.ctrl_reg = pll_ctrl.val;
    }
    return RT_EOK;
}


// 根据当前输入频率计算衰减补偿值
static uint8_t get_decay_compensation(uint8_t channel_idx, uint64_t input_freq_hz)
{
    if (channel_idx >= sizeof(decay_compensation_table) / sizeof(decay_compensation_table[0]))
        return 0;
    uint8_t compensation = 0;
    for (size_t i = 0; i < sizeof(decay_compensation_table[channel_idx]) / sizeof(decay_compensation_table[channel_idx][0]); i++)
    {
        if (input_freq_hz <= decay_compensation_table[channel_idx][i].freq_hz)
        {
            compensation = decay_compensation_table[channel_idx][i].value;
            break;
        }
    }
    return compensation;
}

static rt_err_t set_channel_link_decay(uint8_t channel_idx, uint8_t decay_value)
{
    rt_err_t ret = RT_EOK;
    // ct1-ct6 对应bit1-bit6
    ct_io_map_t ct_map = {0};
    if (channel_idx > 1)
        return -RT_ERROR;
    struct pcf8575c_io_map io_map = {0};
    channel_attr[channel_idx].decay_value = decay_value;
    if (decay_value <= 63) // 衰减没有超过30dB,只使用前级衰减  0b111111 p01_06
    {
        ct_map.p01_06 = decay_value;
        ct_map.p01_06 = channel_attr[channel_idx].decay_compensation_value;
    }
    else
    {
        ct_map.p01_06 = 63;
        ct_map.p11_p16 = decay_value + channel_attr[channel_idx].decay_compensation_value - ct_map.p01_06;
    }
    io_map.mask = 0xFFFF;
    io_map.value = ct_map.val;
    float real_gain = -0.5 * decay_value + 50;
    LOG_D("set chnnel%d link decay raw(%d) real_gain(%.1fdB) compensation(%d)", channel_idx + 1, decay_value, real_gain, channel_attr[channel_idx].decay_compensation_value);
    rt_device_control(pcf8575_dev[channel_idx], PCF8575C_CMD_SET_VALUE, &io_map);
    return ret;
}
// 根据输入频率配置通道
static rt_err_t channel_fre_config(uint8_t channel_idx, uint64_t input_freq_hz, uint8_t if_idx, rt_bool_t channel_output_en)
{
    int ret = 0;
    uint64_t IF2_fre = 0;
    if (channel_idx > 1)
        return -RT_ERROR;
    if (input_freq_hz < INPUT_FREQ_START || input_freq_hz > INPUT_FREQ_END)
        return -RT_ERROR;
    if (channel_idx == 0)
    {
        if (if_idx >= sizeof(CH1_IF2_fre_table) / sizeof(CH1_IF2_fre_table[0]))
            return -RT_ERROR;
        IF2_fre = CH1_IF2_fre_table[if_idx];
    }
    else
    {
        if (if_idx >= sizeof(CH2_IF2_fre_table) / sizeof(CH2_IF2_fre_table[0]))
            return -RT_ERROR;
        IF2_fre = CH2_IF2_fre_table[if_idx];
    }
    Combo combos[64] = {0};
    size_t closest_idx = 0;
    size_t count = find_valid_combinations(input_freq_hz, combos, sizeof(combos) / sizeof(combos[0]), IF2_fre);
    if (count > 0)
    {
        // LOG_D("Valid frequency combinations for input frequency %llu Hz:", input_freq_hz);
        // for (size_t i = 0; i < count; ++i)
        // {
        //     // 输出统一为 Hz
        //     LOG_D("IF_freq: %llu Hz, f1: %llu Hz, f2: %llu Hz",
        //           combos[i].IF_freq, combos[i].f1_2[0], combos[i].f1_2[1]);
        // }
        // 找到最接近 IF_FREQ 的 IF
        closest_idx = find_closest_if_index(combos, count, IF_FREQ);
    }
    else
    {
        LOG_E("No valid combinations found for ch%d input frequency %llu Hz.", channel_idx + 1, input_freq_hz);
        return -RT_ERROR;
    }
    // 配置本振1和本振2节点频率
    for (int i = 0; i < 2; i++)
    {
        if (clk_node[channel_idx][i].freq_set != combos[closest_idx].f1_2[i])
        {
            uint16_t regs[4] = {0};
            regs[0] = ((combos[closest_idx].f1_2[i] >> 48) & 0XFFFF);
            regs[1] = ((combos[closest_idx].f1_2[i] >> 32) & 0XFFFF);
            regs[2] = ((combos[closest_idx].f1_2[i] >> 16) & 0XFFFF);
            regs[3] = ((combos[closest_idx].f1_2[i] >> 0) & 0XFFFF);
            ret = modbus_rtu_excuse(clk_node[channel_idx][i].channel_handle, clk_node[channel_idx][i].mb_id, AGILE_MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 0x0005, 4, &regs);
            if (ret != MODBUS_RT_EOK)
            {
                LOG_E("config channel%d clk%d frequency %llu failed", channel_idx + 1, i + 1, combos[closest_idx].f1_2[i]);
                return -RT_ERROR;
            }
            clk_node[channel_idx][i].freq_set = combos[closest_idx].f1_2[i];
            LOG_D("ch%d Closest IF_freq to %llu Hz: %llu Hz, f1: %llu Hz, f2: %llu Hz",
                  channel_idx + 1, IF_FREQ, combos[closest_idx].IF_freq, combos[closest_idx].f1_2[0], combos[closest_idx].f1_2[1]);
            LOG_D("config channel%d clk%d frequency %llu success", channel_idx + 1, i + 1, combos[closest_idx].f1_2[i]);
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME); // 等待总线空闲
        }
    }
    // 计算衰减补偿值
    channel_attr[channel_idx].input_freq_hz = input_freq_hz;
    channel_attr[channel_idx].decay_compensation_value = get_decay_compensation(channel_idx, input_freq_hz);
    set_channel_link_decay(channel_idx, channel_attr[channel_idx].decay_value); //更新衰减
    // 使能控制
    frecov_node_ctrl_t ctrl_reg = {0};
    ctrl_reg.val = frecov_X2_node[channel_idx].ctrl_reg;
    ctrl_reg.RF_PATH_SEL = if_idx;
    ctrl_reg.PA_LDO_EN = channel_output_en;
    if (ctrl_reg.val != frecov_X2_node[channel_idx].ctrl_reg)
    {
        ret = modbus_rtu_excuse(frecov_X2_node[channel_idx].channel_handle, frecov_X2_node[channel_idx].mb_id, AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0001, 1, &ctrl_reg);
        if (ret != MODBUS_RT_EOK)
        {
            LOG_E("config channel%d frecov ctrl_reg(0x%04x) failed", channel_idx + 1, ctrl_reg.val);
            return -RT_ERROR;
        }
        frecov_X2_node[channel_idx].ctrl_reg = ctrl_reg.val;
        LOG_D("config channel%d frecov ctrl_reg(0x%04x) success", channel_idx + 1, ctrl_reg.val);
        rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME); // 等待总线空闲
    }
    return RT_EOK;
}

rt_err_t mb_fun_read_holding_registers(int addr, unsigned char reg_count, uint16_t *reg_buffer)
{
    rt_err_t ret = RT_EOK;
    int start_addr_index = LWREG_START_ADDR;
    int end_addr_index = addr + reg_count;
    if(addr < start_addr_index || end_addr_index > LWREG_END_ADDR ) return -RT_ERROR;
    for (int addr_index = addr; addr_index < end_addr_index;)
    {
        switch (addr_index)
        {
        case 0x1000:
        case 0x1002:
        case 0x1004:
            {
                uint32_t big_endian_value = 0;
                struct netdev *netdev = netdev_get_by_name("e0");
                if (addr_index == 0x1000)
                {
                    if (netdev)
                    {
                        big_endian_value = htonl(netdev->ip_addr.addr);
                    }
                    else
                    {
                        big_endian_value = htonl(sys_reg.IP);
                    }
                }
                else if (addr_index == 0x1002)
                {
                    if (netdev)
                    {
                        big_endian_value = htonl(netdev->netmask.addr);
                    }
                    else
                    {
                        big_endian_value = htonl(sys_reg.IP_MASK);
                    }
                }
                else if (addr_index == 0x1004)
                {
                    if (netdev)
                    {
                        big_endian_value = htonl(netdev->gw.addr);
                    }
                    else
                    {
                        big_endian_value = htonl(sys_reg.GATE_WAY);
                    }
                }
                reg_buffer[addr_index - start_addr_index] = big_endian_value >> 16;
                reg_buffer[addr_index - start_addr_index + 1] = big_endian_value & 0xFFFF;
                addr_index += 2;
            }
            break;
        case 0x1006:
            {
                for (int i = 0; i < 6; i++)
                {
                    reg_buffer[addr_index - start_addr_index + i] = (sys_reg.SN[i * 2] << 8) | sys_reg.SN[i * 2 + 1];
                }
                addr_index += 6;
            }
            break;
        case 0x100C:  //固件版本
            reg_buffer[addr_index - start_addr_index] = DTU_FIRMWARE_VERSION;
            addr_index += 1;
            break;
        case 0x100F:    //系统状态
            reg_buffer[addr_index - start_addr_index] = sys_status.val;
            addr_index += 1;
            break;
        case 0x1010:  //外设模块状态
            reg_buffer[addr_index - start_addr_index] = node_status.val;
            addr_index += 1;
            break;
        case 0x1011: //外设模块控制
            {
                reg1011_t reg = {0};
                pll_node_ctrl_t pll_ctrl_reg;
                pll_ctrl_reg.val = pll_node.ctrl_reg;
                reg.PLL_INPUT_SEL = pll_ctrl_reg.REF_10M_100M_SEL;
                frecov_node_ctrl_t frecov_ctrl_reg;
                frecov_ctrl_reg.val = frecov_X2_node[0].ctrl_reg;
                reg.CH1_MIDF_SW = frecov_ctrl_reg.RF_PATH_SEL;
                reg.CH1_OUTPUT_EN = frecov_ctrl_reg.PA_LDO_EN;
                frecov_ctrl_reg.val = frecov_X2_node[1].ctrl_reg;
                reg.CH2_MIDF_SW = frecov_ctrl_reg.RF_PATH_SEL;
                reg.CH2_OUTPUT_EN = frecov_ctrl_reg.PA_LDO_EN;
                reg_buffer[addr_index - start_addr_index] = reg.val;
            }
            addr_index += 1;
            break;
        case 0x1012:
            reg_buffer[addr_index - start_addr_index] = sys_reg.CH1_INPUT_FRE >> 16;
            reg_buffer[addr_index - start_addr_index + 1] = sys_reg.CH1_INPUT_FRE & 0xFFFF;
            addr_index += 2;
            break;
        case 0x1014:
            reg_buffer[addr_index - start_addr_index] = sys_reg.CH2_INPUT_FRE >> 16;
            reg_buffer[addr_index - start_addr_index + 1] = sys_reg.CH2_INPUT_FRE & 0xFFFF;
            addr_index += 2;
            break;
        case 0x1016:
            {
                // struct pcf8575c_io_map io_map = {0};
                // ct_io_map_t ct_map = {0};
                reg1016_t reg1016 = {0}; 
                // io_map.mask = 0xFFFF;
                // io_map.value = 0;
                // rt_device_control(pcf8575_dev[0], PCF8575C_CMD_GET_VALUE_CACHE, &io_map);
                // ct_map.val = io_map.value;
                reg1016.CH1_GAIN = channel_attr[0].decay_value;
                // rt_device_control(pcf8575_dev[1], PCF8575C_CMD_GET_VALUE_CACHE, &io_map);
                // ct_map.val = io_map.value;
                reg1016.CH2_GAIN = channel_attr[1].decay_value;
                reg_buffer[addr_index - start_addr_index] = reg1016.val;
            }
            addr_index += 1;
            break;
        case 0x1017: 
            reg_buffer[addr_index - start_addr_index] = pll_node.temperature;
            addr_index += 1;
            break;
        case 0x1018:
            reg_buffer[addr_index - start_addr_index] = clk_node[0][0].temperature;
            addr_index += 1;
            break;
        case 0x1019:
            reg_buffer[addr_index - start_addr_index] = clk_node[0][1].temperature;
            addr_index += 1;
            break;
        case 0x101A:
            reg_buffer[addr_index - start_addr_index] = frecov_X2_node[0].temperature;
            addr_index += 1;
            break;
        case 0x101B:
            reg_buffer[addr_index - start_addr_index] = clk_node[1][0].temperature;
            addr_index += 1;
            break;
        case 0x101C:
            reg_buffer[addr_index - start_addr_index] = clk_node[1][1].temperature;
            addr_index += 1;
            break;
        case 0x101D:
            reg_buffer[addr_index - start_addr_index] = frecov_X2_node[1].temperature;
            addr_index += 1;
            break;
        default:
            addr_index++;
            break;
        }
    }
    return ret;
}

rt_err_t mb_fun_write_registers(int addr, unsigned char reg_count, uint16_t *reg_buffer)
{
    rt_err_t ret = RT_EOK;
    int start_addr_index = LWREG_START_ADDR;
    int end_addr_index = addr + reg_count;
    if(addr < start_addr_index || end_addr_index > LWREG_END_ADDR ) return -RT_ERROR;
    struct fdb_blob blob;
    sys_reg_t sys_reg_temp = {0};
    sys_reg_temp = sys_reg;
    for (int addr_index = addr; addr_index < end_addr_index;)
    {
        switch (addr_index)
        {
        case 0x1000:
            sys_reg_temp.IP = ntohl((reg_buffer[addr_index - start_addr_index] << 16) | reg_buffer[addr_index - start_addr_index + 1]);
            addr_index += 2;
            break;
        case 0x1002:
            sys_reg_temp.IP_MASK = ntohl((reg_buffer[addr_index - start_addr_index] << 16) | reg_buffer[addr_index - start_addr_index + 1]);
            addr_index += 2;
            break;
        case 0x1004:
            sys_reg_temp.GATE_WAY = ntohl((reg_buffer[addr_index - start_addr_index] << 16) | reg_buffer[addr_index - start_addr_index + 1]);
            addr_index += 2;
            break;
        case 0x100E:  //系统控制
            sys_ctrl.val = reg_buffer[addr_index - start_addr_index];
            if (sys_ctrl.reset) // bit0系统重启控制位
            {
                LOG_D("reboot now");
                rt_thread_mdelay(100);  //安全操作
                rt_hw_cpu_reset();
            }
            addr_index += 1;
            break;
        case 0x1011:
            {
                sys_reg_temp.LW1011 = reg_buffer[addr_index - start_addr_index];
                reg1011_t *reg = (reg1011_t *)&sys_reg_temp.LW1011;
                uint64_t input_freq_hz = 0;
                if (sys_status.pll_link == RT_TRUE)
                {
                    pll_inputref_select(reg->PLL_INPUT_SEL);
                }
                if (sys_status.ch1_clk1_link == RT_TRUE && sys_status.ch1_clk2_link == RT_TRUE)
                {
                    input_freq_hz  = (uint64_t)sys_reg_temp.CH1_INPUT_FRE * 1000;
                    channel_fre_config(0, input_freq_hz,reg->CH1_MIDF_SW,reg->CH1_OUTPUT_EN);
                }
                if (sys_status.ch2_clk1_link == RT_TRUE && sys_status.ch2_clk2_link == RT_TRUE)
                {
                    input_freq_hz = (uint64_t)sys_reg_temp.CH2_INPUT_FRE * 1000;
                    channel_fre_config(1, input_freq_hz,reg->CH2_MIDF_SW,reg->CH2_OUTPUT_EN);
                }
                
            }
            addr_index += 1;
            break;
        case 0x1012:
            {
                sys_reg_temp.CH1_INPUT_FRE = (reg_buffer[addr_index - start_addr_index] << 16) | reg_buffer[addr_index - start_addr_index + 1];
                if (sys_status.ch1_clk1_link == RT_TRUE && sys_status.ch1_clk2_link == RT_TRUE)
                {
                    reg1011_t *reg = (reg1011_t *)&sys_reg_temp.LW1011;
                    uint64_t input_freq_hz = (uint64_t)sys_reg_temp.CH1_INPUT_FRE * 1000;
                    channel_fre_config(0, input_freq_hz, reg->CH1_MIDF_SW, reg->CH1_OUTPUT_EN);
                }
            }
            addr_index += 2;
            break;
        case 0x1014:
            {
                sys_reg_temp.CH2_INPUT_FRE = (reg_buffer[addr_index - start_addr_index] << 16) | reg_buffer[addr_index - start_addr_index + 1];
                if (sys_status.ch2_clk1_link == RT_TRUE && sys_status.ch2_clk2_link == RT_TRUE)
                {
                    reg1011_t *reg = (reg1011_t *)&sys_reg_temp.LW1011;
                    uint64_t input_freq_hz = (uint64_t)sys_reg_temp.CH2_INPUT_FRE * 1000;
                    channel_fre_config(1, input_freq_hz, reg->CH2_MIDF_SW, reg->CH2_OUTPUT_EN);
                }
            }
            addr_index += 2;
            break;
        case 0x1016:
            {
                sys_reg_temp.LW1016 = reg_buffer[addr_index - start_addr_index];
                reg1016_t *reg = (reg1016_t *)&sys_reg_temp.LW1016;
                set_channel_link_decay(0, reg->CH1_GAIN);
                set_channel_link_decay(1, reg->CH2_GAIN);
                addr_index += 1;
            }
            break;
        default:
            addr_index++;
            break;
        }
    }
    if (rt_memcmp(&sys_reg, &sys_reg_temp, sizeof(sys_reg)) != 0)
    {
        sys_reg = sys_reg_temp;
        save_sys_regs();
    }
    return ret;
}

int slave_pre_ans_callback(agile_modbus_t *ctx, int slave, int function, int addr, int quantity)
{
    (void)(ctx);
    // LOG_I("slaveid: %d, function: %d, addr: 0x%04x, quantity: %d.", slave, function, addr, quantity);
    if(ctx)
    if (slave == 1)
    {
        rt_err_t ret = RT_EOK;
        switch (function)
        {
        case AGILE_MODBUS_FC_READ_HOLDING_REGISTERS:
            ret = mb_fun_read_holding_registers(addr, quantity, modbus_regs);
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
    modbus_rtu_set_pre_ans_callback(rs, slave_pre_ans_callback);
    modbus_rtu_set_done_callback(rs, slave_done_callback);
    modbus_rtu_open(rs);
    LOG_I("modbus rtu slave open on port %s success", port);
    return rs;
}

rtu_modbus_device_t modbus_rtu_master_init(char *port)
{
    rtu_modbus_device_t rm = modbus_rtu(MODBUS_MASTER);
    RT_ASSERT(rm != NULL);
    modbus_rtu_set_serial(rm, port, 115200, 8, 'N', 1, 0);
    modbus_rtu_set_done_callback(rm, NULL);
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
    modbus_tcp_set_pre_ans_callback(tsu, slave_pre_ans_callback);
    modbus_tcp_set_done_callback(tsu, slave_done_callback);
    modbus_tcp_open(tsu);
    LOG_I("modbus tcp slave for udp open on port %d success", port);
    return tsu;
}

static void netdev_status_callback(struct netdev *netdev, enum netdev_cb_type type)
{
    if (type == NETDEV_CB_STATUS_LINK_UP)
    {
        sys_status.net_link = RT_TRUE;
        LOG_I("netdev event(NETDEV_CB_STATUS_LINK_UP) happend");
        if(tsu == RT_NULL)
        {
            tsu = modbus_tcp_slave_for_udp_init(NULL, 502);
        }
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
    if(netdev_is_link_up(netdev))
        sys_status.net_link = RT_TRUE;
    else
        sys_status.net_link = RT_FALSE;
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
    netdev_set_status_callback(netdev, netdev_status_callback);
    return RT_EOK;
}

static int get_pll_info(pll_node_attr_t *pll_node)
{
    uint16_t regs[10] = {0};
    uint32_t buffer[2] = {0};
    int ret = MODBUS_RT_EOK;
    if (pll_node->channel_handle)
    {
        ret = modbus_rtu_excuse(pll_node->channel_handle, pll_node->mb_id, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, sizeof(regs) / sizeof(regs[0]), &regs);
        if (ret == MODBUS_RT_EOK)
        {
            pll_node->status.val = regs[0];
            pll_node->ctrl_reg = regs[1];
            pll_node->temperature = regs[2];
            pll_node->version = regs[3];
            pll_node->input_ref_vol = regs[4];
            buffer[0] = (regs[5] << 16) | regs[6];
            buffer[1] = (regs[7] << 16) | regs[8];
            pll_node->freq_set = ((uint64_t)buffer[0] << 32) | buffer[1];
        }
    }
    return ret;
}

static int get_clk_info(clk_node_attr_t *clk_node)
{
    uint16_t regs[10] = {0};
    uint32_t buffer[2] = {0};
    int ret = MODBUS_RT_EOK;
    if (clk_node->channel_handle)
    {
        ret = modbus_rtu_excuse(clk_node->channel_handle, clk_node->mb_id, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 10, &regs);
        if (ret == MODBUS_RT_EOK)
        {
            clk_node->status.val = regs[0];
            clk_node->ctrl_reg = regs[1];
            clk_node->temperature = regs[2];
            clk_node->version = regs[3];
            clk_node->spur_param = regs[4];
            buffer[0] = (regs[5] << 16) | regs[6];
            buffer[1] = (regs[7] << 16) | regs[8];
            clk_node->freq_set = ((uint64_t)buffer[0] << 32) | buffer[1];
        }
    }
    return ret;
}

static int get_freqcov_info(frecov_node_attr_t *frecov_node)
{
    uint16_t regs[10] = {0};
    int ret = MODBUS_RT_EOK;
    if (frecov_node->channel_handle)
    {
        ret = modbus_rtu_excuse(frecov_node->channel_handle, frecov_node->mb_id, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, 5, &regs);
        if (ret == MODBUS_RT_EOK)
        {
            frecov_node->status = regs[0];
            frecov_node->ctrl_reg = regs[1];
            frecov_node->temperature = regs[2];
            frecov_node->version = regs[3];
            frecov_node->decay_value = regs[4];
        }
    }
    return ret;
}

static void print_node_info(void)
{
    // 打印锁相源信息
    LOG_D("PLL status 0x%04x,temperature %.1f,lock %d,ctrl_reg 0x%04x,input ref signal voltage %dmV ,frequency %llu Hz",
          pll_node.status.val, (float)pll_node.temperature / 10, pll_node.status.PLL_LOCK, pll_node.ctrl_reg, pll_node.input_ref_vol, pll_node.freq_set);

    // 打印本振信息
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            LOG_D("ch%d clk%d status 0x%04x,temperature %.1f,lock %d,ctrl_reg 0x%04x,spur_param 0x%04x,frequency %llu Hz",
                  i, j, clk_node[i][j].status.val, (float)clk_node[i][j].temperature / 10, clk_node[i][j].status.CLK_LOCK, clk_node[i][j].ctrl_reg, clk_node[i][j].spur_param, clk_node[i][j].freq_set);
        }
    }
    // 打印变频器信息
    for (int i = 0; i < 2; i++)
    {
        LOG_D("ch%d frecovX2 status 0x%04x,temperature %.1f,ctrl_reg 0x%04x,version %d", i + 1, frecov_X2_node[i].status,
              (float)frecov_X2_node[i].temperature / 10, frecov_X2_node[i].ctrl_reg, frecov_X2_node[i].version);
    }
}
MSH_CMD_EXPORT(print_node_info, print all peripheral node info);

static void dtu_thread_entry(void *parameter)
{
    uint16_t regs[10] = {0};
    reg1010_t *reg1010 = RT_NULL;
    reg1010 = (reg1010_t *)&node_status;
    uint32_t buffer[2] = {0};
    int ret = MODBUS_RT_EOK;
    extern uint8_t get_last_reset_code(void);
    uint8_t reset_code = get_last_reset_code();
    sys_status.RESET_CODE = reset_code;
    reg1011_t *reg1011 = (reg1011_t *)&sys_reg.LW1011;    
    // reg1016_t *reg1016 = (reg1016_t *)&sys_reg.LW1016;
    reg1011->PLL_INPUT_SEL = 0;
    reg1011->CH1_OUTPUT_EN = 0;
    reg1011->CH2_OUTPUT_EN = 0;
    set_channel_link_decay(0, 120); // 最大衰减值dB -10 = -0.5*120+50
    set_channel_link_decay(1, 120);
    rt_thread_mdelay(1000); // 等待外设模块上电稳定//根据存储的外设配置配置外设
    uint32_t time_wait = RS485_BUS_IDLE_WAITTIME * 5;
    pll_inputref_select( reg1011->PLL_INPUT_SEL);
    uint64_t input_freq_hz = (uint64_t)sys_reg.CH1_INPUT_FRE * 1000;
    channel_fre_config(0, input_freq_hz, reg1011->CH1_MIDF_SW, reg1011->CH1_OUTPUT_EN);
    input_freq_hz = (uint64_t)sys_reg.CH2_INPUT_FRE * 1000;
    channel_fre_config(1, input_freq_hz, reg1011->CH2_MIDF_SW, reg1011->CH2_OUTPUT_EN);
    while (1)
    {
        int ret = get_pll_info(&pll_node);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.pll_link = RT_TRUE;
            node_status.PLL_REF_STA = pll_node.status.INPUT_REF_STA;
            node_status.PLL_LOCK = pll_node.status.PLL_LOCK;
        }
        else
        {
            sys_status.pll_link = RT_FALSE;
            LOG_E("get pll info error %d", ret);
        }
        // 获取通道1本振、变频器信息
        ret = get_clk_info(&clk_node[0][0]);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.ch1_clk1_link = RT_TRUE;
            node_status.CH1_CLK1_LOCK = clk_node[0][0].status.CLK_LOCK;
        }
        else
        {
            sys_status.ch1_clk1_link = RT_FALSE;
            node_status.CH1_CLK1_LOCK = RT_FALSE;
            LOG_E("get ch1 clk1 info error %d", ret);
        }
        rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        ret = get_clk_info(&clk_node[0][1]);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.ch1_clk2_link = RT_TRUE;
            node_status.CH1_CLK2_LOCK = clk_node[0][1].status.CLK_LOCK;
        }
        else
        {
            sys_status.ch1_clk2_link = RT_FALSE;
            node_status.CH1_CLK2_LOCK = RT_FALSE;
            LOG_E("get ch1 clk2 info error %d", ret);
        }
        rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        ret = get_freqcov_info(&frecov_X2_node[0]);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.ch1_x2_link = RT_TRUE;
        }
        else
        {
            sys_status.ch1_x2_link = RT_FALSE;
            LOG_E("get ch1 frecovX2 info error %d", ret);
        }
        // 获取通道2本振、变频器信息
        ret = get_clk_info(&clk_node[1][0]);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.ch2_clk1_link = RT_TRUE;
            node_status.CH2_CLK1_LOCK = clk_node[1][0].status.CLK_LOCK;
        }
        else
        {
            sys_status.ch2_clk1_link = RT_FALSE;
            node_status.CH2_CLK1_LOCK = RT_FALSE;
            LOG_E("get ch2 clk1 info error %d", ret);
        }
        rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        ret = get_clk_info(&clk_node[1][1]);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.ch2_clk2_link = RT_TRUE;
            node_status.CH2_CLK2_LOCK = clk_node[1][1].status.CLK_LOCK;
        }
        else
        {
            sys_status.ch2_clk2_link = RT_FALSE;
            node_status.CH2_CLK2_LOCK = RT_FALSE;
            LOG_E("get ch2 clk2 info error %d", ret);
        }
        rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        ret = get_freqcov_info(&frecov_X2_node[1]);
        if (ret == MODBUS_RT_EOK)
        {
            sys_status.ch2_x2_link = RT_TRUE;
        }
        else
        {
            sys_status.ch2_x2_link = RT_FALSE;
            LOG_E("get ch2 frecovX2 info error %d", ret);
        }
    }
}

//保存网络参数
rt_err_t save_net_args(void)
{
    char *netdev_name = "e0";
    struct netdev *netdev = RT_NULL;
    ip_addr_t addr;

    netdev = netdev_get_by_name(netdev_name);
    if (netdev == RT_NULL)
    {
        LOG_E("bad network interface device name(%s).", netdev_name);
        return -RT_ERROR;
    }
    sys_reg.IP = netdev->ip_addr.addr;
    sys_reg.IP_MASK = netdev->netmask.addr;
    sys_reg.GATE_WAY = netdev->gw.addr;
    return save_sys_regs();
}
MSH_CMD_EXPORT_ALIAS(save_net_args, save_net_args, save network args);

static int dtu_init(void)
{
    LOG_D("firmware version %d", DTU_FIRMWARE_VERSION);
    char ip[IP4ADDR_STRLEN_MAX + 1] = {0};
    char ip_mask[IP4ADDR_STRLEN_MAX + 1] = {0};
    char gate_way[IP4ADDR_STRLEN_MAX + 1] = {0};
    inet_ntoa_r(sys_reg.IP, ip, IP4ADDR_STRLEN_MAX);
    inet_ntoa_r(sys_reg.IP_MASK, ip_mask, IP4ADDR_STRLEN_MAX);
    inet_ntoa_r(sys_reg.GATE_WAY, gate_way, IP4ADDR_STRLEN_MAX);
    netdev_set_if("e0", ip, gate_way, ip_mask);
    tsu = modbus_tcp_slave_for_udp_init(NULL, 502);
    rtu1 = modbus_rtu_slave_init(COM1_PORT_NAME);  // 屏幕
    rtu2 = modbus_rtu_master_init(COM2_PORT_NAME); // 通道2  -- 锁相源
    pll_node.channel_handle = rtu2;
    rtu3 = modbus_rtu_master_init(COM3_PORT_NAME); // 通道1  --本振 + 变频
    rtu4 = modbus_rtu_master_init(COM4_PORT_NAME); // 通道2  --本振 + 变频
    clk_node[0][0].channel_handle = rtu3;
    clk_node[0][1].channel_handle = rtu3;
    frecov_X2_node[0].channel_handle = rtu3;
    clk_node[1][0].channel_handle = rtu4;
    clk_node[1][1].channel_handle = rtu4;
    frecov_X2_node[1].channel_handle = rtu4;
    rt_thread_t thread = rt_thread_create("dtu", dtu_thread_entry, RT_NULL, 4096, MODBUS_THREAD_PRIO + 1 , 100);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
    return 0;
}
INIT_FS_EXPORT(dtu_init);

