/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <ipc/workqueue.h>
#include "modbus_rtu.h"
#include "env.h"
#include "X_down.h"
#define DBG_TAG "DTU"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#if defined BSP_USING_X_DOWN
// 定义通信相关常量
#define RS485_BUS_IDLE_WAITTIME 100 // ms
#define MODBUS_QUEUE_SIZE 64
#define DEFAULT_PLL_MB_ID 1 // 锁相源模块485通信地址
#define DEFAULT_CLK1_MB_ID 1
#define DEFAULT_CLK2_MB_ID 2
#define DEFAULT_FRECOVX2_MB_ID 3
#define DEFAULT_FRECOVX1_MB_ID 4
// 定义频率相关常量
#define PFD_OFFSET 100000000ULL // 100 MHz
#define PFD1_MIN 10000000ULL    // Hz,距离PFD1频率的最小距离,优化杂散
#define PFD2_MIN 15000000ULL
#define MAX_DECAY_VALUE 120
#define FREQ_MULTI_FACTOR 1
#define INPUT_FREQ_START 7000000000
#define INPUT_FREQ_END 9000000000


#define SCAN_STEP 1000000ULL // 1 MHz

enum
{
    WORK_TYPE_GET_PLL_INFO,
    WORK_TYPE_GET_CLK1_INFO,
    WORK_TYPE_GET_CLK2_INFO,
    WORK_TYPE_GET_CLK3_INFO,
    WORK_TYPE_GET_FRECOVX1_INFO,
    WORK_TYPE_GET_FRECOVX2_INFO, // 变频器X2
    WORK_TYPE_CONFIG_CLK1_FREQ,
    WORK_TYPE_CONFIG_CLK2_FREQ,
    WORK_TYPE_CONFIG_CLK1_OUTPUT,
    WORK_TYPE_CONFIG_CLK2_OUTPUT,
    WORK_TYPE_CONFIG_FRECOVX2_IF_IDX, // 配置变频器X2中频通路选择
    WORK_TYPE_CONFIG_FRECOVX2_LDOEN,  // 配置变频器X2 LDO使能
    WORK_TYPE_CONFIG_FRECOVX1_DECAY,  // 配置变频器X1衰减
    WORK_TYPE_CONFIG_FRECOVX1_LDOEN,  // 配置变频器X1 LDO使能
    WORK_TYPE_CONFIG_PLL_INPUTREF,    // 配置锁相源输入参考
    WORK_TYPE_END
};

typedef struct
{
    uint8_t type;
    struct rt_work work;
    void (*func)(struct rt_work *, void *);
    void *param;
} work_item_t;

static rtu_modbus_device_t rtu2 = RT_NULL;
static rtu_modbus_device_t rtu3 = RT_NULL;
static rtu_modbus_device_t rtu4 = RT_NULL;
extern rt_device_t pcf8575_dev[2];
static struct rt_workqueue *modbus_workqueue = RT_NULL; // modbus工作队列
static pll_node_attr_t pll_node_attr = {
    .mb_id = DEFAULT_PLL_MB_ID,
};
static channel_attr_t channel_attr[2] = {0};
static rt_timer_t timer = RT_NULL;
static struct _reent r = {0}; // 可重入环境
static decay_compensation_t default_decay_compensation_table[X_CHANNEL_NUM][3] =
    {
        DEFAULT_DECAY_COMPENSATION_TABLE(),
        DEFAULT_DECAY_COMPENSATION_TABLE(),
};
static IF_table_t IF_table[X_CHANNEL_NUM][2] = {
    DEFAULT_IF_SELECT_TABLE(),
    DEFAULT_IF_SELECT_TABLE(),
};

static IF1_table_t IF1_table[X_CHANNEL_NUM][5] = {
    
    DEFAULT_IF1_TABLE(),
    DEFAULT_IF1_TABLE(),
};
channel_attr_t   get_channel_attr(int channel_idx)
{
    if (channel_idx >= sizeof(channel_attr) / sizeof(channel_attr[0]))
    {
        return channel_attr[0];
    }
    return channel_attr[channel_idx];
}

static rt_bool_t check_frequency_condition(uint64_t frequency, uint32_t pfd_min)
{
    uint64_t remainder = frequency % PFD_OFFSET;
    if (remainder < pfd_min || (PFD_OFFSET - remainder) < pfd_min)
    {
        return RT_FALSE;
    }
    return RT_TRUE;
}

//根据输入频率查找最接近的if1_freq
static uint64_t find_closest_if1_freq(int channel_idx, uint64_t freq)
{
    if (freq < INPUT_FREQ_START || freq > INPUT_FREQ_END)
        return 0;
    uint64_t if1_freq = 0;
    size_t table_size = sizeof(IF1_table[channel_idx]) / sizeof(IF1_table[channel_idx][0]);
    for (size_t i = 0; i < table_size; i++)
    {
        if (freq <= IF1_table[channel_idx][i].freq)
        {
            if1_freq = IF1_table[channel_idx][i].if1_freq;
            break;
        }
    }
    return if1_freq;
}


// 查找满足条件的组合，写入 combos，返回写入的数量
static size_t find_valid_combinations(uint64_t freq, Combo *combos, size_t max_combos, IF_table_t IF,uint64_t if1_freq)
{
#define IF1_OFFSET 20000000ULL
    size_t count = 0;
    uint64_t f1, f2;
    uint64_t IF1_MIN = if1_freq - IF1_OFFSET;
    uint64_t IF1_MAX = if1_freq + IF1_OFFSET;
    for (uint64_t IF_freq = IF1_MIN; IF_freq <= IF1_MAX; IF_freq += SCAN_STEP) // 扫描1 MHz
    {
        f1 = (freq + IF_freq) / FREQ_MULTI_FACTOR;
        f2 = IF_freq + IF.IF_freq;
        if (check_frequency_condition(f1, IF.pfd1_min) && check_frequency_condition(f2, IF.pfd2_min))
        {
            if (count < max_combos)
            {
                combos[count].IF_freq = IF_freq;
                combos[count].f1_2[0] = f1;
                combos[count].f1_2[1] = f2;
                count++;
            }
        }
    }
    return count;
}

// 返回最接近 target 的 IF1 的索引
static size_t find_closest_if1_index(const Combo *combos, size_t count, uint64_t target)
{
    if (count == 0)
    {
        return -1;
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

// 根据当前输入频率计算衰减补偿值
static uint8_t channel_get_decay_compensation(int channel_idx, uint64_t freq)
{
    if (freq < INPUT_FREQ_START || freq > INPUT_FREQ_END)
        return 0;
    uint8_t compensation = 0;
    size_t table_size = sizeof(default_decay_compensation_table[channel_idx]) / sizeof(default_decay_compensation_table[channel_idx][0]);
    for (size_t i = 0; i < table_size; i++)
    {
        if (freq <= default_decay_compensation_table[channel_idx][i].freq)
        {
            compensation = default_decay_compensation_table[channel_idx][i].value;
            break;
        }
    }
    return compensation;
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

static void print_channel_info(void)
{
    // 打印通道信息
    for (int i = 0; i < sizeof(channel_attr) / sizeof(channel_attr[0]); i++)
    {
        channel_attr_t *channel = &channel_attr[i];
        LOG_D("channel %d:", i);
        LOG_D("input_freq_hz %llu,decay_value %d,decay_compensation_value %d,if_idx %d,if1_freq %llu", channel->input_freq_hz, channel->decay_value, channel->decay_compensation_value, channel->if_idx, channel->if1_freq);
        LOG_D("IF_freq %llu,pfd1_min %lu,pfd2_min %lu", IF_table[i][channel->if_idx].IF_freq, IF_table[i][channel->if_idx].pfd1_min, IF_table[i][channel->if_idx].pfd2_min);
        pll_node_attr_t *pll_node = channel->pll_node;
        LOG_D("PLL status 0x%04x,temperature %.1f,lock %d,ctrl_reg 0x%04x,input ref signal voltage %dmV ,frequency %llu Hz",
              pll_node->status.val, (float)pll_node->temperature / 10, pll_node->status.PLL_LOCK, pll_node->ctrl_reg_get, pll_node->input_ref_vol, pll_node->freq_get);
        // 打印本振信息
        for (int i = 0; i < sizeof(channel->clk_node) / sizeof(channel->clk_node[0]); i++)
        {
            clk_node_attr_t *node = &channel->clk_node[i];
            LOG_D("clk%d status 0x%04x,temperature %.1f,lock %d,ctrl_reg 0x%04x,spur_param 0x%04x,frequency %llu Hz",
                  i, node->status.val, (float)node->temperature / 10, node->status.CLK_LOCK, node->ctrl_reg_get, node->spur_param_get, node->freq_get);
        }
        // 打印变频器信息
        for (int i = 0; i < sizeof(channel->frecov_node) / sizeof(channel->frecov_node[0]); i++)
        {
            frecov_node_attr_t *node = &channel->frecov_node[i];
            LOG_D("frecov%d status 0x%04x,temperature %.1f,ctrl_reg 0x%04x,decay_set %d,decay_get %d",
                  i, node->status, (float)node->temperature / 10, node->ctrl_reg_get, node->decay_value_set, node->decay_value_get);
        }
    }
}
MSH_CMD_EXPORT(print_channel_info, print all channel info);

// 获取锁相源信息
void work_func0(struct rt_work *work, void *work_data)
{
    uint16_t regs[10] = {0};
    uint32_t buffer[2] = {0};
    pll_node_attr_t *pll_node = (pll_node_attr_t *)work_data;
    if (pll_node && pll_node->channel_handle)
    {
        int ret = modbus_rtu_excuse(pll_node->channel_handle, pll_node->mb_id, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, sizeof(regs) / sizeof(regs[0]), &regs);
        if (ret == MODBUS_RT_EOK)
        {
            pll_node->status.val = regs[0];
            pll_node->ctrl_reg_get.val = regs[1];
            pll_node->temperature = regs[2];
            pll_node->version = regs[3];
            pll_node->input_ref_vol = regs[4];
            buffer[0] = (regs[5] << 16) | regs[6];
            buffer[1] = (regs[7] << 16) | regs[8];
            pll_node->freq_get = ((uint64_t)buffer[0] << 32) | buffer[1];
            pll_node->link_status = 1;
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
        else
        {
            pll_node->link_status = 0;
            LOG_E("get pll info failed");
        }
    }
}

// 获取本振信息
void work_func1(struct rt_work *work, void *work_data)
{
    uint16_t regs[10] = {0};
    uint32_t buffer[2] = {0};
    clk_node_attr_t *clk_node = (clk_node_attr_t *)work_data;
    if (clk_node && clk_node->channel_handle)
    {
        int ret = modbus_rtu_excuse(clk_node->channel_handle, clk_node->mb_id, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, sizeof(regs) / sizeof(regs[0]), &regs);
        if (ret == MODBUS_RT_EOK)
        {
            clk_node->status.val = regs[0];
            clk_node->ctrl_reg_get.val = regs[1];
            clk_node->temperature = regs[2];
            clk_node->version = regs[3];
            clk_node->spur_param_get = regs[4];
            buffer[0] = (regs[5] << 16) | regs[6];
            buffer[1] = (regs[7] << 16) | regs[8];
            clk_node->freq_get = ((uint64_t)buffer[0] << 32) | buffer[1];
            clk_node->link_status = 1;
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
        else
        {
            clk_node->link_status = 0;
            LOG_E("get clk info failed");
        }
    }
}

// 获取变频器信息
void work_func2(struct rt_work *work, void *work_data)
{
    frecov_node_attr_t *frecov_node = (frecov_node_attr_t *)work_data;
    uint16_t regs[10] = {0};
    if (frecov_node && frecov_node->channel_handle)
    {
        int ret = modbus_rtu_excuse(frecov_node->channel_handle, frecov_node->mb_id, AGILE_MODBUS_FC_READ_HOLDING_REGISTERS, 0x0000, sizeof(regs) / sizeof(regs[0]), &regs);
        if (ret == MODBUS_RT_EOK)
        {
            frecov_node->status = regs[0];
            frecov_node->ctrl_reg_get = regs[1];
            frecov_node->temperature = regs[2];
            frecov_node->version = regs[3];
            frecov_node->decay_value_get = regs[4];
            frecov_node->link_status = 1;
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
        else
        {
            frecov_node->link_status = 0;
            LOG_E("get frecov info failed");
        }
    }
}

void work_func3(struct rt_work *work, void *work_data)
{
    clk_node_attr_t *clk_node = (clk_node_attr_t *)work_data;
    if (clk_node && clk_node->channel_handle && clk_node->link_status)
    {
        if (clk_node->freq_set != clk_node->freq_get)
        {
            uint64_t freq_hz = clk_node->freq_set;
            uint16_t regs[4] = {0};
            regs[0] = ((freq_hz >> 48) & 0XFFFF);
            regs[1] = ((freq_hz >> 32) & 0XFFFF);
            regs[2] = ((freq_hz >> 16) & 0XFFFF);
            regs[3] = ((freq_hz >> 0) & 0XFFFF);
            int ret = modbus_rtu_excuse(clk_node->channel_handle, clk_node->mb_id, AGILE_MODBUS_FC_WRITE_MULTIPLE_REGISTERS, 0x0005, 4, &regs);
            if (ret != MODBUS_RT_EOK)
            {
                LOG_E("config clk freq 0x%llu failed", freq_hz);
                return;
            }
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
    }
}

void work_func4(struct rt_work *work, void *work_data)
{
    clk_node_attr_t *clk_node = (clk_node_attr_t *)work_data;
    if (clk_node && clk_node->channel_handle && clk_node->link_status)
    {
        if (clk_node->ctrl_reg_set.val != clk_node->ctrl_reg_get.val)
        {
            int ret = modbus_rtu_excuse(clk_node->channel_handle, clk_node->mb_id, AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0001, 1, &clk_node->ctrl_reg_set.val);
            if (ret != MODBUS_RT_EOK)
            {
                LOG_E("config clk output failed");
                return;
            }
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
    }
}

// 配置变频器衰减
void work_func5(struct rt_work *work, void *work_data)
{
    frecov_node_attr_t *frecov_node = (frecov_node_attr_t *)work_data;
    if (frecov_node && frecov_node->channel_handle && frecov_node->link_status)
    {
        if (frecov_node->decay_value_set != frecov_node->decay_value_get)
        {
            float real_gain = -0.5 * frecov_node->decay_value_set + 30;
            int ret = modbus_rtu_excuse(frecov_node->channel_handle, frecov_node->mb_id, AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0004, 1, &frecov_node->decay_value_set);
            if (ret != MODBUS_RT_EOK)
            {
                LOG_E("set link decay raw(%d) real_gain(%.1fdB) failed", frecov_node->decay_value_set, real_gain);
            }
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
    }
}

// 配置变频器输出通路和使能
void work_func6(struct rt_work *work, void *work_data)
{
    frecov_node_attr_t *frecov_node = (frecov_node_attr_t *)work_data;
    if (frecov_node && frecov_node->channel_handle && frecov_node->link_status)
    {
        if (frecov_node->ctrl_reg_set != frecov_node->ctrl_reg_get)
        {
            int ret = modbus_rtu_excuse(frecov_node->channel_handle, frecov_node->mb_id, AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0001, 1, &frecov_node->ctrl_reg_set);
            if (ret != MODBUS_RT_EOK)
            {
                LOG_E("config frecov ctrl_reg(0x%04x) failed", frecov_node->ctrl_reg_set);
            }
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
    }
}

void work_func7(struct rt_work *work, void *work_data)
{
    pll_node_attr_t *pll_node = (pll_node_attr_t *)work_data;
    if (pll_node && pll_node->channel_handle && pll_node->link_status)
    {
        if (pll_node->ctrl_reg_set.val != pll_node->ctrl_reg_get.val)
        {
            int ret = modbus_rtu_excuse(pll_node->channel_handle, pll_node->mb_id, AGILE_MODBUS_FC_WRITE_SINGLE_REGISTER, 0x0001, 1, &pll_node->ctrl_reg_set);
            if (ret != MODBUS_RT_EOK)
            {
                LOG_E("config pll ctrl_reg value(0x%02x) failed", pll_node->ctrl_reg_set);
            }
            rt_thread_mdelay(RS485_BUS_IDLE_WAITTIME);
        }
    }
}

static work_item_t work_items[X_CHANNEL_NUM][WORK_TYPE_END] = {
    {
        // 通道1
        {WORK_TYPE_GET_PLL_INFO, {0}, work_func0, &pll_node_attr},                // 获取锁相源信息
        {WORK_TYPE_GET_CLK1_INFO, {0}, work_func1, &channel_attr[0].clk_node[0]}, // 获取本振信息
        {WORK_TYPE_GET_CLK2_INFO, {0}, work_func1, &channel_attr[0].clk_node[1]},
        {WORK_TYPE_GET_FRECOVX1_INFO, {0}, work_func2, &channel_attr[0].frecov_node[0]}, // 获取变频器x1信息
        {WORK_TYPE_GET_FRECOVX2_INFO, {0}, work_func2, &channel_attr[0].frecov_node[1]}, // 获取变频器x2信息
        {WORK_TYPE_CONFIG_CLK1_FREQ, {0}, work_func3, &channel_attr[0].clk_node[0]},     // 配置本振频率
        {WORK_TYPE_CONFIG_CLK2_FREQ, {0}, work_func3, &channel_attr[0].clk_node[1]},
        {WORK_TYPE_CONFIG_CLK1_OUTPUT, {0}, work_func4, &channel_attr[0].clk_node[0]},
        {WORK_TYPE_CONFIG_CLK2_OUTPUT, {0}, work_func4, &channel_attr[0].clk_node[1]},
        {WORK_TYPE_CONFIG_FRECOVX2_IF_IDX, {0}, work_func6, &channel_attr[0].frecov_node[1]},
        {WORK_TYPE_CONFIG_FRECOVX2_LDOEN, {0}, work_func6, &channel_attr[0].frecov_node[1]},
        {WORK_TYPE_CONFIG_FRECOVX1_DECAY, {0}, work_func5, &channel_attr[0].frecov_node[0]},
        {WORK_TYPE_CONFIG_FRECOVX1_LDOEN, {0}, work_func6, &channel_attr[0].frecov_node[0]},
        {WORK_TYPE_CONFIG_PLL_INPUTREF, {0}, work_func7, &pll_node_attr},
    },
    {
        // 通道2
        {WORK_TYPE_GET_PLL_INFO, {0}, work_func0, &pll_node_attr},                // 获取锁相源信息
        {WORK_TYPE_GET_CLK1_INFO, {0}, work_func1, &channel_attr[1].clk_node[0]}, // 获取本振信息
        {WORK_TYPE_GET_CLK2_INFO, {0}, work_func1, &channel_attr[1].clk_node[1]},
        {WORK_TYPE_GET_FRECOVX1_INFO, {0}, work_func2, &channel_attr[1].frecov_node[0]}, // 获取变频器x1信息
        {WORK_TYPE_GET_FRECOVX2_INFO, {0}, work_func2, &channel_attr[1].frecov_node[1]}, // 获取变频器x2信息
        {WORK_TYPE_CONFIG_CLK1_FREQ, {0}, work_func3, &channel_attr[1].clk_node[0]},     // 配置本振频率
        {WORK_TYPE_CONFIG_CLK2_FREQ, {0}, work_func3, &channel_attr[1].clk_node[1]},
        {WORK_TYPE_CONFIG_CLK1_OUTPUT, {0}, work_func4, &channel_attr[1].clk_node[0]},
        {WORK_TYPE_CONFIG_CLK2_OUTPUT, {0}, work_func4, &channel_attr[1].clk_node[1]},
        {WORK_TYPE_CONFIG_FRECOVX2_IF_IDX, {0}, work_func6, &channel_attr[1].frecov_node[1]},
        {WORK_TYPE_CONFIG_FRECOVX2_LDOEN, {0}, work_func6, &channel_attr[1].frecov_node[1]},
        {WORK_TYPE_CONFIG_FRECOVX1_DECAY, {0}, work_func5, &channel_attr[1].frecov_node[0]},
        {WORK_TYPE_CONFIG_FRECOVX1_LDOEN, {0}, work_func6, &channel_attr[1].frecov_node[0]},
        {WORK_TYPE_CONFIG_PLL_INPUTREF, {0}, work_func7, &pll_node_attr},

    }};

static work_item_t *get_work_item(int channel_idx, int type)
{
    RT_ASSERT(channel_idx < X_CHANNEL_NUM);
    for (int i = 0; i < WORK_TYPE_END; i++)
    {
        if (work_items[channel_idx][i].type == type)
        {
            return &work_items[channel_idx][i];
        }
    }
    return NULL;
}
// 配置变频器衰减(x1)
rt_err_t config_channel_decay(int channel_idx, uint8_t decay_value)
{
    RT_ASSERT(channel_idx < X_CHANNEL_NUM);
    channel_attr[channel_idx].decay_value = decay_value;
    channel_attr[channel_idx].decay_compensation_value = 0;
    channel_attr[channel_idx].frecov_node[0].decay_value_set = decay_value;
    work_item_t *work_item = get_work_item(channel_idx, WORK_TYPE_CONFIG_FRECOVX1_DECAY);
    if (work_item)
        rt_workqueue_submit_work(modbus_workqueue, &work_item->work, 0);
    return RT_EOK;
}

// 配置通道输出(x1和X2)
rt_err_t config_channel_output(int channel_idx, rt_bool_t output_en)
{
    RT_ASSERT(channel_idx < X_CHANNEL_NUM);
    channel_attr[channel_idx].output_en = output_en;
    frecov_node_ctrl_t *ctrl;
    // ctrl = (frecov_node_ctrl_t *)&channel_attr[channel_idx].frecov_node[0].ctrl_reg_set;
    // ctrl->PA_LDO_EN = RT_TRUE;
    ctrl = (frecov_node_ctrl_t *)&channel_attr[channel_idx].frecov_node[1].ctrl_reg_set;
    ctrl->PA_LDO_EN = channel_attr[channel_idx].output_en;
    channel_attr[channel_idx].clk_node[0].ctrl_reg_set.OUTPUT_EN = channel_attr[channel_idx].output_en;
    channel_attr[channel_idx].clk_node[1].ctrl_reg_set.OUTPUT_EN = channel_attr[channel_idx].output_en;
    work_item_t *work_item[4] = {0};
    work_item[3] = get_work_item(channel_idx, WORK_TYPE_CONFIG_CLK1_OUTPUT);
    work_item[2] = get_work_item(channel_idx, WORK_TYPE_CONFIG_CLK2_OUTPUT);
    // work_item[1] = get_work_item(channel_idx, WORK_TYPE_CONFIG_FRECOVX1_LDOEN);
    work_item[0] = get_work_item(channel_idx, WORK_TYPE_CONFIG_FRECOVX2_LDOEN);
    if (work_item[0])
        rt_workqueue_submit_work(modbus_workqueue, &work_item[0]->work, 0);
    if (work_item[1])
        rt_workqueue_submit_work(modbus_workqueue, &work_item[1]->work, 0);
    if (work_item[2])
        rt_workqueue_submit_work(modbus_workqueue, &work_item[2]->work, 0);
    if (work_item[3])
        rt_workqueue_submit_work(modbus_workqueue, &work_item[3]->work, 0);
    return RT_EOK;
}

// 配置本振频率
rt_err_t config_channel_fre(int channel_idx, uint64_t freq_hz, uint8_t if_idx)
{
    if (channel_idx < 0 || channel_idx >= sizeof(channel_attr) / sizeof(channel_attr[0]))
        return -RT_ERROR;
    if (freq_hz < INPUT_FREQ_START || freq_hz > INPUT_FREQ_END)
        return -RT_ERROR;
    channel_attr[channel_idx].input_freq_hz = freq_hz;
    channel_attr[channel_idx].if1_freq = find_closest_if1_freq(channel_idx, freq_hz);
    size_t closest_idx = 0;
    // 查找所有有效组合
    size_t count = find_valid_combinations(freq_hz, channel_attr[channel_idx].IF_info,
                                           sizeof(channel_attr[channel_idx].IF_info) / sizeof(channel_attr[channel_idx].IF_info[0]), IF_table[channel_idx][if_idx], channel_attr[channel_idx].if1_freq);
    if (count > 0)
    { // 找到最接近目标IF1频率组合的索引
        closest_idx = find_closest_if1_index(channel_attr[channel_idx].IF_info, count, channel_attr[channel_idx].if1_freq);
    }
    else
    {
        LOG_E("channel %d, No valid combinations found for frequency %llu Hz.", channel_idx, freq_hz);
        return -RT_ERROR;
    }

    // 配置本振1和本振2节点频率
    channel_attr[channel_idx].clk_node[0].freq_set = channel_attr[channel_idx].IF_info[closest_idx].f1_2[0];
    work_item_t *work_item = get_work_item(channel_idx, WORK_TYPE_CONFIG_CLK1_FREQ);
    if (work_item)
        rt_workqueue_urgent_work(modbus_workqueue, &work_item->work);
    channel_attr[channel_idx].clk_node[1].freq_set = channel_attr[channel_idx].IF_info[closest_idx].f1_2[1];
    work_item = get_work_item(channel_idx, WORK_TYPE_CONFIG_CLK2_FREQ);
    if (work_item)
        rt_workqueue_urgent_work(modbus_workqueue, &work_item->work);
    //  配置中频
    channel_attr[channel_idx].if_idx = if_idx;
    frecov_node_ctrl_t *ctrl = (frecov_node_ctrl_t *)&channel_attr[channel_idx].frecov_node[1].ctrl_reg_set;
    ctrl->RF_PATH_SEL = channel_attr[channel_idx].if_idx;
    work_item = get_work_item(channel_idx, WORK_TYPE_CONFIG_FRECOVX2_IF_IDX);
    if (work_item)
        rt_workqueue_urgent_work(modbus_workqueue, &work_item->work);
    // 计算当前频率所需要的衰减补偿值
    //  uint8_t compensation = get_decay_compensation(freq_hz, channel_attr[channel_idx].decay_compensation_table,
    //                                                sizeof(channel_attr[channel_idx].decay_compensation_table) / sizeof(channel_attr[channel_idx].decay_compensation_table[0]));
    //  // 配置衰减补偿值
    //  config_channel_link_decay(channel_idx, channel_attr[channel_idx].decay_dev.decay_value_set, compensation);
    char strtemp[32];
    if (channel_idx == 0)
    {
        rt_snprintf(strtemp, sizeof(strtemp), "%llu", freq_hz);
        _setenv_r(&r, "CH1_INPUT_FRE", strtemp, 1);
        _setenv_r(&r, "CH1_IF_LEVEL", __itoa(if_idx, strtemp, 10), 1);
        env_save();
    }
    if (channel_idx == 1)
    {
        rt_snprintf(strtemp, sizeof(strtemp), "%llu", freq_hz);
        _setenv_r(&r, "CH2_INPUT_FRE", strtemp, 1);
        _setenv_r(&r, "CH2_IF_LEVEL", __itoa(if_idx, strtemp, 10), 1);
        env_save();
    }
    return RT_EOK;
}

rt_err_t get_pll_info(void)
{
    work_item_t *work_item = get_work_item(0, WORK_TYPE_GET_PLL_INFO);
    if (work_item)
        return rt_workqueue_submit_work(modbus_workqueue, &work_item->work, 0);
    return -RT_ERROR;
}

// 锁相源输入选择
rt_err_t select_pll_input_ref(uint8_t input_ref)
{
    pll_node_ctrl_t *ctrl = (pll_node_ctrl_t *)&pll_node_attr.ctrl_reg_set;
    ctrl->REF_10M_100M_SEL = input_ref;
    work_item_t *work_item = get_work_item(0, WORK_TYPE_CONFIG_PLL_INPUTREF);
    if (work_item)
        return rt_workqueue_submit_work(modbus_workqueue, &work_item->work, 0);
    return -RT_ERROR;
}

// 请求获取所有节点信息
static void request_get_node_info_timer_callback(void *parameter)
{
    static int init_work_table[] = {
        // WORK_TYPE_GET_PLL_INFO,    //由于硬件锁相源485通信会导致电源不稳，从而导致射频指标异常，所以改为手动触发采集减小影响
        WORK_TYPE_GET_CLK1_INFO,
        WORK_TYPE_GET_CLK2_INFO,
        WORK_TYPE_GET_CLK3_INFO,
        WORK_TYPE_GET_FRECOVX1_INFO,
        WORK_TYPE_GET_FRECOVX2_INFO,
    };

    for (int channel_idx = 0; channel_idx < X_CHANNEL_NUM; channel_idx++)
    {
        for (int i = 0; i < sizeof(init_work_table) / sizeof(init_work_table[0]); i++)
        {
            work_item_t *work_item = get_work_item(channel_idx, init_work_table[i]);
            if (work_item)
                rt_workqueue_submit_work(modbus_workqueue, &work_item->work, 0);
        }
    }
}

// rt_err_t config_clck3_freq(int channel_idx, uint64_t freq_hz)
// {
//     if (channel_idx < 0 || channel_idx >= sizeof(channel_attr) / sizeof(channel_attr[0]))
//         return -RT_ERROR;
//     channel_attr[channel_idx].clk_node[2].freq_set = freq_hz;
//     work_item_t *work_item = get_work_item(WORK_TYPE_CONFIG_CLK3_FREQ);
//     if (work_item)
//         rt_work_submit(&work_item->work, 0);
//     return RT_EOK;
// }

int channel_init(int channel_id, rtu_modbus_device_t rtu, uint64_t input_freq_hz, uint8_t if_idx)
{
    RT_ASSERT(channel_id >= 0 && channel_id < sizeof(channel_attr) / sizeof(channel_attr[0]));

    channel_attr[channel_id].pll_node = &pll_node_attr;
    channel_attr[channel_id].frecov_node[0].mb_id = DEFAULT_FRECOVX1_MB_ID;
    channel_attr[channel_id].frecov_node[1].mb_id = DEFAULT_FRECOVX2_MB_ID;
    channel_attr[channel_id].clk_node[0].mb_id = DEFAULT_CLK1_MB_ID;
    channel_attr[channel_id].clk_node[1].mb_id = DEFAULT_CLK2_MB_ID;
    channel_attr[channel_id].frecov_node[0].channel_handle = rtu;
    channel_attr[channel_id].frecov_node[1].channel_handle = rtu;
    channel_attr[channel_id].clk_node[0].channel_handle = rtu;
    channel_attr[channel_id].clk_node[1].channel_handle = rtu;
    channel_attr[channel_id].input_freq_hz = input_freq_hz;
    channel_attr[channel_id].if_idx = if_idx;
    channel_attr[channel_id].if1_freq = find_closest_if1_freq(channel_id, input_freq_hz);
    channel_attr[channel_id].decay_value = MAX_DECAY_VALUE;
    channel_attr[channel_id].decay_compensation_value = channel_get_decay_compensation(channel_id, input_freq_hz);
    channel_attr[channel_id].output_en = RT_FALSE; // 默认关闭输出
    return RT_EOK;
}

int controler_init(void)
{
    work_item_t *work_item;
    rtu2 = modbus_rtu_master_init(COM2_PORT_NAME); //-- 锁相源
    RT_ASSERT(rtu2 != NULL);
    rtu3 = modbus_rtu_master_init(COM3_PORT_NAME); // 通道1 --本振 + 变频
    RT_ASSERT(rtu3 != NULL);
    rtu4 = modbus_rtu_master_init(COM4_PORT_NAME); // 通道2  --本振 + 变频
    RT_ASSERT(rtu4 != NULL);
    modbus_workqueue = rt_workqueue_create("modbus_wq", 4096, MODBUS_THREAD_PRIO - 1);
    RT_ASSERT(modbus_workqueue);
    // 初始化所有工作项
    for (int i = 0; i < sizeof(work_items) / sizeof(work_items[0]); i++)
    {
        for (int j = 0; j < sizeof(work_items[i]) / sizeof(work_items[i][0]); j++)
        {
            work_item = &work_items[i][j];
            if (work_item->func && work_item->param)
                rt_work_init(&work_item->work, work_item->func, work_item->param);
        }
    }
    // 锁相源初始化
    pll_node_attr.channel_handle = rtu2;
    pll_node_attr.mb_id = DEFAULT_PLL_MB_ID;
    get_pll_info(); //  启动锁相源采集一次
    // 通道1初始化
    uint64_t input_freq_hz = atoll(_getenv_r(&r, "CH1_INPUT_FRE"));
    uint8_t if_idx = atoi(_getenv_r(&r, "CH1_IF_LEVEL"));
    channel_init(0, rtu3, input_freq_hz, if_idx);
    // 通道2初始化
    input_freq_hz = atoll(_getenv_r(&r, "CH2_INPUT_FRE"));
    if_idx = atoi(_getenv_r(&r, "CH2_IF_LEVEL"));
    channel_init(1, rtu4, input_freq_hz, if_idx);
    // 启动定时采集任务
    timer = rt_timer_create("dtu_timer", request_get_node_info_timer_callback, RT_NULL, rt_tick_from_millisecond(1000), RT_TIMER_FLAG_PERIODIC);
    RT_ASSERT(timer);
    rt_timer_start(timer);
    // 等待采集一次数据,配置通道衰减值,频率,输出使能到默认状态
    rt_thread_mdelay(2000);
    for (int channel_id = 0; channel_id < X_CHANNEL_NUM; channel_id++)
    {
        config_channel_decay(channel_id, channel_attr[channel_id].decay_value);
        config_channel_fre(channel_id, channel_attr[channel_id].input_freq_hz, channel_attr[channel_id].if_idx);
        config_channel_output(channel_id, channel_attr[channel_id].output_en);
    }
    return 0;
}
// INIT_FS_EXPORT(controler_init);
#endif