#ifndef __X_DOWN_H__
#define __X_DOWN_H__

#include "modbus_rtu.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if defined BSP_USING_X_DOWN


#define X_CHANNEL_NUM (2)
typedef union
{
    struct
    {
        uint16_t CLK_LOCK : 1;
        uint16_t RESET_CODE : 15;
    };
    uint16_t val;
} clk_node_status_t;
typedef union
{
    struct
    {
        uint16_t OUTPUT_EN : 1; // 与本振节点定义保持一致
        uint16_t RESERVED : 15;
    };
    uint16_t val;
} clk_node_ctrl_t;
// 本振节点属性
typedef struct
{
    uint16_t mb_id;
    rtu_modbus_device_t channel_handle;
    clk_node_status_t status;
    clk_node_ctrl_t ctrl_reg_get;
    clk_node_ctrl_t ctrl_reg_set;
    uint16_t temperature;
    uint16_t version;
    uint16_t spur_param_set;
    uint16_t spur_param_get;
    uint64_t freq_set;
    uint64_t freq_get;
    rt_bool_t link_status;
} clk_node_attr_t;

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
    pll_node_ctrl_t ctrl_reg_set;
    pll_node_ctrl_t ctrl_reg_get;
    uint16_t temperature;
    uint16_t version;
    uint16_t input_ref_vol;
    uint64_t freq_set;
    uint64_t freq_get;
    rt_bool_t link_status;
} pll_node_attr_t;

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
    uint16_t ctrl_reg_set;
    uint16_t ctrl_reg_get;
    uint16_t temperature;
    uint16_t version;
    uint16_t decay_value_set;
    uint16_t decay_value_get;
    rt_bool_t link_status;
} frecov_node_attr_t;

// 二次变频频率组合定义
typedef struct
{
    uint64_t IF_freq; // Hz
    uint64_t f1_2[2]; // Hz
} Combo;


// 2中频属性定义
typedef struct
{
    uint64_t IF_freq;  // Hz
    uint32_t pfd1_min; // Hz  距离PFD频率的最小距离,优化杂散
    uint32_t pfd2_min; // Hz
    /* data */
} IF_table_t;
#define DEFAULT_IF_SELECT_TABLE()             \
{                                             \
    {720000000ULL, 10000000ULL, 15000000ULL}, \
    {1200000000ULL, 10000000ULL, 15000000ULL},       \
}
// 定义一个衰减补偿表给后级衰减使用
typedef struct
{
    uint64_t freq;
    uint8_t value;
} decay_compensation_t; // 衰减补偿

#define DEFAULT_DECAY_COMPENSATION_TABLE() \
{                                          \
    {24000000000ULL, 0},                   \
    {25000000000ULL, 0},                   \
    {25500000000ULL, 0}                    \
}

typedef struct
{
    uint64_t freq;
    uint64_t if1_freq;
} IF1_table_t;

#define DEFAULT_IF1_TABLE()         \
{                                   \
    {7100000000ULL, 2900000000ULL}, \
    {7230000000ULL, 3000000000ULL}, \
    {7300000000ULL, 3100000000ULL}, \
    {8000000000ULL, 3100000000ULL}, \
    {9000000000ULL, 3100000000ULL}, \
}

typedef struct
{
    uint64_t input_freq_hz;
    uint8_t if_idx;
    uint64_t if1_freq;
    uint8_t decay_compensation_value;
    uint8_t decay_value;
    Combo IF_info[128];
    pll_node_attr_t *pll_node;
    frecov_node_attr_t frecov_node[2];
    clk_node_attr_t clk_node[2];
    rt_bool_t output_en;
} channel_attr_t;


channel_attr_t get_channel_attr(int channel_idx);
rt_err_t config_channel_fre(int channel_idx, uint64_t freq_hz, uint8_t if_idx);
rt_err_t config_channel_decay(int channel_idx, uint8_t decay_value);
rt_err_t config_channel_output(int channel_idx, rt_bool_t output_en);
rt_err_t select_pll_input_ref(uint8_t input_ref);
rt_err_t get_pll_info(void);
int controler_init(void);
#endif

#ifdef __cplusplus
}
#endif
#endif
