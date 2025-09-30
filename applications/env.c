/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */


#include "env.h"

#define DBG_TAG "env"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

sys_reg_t sys_reg = {
    .IP = 0x0a01a8c0, // 192.168.1.10
    .IP_MASK = 0x00ffffff,
    .GATE_WAY = 0x0101a8c0,
    .SN = DTU_DEVICE_DEFAULT_SN,
    .LW1011 = 0,
#if defined DEVICE_FRE_CONTROL_RANGE_KA
    .CH1_INPUT_FRE = 17000000,
    .CH2_INPUT_FRE = 17000000,
#elif defined DEVICE_FRE_CONTROL_RANGE_Q
#elif defined DEVICE_FRE_CONTROL_RANGE_X
    .CH1_INPUT_FRE = 7000000,
    .CH2_INPUT_FRE = 7000000,
#endif
    .LW1016 = 0,
};

static struct fdb_kvdb kvdb = {0};
static struct rt_mutex lock;
static struct fdb_default_kv_node default_kv_table[] = {
    {"sys_reg", &sys_reg, sizeof(sys_reg_t)},
};

static void kvdb_lock(fdb_db_t db)
{
    rt_mutex_take(&lock, RT_WAITING_FOREVER);
}

static void kvdb_unlock(fdb_db_t db)
{
    rt_mutex_release(&lock);
}

static int env_print(uint8_t argc, char **argv)
{
    struct fdb_blob blob;
    sys_reg_t regs = {0};
    fdb_kv_print(&kvdb);
    fdb_kv_get_blob(&kvdb, "sys_reg", fdb_blob_make(&blob, &regs, sizeof(regs)));
    if (blob.saved.len > 0)
    {
        LOG_D("IP:%d.%d.%d.%d\nMASK:%d.%d.%d.%d\nGW:%d.%d.%d.%d\nCH1_INPUT_FREQ:%d\nCH2_INPUT_FREQ:%d\nSN:%s\nLW1011:%d\nLW1016:%d",
              (regs.IP >> 0) & 0xff, (regs.IP >> 8) & 0xff, (regs.IP >> 16) & 0xff, (regs.IP >> 24) & 0xff,
              (regs.IP_MASK >> 0) & 0xff, (regs.IP_MASK >> 8) & 0xff, (regs.IP_MASK >> 16) & 0xff, (regs.IP_MASK >> 24) & 0xff,
              (regs.GATE_WAY >> 0) & 0xff, (regs.GATE_WAY >> 8) & 0xff, (regs.GATE_WAY >> 16) & 0xff, (regs.GATE_WAY >> 24) & 0xff,
              regs.CH1_INPUT_FRE,
              regs.CH2_INPUT_FRE, regs.SN, regs.LW1011, regs.LW1016);
        return 0;
    }
    return -1;
}
MSH_CMD_EXPORT_ALIAS(env_print, env_print, print env saved in env);

rt_err_t save_sys_regs(void)
{
    struct fdb_blob blob;
    fdb_err_t ret = fdb_kv_set_blob(&kvdb, "sys_reg", fdb_blob_make(&blob, &sys_reg, sizeof(sys_reg)));
    if(ret != FDB_NO_ERR)
    {
        LOG_E("save sys regs failed,err(%d)",ret);
        return -RT_ERROR;
    }
    return RT_EOK;
}

rt_err_t sn_set(int argc, char **argv)
{
    if(argc != 2)
    {
        return -RT_ERROR;
    }
    rt_strncpy(sys_reg.SN,argv[1],DTU_DEVICE_SN_MAX_LEN-1);
    sys_reg.SN[DTU_DEVICE_SN_MAX_LEN-1] = '\0';
    return save_sys_regs();
}
MSH_CMD_EXPORT_ALIAS(sn_set, sn_set, set sn and save in env);

rt_err_t env_default(void)
{
    return fdb_kv_set_default(&kvdb);
}
MSH_CMD_EXPORT_ALIAS(env_default, env_default, set env to default);

rt_err_t env_clean(void)
{
    return fdb_kv_clean(&kvdb);
}
MSH_CMD_EXPORT_ALIAS(env_clean, env_clean, clean env);

static int env_kvdb_init(void)
{
#define dir_path "spiflash1"
    struct fdb_blob blob;
    fdb_err_t result;
    // bool file_mode = true;
    // bool not_formatable = true;
    // uint32_t sec_size = 4096;
    // uint32_t db_size = sec_size * 32;
    struct fdb_default_kv default_kv;
    default_kv.kvs = default_kv_table;
    default_kv.num = sizeof(default_kv_table) / sizeof(default_kv_table[0]);
#ifdef FDB_KV_AUTO_UPDATE
    kvdb.ver_num = 1;
#endif
    rt_mutex_init(&lock, "env", RT_IPC_FLAG_FIFO);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_LOCK, kvdb_lock);
    fdb_kvdb_control(&kvdb, FDB_KVDB_CTRL_SET_UNLOCK, kvdb_unlock);
    result = fdb_kvdb_init(&kvdb, "env", dir_path, &default_kv, NULL);
    if (result != FDB_NO_ERR)
    {
        return -RT_ERROR;
    }
    fdb_kv_get_blob(&kvdb, "sys_reg", fdb_blob_make(&blob, &sys_reg, sizeof(sys_reg)));
    return RT_EOK;
}
INIT_APP_EXPORT(env_kvdb_init);
