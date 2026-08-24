/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-03-15     RT-Thread    First version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

#include "env.h"

#define DBG_TAG "env_cmd"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static struct _reent r = {0}; // 可重入环境
/**
 * @brief Console command "getenv" implementation (U-Boot style)
 * Get the value of an environment variable
 */
static int getenv_cmd(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_RAW("Usage: getenv <key>\n");
        return 1;
    }

    char *value = _getenv_r(&r, argv[1]);
    if (value != RT_NULL)
    {
        LOG_RAW("%s\n", value);
        return 0;
    }
    else
    {
        LOG_RAW("Error: Environment variable not found: %s\n", argv[1]);
        return 1;
    }
}
MSH_CMD_EXPORT_ALIAS(getenv_cmd, getenv, get environment variable);

/**
 * @brief Console command "setenv" implementation (U-Boot style)
 * Set the value of an environment variable
 */
static int setenv_cmd(int argc, char **argv)
{
    if (argc != 3)
    {
        LOG_RAW("Usage: setenv <key> <value>\n");
        return 1;
    }

    if (_setenv_r(&r, argv[1], argv[2], 1) != 0)
    {
        LOG_RAW("Error: Failed to set environment variable %s\n", argv[1]);
        return 1;
    }

    LOG_RAW("setenv %s=%s\n", argv[1], argv[2]);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(setenv_cmd, setenv, set environment variable);

/**
 * @brief Console command "delenv" implementation (U-Boot style)
 * Delete an environment variable
 */
static int unsetenv_cmd(int argc, char **argv)
{
    if (argc != 2)
    {
        LOG_RAW("Usage: unsetenv <key>\n");
        return 1;
    }

    if (_unsetenv_r(&r, argv[1]) != 0)
    {
        LOG_RAW("Error: Failed to delete environment variable %s\n", argv[1]);
        return 1;
    }

    LOG_RAW("Deleted environment variable: %s\n", argv[1]);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(unsetenv_cmd, unsetenv, delete environment variable);

/**
 * @brief Console command "saveenv" implementation (U-Boot style)
 * Save environment variables to file system
 */
static int saveenv_cmd(int argc, char **argv)
{
    env_save();
    return 0;
}
MSH_CMD_EXPORT_ALIAS(saveenv_cmd, saveenv, save environment variable);

/**
 * @brief Console command "loadenv" implementation (U-Boot style)
 * Load environment variables from file system
 */
static int loadenv_cmd(int argc, char **argv)
{
    env_load();
    return 0;
}
MSH_CMD_EXPORT_ALIAS(loadenv_cmd, loadenv, load environment variable);

/**
 * @brief Console command "printenv" implementation (U-Boot style)
 * Print all environment variables
 */
static int printenv_cmd(int argc, char **argv)
{
    return env_print();
}
MSH_CMD_EXPORT_ALIAS(printenv_cmd, printenv, print environment variable);