#include "env.h"

#define DBG_TAG "env"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
// 环境变量全局缓存，用于比较是否发生变化
static char **env_cache = NULL;
static int env_cache_count = 0;
static struct _reent r = {0}; // 可重入环境
static void update_env_cache(void);

// 默认环境变量表，方便集中管理和修改
static const struct
{
    const char *key;
    const char *value;
} default_env_vars[] = DEFAULT_ENV_PARAM();

/**
 * @brief 初始化环境变量模块
 * @return 0成功，负数失败
 */
int env_init(void)
{
    memset(&r, 0, sizeof(r)); // 初始化可重入环境
    // 先设置默认环境变量
    int i;
    for (i = 0; i < sizeof(default_env_vars) / sizeof(default_env_vars[0]); i++)
    {
        _setenv_r(&r, default_env_vars[i].key, default_env_vars[i].value, 1);
    }

    // 再尝试从文件系统加载环境变量，文件系统中的变量会覆盖默认值
    env_load();

    // 初始化缓存
    if (env_cache == NULL)
    {
        update_env_cache();
    }

    return 0;
}
// INIT_FS_EXPORT(env_init);
/**
 * @brief 初始化环境变量迭代器
 * @param iter 迭代器指针
 */
void env_iter_init(env_iterator_t *iter)
{
    extern char **environ;
    iter->current = environ;
}

/**
 * @brief 获取下一个环境变量的键和值
 * @param iter 迭代器指针
 * @param key 输出参数，用于存储键名指针
 * @param key_len 输出参数，用于存储键名长度
 * @param value 输出参数，用于存储值指针
 * @param value_len 输出参数，用于存储值长度
 * @return 0表示成功获取到环境变量，-1表示已到达末尾
 * @note 此函数不会修改原始环境变量字符串
 */
int env_iter_next(env_iterator_t *iter, char **key, size_t *key_len, char **value, size_t *value_len)
{
    char *line = *iter->current;
    char *equals_pos;

    if (line == NULL)
    {
        return -1; // 已到达末尾
    }

    // 查找等号位置
    equals_pos = strchr(line, '=');
    if (equals_pos == NULL)
    {
        // 没有等号的情况
        *key = line;
        *key_len = strlen(line);
        *value = NULL;
        *value_len = 0;
    }
    else
    {
        // 计算键名长度
        *key_len = equals_pos - line;
        *key = line;

        // 计算值长度
        *value = equals_pos + 1;
        *value_len = strlen(*value);
    }

    // 移动迭代器到下一个位置
    iter->current++;

    return 0;
}

/**
 * @brief 获取下一个完整的环境变量字符串
 * @param iter 迭代器指针
 * @return 环境变量字符串，NULL表示已到达末尾
 */
char *env_iter_next_full(env_iterator_t *iter)
{
    char *line = *iter->current;

    if (line == NULL)
    {
        return NULL; // 已到达末尾
    }

    // 移动迭代器到下一个位置
    iter->current++;

    return line;
}

/**
 * @brief 释放环境变量缓存
 */
static void free_env_cache(void)
{
    if (env_cache != NULL)
    {
        for (int i = 0; i < env_cache_count; i++)
        {
            if (env_cache[i] != NULL)
            {
                rt_free(env_cache[i]);
            }
        }
        rt_free(env_cache);
        env_cache = NULL;
        env_cache_count = 0;
    }
}

/**
 * @brief 更新环境变量缓存
 */
static void update_env_cache(void)
{
    // 释放旧缓存
    free_env_cache();

    // 计算环境变量数量
    env_iterator_t iter;
    char *line;
    int count = 0;

    env_iter_init(&iter);
    while ((line = env_iter_next_full(&iter)) != NULL)
    {
        count++;
    }

    if (count == 0)
    {
        return;
    }

    // 分配新缓存
    env_cache = (char **)rt_malloc(sizeof(char *) * count);
    if (env_cache == NULL)
    {
        LOG_E("Failed to allocate env cache");
        return;
    }

    // 填充缓存
    env_cache_count = count;
    count = 0;

    env_iter_init(&iter);
    while ((line = env_iter_next_full(&iter)) != NULL)
    {
        env_cache[count] = rt_strdup(line);
        if (env_cache[count] == NULL)
        {
            LOG_E("Failed to duplicate env line");
            free_env_cache();
            return;
        }
        count++;
    }
}

/**
 * @brief 检查环境变量是否发生变化
 * @return 1表示发生变化，0表示未变化
 */
static int is_env_changed(void)
{
    if (env_cache == NULL)
    {
        return 1; // 缓存未初始化，认为发生变化
    }

    // 比较环境变量数量
    env_iterator_t iter;
    char *line;
    int count = 0;

    env_iter_init(&iter);
    while ((line = env_iter_next_full(&iter)) != NULL)
    {
        count++;
    }

    if (count != env_cache_count)
    {
        return 1; // 数量变化
    }

    // 比较每个环境变量
    int i = 0;
    env_iter_init(&iter);
    while ((line = env_iter_next_full(&iter)) != NULL)
    {
        if (strcmp(line, env_cache[i]) != 0)
        {
            return 1; // 内容变化
        }
        i++;
    }

    return 0; // 未发生变化
}

/**
 * @brief 保存环境变量到文件系统
 * @return 0成功，负数失败
 */
int env_save(void)
{
    int fd;
    env_iterator_t iter;
    char *line;

    // 检查环境变量是否发生变化
    if (!is_env_changed())
    {
        // LOG_I("Environment variables unchanged, skipping save");
        return 0;
    }

    // 有变化，执行写入操作
    LOG_I("Environment variables changed, saving to file");

    // 打开文件，创建或截断
    fd = open(ENV_FILE_PATH, O_WRONLY | O_CREAT);
    if (fd < 0)
    {
        LOG_E("Failed to open environment file for writing: %d", errno);
        return -1;
    }

    // 使用迭代器遍历环境变量表，写入所有环境变量
    env_iter_init(&iter);
    while ((line = env_iter_next_full(&iter)) != NULL)
    {
        int len = strlen(line);
        if (write(fd, line, len) < len)
        {
            LOG_E("Failed to write environment variable to file: %d", errno);
            close(fd);
            return -1;
        }
        // 写入换行符
        if (write(fd, "\n", 1) < 1)
        {
            LOG_E("Failed to write newline to environment file: %d", errno);
            close(fd);
            return -1;
        }
    }

    // 关闭文件
    if (close(fd) < 0)
    {
        LOG_E("Failed to close environment file: %d", errno);
        return -1;
    }

    // 更新缓存
    update_env_cache();

    LOG_I("Environment variables saved successfully");
    return 0;
}

/**
 * @brief 从文件系统加载环境变量
 * @return 0成功，负数失败
 */
int env_load(void)
{
    int fd;
    int read_len;
    static char buffer[4096];
    char *line;
    char *saveptr1, *saveptr2;

    // 打开环境变量文件
    fd = open(ENV_FILE_PATH, O_RDONLY);
    if (fd < 0)
    {
        if (errno == ENOENT)
        {
            LOG_I("Environment file not found, using default values");
            return 0;
        }
        else
        {
            LOG_E("Failed to open environment file: %d", errno);
            return -1;
        }
    }

    // 读取文件内容
    read_len = read(fd, buffer, sizeof(buffer) - 1);
    if (read_len < 0)
    {
        LOG_E("Failed to read environment file: %d", errno);
        close(fd);
        return -1;
    }

    // 添加null终止符
    buffer[read_len] = '\0';

    // 关闭文件
    if (close(fd) < 0)
    {
        LOG_E("Failed to close environment file: %d", errno);
        // 继续处理已读取的内容
    }

    // 解析文件内容，设置环境变量
    line = strtok_r(buffer, "\n", &saveptr1);
    while (line != NULL)
    {
        // 跳过空行和注释行
        if (*line == '\0' || *line == '#')
        {
            line = strtok_r(NULL, "\n", &saveptr1);
            continue;
        }

        // 直接使用原始格式设置环境变量
        char *key = strtok_r(line, "=", &saveptr2);
        char *value = strtok_r(NULL, "", &saveptr2);
        if (key != NULL && value != NULL)
        {
            _setenv_r(&r, key, value, 1);
        }

        // 获取下一行
        line = strtok_r(NULL, "\n", &saveptr1);
    }

    // 更新缓存
    update_env_cache();

    LOG_I("Environment variables loaded successfully");
    return 0;
}

/**
 * @brief 打印所有环境变量
 * @return 0成功，负数失败
 */
int env_print(void)
{
    env_iterator_t iter;
    char *line;
    int count = 0;

    // 使用迭代器遍历环境变量列表
    env_iter_init(&iter);
    while ((line = env_iter_next_full(&iter)) != NULL)
    {
        LOG_RAW("%s\n", line);
        count++;
    }

    if (count == 0)
    {
        LOG_RAW("No environment variables found\n");
    }

    LOG_RAW("Total: %d environment variables\n", count);
    return 0;
}

/**
 * @brief 清除所有存储的环境变量
 * @return 0成功，负数失败
 */
int env_clean(void)
{
    env_iterator_t iter;
    char *key_ptr;
    size_t key_len;
    char *value_ptr;
    size_t value_len;
    char key_buf[256]; // 临时缓冲区，用于存储null终止的键名

    // 使用迭代器遍历并清除所有环境变量
    env_iter_init(&iter);
    while (env_iter_next(&iter, &key_ptr, &key_len, &value_ptr, &value_len) == 0)
    {
        if (key_ptr != NULL && key_len > 0)
        {
            // 确保键名长度不超过缓冲区大小
            if (key_len >= sizeof(key_buf))
            {
                LOG_E("Environment variable key too long");
                continue;
            }

            // 复制键名到临时缓冲区并添加null终止符
            memcpy(key_buf, key_ptr, key_len);
            key_buf[key_len] = '\0';

            // 清除环境变量
            _unsetenv_r(&r, key_buf);
        }
    }

    // 删除文件系统中的环境变量文件
    if (remove(ENV_FILE_PATH) < 0 && errno != ENOENT)
    {
        LOG_E("Failed to remove environment file: %d", errno);
        return -1;
    }

    // 清除后更新缓存
    update_env_cache();

    LOG_I("Environment variables cleaned successfully");
    return 0;
}

/**
 * @brief 恢复默认环境变量
 * @return 0成功，负数失败
 */
int env_restore_default(void)
{
    int i;
    env_clean();

    // 重新设置默认环境变量
    for (i = 0; i < sizeof(default_env_vars) / sizeof(default_env_vars[0]); i++)
    {
        _setenv_r(&r, default_env_vars[i].key, default_env_vars[i].value, 1);
    }

    // 恢复后更新缓存
    update_env_cache();

    LOG_I("Environment variables restored to default values");
    return 0;
}