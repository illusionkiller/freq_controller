# 中频模块频率的范围（）
IF_MIN = 4990000000  # 6000 MHz ± 20 MHz -> 5980 MHz 
IF_MAX = 5010000000  # 6020 MHz 
SCAN_STEP = 1000000  # 2 MHz
PFD1_MIN = 10000000  
PFD2_MIN = 10000000  

PFD_OFFSET = 100000000  # 100 MHz偏移量

# 计算本振1和本振2的频率 
def calculate_oscillators(input_freq, IF_freq):
    # 本振1频率
    f1 = (input_freq + IF_freq) / 2
    # 本振2频率
    f2 = IF_freq + 720000000  
    return f1, f2

# 检查f1和f2是否与100MHz的整数倍距离小于20MHz
def check_frequency_condition1(frequency):
    # 计算与最近的100 MHz整数倍的差值
    remainder = frequency % PFD_OFFSET
    # 判断该频率与最近的 100 MHz 的整数倍之间的差距
    if remainder < PFD1_MIN or (PFD_OFFSET - remainder) < PFD1_MIN:
        return False  # 如果距离小于20MHz，则返回 False
    return True

def check_frequency_condition2(frequency):
    # 计算与最近的100 MHz整数倍的差值
    remainder = frequency % PFD_OFFSET
    # 判断该频率与最近的 100 MHz 的整数倍之间的差距
    if remainder < PFD2_MIN or (PFD_OFFSET - remainder) < PFD2_MIN:
        return False  # 如果距离小于20MHz，则返回 False
    return True

# 给定输入频率，输出满足条件的中频频率（IF_freq）、本振1（f1）、本振2（f2）
def find_valid_combinations(input_freq):
    valid_combinations = []
    
    # 遍历中频模块频率范围
    for IF_freq in range(IF_MIN, IF_MAX + 1, SCAN_STEP):  # 步进2 MHz
        # 计算本振1和本振2频率
        f1, f2 = calculate_oscillators(input_freq, IF_freq)
        
        # 检查是否满足频率条件：f1 和 f2 分别与 100 MHz 的整数倍距离
        # https://www.analog.com/cn/resources/analog-dialogue/articles/analyzing-integer-boundary-spurs.html
        if check_frequency_condition1(f1) and check_frequency_condition2(f2):
            valid_combinations.append((IF_freq, f1, f2))  # 输出频率以 Hz 为单位

    return valid_combinations

# 自动步进扫描频率范围
def auto_scan_frequencies():
    # 频率范围和步进
    start_freq = 17000000000  # 17 GHz
    end_freq = 28000000000    # 28 GHz
    step_freq = 1000          # 1 kHz步进
    
    # 结果存储
    results = []
    not_found_frequencies = []
    
    # 计算总步数
    total_steps = (end_freq - start_freq) // step_freq + 1
    current_step = 0
    
    # 扫描所有频率
    for input_freq in range(start_freq, end_freq + 1, step_freq):
        # 更新进度
        current_step += 1
        progress_percent = (current_step / total_steps) * 100
        
        # 每1%或每1000步更新一次进度显示
        if current_step % 1000 == 0 or current_step == 1 or current_step == total_steps:
            print(f"\r扫描进度: {progress_percent:.2f}% ({current_step}/{total_steps}) - 当前频率: {input_freq/1000000:.3f} MHz", end="")
        
        valid_combinations = find_valid_combinations(input_freq)
        
        if valid_combinations:
            # 找到最接近6000000000Hz的IF_freq
            closest_IF_freq = min(valid_combinations, key=lambda x: abs(x[0] - 6000000000))
            results.append((input_freq, closest_IF_freq))
        else:
            not_found_frequencies.append(input_freq)
    
    # 完成后换行
    print()
    return results, not_found_frequencies

# 主程序
if __name__ == "__main__":
    print("开始自动扫描频率范围 (17 GHz 到 28 GHz)，步进为 1 kHz...")
    results, not_found_frequencies = auto_scan_frequencies()
    
    # 输出结果
    print(f"\n找到有效组合的频率数量: {len(results)}")
    print(f"未找到有效组合的频率数量: {len(not_found_frequencies)}")
    
    # 保存结果到文件
    with open("frequency_scan_results.txt", "w") as f:
        f.write("输入频率(Hz),IF频率(Hz),f1(Hz),f2(Hz)\n")
        for input_freq, combo in results:
            f.write(f"{input_freq},{combo[0]},{combo[1]},{combo[2]}\n")
    
    # 保存未找到有效组合的频率
    with open("not_found_frequencies.txt", "w") as f:
        f.write("输入频率(Hz)\n")
        for freq in not_found_frequencies:
            f.write(f"{freq}\n")
    
    print("\n结果已保存到 frequency_scan_results.txt 和 not_found_frequencies.txt 文件中")
    
    # 打印前10个结果作为示例
    if results:
        print("\n前10个结果示例:")
        for i, (input_freq, combo) in enumerate(results[:10]):
            print(f"输入频率: {input_freq} Hz, IF频率: {combo[0]} Hz, f1: {combo[1]} Hz, f2: {combo[2]} Hz")
    
    # 打印前10个未找到有效组合的频率
    if not_found_frequencies:
        print("\n前10个未找到有效组合的频率示例:")
        for freq in not_found_frequencies[:10]:
            print(f"{freq} Hz")
