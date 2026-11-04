# 中频模块频率的范围（）
IF_MIN = 5980000000  # 6000 MHz ± 20 MHz -> 5980 MHz 
IF_MAX = 6020000000  # 6020 MHz 

PFD1_MIN = 5000000 
PFD2_MIN = 10000000  
PFD_OFFSET = 100000000  # 100 MHz偏移量

# 计算本振1和本振2的频率
def calculate_oscillators(input_freq, IF_freq):
    # 本振1频率
    f1 = (input_freq + IF_freq) / 3 
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
    for IF_freq in range(IF_MIN, IF_MAX + 1, 1000000):  # 步进1 MHz
        # 计算本振1和本振2频率
        f1, f2 = calculate_oscillators(input_freq, IF_freq)
        
        # 检查是否满足频率条件：f1 和 f2 分别与 100 MHz 的整数倍距离至少 20 MHz
        if check_frequency_condition1(f1) and check_frequency_condition2(f2):
            valid_combinations.append((IF_freq, f1, f2))  # 输出频率以 Hz 为单位

    return valid_combinations

# 从控制台获取输入频率
def get_input_frequency():
    while True:
        try:
            input_freq = int(input("请输入输入频率 (17000000000 Hz 到 28000000000 Hz): "))
            if 17000000000 <= input_freq <= 28000000000:
                return input_freq
            else:
                print("输入频率超出范围，请输入 17000000000 Hz 到 28000000000 Hz 之间的值。")
        except ValueError:
            print("无效输入，请输入一个有效的整数。")

# 主程序
if __name__ == "__main__":
    input_freq = get_input_frequency()

    valid_combinations = find_valid_combinations(input_freq)

    # 输出有效组合
    if valid_combinations:
        print(f"Valid frequency combinations for input frequency {input_freq} Hz:")
        for combo in valid_combinations:
            print(f"IF_freq: {combo[0]} kHz, f1: {combo[1]} kHz, f2: {combo[2]} Hz")
        #打印IF_freq 最接近6000000000Hz 对应的f1 和f2
        closest_IF_freq = min(valid_combinations, key=lambda x: abs(x[0] - 6000000000))
        print(f"Closest IF_freq to 6000000000 Hz: {closest_IF_freq[0]} Hz, f1: {closest_IF_freq[1]} Hz, f2: {closest_IF_freq[2]} Hz")
    else:
        print(f"No valid combinations found for input frequency {input_freq} Hz.")
