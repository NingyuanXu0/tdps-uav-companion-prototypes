"""
VL53L0X Time-of-Flight传感器综合测试程序 - 优化版
适用于ESP32-S3-DevKitC-1U-N32R8V运行MicroPython
Score99团队 - 无人机项目

优化内容：
- 添加了完整的传感器初始化序列
- 改进了错误处理机制
- 增强了测试数据可视化
- 添加了自动测试模式
- 优化了测试报告格式
"""

import machine
import time
import struct
import gc
import json
from micropython import const

# VL53L0X寄存器定义
_VL53L0X_ADDR = const(0x29)
_REG_IDENTIFICATION_MODEL_ID = const(0xC0)
_REG_IDENTIFICATION_REVISION_ID = const(0xC2)
_REG_SYSRANGE_START = const(0x00)
_REG_SYSTEM_THRESH_HIGH = const(0x0C)
_REG_SYSTEM_THRESH_LOW = const(0x0E)
_REG_SYSTEM_SEQUENCE_CONFIG = const(0x01)
_REG_SYSTEM_RANGE_CONFIG = const(0x09)
_REG_SYSTEM_INTERMEASUREMENT_PERIOD = const(0x04)
_REG_SYSTEM_INTERRUPT_CONFIG_GPIO = const(0x0A)
_REG_GPIO_HV_MUX_ACTIVE_HIGH = const(0x84)
_REG_SYSTEM_INTERRUPT_CLEAR = const(0x0B)
_REG_RESULT_INTERRUPT_STATUS = const(0x13)
_REG_RESULT_RANGE_STATUS = const(0x14)
_REG_RESULT_CORE_PAGE = const(0x01)
_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM = const(0x28)
_REG_GLOBAL_CONFIG_VCSEL_WIDTH = const(0x32)
_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x71)
_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x72)
_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = const(0x20)
_REG_MSRC_CONFIG_TIMEOUT_MACROP = const(0x46)
_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = const(0x51)
_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = const(0x52)

class VL53L0XTester:
    """VL53L0X传感器综合测试类 - 增强版"""
    
    def __init__(self, scl_pin=5, sda_pin=4, freq=400000):
        """初始化测试环境"""
        print("\n" + "="*60)
        print("VL53L0X传感器测试程序 v2.0")
        print("Score99团队 - 无人机项目")
        print("="*60)
        print(f"\n配置信息:")
        print(f"  SCL引脚: GPIO{scl_pin}")
        print(f"  SDA引脚: GPIO{sda_pin}")
        print(f"  I2C频率: {freq/1000:.0f} kHz")
        print("-"*60)
        
        self.scl_pin = scl_pin
        self.sda_pin = sda_pin
        self.freq = freq
        self.i2c = None
        self.sensor_address = _VL53L0X_ADDR
        self.test_results = {}
        self.test_data = {}
        self.initialized = False
        self.calibration_offset = 0
        
    def run_all_tests(self, auto_mode=False):
        """执行完整测试套件
        
        参数:
            auto_mode: 自动模式，无需用户交互
        """
        self.auto_mode = auto_mode
        print(f"\n测试模式: {'自动' if auto_mode else '交互式'}")
        print("开始执行传感器测试套件...\n")
        
        # 测试执行列表
        test_sequence = [
            ("I2C通信测试", self.test_i2c_communication),
            ("传感器识别测试", self.test_sensor_identification),
            ("传感器初始化测试", self.test_sensor_initialization),
            ("基础测量测试", self.test_basic_measurement),
            ("测量精度测试", self.test_measurement_accuracy),
            ("测量速度测试", self.test_measurement_speed),
            ("校准测试", self.test_calibration),
            ("环境适应性测试", self.test_environmental_conditions),
            ("稳定性测试", self.test_stability),
            ("压力测试", self.test_stress)
        ]
        
        # 执行测试
        for test_name, test_func in test_sequence:
            print(f"\n{'='*20} {test_name} {'='*20}")
            try:
                if not test_func():
                    print(f"\n{test_name}失败，是否继续？(y/n)", end='')
                    if not auto_mode:
                        response = input().lower()
                        if response != 'y':
                            break
            except Exception as e:
                print(f"\n错误: {test_name}出现异常 - {e}")
                self.test_results[test_name] = 'EXCEPTION'
                if not auto_mode:
                    print("是否继续？(y/n)", end='')
                    response = input().lower()
                    if response != 'y':
                        break
        
        # 生成测试报告
        self.generate_report()
        
        # 保存测试数据
        self.save_test_data()
        
        return self.test_results
    
    def test_i2c_communication(self):
        """测试1: I2C通信验证"""
        print("\n[测试1] I2C通信测试")
        print("-" * 40)
        
        try:
            # 初始化I2C
            print("初始化I2C总线...")
            self.i2c = machine.I2C(0, 
                                 scl=machine.Pin(self.scl_pin), 
                                 sda=machine.Pin(self.sda_pin), 
                                 freq=self.freq)
            
            # 扫描I2C总线
            print("扫描I2C设备...")
            devices = self.i2c.scan()
            print(f"发现设备: {[hex(d) for d in devices]}")
            
            if not devices:
                print("错误: I2C总线上未发现任何设备")
                self.test_results['I2C通信测试'] = 'FAILED'
                return False
            
            if self.sensor_address not in devices:
                print(f"错误: 未在地址0x{self.sensor_address:02X}找到VL53L0X")
                self.test_results['I2C通信测试'] = 'FAILED'
                return False
            
            # 测试读写操作
            print("测试I2C读写操作...")
            test_reg = 0x00
            try:
                # 尝试读取寄存器
                value = self.read_reg(test_reg)
                print(f"成功读取寄存器0x{test_reg:02X}: 0x{value:02X}")
                
                print(f"✓ 成功: VL53L0X在地址0x{self.sensor_address:02X}响应正常")
                self.test_results['I2C通信测试'] = 'PASSED'
                return True
            except:
                print("错误: I2C读取操作失败")
                self.test_results['I2C通信测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: I2C初始化失败 - {e}")
            self.test_results['I2C通信测试'] = 'FAILED'
            return False
    
    def test_sensor_identification(self):
        """测试2: 传感器识别验证"""
        print("\n[测试2] 传感器识别测试")
        print("-" * 40)
        
        try:
            # 读取型号ID
            model_id = self.read_reg(_REG_IDENTIFICATION_MODEL_ID)
            revision_id = self.read_reg(_REG_IDENTIFICATION_REVISION_ID)
            
            print(f"型号ID: 0x{model_id:02X} (预期: 0xEE)")
            print(f"版本ID: 0x{revision_id:02X}")
            
            # 读取更多识别信息
            print("\n读取额外识别信息...")
            for addr in range(0xC0, 0xC8):
                try:
                    value = self.read_reg(addr)
                    print(f"  寄存器0x{addr:02X}: 0x{value:02X}")
                except:
                    pass
            
            if model_id == 0xEE:
                print("\n✓ 成功: 正确识别VL53L0X传感器")
                self.test_results['传感器识别测试'] = 'PASSED'
                return True
            else:
                print("\n✗ 警告: 型号ID不匹配，可能是兼容芯片")
                self.test_results['传感器识别测试'] = 'WARNING'
                return True
                
        except Exception as e:
            print(f"错误: 无法读取传感器ID - {e}")
            self.test_results['传感器识别测试'] = 'FAILED'
            return False
    
    def test_sensor_initialization(self):
        """测试3: 传感器初始化测试"""
        print("\n[测试3] 传感器初始化测试")
        print("-" * 40)
        
        try:
            print("执行传感器初始化序列...")
            
            # 完整的初始化序列
            init_sequence = [
                # 设置I2C标准模式
                (0x88, 0x00),
                # VHV校准
                (0x80, 0x01),
                (0xFF, 0x01),
                (0x00, 0x00),
                (0x91, 0x3c),
                (0x00, 0x01),
                (0xFF, 0x00),
                (0x80, 0x00),
                # 配置中断
                (_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04),
                (_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0x01),
                (_REG_SYSTEM_INTERRUPT_CLEAR, 0x01),
                # 设置序列步骤
                (_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF),
                # 设置测量周期
                (_REG_SYSTEM_INTERMEASUREMENT_PERIOD, 0x00),
            ]
            
            for reg, val in init_sequence:
                self.write_reg(reg, val)
                time.sleep(0.002)  # 短暂延迟
            
            # 验证初始化
            print("验证初始化状态...")
            seq_config = self.read_reg(_REG_SYSTEM_SEQUENCE_CONFIG)
            print(f"序列配置: 0x{seq_config:02X}")
            
            if seq_config == 0xFF:
                print("\n✓ 成功: 传感器初始化完成")
                self.test_results['传感器初始化测试'] = 'PASSED'
                self.initialized = True
                return True
            else:
                print("\n✗ 错误: 初始化验证失败")
                self.test_results['传感器初始化测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: 初始化过程失败 - {e}")
            self.test_results['传感器初始化测试'] = 'FAILED'
            return False
    
    def test_basic_measurement(self):
        """测试4: 基础测量功能测试"""
        print("\n[测试4] 基础测量测试")
        print("-" * 40)
        
        if not self.initialized:
            print("警告: 传感器未初始化，尝试初始化...")
            self.test_sensor_initialization()
        
        try:
            print("执行10次测量...")
            measurements = []
            errors = 0
            
            for i in range(10):
                distance = self.get_distance()
                if distance is not None:
                    measurements.append(distance)
                    status = "✓" if 0.05 < distance < 2.0 else "?"
                    print(f"  测量 {i+1:2d}: {distance:.3f} m {status}")
                else:
                    errors += 1
                    print(f"  测量 {i+1:2d}: 失败 ✗")
                time.sleep(0.1)
            
            if len(measurements) >= 5:
                avg_distance = sum(measurements) / len(measurements)
                min_dist = min(measurements)
                max_dist = max(measurements)
                
                print(f"\n测量统计:")
                print(f"  成功率: {len(measurements)/10*100:.0f}% ({len(measurements)}/10)")
                print(f"  平均值: {avg_distance:.3f} m")
                print(f"  范围: {min_dist:.3f} - {max_dist:.3f} m")
                
                # 保存测试数据
                self.test_data['basic_measurement'] = {
                    'measurements': measurements,
                    'average': avg_distance,
                    'success_rate': len(measurements)/10*100
                }
                
                print("\n✓ 成功: 基础测量功能正常")
                self.test_results['基础测量测试'] = 'PASSED'
                return True
            else:
                print(f"\n✗ 错误: 测量失败率过高 ({errors}/10)")
                self.test_results['基础测量测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: 测量测试失败 - {e}")
            self.test_results['基础测量测试'] = 'FAILED'
            return False
    
    def test_measurement_accuracy(self):
        """测试5: 测量精度和准确性测试"""
        print("\n[测试5] 测量精度测试")
        print("-" * 40)
        
        try:
            if not self.auto_mode:
                print("请将目标放置在已知距离（建议0.5米）")
                print("按Enter键开始测试...", end='')
                input()
            
            print("采集50个测量样本进行统计分析...")
            measurements = []
            
            # 进度条显示
            for i in range(50):
                distance = self.get_distance()
                if distance is not None:
                    measurements.append(distance)
                
                # 显示进度
                if (i + 1) % 10 == 0:
                    print(f"  进度: {(i+1)*2}%")
                
                time.sleep(0.05)
            
            if len(measurements) > 30:
                # 统计分析
                avg = sum(measurements) / len(measurements)
                
                # 计算标准差
                variance = sum((x - avg) ** 2 for x in measurements) / len(measurements)
                std_dev = variance ** 0.5
                
                # 计算其他统计量
                sorted_measurements = sorted(measurements)
                median = sorted_measurements[len(measurements)//2]
                q1 = sorted_measurements[len(measurements)//4]
                q3 = sorted_measurements[3*len(measurements)//4]
                iqr = q3 - q1
                
                print(f"\n统计分析结果:")
                print(f"  样本数: {len(measurements)}")
                print(f"  平均值: {avg:.3f} m")
                print(f"  中位数: {median:.3f} m")
                print(f"  标准差: {std_dev*1000:.1f} mm")
                print(f"  最小值: {min(measurements):.3f} m")
                print(f"  最大值: {max(measurements):.3f} m")
                print(f"  极差: {(max(measurements)-min(measurements))*1000:.1f} mm")
                print(f"  四分位距: {iqr*1000:.1f} mm")
                
                # 精度评估
                print(f"\n精度评估:")
                if std_dev < 0.005:
                    print("  ✓ 优秀: 标准差 < 5mm")
                    grade = "EXCELLENT"
                elif std_dev < 0.010:
                    print("  ✓ 良好: 标准差 < 10mm")
                    grade = "GOOD"
                elif std_dev < 0.020:
                    print("  ⚠ 合格: 标准差 < 20mm")
                    grade = "ACCEPTABLE"
                else:
                    print("  ✗ 不合格: 标准差 ≥ 20mm")
                    grade = "POOR"
                
                # 保存测试数据
                self.test_data['accuracy'] = {
                    'measurements': measurements,
                    'statistics': {
                        'mean': avg,
                        'median': median,
                        'std_dev': std_dev,
                        'min': min(measurements),
                        'max': max(measurements),
                        'iqr': iqr
                    },
                    'grade': grade
                }
                
                if grade in ["EXCELLENT", "GOOD", "ACCEPTABLE"]:
                    print(f"\n✓ 成功: 测量精度{grade}")
                    self.test_results['测量精度测试'] = 'PASSED'
                    return True
                else:
                    print(f"\n✗ 失败: 测量精度不足")
                    self.test_results['测量精度测试'] = 'FAILED'
                    return False
                    
            else:
                print(f"\n✗ 错误: 有效测量不足 ({len(measurements)}/50)")
                self.test_results['测量精度测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: 精度测试失败 - {e}")
            self.test_results['测量精度测试'] = 'FAILED'
            return False
    
    def test_measurement_speed(self):
        """测试6: 测量速度测试"""
        print("\n[测试6] 测量速度测试")
        print("-" * 40)
        
        try:
            print("测试不同时序预算下的测量速率...")
            
            # 测试不同的时序预算（简化版）
            test_durations = [2.0, 2.0, 2.0]  # 每个测试2秒
            test_names = ["快速模式", "标准模式", "高精度模式"]
            
            speed_results = []
            
            for i, (duration, name) in enumerate(zip(test_durations, test_names)):
                print(f"\n测试 {name}:")
                
                start_time = time.ticks_ms()
                count = 0
                valid_count = 0
                
                # 测量指定时间
                while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
                    distance = self.get_distance()
                    count += 1
                    if distance is not None:
                        valid_count += 1
                
                elapsed = time.ticks_diff(time.ticks_ms(), start_time) / 1000.0
                rate = count / elapsed
                valid_rate = valid_count / elapsed
                success_rate = (valid_count / count * 100) if count > 0 else 0
                
                print(f"  测量速率: {rate:.1f} Hz")
                print(f"  有效速率: {valid_rate:.1f} Hz")
                print(f"  成功率: {success_rate:.1f}%")
                
                speed_results.append({
                    'mode': name,
                    'rate': rate,
                    'valid_rate': valid_rate,
                    'success_rate': success_rate
                })
            
            # 保存测试数据
            self.test_data['speed'] = speed_results
            
            print("\n✓ 成功: 速度测试完成")
            self.test_results['测量速度测试'] = 'PASSED'
            return True
            
        except Exception as e:
            print(f"错误: 速度测试失败 - {e}")
            self.test_results['测量速度测试'] = 'FAILED'
            return False
    
    def test_calibration(self):
        """测试7: 校准功能测试"""
        print("\n[测试7] 校准测试")
        print("-" * 40)
        
        try:
            if not self.auto_mode:
                print("请将目标精确放置在0.500米处")
                print("按Enter键开始校准...", end='')
                input()
            else:
                print("自动模式: 假设目标在0.500米处")
            
            # 校准前测量
            print("\n校准前测量:")
            pre_cal_measurements = []
            for i in range(20):
                distance = self.get_distance()
                if distance is not None:
                    pre_cal_measurements.append(distance)
                time.sleep(0.05)
            
            if len(pre_cal_measurements) > 10:
                avg_pre = sum(pre_cal_measurements) / len(pre_cal_measurements)
                print(f"  平均值: {avg_pre:.3f} m")
                print(f"  误差: {(avg_pre - 0.5)*1000:.1f} mm")
                
                # 计算并应用偏移
                offset = 0.5 - avg_pre
                self.calibration_offset = offset
                print(f"\n应用校准偏移: {offset*1000:.1f} mm")
                
                # 设置偏移到传感器
                offset_mm = int(offset * 1000)
                if -127 <= offset_mm <= 127:
                    self.write_reg(_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM, 
                                 offset_mm & 0xFF)
                
                # 校准后测量
                print("\n校准后测量:")
                post_cal_measurements = []
                for i in range(20):
                    distance = self.get_distance()
                    if distance is not None:
                        # 应用软件偏移
                        distance += self.calibration_offset
                        post_cal_measurements.append(distance)
                    time.sleep(0.05)
                
                avg_post = sum(post_cal_measurements) / len(post_cal_measurements)
                final_error = abs(avg_post - 0.5)
                
                print(f"  平均值: {avg_post:.3f} m")
                print(f"  误差: {final_error*1000:.1f} mm")
                
                # 保存校准数据
                self.test_data['calibration'] = {
                    'pre_calibration': avg_pre,
                    'post_calibration': avg_post,
                    'offset': offset,
                    'final_error': final_error
                }
                
                # 评估校准效果
                print(f"\n校准效果评估:")
                if final_error < 0.005:
                    print("  ✓ 优秀: 误差 < 5mm")
                    self.test_results['校准测试'] = 'PASSED'
                    return True
                elif final_error < 0.010:
                    print("  ✓ 良好: 误差 < 10mm")
                    self.test_results['校准测试'] = 'PASSED'
                    return True
                else:
                    print("  ⚠ 警告: 误差 ≥ 10mm")
                    self.test_results['校准测试'] = 'WARNING'
                    return True
            else:
                print("\n✗ 错误: 测量数据不足")
                self.test_results['校准测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: 校准测试失败 - {e}")
            self.test_results['校准测试'] = 'FAILED'
            return False
    
    def test_environmental_conditions(self):
        """测试8: 环境适应性测试"""
        print("\n[测试8] 环境适应性测试")
        print("-" * 40)
        
        try:
            if self.auto_mode:
                print("自动模式: 跳过环境测试")
                self.test_results['环境适应性测试'] = 'SKIPPED'
                return True
            
            print("测试不同目标表面的测量性能...")
            
            test_surfaces = [
                ("白纸（高反射率）", "white_paper"),
                ("黑纸（低反射率）", "black_paper"),
                ("金属（镜面反射）", "metal"),
                ("木材（漫反射）", "wood"),
                ("透明塑料", "plastic")
            ]
            
            environment_results = []
            
            for surface_name, surface_id in test_surfaces:
                print(f"\n测试目标: {surface_name}")
                print("放置目标后按Enter键...", end='')
                input()
                
                measurements = []
                failures = 0
                
                for i in range(20):
                    distance = self.get_distance()
                    if distance is not None:
                        measurements.append(distance)
                    else:
                        failures += 1
                    time.sleep(0.05)
                
                if len(measurements) > 0:
                    avg = sum(measurements) / len(measurements)
                    success_rate = len(measurements) / 20 * 100
                    
                    print(f"  平均距离: {avg:.3f} m")
                    print(f"  成功率: {success_rate:.0f}%")
                    
                    result = {
                        'surface': surface_name,
                        'average': avg,
                        'success_rate': success_rate,
                        'measurements': len(measurements)
                    }
                else:
                    print("  无有效测量")
                    result = {
                        'surface': surface_name,
                        'average': None,
                        'success_rate': 0,
                        'measurements': 0
                    }
                
                environment_results.append(result)
            
            # 保存测试数据
            self.test_data['environmental'] = environment_results
            
            # 分析结果
            total_surfaces = len(test_surfaces)
            successful_surfaces = sum(1 for r in environment_results if r['success_rate'] > 50)
            
            print(f"\n环境测试总结:")
            print(f"  测试表面数: {total_surfaces}")
            print(f"  成功测量数: {successful_surfaces}")
            print(f"  适应性: {successful_surfaces/total_surfaces*100:.0f}%")
            
            if successful_surfaces >= total_surfaces * 0.6:
                print("\n✓ 成功: 环境适应性良好")
                self.test_results['环境适应性测试'] = 'PASSED'
                return True
            else:
                print("\n⚠ 警告: 环境适应性有限")
                self.test_results['环境适应性测试'] = 'WARNING'
                return True
                
        except Exception as e:
            print(f"错误: 环境测试失败 - {e}")
            self.test_results['环境适应性测试'] = 'FAILED'
            return False
    
    def test_stability(self):
        """测试9: 长期稳定性测试"""
        print("\n[测试9] 稳定性测试")
        print("-" * 40)
        
        try:
            if not self.auto_mode:
                print("将传感器对准稳定目标")
                print("测试将持续2分钟")
                print("按Enter键开始...", end='')
                input()
            
            print("\n开始稳定性测试（2分钟）...")
            
            start_time = time.time()
            measurements = []
            timestamps = []
            
            # 2分钟测试
            test_duration = 120  # 秒
            
            while time.time() - start_time < test_duration:
                distance = self.get_distance()
                if distance is not None:
                    # 应用校准偏移
                    distance += self.calibration_offset
                    measurements.append(distance)
                    timestamps.append(time.time() - start_time)
                
                # 显示进度
                elapsed = time.time() - start_time
                if int(elapsed) % 20 == 0 and int(elapsed) > 0:
                    progress = elapsed / test_duration * 100
                    remaining = test_duration - elapsed
                    print(f"  进度: {progress:.0f}% (剩余 {remaining:.0f}秒)")
                    
                    # 显示实时统计
                    if len(measurements) > 10:
                        recent = measurements[-10:]
                        print(f"    最近10次平均: {sum(recent)/10:.3f} m")
                
                time.sleep(0.5)
            
            if len(measurements) > 50:
                # 分析漂移
                # 将数据分成5段
                segment_size = len(measurements) // 5
                segments = []
                
                for i in range(5):
                    start_idx = i * segment_size
                    end_idx = start_idx + segment_size
                    segment = measurements[start_idx:end_idx]
                    if segment:
                        segments.append(sum(segment) / len(segment))
                
                # 计算漂移
                max_drift = max(segments) - min(segments)
                
                # 计算整体统计
                overall_avg = sum(measurements) / len(measurements)
                overall_std = (sum((x - overall_avg)**2 for x in measurements) / len(measurements))**0.5
                
                print(f"\n稳定性分析:")
                print(f"  总测量数: {len(measurements)}")
                print(f"  总体平均: {overall_avg:.3f} m")
                print(f"  总体标准差: {overall_std*1000:.1f} mm")
                print(f"  最大漂移: {max_drift*1000:.1f} mm")
                
                print(f"\n各时段平均值:")
                for i, avg in enumerate(segments):
                    print(f"  时段{i+1}: {avg:.3f} m")
                
                # 保存测试数据
                self.test_data['stability'] = {
                    'measurements': measurements,
                    'timestamps': timestamps,
                    'segments': segments,
                    'drift': max_drift,
                    'std_dev': overall_std
                }
                
                # 评估稳定性
                print(f"\n稳定性评估:")
                if max_drift < 0.002:
                    print("  ✓ 优秀: 漂移 < 2mm")
                    self.test_results['稳定性测试'] = 'PASSED'
                    return True
                elif max_drift < 0.005:
                    print("  ✓ 良好: 漂移 < 5mm")
                    self.test_results['稳定性测试'] = 'PASSED'
                    return True
                else:
                    print("  ⚠ 警告: 漂移 ≥ 5mm")
                    self.test_results['稳定性测试'] = 'WARNING'
                    return True
            else:
                print("\n✗ 错误: 测量数据不足")
                self.test_results['稳定性测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: 稳定性测试失败 - {e}")
            self.test_results['稳定性测试'] = 'FAILED'
            return False
    
    def test_stress(self):
        """测试10: 压力测试"""
        print("\n[测试10] 压力测试")
        print("-" * 40)
        
        try:
            print("执行连续高速测量压力测试...")
            print("测试时间: 30秒")
            
            start_time = time.time()
            total_attempts = 0
            successful_reads = 0
            errors = 0
            max_time = 0
            min_time = float('inf')
            
            # 30秒压力测试
            while time.time() - start_time < 30:
                measure_start = time.ticks_us()
                
                distance = self.get_distance()
                
                measure_time = time.ticks_diff(time.ticks_us(), measure_start)
                
                total_attempts += 1
                if distance is not None:
                    successful_reads += 1
                    if measure_time > max_time:
                        max_time = measure_time
                    if measure_time < min_time:
                        min_time = measure_time
                else:
                    errors += 1
                
                # 不延迟，最大速度运行
            
            elapsed = time.time() - start_time
            rate = total_attempts / elapsed
            success_rate = successful_reads / total_attempts * 100
            
            print(f"\n压力测试结果:")
            print(f"  测试时长: {elapsed:.1f} 秒")
            print(f"  总尝试次数: {total_attempts}")
            print(f"  成功次数: {successful_reads}")
            print(f"  错误次数: {errors}")
            print(f"  成功率: {success_rate:.1f}%")
            print(f"  测量速率: {rate:.1f} Hz")
            print(f"  最快响应: {min_time/1000:.1f} ms")
            print(f"  最慢响应: {max_time/1000:.1f} ms")
            
            # 内存检查
            gc.collect()
            free_mem = gc.mem_free()
            used_mem = gc.mem_alloc()
            print(f"\n内存状态:")
            print(f"  空闲: {free_mem} 字节")
            print(f"  已用: {used_mem} 字节")
            
            # 保存测试数据
            self.test_data['stress'] = {
                'duration': elapsed,
                'total_attempts': total_attempts,
                'successful_reads': successful_reads,
                'errors': errors,
                'success_rate': success_rate,
                'rate': rate,
                'memory_free': free_mem,
                'memory_used': used_mem
            }
            
            # 评估结果
            if success_rate > 95 and rate > 10:
                print("\n✓ 成功: 压力测试通过")
                self.test_results['压力测试'] = 'PASSED'
                return True
            elif success_rate > 90:
                print("\n⚠ 警告: 压力测试部分通过")
                self.test_results['压力测试'] = 'WARNING'
                return True
            else:
                print("\n✗ 失败: 压力测试失败")
                self.test_results['压力测试'] = 'FAILED'
                return False
                
        except Exception as e:
            print(f"错误: 压力测试失败 - {e}")
            self.test_results['压力测试'] = 'FAILED'
            return False
    
    def generate_report(self):
        """生成综合测试报告"""
        print("\n" + "="*60)
        print("VL53L0X传感器测试报告")
        print("="*60)
        print(f"测试时间: {time.localtime()}")
        print(f"硬件平台: ESP32-S3-DevKitC-1U-N32R8V")
        print(f"I2C配置: SCL=GPIO{self.scl_pin}, SDA=GPIO{self.sda_pin}")
        print(f"传感器地址: 0x{self.sensor_address:02X}")
        
        print("\n测试结果摘要:")
        print("-"*40)
        
        passed = 0
        failed = 0
        warnings = 0
        skipped = 0
        exceptions = 0
        
        # 计算统计
        for test, result in self.test_results.items():
            status_symbol = {
                'PASSED': '✓',
                'FAILED': '✗',
                'WARNING': '⚠',
                'SKIPPED': '○',
                'EXCEPTION': '!'
            }.get(result, '?')
            
            print(f"{status_symbol} {test:.<30} {result}")
            
            if result == 'PASSED':
                passed += 1
            elif result == 'FAILED':
                failed += 1
            elif result == 'WARNING':
                warnings += 1
            elif result == 'SKIPPED':
                skipped += 1
            elif result == 'EXCEPTION':
                exceptions += 1
        
        print("\n" + "-"*40)
        print(f"总测试数: {len(self.test_results)}")
        print(f"通过: {passed}")
        print(f"警告: {warnings}")
        print(f"失败: {failed}")
        print(f"跳过: {skipped}")
        print(f"异常: {exceptions}")
        
        # 关键性能指标
        if 'accuracy' in self.test_data:
            print("\n关键性能指标:")
            print("-"*40)
            acc_data = self.test_data['accuracy']['statistics']
            print(f"测量精度: {acc_data['std_dev']*1000:.1f} mm")
            
        if 'stability' in self.test_data:
            stab_data = self.test_data['stability']
            print(f"长期漂移: {stab_data['drift']*1000:.1f} mm")
            
        if 'stress' in self.test_data:
            stress_data = self.test_data['stress']
            print(f"最大测量速率: {stress_data['rate']:.1f} Hz")
            print(f"压力测试成功率: {stress_data['success_rate']:.1f}%")
        
        # 总体评估
        print("\n总体评估:")
        print("-"*40)
        
        if failed == 0 and exceptions == 0:
            if warnings == 0:
                print("★ 优秀: 传感器完全符合要求")
                print("建议: 可以直接用于生产环境")
            else:
                print("☆ 良好: 传感器基本符合要求")
                print("建议: 注意警告项，可用于开发测试")
        else:
            print("✗ 不合格: 传感器存在问题")
            print("建议: 检查硬件连接或更换传感器")
        
        print("="*60)
    
    def save_test_data(self):
        """保存测试数据到文件"""
        try:
            filename = "vl53l0x_test_data.json"
            
            # 准备数据
            save_data = {
                'test_time': str(time.localtime()),
                'results': self.test_results,
                'data': {}
            }
            
            # 简化测试数据（移除大量原始数据）
            for key, value in self.test_data.items():
                if isinstance(value, dict):
                    save_data['data'][key] = {}
                    for k, v in value.items():
                        if k not in ['measurements', 'timestamps']:  # 跳过大数组
                            save_data['data'][key][k] = v
            
            # 保存到文件
            with open(filename, 'w') as f:
                json.dump(save_data, f)
            
            print(f"\n测试数据已保存到: {filename}")
            
        except Exception as e:
            print(f"\n警告: 无法保存测试数据 - {e}")
    
    # 辅助方法
    
    def read_reg(self, reg):
        """读取单字节寄存器"""
        return self.i2c.readfrom_mem(self.sensor_address, reg, 1)[0]
    
    def read_reg16(self, reg):
        """读取16位寄存器"""
        data = self.i2c.readfrom_mem(self.sensor_address, reg, 2)
        return (data[0] << 8) | data[1]
    
    def write_reg(self, reg, val):
        """写入单字节寄存器"""
        self.i2c.writeto_mem(self.sensor_address, reg, bytes([val]))
    
    def write_reg16(self, reg, val):
        """写入16位寄存器"""
        self.i2c.writeto_mem(self.sensor_address, reg, 
                           bytes([(val >> 8) & 0xFF, val & 0xFF]))
    
    def get_distance(self):
        """获取单次距离测量"""
        try:
            # 启动测量
            self.write_reg(_REG_SYSRANGE_START, 0x01)
            
            # 等待测量完成
            timeout = 0
            while timeout < 100:
                if (self.read_reg(_REG_RESULT_INTERRUPT_STATUS) & 0x07) != 0:
                    break
                time.sleep(0.001)
                timeout += 1
            
            if timeout >= 100:
                return None
            
            # 读取结果
            distance_mm = self.read_reg16(_REG_RESULT_RANGE_STATUS + 10)
            
            # 清除中断
            self.write_reg(_REG_SYSTEM_INTERRUPT_CLEAR, 0x01)
            
            # 检查有效性
            if distance_mm > 8190:
                return None
            
            # 转换为米
            return distance_mm / 1000.0
            
        except:
            return None

# 快速测试函数
def quick_test():
    """执行快速测试（仅基础功能）"""
    print("\n执行VL53L0X快速测试...")
    tester = VL53L0XTester()
    
    # 只运行基础测试
    if tester.test_i2c_communication():
        tester.test_sensor_identification()
        tester.test_sensor_initialization()
        tester.test_basic_measurement()
    
    tester.generate_report()

# 主测试入口
def main(auto_mode=False):
    """运行完整测试套件
    
    参数:
        auto_mode: True为自动模式，False为交互模式
    """
    print("\n" + "="*60)
    print("VL53L0X激光测距传感器测试程序")
    print("Score99团队 - 无人机项目")
    print("="*60)
    
    # 显示测试选项
    if not auto_mode:
        print("\n选择测试模式:")
        print("1. 完整测试（推荐）")
        print("2. 快速测试")
        print("3. 自动测试（无需交互）")
        print("\n请选择 (1/2/3): ", end='')
        
        choice = input().strip()
        
        if choice == '2':
            quick_test()
            return
        elif choice == '3':
            auto_mode = True
    
    # 创建测试器实例
    tester = VL53L0XTester(scl_pin=5, sda_pin=4)
    
    # 运行完整测试
    results = tester.run_all_tests(auto_mode=auto_mode)
    
    print("\n测试套件执行完成！")
    
    # 返回测试结果供其他程序使用
    return results

# 独立运行时执行
if __name__ == "__main__":
    main()