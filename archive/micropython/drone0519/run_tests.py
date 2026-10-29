"""
无人机控制系统完整测试流程
运行单元测试和模拟测试
"""
import os
import sys
import time
import subprocess
import argparse
import importlib.util

def setup_environment():
    """设置测试环境"""
    
    # 确保所需的测试模块可用
    modules_to_check = [
        'simple_test_framework.py',
        'mock_fcm.py',
        'mock_sensors.py', 
        'mock_wireless.py',
        'simulation_runner.py'
    ]
    
    missing_modules = []
    for module in modules_to_check:
        if not os.path.exists(module):
            missing_modules.append(module)
    
    if missing_modules:
        print("警告：以下测试模块不可用：")
        for module in missing_modules:
            print(f" - {module}")
        return False
    
    return True

def run_unit_tests():
    """运行所有单元测试"""
    print("\n============= 开始单元测试 =============")
    
    # 运行简化的测试框架
    if os.path.exists('simple_test_framework.py'):
        try:
            # 动态导入测试框架
            spec = importlib.util.spec_from_file_location("simple_test_framework", "simple_test_framework.py")
            test_framework = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(test_framework)
            
            # 发现并运行测试
            if hasattr(test_framework, 'main'):
                return test_framework.main() == 0
        except Exception as e:
            print(f"运行单元测试时出错: {e}")
    
    # 如果简化框架失败，尝试直接运行测试文件
    test_files = [f for f in os.listdir('.') if f.startswith('test_') and f.endswith('.py')]
    
    if not test_files:
        print("未找到测试文件")
        return False
    
    success = True
    for test_file in test_files:
        test_path = test_file
        print(f"运行测试: {test_file}")
        try:
            result = subprocess.run([sys.executable, test_path], check=False)
            if result.returncode != 0:
                success = False
                print(f"测试 {test_file} 失败")
        except Exception as e:
            success = False
            print(f"运行测试 {test_file} 时出错: {e}")
    
    print("\n============= 单元测试完成 =============")
    return success

def run_simulation_test(duration=60):
    """
    运行模拟测试
    
    Args:
        duration: 模拟持续时间（秒）
    """
    print(f"\n============= 开始模拟测试（{duration}秒）=============")
    
    if not os.path.exists('simulation_runner.py'):
        print("找不到模拟器脚本")
        return False
    
    try:
        # 运行模拟器脚本
        cmd = [sys.executable, 'simulation_runner.py', '--duration', str(duration)]
        process = subprocess.run(cmd, check=False)
        
        success = (process.returncode == 0)
        
        print("\n============= 模拟测试完成 =============")
        return success
    
    except Exception as e:
        print(f"运行模拟测试时出错: {e}")
        return False

def generate_report(results):
    """
    生成测试报告
    
    Args:
        results: 测试结果字典
    """
    print("\n============= 测试报告 =============")
    
    all_passed = True
    for test_name, result in results.items():
        status = "通过" if result else "失败"
        print(f"{test_name}: {status}")
        if not result:
            all_passed = False
    
    print("\n总体结果: " + ("通过" if all_passed else "失败"))
    
    # 将结果写入报告文件
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    report_file = f"test_report_{timestamp}.txt"
    
    try:
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("无人机控制系统测试报告\n")
            f.write(f"生成时间: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            for test_name, result in results.items():
                status = "通过" if result else "失败"
                f.write(f"{test_name}: {status}\n")
            
            f.write(f"\n总体结果: {('通过' if all_passed else '失败')}\n")
        
        print(f"测试报告已保存到 {report_file}")
        
    except Exception as e:
        print(f"保存测试报告时出错: {e}")
    
    return all_passed

def main():
    """主函数"""
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    parser = argparse.ArgumentParser(description='无人机控制系统测试运行器')
    parser.add_argument('--unit', action='store_true', help='运行单元测试')
    parser.add_argument('--simulation', action='store_true', help='运行模拟测试')
    parser.add_argument('--all', action='store_true', help='运行所有测试')
    parser.add_argument('--sim-duration', type=int, default=60, help='模拟测试持续时间（秒）')
    
    args = parser.parse_args()
    
    # 如果没有指定任何测试，则默认运行所有测试
    if not (args.unit or args.simulation or args.all):
        args.all = True
    
    # 设置测试环境
    if not setup_environment():
        print("测试环境设置失败")
        return 1
    
    # 记录测试结果
    results = {}
    
    # 运行选定的测试
    if args.all or args.unit:
        results['单元测试'] = run_unit_tests()
    
    if args.all or args.simulation:
        results['模拟测试'] = run_simulation_test(args.sim_duration)
    
    # 生成测试报告
    success = generate_report(results)
    
    # 返回适当的退出代码
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())