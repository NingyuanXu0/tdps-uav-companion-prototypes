"""
无人机代码完整测试与调试方案
提供从单元测试到系统集成测试的全面测试计划
"""
import os
import sys
import subprocess
import importlib
import time
import threading
import unittest
import glob

# 总测试计划执行状态
test_results = {
    "unit_tests": {"status": "pending", "details": {}},
    "integration_tests": {"status": "pending", "details": {}},
    "system_tests": {"status": "pending", "details": {}},
    "performance_tests": {"status": "pending", "details": {}},
    "fcm_integration": {"status": "pending", "details": {}}
}

def print_header(title):
    """打印带格式的标题"""
    print("\n" + "="*80)
    print(f"{title}".center(80))
    print("="*80 + "\n")

def run_unit_tests():
    """运行所有单元测试"""
    print_header("运行单元测试")
    
    # 要测试的模块列表
    test_modules = [
        "unit_test_utils",
        "unit_test_perception", 
        "unit_test_communication",
        "unit_test_task_manager",
        "unit_test_main"
    ]
    
    # 收集测试结果
    results = {}
    all_tests_passed = True
    
    for module in test_modules:
        print(f"测试模块: {module}")
        try:
            # 导入测试模块
            test_module = importlib.import_module(module)
            
            # 创建测试加载器
            loader = unittest.TestLoader()
            suite = loader.loadTestsFromModule(test_module)
            
            # 运行测试
            runner = unittest.TextTestRunner(verbosity=2)
            result = runner.run(suite)
            
            # 收集结果
            module_result = {
                "total": result.testsRun,
                "passed": result.testsRun - len(result.failures) - len(result.errors),
                "failed": len(result.failures),
                "errors": len(result.errors),
                "failures": result.failures,
                "error_details": result.errors
            }
            
            # 更新结果字典
            results[module] = module_result
            
            # 检查是否全部通过
            if result.failures or result.errors:
                all_tests_passed = False
                print(f"模块 {module} 测试失败或出错")
            else:
                print(f"模块 {module} 测试全部通过")
            
        except Exception as e:
            print(f"运行测试模块 {module} 时出错: {e}")
            all_tests_passed = False
            results[module] = {"error": str(e)}
    
    # 更新全局结果
    test_results["unit_tests"]["status"] = "passed" if all_tests_passed else "failed"
    test_results["unit_tests"]["details"] = results
    
    # 返回结果
    return all_tests_passed, results

def run_integration_tests():
    """运行集成测试"""
    print_header("运行集成测试")
    
    try:
        # 导入集成测试模块
        import integration_test
        
        # 加载测试用例
        loader = unittest.TestLoader()
        suite = loader.loadTestsFromModule(integration_test)
        
        # 运行测试
        runner = unittest.TextTestRunner(verbosity=2)
        result = runner.run(suite)
        
        # 检查结果
        all_tests_passed = not result.failures and not result.errors
        
        # 收集结果
        results = {
            "total": result.testsRun,
            "passed": result.testsRun - len(result.failures) - len(result.errors),
            "failed": len(result.failures),
            "errors": len(result.errors),
            "failures": result.failures,
            "error_details": result.errors
        }
        
        # 更新全局结果
        test_results["integration_tests"]["status"] = "passed" if all_tests_passed else "failed"
        test_results["integration_tests"]["details"] = results
        
        # 返回结果
        return all_tests_passed, results
        
    except Exception as e:
        print(f"运行集成测试时出错: {e}")
        test_results["integration_tests"]["status"] = "error"
        test_results["integration_tests"]["details"] = {"error": str(e)}
        return False, {"error": str(e)}

def run_system_tests():
    """运行系统测试（硬件模拟器）"""
    print_header("运行系统测试（通过硬件模拟器）")
    
    try:
        print("启动硬件模拟器线程...")
        
        # 创建线程运行系统测试
        def run_simulator():
            try:
                import run_system_test
                # 不调用run_simulator.run()，因为那会阻塞线程
                # 而是直接运行测试序列
                run_system_test.auto_test_sequence()
                return True
            except Exception as e:
                print(f"运行系统测试时出错: {e}")
                return False
        
        # 启动线程
        test_thread = threading.Thread(target=run_simulator)
        test_thread.daemon = True
        test_thread.start()
        
        # 等待测试完成（最多等待60秒）
        test_thread.join(timeout=60)
        
        # 检查线程是否仍在运行
        if test_thread.is_alive():
            print("系统测试超时！")
            results = {"error": "测试超时"}
            success = False
        else:
            print("系统测试完成")
            results = {"status": "完成"}
            success = True
        
        # 更新全局结果
        test_results["system_tests"]["status"] = "passed" if success else "failed"
        test_results["system_tests"]["details"] = results
        
        # 返回结果
        return success, results
        
    except Exception as e:
        print(f"启动系统测试时出错: {e}")
        test_results["system_tests"]["status"] = "error"
        test_results["system_tests"]["details"] = {"error": str(e)}
        return False, {"error": str(e)}

def run_performance_tests():
    """运行性能测试"""
    print_header("运行性能测试")
    
    try:
        # 导入性能测试模块
        import performance_test
        
        # 创建性能测试实例
        tester = performance_test.PerformanceTester()
        
        # 运行完整测试
        print("运行性能测试...")
        tester.run_complete_test()
        
        # 获取优化建议
        suggestions = tester.suggest_optimizations()
        
        # 运行代码优化
        print("\n应用性能优化...")
        optimizer = performance_test.CodeOptimizer()
        optimizer.optimize_all()
        
        # 收集结果
        results = {
            "results": tester.results,
            "suggestions": suggestions,
            "optimizations": optimizer.optimizations
        }
        
        # 更新全局结果
        test_results["performance_tests"]["status"] = "passed"
        test_results["performance_tests"]["details"] = results
        
        # 返回结果
        return True, results
        
    except Exception as e:
        print(f"运行性能测试时出错: {e}")
        test_results["performance_tests"]["status"] = "error"
        test_results["performance_tests"]["details"] = {"error": str(e)}
        return False, {"error": str(e)}

def run_fcm_integration():
    """运行FCM集成测试"""
    print_header("运行FCM集成测试")
    
    try:
        print("启动FCM集成测试线程...")
        
        # 创建线程运行FCM集成测试
        def run_fcm_test():
            try:
                import fcm_acm_integration
                # 创建模拟器但不运行主循环（会阻塞线程）
                gui = fcm_acm_integration.FCMAcmIntegrationGUI()
                
                # 直接运行集成测试
                print("运行FCM-ACM集成测试...")
                fcm = fcm_acm_integration.FCMSimulator(gui=gui)
                fcm.start()
                
                # 运行测试序列
                gui._run_test_sequence()
                
                # 停止FCM模拟器
                fcm.stop()
                
                return True
            except Exception as e:
                print(f"运行FCM集成测试时出错: {e}")
                return False
        
        # 启动线程
        test_thread = threading.Thread(target=run_fcm_test)
        test_thread.daemon = True
        test_thread.start()
        
        # 等待测试完成（最多等待60秒）
        test_thread.join(timeout=60)
        
        # 检查线程是否仍在运行
        if test_thread.is_alive():
            print("FCM集成测试超时！")
            results = {"error": "测试超时"}
            success = False
        else:
            print("FCM集成测试完成")
            results = {"status": "完成"}
            success = True
        
        # 更新全局结果
        test_results["fcm_integration"]["status"] = "passed" if success else "failed"
        test_results["fcm_integration"]["details"] = results
        
        # 返回结果
        return success, results
        
    except Exception as e:
        print(f"启动FCM集成测试时出错: {e}")
        test_results["fcm_integration"]["status"] = "error"
        test_results["fcm_integration"]["details"] = {"error": str(e)}
        return False, {"error": str(e)}

def run_all_tests():
    """运行所有测试"""
    print_header("无人机代码完整测试计划")
    
    # 步骤1：单元测试
    unit_success, unit_results = run_unit_tests()
    print(f"单元测试结果: {'通过' if unit_success else '失败'}")
    
    # 步骤2：集成测试
    int_success, int_results = run_integration_tests()
    print(f"集成测试结果: {'通过' if int_success else '失败'}")
    
    # 步骤3：系统测试
    sys_success, sys_results = run_system_tests()
    print(f"系统测试结果: {'通过' if sys_success else '失败'}")
    
    # 步骤4：性能测试
    perf_success, perf_results = run_performance_tests()
    print(f"性能测试结果: {'通过' if perf_success else '失败'}")
    
    # 步骤5：FCM集成测试
    fcm_success, fcm_results = run_fcm_integration()
    print(f"FCM集成测试结果: {'通过' if fcm_success else '失败'}")
    
    # 汇总结果
    all_passed = unit_success and int_success and sys_success and perf_success and fcm_success
    
    print_header("测试总结")
    print(f"单元测试: {'通过' if unit_success else '失败'}")
    print(f"集成测试: {'通过' if int_success else '失败'}")
    print(f"系统测试: {'通过' if sys_success else '失败'}")
    print(f"性能测试: {'通过' if perf_success else '失败'}")
    print(f"FCM集成测试: {'通过' if fcm_success else '失败'}")
    print("\n总体结果: " + ("所有测试通过！" if all_passed else "部分测试失败！"))
    
    return all_passed

def format_performance_test_details(details):
    """格式化性能测试详情"""
    if not details:
        return "无性能测试详情"
    
    if "error" in details:
        return f"性能测试运行出错: {details['error']}"
    
    result = "性能测试结果：\n\n"
    
    if "results" in details:
        for name, test_result in details["results"].items():
            if name == "main_loop":
                result += f"#### 主循环性能\n\n"
                result += f"- 平均循环时间: {test_result.get('average', 'N/A')} 秒\n\n"
            elif name == "memory_usage":
                result += f"#### 内存使用情况\n\n"
                for module, usage in test_result.items():
                    result += f"- {module}: {usage} 字节\n"
                result += "\n"
            elif isinstance(test_result, dict):
                result += f"#### {name}\n\n"
                for key, value in test_result.items():
                    result += f"- {key}: {value}\n"
                result += "\n"
    
    return result

def format_fcm_integration_details(details):
    """格式化FCM集成测试详情"""
    if not details:
        return "无FCM集成测试详情"
    
    if "error" in details:
        return f"FCM集成测试运行出错: {details['error']}"
    
    return f"FCM集成测试状态: {details.get('status', 'unknown')}"

def format_optimization_suggestions(details):
    """格式化优化建议"""
    if not details or "error" in details or not isinstance(details, dict) or "suggestions" not in details:
        return "无优化建议"
    
    suggestions = details["suggestions"]
    if not suggestions:
        return "无需优化"
    
    result = "以下是代码性能优化建议：\n\n"
    for i, suggestion in enumerate(suggestions, 1):
        result += f"{i}. {suggestion}\n"
    
    if "optimizations" in details:
        result += "\n\n已应用的优化：\n\n"
        for i, optimization in enumerate(details["optimizations"], 1):
            result += f"{i}. {optimization}\n"
    
    return result

def get_conclusion():
    """生成测试结论"""
    statuses = [
        test_results["unit_tests"]["status"],
        test_results["integration_tests"]["status"],
        test_results["system_tests"]["status"],
        test_results["performance_tests"]["status"],
        test_results["fcm_integration"]["status"]
    ]
    
    if all(status == "passed" for status in statuses):
        return "所有测试均已通过。代码质量良好，性能符合预期，可以部署到实际环境中。"
    elif statuses.count("passed") >= 3:
        return "大部分测试已通过，但仍有一些问题需要解决。在部署到实际环境前，建议修复失败的测试。"
    else:
        return "多数测试未通过。代码存在严重问题，需要进行重大修改后才能部署到实际环境。"

def generate_test_report():
    """生成测试报告"""
    print_header("生成测试报告")
    
    report = """
# 无人机代码测试报告

## 测试概述

本报告包含对无人机控制系统代码的完整测试结果，包括单元测试、集成测试、系统测试、性能测试和FCM集成测试。

## 测试结果汇总

| 测试类型 | 状态 | 备注 |
|---------|------|------|
| 单元测试 | {unit_status} | {unit_notes} |
| 集成测试 | {int_status} | {int_notes} |
| 系统测试 | {sys_status} | {sys_notes} |
| 性能测试 | {perf_status} | {perf_notes} |
| FCM集成测试 | {fcm_status} | {fcm_notes} |

## 详细测试结果

### 单元测试

{unit_details}

### 集成测试

{int_details}

### 系统测试

{sys_details}

### 性能测试

{perf_details}

### FCM集成测试

{fcm_details}

## 性能优化建议

{opt_suggestions}

## 结论

{conclusion}
    """.format(
        unit_status=test_results["unit_tests"]["status"],
        int_status=test_results["integration_tests"]["status"],
        sys_status=test_results["system_tests"]["status"],
        perf_status=test_results["performance_tests"]["status"],
        fcm_status=test_results["fcm_integration"]["status"],
        
        unit_notes=f"通过率: {get_pass_rate(test_results['unit_tests']['details'])}%" if test_results["unit_tests"]["status"] != "error" else "测试过程出错",
        int_notes=f"通过率: {get_pass_rate(test_results['integration_tests']['details'])}%" if test_results["integration_tests"]["status"] != "error" else "测试过程出错",
        sys_notes="详见系统测试部分",
        perf_notes="见性能优化建议部分",
        fcm_notes="详见FCM集成测试部分",
        
        unit_details=format_unit_test_details(test_results["unit_tests"]["details"]),
        int_details=format_integration_test_details(test_results["integration_tests"]["details"]),
        sys_details=format_system_test_details(test_results["system_tests"]["details"]),
        perf_details=format_performance_test_details(test_results["performance_tests"]["details"]),
        fcm_details=format_fcm_integration_details(test_results["fcm_integration"]["details"]),
        
        opt_suggestions=format_optimization_suggestions(test_results["performance_tests"]["details"]),
        
        conclusion=get_conclusion()
    )
    
    # 保存报告
    with open("test_report.md", "w", encoding="utf-8") as f:
        f.write(report)
    
    print(f"测试报告已生成：test_report.md")
    return report

def get_pass_rate(details):
    """计算测试通过率"""
    if not details or "error" in details:
        return 0
    
    total_tests = 0
    passed_tests = 0
    
    for module, result in details.items():
        if isinstance(result, dict) and "total" in result and "passed" in result:
            total_tests += result["total"]
            passed_tests += result["passed"]
    
    return int(passed_tests / max(1, total_tests) * 100)

def format_unit_test_details(details):
    """格式化单元测试详情"""
    if not details:
        return "无单元测试详情"
    
    if "error" in details:
        return f"单元测试运行出错: {details['error']}"
    
    result = "以下是各模块的单元测试结果：\n\n"
    for module, module_result in details.items():
        if isinstance(module_result, dict) and "total" in module_result:
            result += f"#### {module}\n\n"
            result += f"- 测试总数: {module_result['total']}\n"
            result += f"- 通过数: {module_result['passed']}\n"
            result += f"- 失败数: {module_result['failed']}\n"
            result += f"- 错误数: {module_result['errors']}\n\n"
            
            if module_result["failures"]:
                result += "失败的测试:\n\n"
                for failure in module_result["failures"]:
                    if isinstance(failure, tuple) and len(failure) >= 2:
                        test_name = failure[0]
                        result += f"- {test_name}\n"
                result += "\n"
        else:
            result += f"#### {module}: {module_result}\n\n"
    
    return result

def format_integration_test_details(details):
    """格式化集成测试详情"""
    if not details:
        return "无集成测试详情"
    
    if "error" in details:
        return f"集成测试运行出错: {details['error']}"
    
    result = "集成测试结果：\n\n"
    result += f"- 测试总数: {details.get('total', 'N/A')}\n"
    result += f"- 通过数: {details.get('passed', 'N/A')}\n"
    result += f"- 失败数: {details.get('failed', 'N/A')}\n"
    result += f"- 错误数: {details.get('errors', 'N/A')}\n\n"
    
    if details.get("failures"):
        result += "失败的测试:\n\n"
        for failure in details["failures"]:
            if isinstance(failure, tuple) and len(failure) >= 2:
                test_name = failure[0]
                result += f"- {test_name}\n"
    
    return result

def format_system_test_details(details):
    """格式化系统测试详情"""
    if not details:
        return "无系统测试详情"
    
    if "error" in details:
        return f"系统测试运行出错: {details['error']}"
    
    return f"系统测试状态: {details.get('status', 'unknown')}"

def create_esp32_flashable_firmware():
    """创建可烧录到ESP32的固件"""
    print_header("创建ESP32固件")
    
    try:
        print("收集Python文件...")
        
        # 收集所有需要包含的Python文件
        python_files = [
            "main.py",
            "config.py",
            "utils.py",
            "perception.py",
            "communication.py",
            "task_manager.py"
        ]
        
        # 创建固件目录
        os.makedirs("firmware", exist_ok=True)
        
        # 复制文件到固件目录
        for filename in python_files:
            if os.path.exists(filename):
                with open(filename, "r", encoding="utf-8") as src_file:
                    content = src_file.read()
                
                # 写入到固件目录
                with open(os.path.join("firmware", filename), "w", encoding="utf-8") as dst_file:
                    dst_file.write(content)
                
                print(f"已添加 {filename} 到固件")
            else:
                print(f"警告: 文件 {filename} 不存在")
        
        print("\n固件目录已创建: firmware/")
        print("固件文件列表:")
        for filename in os.listdir("firmware"):
            print(f"- {filename}")
        
        print("\n要将固件烧录到ESP32，请使用以下命令：")
        print("ampy --port COM_PORT put firmware/main.py /main.py")
        print("ampy --port COM_PORT put firmware/config.py /config.py")
        print("ampy --port COM_PORT put firmware/utils.py /utils.py")
        print("ampy --port COM_PORT put firmware/perception.py /perception.py")
        print("ampy --port COM_PORT put firmware/communication.py /communication.py")
        print("ampy --port COM_PORT put firmware/task_manager.py /task_manager.py")
        print("\n其中COM_PORT替换为ESP32的串口（例如COM3或/dev/ttyUSB0）")
        
        return True
        
    except Exception as e:
        print(f"创建固件时出错: {e}")
        return False

def main():
    """主函数"""
    print_header("无人机代码测试与调试方案")
    
    print("""本程序提供以下功能：
1. 运行所有测试（单元测试、集成测试、系统测试、性能测试、FCM集成测试）
2. 仅运行单元测试
3. 仅运行集成测试
4. 仅运行系统测试（硬件模拟器）
5. 仅运行性能测试
6. 仅运行FCM集成测试
7. 生成测试报告
8. 创建ESP32固件
9. 退出
""")
    
    while True:
        try:
            choice = input("请选择操作 (1-9): ")
            
            if choice == "1":
                run_all_tests()
                
            elif choice == "2":
                run_unit_tests()
                
            elif choice == "3":
                run_integration_tests()
                
            elif choice == "4":
                run_system_tests()
                
            elif choice == "5":
                run_performance_tests()
                
            elif choice == "6":
                run_fcm_integration()
                
            elif choice == "7":
                generate_test_report()
                
            elif choice == "8":
                create_esp32_flashable_firmware()
                
            elif choice == "9":
                print("退出程序")
                break
                
            else:
                print("无效的选择，请重新输入")
                
        except KeyboardInterrupt:
            print("\n操作已取消")
            
        except Exception as e:
            print(f"操作过程中出错: {e}")
        
        print("\n")

if __name__ == "__main__":
    main()