"""
简化的测试框架 - 不依赖于复杂的测试库
用于无人机控制系统代码的单元测试
"""
import sys
import traceback
import time

class TestCase:
    """简化的测试用例基类"""
    
    def __init__(self, name):
        """初始化测试用例"""
        self.name = name
        self.results = []
        self.setup_done = False
        self.teardown_done = False
    
    def setUp(self):
        """测试前准备（子类可重写）"""
        pass
    
    def tearDown(self):
        """测试后清理（子类可重写）"""
        pass
    
    def assertTrue(self, condition, message=None):
        """断言条件为True"""
        if condition:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or "断言失败：期望True，实际为False"})
    
    def assertFalse(self, condition, message=None):
        """断言条件为False"""
        if not condition:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or "断言失败：期望False，实际为True"})
    
    def assertEqual(self, actual, expected, message=None):
        """断言两个值相等"""
        if actual == expected:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or 
                               f"断言失败：期望 {expected}，实际为 {actual}"})
    
    def assertNotEqual(self, actual, expected, message=None):
        """断言两个值不相等"""
        if actual != expected:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or 
                               f"断言失败：期望不等于 {expected}，实际为 {actual}"})
    
    def assertIsNone(self, obj, message=None):
        """断言对象为None"""
        if obj is None:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or 
                               f"断言失败：期望None，实际为 {obj}"})
    
    def assertIsNotNone(self, obj, message=None):
        """断言对象不为None"""
        if obj is not None:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or 
                               "断言失败：期望非None，实际为None"})
    
    def assertIsInstance(self, obj, cls, message=None):
        """断言对象是指定类的实例"""
        if isinstance(obj, cls):
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or 
                               f"断言失败：期望类型 {cls.__name__}，实际为 {type(obj).__name__}"})
    
    def assertAlmostEqual(self, actual, expected, places=7, message=None):
        """断言两个浮点数近似相等"""
        if round(abs(actual - expected), places) == 0:
            self.results.append({"result": True, "message": message})
        else:
            self.results.append({"result": False, "message": message or 
                               f"断言失败：期望约等于 {expected}，实际为 {actual}"})
    
    def run_test_method(self, method_name):
        """运行单个测试方法"""
        self.setup_done = False
        self.teardown_done = False
        method = getattr(self, method_name)
        
        if not self.setup_done:
            try:
                self.setUp()
                self.setup_done = True
            except Exception as e:
                print(f"  测试前准备失败：{e}")
                traceback.print_exc()
                return False
        
        try:
            method()
            return True
        except Exception as e:
            self.results.append({"result": False, "message": f"测试过程中出错：{e}"})
            traceback.print_exc()
            return False
        finally:
            if not self.teardown_done:
                try:
                    self.tearDown()
                    self.teardown_done = True
                except Exception as e:
                    print(f"  测试后清理失败：{e}")
                    traceback.print_exc()
    
    def run(self):
        """运行所有测试方法"""
        print(f"运行测试用例：{self.name}")
        
        # 获取所有test_开头的方法
        test_methods = [method for method in dir(self) if method.startswith("test_")]
        
        if not test_methods:
            print("  警告：没有找到测试方法")
            return False
        
        success = True
        for method_name in test_methods:
            print(f"  运行测试方法：{method_name}")
            self.results = []  # 清空之前的结果
            
            start_time = time.time()
            method_success = self.run_test_method(method_name)
            execution_time = time.time() - start_time
            
            if not method_success:
                success = False
                print(f"  测试方法 {method_name} 运行出错 ({execution_time:.3f}秒)")
                continue
            
            # 检查测试结果
            failures = [r for r in self.results if not r["result"]]
            if failures:
                success = False
                print(f"  测试方法 {method_name} 失败 ({execution_time:.3f}秒):")
                for failure in failures:
                    print(f"    {failure['message']}")
            else:
                print(f"  测试方法 {method_name} 通过 ({execution_time:.3f}秒)")
        
        return success

class TestSuite:
    """测试套件，运行多个测试用例"""
    
    def __init__(self):
        """初始化测试套件"""
        self.test_cases = []
        self.discovery_failed = False
    
    def add_test_case(self, test_case):
        """添加测试用例"""
        self.test_cases.append(test_case)
    
    def run(self):
        """运行所有测试用例"""
        print(f"运行测试套件：总共 {len(self.test_cases)} 个测试用例")
        
        success = bool(self.test_cases) and not self.discovery_failed
        start_time = time.time()
        
        for test_case in self.test_cases:
            case_success = test_case.run()
            if not case_success:
                success = False
        
        execution_time = time.time() - start_time
        
        print(f"测试套件运行完成 ({execution_time:.3f}秒)")
        if success:
            print("所有测试通过！")
        else:
            print("有测试失败！")
        
        return success

def discover_tests(module_names):
    """
    发现并加载测试用例
    
    Args:
        module_names: 测试模块名称列表
    
    Returns:
        TestSuite实例，包含发现的所有测试用例
    """
    suite = TestSuite()
    
    for module_name in module_names:
        try:
            # 动态导入模块
            __import__(module_name)
            module = sys.modules[module_name]
            
            # 查找模块中的所有TestCase子类
            for name in dir(module):
                obj = getattr(module, name)
                if isinstance(obj, type) and issubclass(obj, TestCase) and obj != TestCase:
                    # 创建测试用例实例并添加到套件
                    suite.add_test_case(obj(name))
        
        except ImportError:
            suite.discovery_failed = True
            print(f"警告：无法导入模块 {module_name}")
        except Exception as e:
            suite.discovery_failed = True
            print(f"发现测试时出错：{e}")
            traceback.print_exc()
    
    return suite

def main():
    """测试框架主函数"""
    # 可以在这里指定测试模块，或者从命令行参数获取
    test_modules = ["test_communication", "test_perception", "test_utils"]
    
    # 发现并运行测试
    suite = discover_tests(test_modules)
    success = suite.run()
    
    # 返回适当的退出代码
    return 0 if success else 1

if __name__ == "__main__":
    sys.exit(main())