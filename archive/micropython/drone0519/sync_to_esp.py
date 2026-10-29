"""
文件同步工具 - 将选定的代码同步到ESP32设备；本发布未运行测试。
"""
import os
import time
import argparse
import subprocess
import platform

def detect_port():
    """检测ESP32设备的串口"""
    system = platform.system()
    
    if system == 'Windows':
        # Windows系统下检测可用COM端口
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            for port in ports:
                if 'CP210' in port.description or 'Silicon Labs' in port.description:
                    return port.device
        except ImportError:
            print("警告：pyserial未安装，无法自动检测端口")
    
    elif system == 'Linux':
        # Linux系统下检测常见USB串口
        for port in ['/dev/ttyUSB0', '/dev/ttyACM0']:
            if os.path.exists(port):
                return port
    
    elif system == 'Darwin':  # macOS
        # macOS系统下检测常见USB串口
        for port_pattern in ['/dev/tty.SLAB_USBtoUART', '/dev/tty.usbserial-*']:
            import glob
            matches = glob.glob(port_pattern)
            if matches:
                return matches[0]
    
    return None

def list_files(directory, extensions=['.py']):
    """列出指定目录中的所有Python文件"""
    files = []
    for file in os.listdir(directory):
        if os.path.isfile(os.path.join(directory, file)):
            _, ext = os.path.splitext(file)
            if ext.lower() in extensions:
                files.append(file)
    return files

def sync_file_mpremote(file_path, port):
    """
    使用mpremote同步单个文件到ESP32
    
    Args:
        file_path: 文件路径
        port: 串口名称
    
    Returns:
        同步成功返回True，否则返回False
    """
    filename = os.path.basename(file_path)
    print(f"同步文件: {filename}")
    
    try:
        # 使用mpremote工具
        cmd = ['mpremote', 'connect', port, 'fs', 'cp', file_path, ':' + filename]
        result = subprocess.run(cmd, capture_output=True, text=True)
        
        if result.returncode != 0:
            print(f"同步失败: {result.stderr}")
            return False
            
        print(f"文件 {filename} 同步成功")
        return True
        
    except Exception as e:
        print(f"同步文件 {filename} 时出错: {e}")
        return False

def sync_file_manual(file_path, port):
    """
    使用串口手动同步单个文件到ESP32
    
    Args:
        file_path: 文件路径
        port: 串口名称
    
    Returns:
        同步成功返回True，否则返回False
    """
    filename = os.path.basename(file_path)
    print(f"手动同步文件: {filename}")
    
    try:
        import serial
        ser = serial.Serial(port, 115200, timeout=3)
        
        # 进入REPL模式
        ser.write(b'\x03\x01')  # Ctrl+C, Ctrl+A
        time.sleep(0.1)
        
        # 读取并丢弃现有输出
        ser.read_all()
        
        # 使用paste模式上传文件
        ser.write(b'\x05')  # Ctrl+E 进入paste模式
        time.sleep(0.1)
        
        # 读取并确认进入paste模式
        response = ser.read_all()
        if b'paste mode' not in response:
            print("无法进入paste模式")
            ser.close()
            return False
        
        # 读取文件内容
        with open(file_path, 'r') as f:
            content = f.read()
        
        # 发送文件内容
        ser.write(content.encode('utf-8'))
        time.sleep(0.5)
        
        # 退出paste模式
        ser.write(b'\x04')  # Ctrl+D
        time.sleep(0.5)
        
        # 保存文件
        cmd = f"with open('{filename}', 'w') as f:\n    f.write('''".encode('utf-8')
        ser.write(cmd)
        time.sleep(0.1)
        ser.write(content.encode('utf-8'))
        ser.write(b"''')\n")
        time.sleep(0.5)
        
        # 关闭串口
        ser.close()
        
        print(f"文件 {filename} 手动同步成功")
        return True
        
    except Exception as e:
        print(f"手动同步文件 {filename} 时出错: {e}")
        return False

def reset_device(port):
    """
    重置ESP32设备
    
    Args:
        port: 串口名称
    
    Returns:
        重置成功返回True，否则返回False
    """
    print("重置设备...")
    
    try:
        # 尝试使用mpremote
        try:
            cmd = ['mpremote', f'connect={port}', 'reset']
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0:
                print("设备重置成功（通过mpremote）")
                return True
        except:
            pass
            
        # 如果mpremote失败，使用串口方式重置
        import serial
        ser = serial.Serial(port, 115200, timeout=1)
        ser.write(b'\x03\x04')  # Ctrl+C followed by Ctrl+D
        ser.close()
        
        print("设备重置成功（通过串口）")
        return True
        
    except Exception as e:
        print(f"重置设备时出错: {e}")
        return False

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='ESP32文件同步工具')
    parser.add_argument('--port', type=str, help='ESP32设备串口')
    parser.add_argument('--dir', type=str, default='.', help='源代码目录')
    parser.add_argument('--manual', action='store_true', help='使用手动串口模式而非mpremote')
    
    args = parser.parse_args()
    
    # 如果未指定端口，尝试自动检测
    port = args.port
    if not port:
        port = detect_port()
        if port:
            print(f"自动检测到ESP32设备端口: {port}")
        else:
            print("未能自动检测到ESP32设备，请指定--port参数")
            return 1
    
    # 检查源代码目录
    if not os.path.isdir(args.dir):
        print(f"目录不存在: {args.dir}")
        return 1
    
    # 列出所有Python文件
    files = list_files(args.dir)
    if not files:
        print(f"在目录 {args.dir} 中未找到Python文件")
        return 1
    
    print(f"找到 {len(files)} 个Python文件，准备同步...")
    
    # 同步所有文件
    success_count = 0
    for file in files:
        file_path = os.path.join(args.dir, file)
        if args.manual:
            if sync_file_manual(file_path, port):
                success_count += 1
        else:
            if sync_file_mpremote(file_path, port):
                success_count += 1
        
        # 短暂暂停，避免设备过载
        time.sleep(0.5)
    
    # 同步结果
    if success_count == len(files):
        print(f"所有 {success_count} 个文件已成功同步")
        
        # 重置设备，使新代码生效
        reset_device(port)
        
        return 0
    else:
        print(f"同步完成，{success_count}/{len(files)} 个文件成功")
        return 1

if __name__ == "__main__":
    exit(main())
