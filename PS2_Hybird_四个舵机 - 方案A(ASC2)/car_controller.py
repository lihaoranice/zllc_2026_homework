"""
PS2 Hybrid Car 上位机控制程序
支持手动控制和自动寻路功能
"""

import serial
import time
import sys

class CarController:
    """小车控制器类"""
    
    def __init__(self, port='COM3', baudrate=115200, timeout=1):
        """
        初始化小车控制器
        :param port: 串口号（Windows: COM3, Linux: /dev/ttyUSB0）
        :param baudrate: 波特率
        :param timeout: 超时时间
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # 等待连接稳定
            print(f"成功连接到 {port}，波特率 {baudrate}")
            
            # 切换到上位机控制模式
            response = self.set_mode(1)
            print(response)
            
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            sys.exit(1)
    
    def send_command(self, command):
        """
        发送命令并接收响应
        :param command: 命令字符串
        :return: 响应字符串
        """
        try:
            self.ser.write((command + '\n').encode())
            time.sleep(0.05)
            
            # 读取所有可用响应
            response = ""
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode().strip()
                if line:
                    response += line + "\n"
            
            return response.strip() if response else "No response"
            
        except Exception as e:
            return f"Error: {e}"
    
    def move_forward(self, speed=500):
        """
        前进
        :param speed: 速度 (150-900)
        """
        return self.send_command(f'CMD:MOVE,F,{speed}')
    
    def move_backward(self, speed=500):
        """
        后退
        :param speed: 速度 (150-900)
        """
        return self.send_command(f'CMD:MOVE,B,{speed}')
    
    def turn_left(self, speed=300):
        """
        原地左转
        :param speed: 速度 (150-900)
        """
        return self.send_command(f'CMD:MOVE,L,{speed}')
    
    def turn_right(self, speed=300):
        """
        原地右转
        :param speed: 速度 (150-900)
        """
        return self.send_command(f'CMD:MOVE,R,{speed}')
    
    def stop(self):
        """停止"""
        return self.send_command('CMD:STOP')
    
    def get_status(self):
        """获取状态"""
        return self.send_command('CMD:STATUS')
    
    def reset_encoder(self):
        """复位编码器"""
        return self.send_command('CMD:RESET_ENC')
    
    def set_mode(self, mode):
        """
        设置控制模式
        :param mode: 0=PS2, 1=UART
        """
        return self.send_command(f'CMD:MODE,{mode}')
    
    def close(self):
        """关闭串口"""
        if self.ser.is_open:
            self.ser.close()
            print("串口已关闭")


class AutoNavigator:
    """自动寻路类"""
    
    def __init__(self, car):
        """
        初始化自动寻路器
        :param car: CarController实例
        """
        self.car = car
    
    def execute_path(self, path):
        """
        执行路径
        :param path: 路径列表，格式：[('F', 500, 2.0), ('R', 300, 0.8), ...]
                    每个元素：(方向, 速度, 持续时间)
        """
        print("\n开始执行自动寻路...")
        print("=" * 50)
        
        self.car.reset_encoder()
        time.sleep(0.5)
        
        step = 1
        for direction, speed, duration in path:
            print(f"\n步骤 {step}:", end=" ")
            
            if direction == 'F':
                print(f"前进，速度={speed}，持续{duration:.1f}秒")
                self.car.move_forward(speed)
            elif direction == 'B':
                print(f"后退，速度={speed}，持续{duration:.1f}秒")
                self.car.move_backward(speed)
            elif direction == 'L':
                print(f"左转，速度={speed}，持续{duration:.1f}秒")
                self.car.turn_left(speed)
            elif direction == 'R':
                print(f"右转，速度={speed}，持续{duration:.1f}秒")
                self.car.turn_right(speed)
            else:
                print(f"未知方向: {direction}")
                continue
            
            time.sleep(duration)
            self.car.stop()
            time.sleep(0.5)
            
            # 查询状态
            status = self.car.get_status()
            print(f"  状态: {status}")
            
            step += 1
        
        print("\n" + "=" * 50)
        print("自动寻路完成！")
    
    def square_path(self, side_time=2.0, turn_time=0.8, speed=500):
        """
        正方形路径
        :param side_time: 每边前进时间
        :param turn_time: 每次转向时间
        :param speed: 速度
        """
        path = []
        for _ in range(4):
            path.append(('F', speed, side_time))
            path.append(('R', 300, turn_time))
        return path
    
    def triangle_path(self, side_time=2.0, turn_time=1.0, speed=500):
        """
        三角形路径（120度转向）
        :param side_time: 每边前进时间
        :param turn_time: 每次转向时间
        :param speed: 速度
        """
        path = []
        for _ in range(3):
            path.append(('F', speed, side_time))
            path.append(('R', 300, turn_time))
        return path
    
    def eight_path(self, radius_time=1.5, speed=500):
        """
        8字形路径
        :param radius_time: 每个圆的时间
        :param speed: 速度
        """
        path = [
            ('F', speed, 1.0),
            ('R', 250, radius_time),
            ('F', speed, 1.0),
            ('L', 250, radius_time),
        ]
        return path


def manual_control_menu(car):
    """手动控制菜单"""
    print("\n" + "=" * 50)
    print("手动控制模式")
    print("=" * 50)
    print("w - 前进    s - 后退")
    print("a - 左转    d - 右转")
    print("x - 停止")
    print("q - 退出手动模式")
    print("=" * 50)
    
    speed = 500
    
    while True:
        key = input("\n请输入命令 (w/s/a/d/x/q): ").lower().strip()
        
        if key == 'w':
            print(car.move_forward(speed))
        elif key == 's':
            print(car.move_backward(speed))
        elif key == 'a':
            print(car.turn_left(300))
        elif key == 'd':
            print(car.turn_right(300))
        elif key == 'x':
            print(car.stop())
        elif key == 'q':
            car.stop()
            print("退出手动控制模式")
            break
        else:
            print("无效命令！")


def auto_navigation_menu(car):
    """自动寻路菜单"""
    navigator = AutoNavigator(car)
    
    while True:
        print("\n" + "=" * 50)
        print("自动寻路模式")
        print("=" * 50)
        print("1 - 正方形路径")
        print("2 - 三角形路径")
        print("3 - 8字形路径")
        print("4 - 自定义路径")
        print("0 - 返回主菜单")
        print("=" * 50)
        
        choice = input("请选择: ").strip()
        
        if choice == '1':
            path = navigator.square_path()
            navigator.execute_path(path)
        elif choice == '2':
            path = navigator.triangle_path()
            navigator.execute_path(path)
        elif choice == '3':
            path = navigator.eight_path()
            navigator.execute_path(path)
        elif choice == '4':
            print("\n自定义路径格式：方向,速度,时间（例如：F,500,2.0）")
            print("输入 'done' 完成路径输入")
            custom_path = []
            while True:
                cmd = input("输入命令: ").strip()
                if cmd.lower() == 'done':
                    break
                try:
                    parts = cmd.split(',')
                    direction = parts[0].upper()
                    speed = int(parts[1])
                    duration = float(parts[2])
                    custom_path.append((direction, speed, duration))
                    print(f"已添加: {direction}, {speed}, {duration}")
                except:
                    print("格式错误，请重新输入")
            
            if custom_path:
                navigator.execute_path(custom_path)
        elif choice == '0':
            break
        else:
            print("无效选择！")


def main_menu(car):
    """主菜单"""
    while True:
        print("\n" + "=" * 50)
        print("PS2 Hybrid Car 上位机控制系统")
        print("=" * 50)
        print("1 - 手动控制")
        print("2 - 自动寻路")
        print("3 - 查询状态")
        print("4 - 复位编码器")
        print("5 - 切换到PS2模式")
        print("0 - 退出程序")
        print("=" * 50)
        
        choice = input("请选择: ").strip()
        
        if choice == '1':
            manual_control_menu(car)
        elif choice == '2':
            auto_navigation_menu(car)
        elif choice == '3':
            status = car.get_status()
            print(f"\n当前状态: {status}")
        elif choice == '4':
            result = car.reset_encoder()
            print(f"\n{result}")
        elif choice == '5':
            result = car.set_mode(0)
            print(f"\n{result}")
            print("已切换到PS2手柄模式，程序将退出")
            return
        elif choice == '0':
            car.stop()
            print("\n正在退出...")
            return
        else:
            print("无效选择！")


def main():
    """主程序"""
    print("=" * 50)
    print("PS2 Hybrid Car 上位机控制程序")
    print("=" * 50)
    
    # 获取串口号
    port = input("请输入串口号 (默认 COM3): ").strip()
    if not port:
        port = 'COM3'
    
    # 创建控制器
    car = CarController(port)
    
    try:
        # 进入主菜单
        main_menu(car)
    except KeyboardInterrupt:
        print("\n\n程序被用户中断")
    finally:
        car.stop()
        car.close()
        print("程序已退出")


if __name__ == '__main__':
    main()
