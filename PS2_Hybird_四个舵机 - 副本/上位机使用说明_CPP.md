# PS2混合小车上位机控制说明 (C++版本)

## 系统概述

本系统是基于STM32F103C8T6的智能小车控制系统的C++上位机实现，支持两种控制模式：
- **PS2手柄控制模式**：通过PS2无线手柄实时控制小车运动
- **上位机控制模式**：通过串口接收上位机命令，实现自动寻路和精确控制

## 硬件配置

### 主控芯片
- STM32F103C8T6（72MHz主频）

### 电机驱动
- 4个JGB37-520带编码器直流电机
- 2个L298N电机驱动模块
- PWM控制（TIM1 CH1/CH2）

### 传感器
- 编码器反馈（TIM2/TIM3编码器模式）

### 通信接口
- USART3：115200波特率，8位数据位，1位停止位，无校验
- 引脚：PB10(TX)，PB11(RX)

### PS2手柄接口
- PB12: DI（数据输入）
- PB13: CMD（命令）
- PB14: CS（片选）
- PB15: CLK（时钟）

## C++上位机程序

### 编译环境要求

#### Windows环境
- **编译器**：MinGW-w64 GCC 或 MSVC
- **C++标准**：C++17或更高
- **操作系统**：Windows 7及以上

#### Linux环境
- **编译器**：GCC 7.0+
- **C++标准**：C++17或更高
- **操作系统**：Ubuntu 18.04+, Debian 10+, Fedora 28+等

### 编译方法

#### Windows下编译

**使用MinGW-w64:**
```bash
# 基本编译
g++ -std=c++17 car_controller.cpp -o car_controller.exe

# 优化编译
g++ -std=c++17 -O2 car_controller.cpp -o car_controller.exe

# 静态链接（便于分发）
g++ -std=c++17 -O2 -static car_controller.cpp -o car_controller.exe
```

**使用MSVC (Visual Studio):**
```bash
# 打开 x64 Native Tools Command Prompt
cl /EHsc /std:c++17 car_controller.cpp /Fe:car_controller.exe
```

#### Linux下编译

```bash
# 基本编译
g++ -std=c++17 car_controller.cpp -o car_controller

# 优化编译
g++ -std=c++17 -O2 car_controller.cpp -o car_controller

# 添加调试信息
g++ -std=c++17 -g car_controller.cpp -o car_controller

# 设置可执行权限
chmod +x car_controller
```

#### 跨平台编译（使用CMake）

创建 `CMakeLists.txt` 文件：
```cmake
cmake_minimum_required(VERSION 3.10)
project(CarController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(car_controller car_controller.cpp)

# Windows下需要链接ws2_32库
if(WIN32)
    target_link_libraries(car_controller ws2_32)
endif()
```

编译步骤：
```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### 运行程序

#### Windows
```bash
# 直接运行
car_controller.exe

# 或者双击 car_controller.exe
```

#### Linux
```bash
# 需要串口访问权限
sudo usermod -a -G dialout $USER
# 注销后重新登录

# 运行程序
./car_controller

# 或者使用sudo（不推荐）
sudo ./car_controller
```

### 串口配置

#### Windows串口名称
- COM1, COM2, COM3, ... COM9
- 对于COM10及以上：`\\.\COM10`, `\\.\COM11` 等

查看可用串口：
```powershell
# PowerShell
Get-WmiObject Win32_SerialPort | Select-Object DeviceID,Description

# 或者在设备管理器中查看
```

#### Linux串口名称
- USB转串口：`/dev/ttyUSB0`, `/dev/ttyUSB1` 等
- 内置串口：`/dev/ttyS0`, `/dev/ttyS1` 等
- Arduino：`/dev/ttyACM0` 等

查看可用串口：
```bash
# 列出所有串口设备
ls /dev/tty*

# 查看USB串口
ls /dev/ttyUSB*

# 查看串口详细信息
dmesg | grep tty
```

## 程序功能说明

### 主菜单

程序启动后会显示主菜单：
```
==================================================
PS2 Hybrid Car 上位机控制系统
==================================================
1 - 手动控制
2 - 自动寻路
3 - 查询状态
4 - 复位编码器
5 - 切换到PS2模式
0 - 退出程序
==================================================
```

### 1. 手动控制模式

进入手动控制模式后，可以使用键盘实时控制小车：

```
==================================================
手动控制模式
==================================================
w - 前进    s - 后退
a - 左转    d - 右转
x - 停止
q - 退出手动模式
==================================================
```

**操作示例：**
```
请输入命令 (w/s/a/d/x/q): w
OK: Moving forward at 500

请输入命令 (w/s/a/d/x/q): d
OK: Turning right at 300

请输入命令 (w/s/a/d/x/q): x
OK: Car stopped
```

### 2. 自动寻路模式

支持预设路径和自定义路径：

```
==================================================
自动寻路模式
==================================================
1 - 正方形路径
2 - 三角形路径
3 - 8字形路径
4 - 自定义路径
0 - 返回主菜单
==================================================
```

#### 预设路径

**正方形路径：** 小车行走一个正方形
- 每边前进2秒
- 每次右转0.8秒（约90度）
- 重复4次

**三角形路径：** 小车行走一个三角形
- 每边前进2秒
- 每次右转1.0秒（约120度）
- 重复3次

**8字形路径：** 小车行走8字形
- 前进→右转→前进→左转

#### 自定义路径

格式：`方向,速度,时间`

示例：
```
输入命令: F,500,2.0
已添加: F, 500, 2.0

输入命令: R,300,0.8
已添加: R, 300, 0.8

输入命令: F,500,1.5
已添加: F, 500, 1.5

输入命令: done
```

**方向参数：**
- `F` - 前进
- `B` - 后退
- `L` - 左转
- `R` - 右转

**速度参数：** 150-900（建议300-600）

**时间参数：** 秒（支持小数）

### 3. 查询状态

返回小车当前状态：
```
当前状态: STATUS: LeftEnc=12345, RightEnc=12340, Mode=1
```

### 4. 复位编码器

将编码器计数清零：
```
OK: Encoder reset
```

### 5. 切换到PS2模式

切换到PS2手柄控制，程序将退出：
```
OK: Mode set to 0 (PS2)
已切换到PS2手柄模式，程序将退出
```

## C++ API参考

### CarController类

```cpp
// 构造函数
CarController(const string& portName, int baudRate = 115200);

// 移动控制
string moveForward(int speed = 500);      // 前进
string moveBackward(int speed = 500);     // 后退
string turnLeft(int speed = 300);         // 左转
string turnRight(int speed = 300);        // 右转
string stop();                            // 停止

// 状态查询
string getStatus();                       // 获取状态
string resetEncoder();                    // 复位编码器
string setMode(int mode);                 // 设置模式(0=PS2, 1=UART)

// 底层通信
string sendCommand(const string& command); // 发送自定义命令
```

### AutoNavigator类

```cpp
// 构造函数
AutoNavigator(CarController* controller);

// 执行路径
void executePath(const vector<PathStep>& path);

// 预设路径
vector<PathStep> squarePath(double sideTime = 2.0, 
                            double turnTime = 0.8, 
                            int speed = 500);

vector<PathStep> trianglePath(double sideTime = 2.0, 
                              double turnTime = 1.0, 
                              int speed = 500);

vector<PathStep> eightPath(double radiusTime = 1.5, 
                          int speed = 500);
```

### PathStep结构体

```cpp
struct PathStep {
    char direction;  // 方向: F, B, L, R
    int speed;       // 速度: 150-900
    double duration; // 持续时间(秒)
};
```

## C++编程示例

### 示例1：简单的前进后退

```cpp
#include "car_controller.cpp"

int main() {
    try {
        // 创建控制器
        CarController car("COM3");
        
        // 前进2秒
        cout << car.moveForward(500) << endl;
        this_thread::sleep_for(chrono::seconds(2));
        
        // 停止
        cout << car.stop() << endl;
        this_thread::sleep_for(chrono::milliseconds(500));
        
        // 后退2秒
        cout << car.moveBackward(500) << endl;
        this_thread::sleep_for(chrono::seconds(2));
        
        // 停止
        cout << car.stop() << endl;
        
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
```

### 示例2：正方形路径

```cpp
#include "car_controller.cpp"

int main() {
    try {
        CarController car("COM3");
        AutoNavigator navigator(&car);
        
        // 生成正方形路径
        auto path = navigator.squarePath(2.0, 0.8, 500);
        
        // 执行路径
        navigator.executePath(path);
        
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
```

### 示例3：自定义复杂路径

```cpp
#include "car_controller.cpp"

int main() {
    try {
        CarController car("COM3");
        AutoNavigator navigator(&car);
        
        // 创建自定义路径
        vector<PathStep> customPath = {
            PathStep('F', 500, 2.0),   // 前进2秒
            PathStep('R', 300, 0.4),   // 右转45度
            PathStep('F', 500, 1.5),   // 前进1.5秒
            PathStep('L', 300, 0.8),   // 左转90度
            PathStep('F', 500, 2.0),   // 前进2秒
            PathStep('R', 300, 0.4),   // 右转45度
            PathStep('B', 400, 1.0)    // 后退1秒
        };
        
        // 执行路径
        navigator.executePath(customPath);
        
        // 查询最终状态
        cout << "\n最终状态: " << car.getStatus() << endl;
        
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
```

### 示例4：基于编码器的精确控制

```cpp
#include "car_controller.cpp"
#include <sstream>

// 解析状态字符串获取编码器值
pair<int, int> parseEncoders(const string& status) {
    // STATUS: LeftEnc=12345, RightEnc=12340, Mode=1
    int leftEnc = 0, rightEnc = 0;
    
    size_t leftPos = status.find("LeftEnc=");
    size_t rightPos = status.find("RightEnc=");
    
    if (leftPos != string::npos) {
        leftEnc = stoi(status.substr(leftPos + 8));
    }
    if (rightPos != string::npos) {
        rightEnc = stoi(status.substr(rightPos + 9));
    }
    
    return {leftEnc, rightEnc};
}

// 精确移动指定编码器计数
void moveByEncoder(CarController& car, int targetCount, int speed = 500) {
    car.resetEncoder();
    car.moveForward(speed);
    
    while (true) {
        string status = car.getStatus();
        auto [leftEnc, rightEnc] = parseEncoders(status);
        
        cout << "当前编码器: Left=" << leftEnc 
             << ", Right=" << rightEnc << "\r" << flush;
        
        if (abs(leftEnc) >= targetCount) {
            car.stop();
            cout << "\n到达目标位置！" << endl;
            break;
        }
        
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

int main() {
    try {
        CarController car("COM3");
        
        // 移动1000个编码器计数
        cout << "开始移动..." << endl;
        moveByEncoder(car, 1000, 500);
        
        this_thread::sleep_for(chrono::seconds(1));
        
        // 返回原点
        cout << "返回原点..." << endl;
        moveByEncoder(car, 1000, 500);
        
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }
    
    return 0;
}
```

### 示例5：多线程状态监控

```cpp
#include "car_controller.cpp"
#include <atomic>

atomic<bool> monitoring(true);

// 状态监控线程
void monitorStatus(CarController& car) {
    while (monitoring) {
        string status = car.getStatus();
        cout << "[监控] " << status << endl;
        this_thread::sleep_for(chrono::seconds(1));
    }
}

int main() {
    try {
        CarController car("COM3");
        
        // 启动监控线程
        thread monitorThread(monitorStatus, ref(car));
        
        // 执行路径
        AutoNavigator navigator(&car);
        auto path = navigator.squarePath();
        navigator.executePath(path);
        
        // 停止监控
        monitoring = false;
        monitorThread.join();
        
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        monitoring = false;
        return 1;
    }
    
    return 0;
}
```

## 串口通信协议

### 通信参数
- **波特率**：115200
- **数据位**：8
- **停止位**：1
- **校验位**：无
- **流控**：无

### 命令格式

所有命令采用ASCII文本格式，以换行符（`\n`）结束。

#### 移动控制命令
```
CMD:MOVE,<方向>,<速度>
```

#### 停止命令
```
CMD:STOP
```

#### 状态查询命令
```
CMD:STATUS
```

#### 编码器复位命令
```
CMD:RESET_ENC
```

#### 模式切换命令
```
CMD:MODE,<模式>
```

详细协议说明请参考 Python 版本文档。

## 性能优化建议

### 1. 编译优化

```bash
# 启用O2优化
g++ -std=c++17 -O2 car_controller.cpp -o car_controller

# 启用O3优化（更激进）
g++ -std=c++17 -O3 car_controller.cpp -o car_controller

# 针对本机CPU优化
g++ -std=c++17 -O2 -march=native car_controller.cpp -o car_controller
```

### 2. 减小可执行文件大小

```bash
# 去除调试信息
strip car_controller

# 编译时优化大小
g++ -std=c++17 -Os car_controller.cpp -o car_controller
```

### 3. 串口通信优化

- 调整 `SerialPort::read()` 中的延迟时间
- 根据实际情况调整超时参数
- 对于实时性要求高的应用，可以使用异步IO

## 常见问题

### Q1: Windows下提示"无法打开串口"
**A**:
1. 检查串口号是否正确（COM1-COM9直接使用，COM10及以上需要使用`\\.\COM10`格式）
2. 确认串口未被其他程序占用
3. 检查USB驱动是否正确安装
4. 尝试以管理员权限运行

### Q2: Linux下提示"Permission denied"
**A**:
```bash
# 方法1：添加用户到dialout组
sudo usermod -a -G dialout $USER
# 注销后重新登录

# 方法2：临时修改权限
sudo chmod 666 /dev/ttyUSB0

# 方法3：使用sudo运行（不推荐）
sudo ./car_controller
```

### Q3: 编译时出现"error: 'this_thread' is not a member of 'std'"
**A**:
确保使用C++11或更高标准：
```bash
g++ -std=c++17 car_controller.cpp -o car_controller
```

### Q4: Windows下编译提示找不到头文件
**A**:
确保已安装MinGW-w64或Visual Studio，并正确配置环境变量。

### Q5: 小车不响应命令
**A**:
1. 检查串口连接是否正常
2. 确认波特率设置正确（115200）
3. 检查是否已切换到上位机模式
4. 使用串口调试工具验证STM32是否有响应

### Q6: 如何在代码中使用其他串口参数
**A**:
修改 `SerialPort` 构造函数中的串口配置代码，调整波特率、数据位、停止位等参数。

## 与Python版本的对比

| 特性 | C++版本 | Python版本 |
|------|---------|-----------|
| 性能 | 更快，资源占用少 | 较慢，但足够 |
| 部署 | 需要编译，无依赖 | 需要Python环境和pyserial库 |
| 跨平台 | 支持Windows/Linux | 支持Windows/Linux/macOS |
| 开发速度 | 较慢 | 快速 |
| 内存占用 | 极低（<1MB） | 较高（~20MB） |
| 启动速度 | 极快 | 较慢 |
| 适用场景 | 嵌入式、资源受限环境 | 快速原型开发、科研 |

## 项目结构建议

```
project/
├── car_controller.cpp          # 主程序
├── CMakeLists.txt              # CMake配置（可选）
├── Makefile                    # Makefile（可选）
├── README.md                   # 说明文档
└── examples/                   # 示例代码
    ├── simple_move.cpp
    ├── square_path.cpp
    ├── encoder_control.cpp
    └── monitor.cpp
```

## 进阶开发

### 将代码模块化

可以将代码拆分为多个文件：

**serial_port.h / serial_port.cpp** - 串口通信类

**car_controller.h / car_controller.cpp** - 小车控制类

**auto_navigator.h / auto_navigator.cpp** - 自动寻路类

**main.cpp** - 主程序

### 添加日志功能

```cpp
#include <fstream>
#include <ctime>

class Logger {
private:
    ofstream logFile;
    
public:
    Logger(const string& filename) {
        logFile.open(filename, ios::app);
    }
    
    void log(const string& message) {
        auto t = time(nullptr);
        auto tm = *localtime(&t);
        
        logFile << put_time(&tm, "%Y-%m-%d %H:%M:%S") 
                << " - " << message << endl;
        logFile.flush();
    }
    
    ~Logger() {
        logFile.close();
    }
};
```

### 添加配置文件支持

创建 `config.txt`:
```ini
port=COM3
baudrate=115200
default_speed=500
turn_speed=300
```

### 添加图形界面（Qt）

可以使用Qt框架开发图形界面版本，提供更友好的用户体验。

## 技术支持

如有问题，请提供以下信息：
1. 操作系统版本
2. 编译器版本
3. 错误信息和日志
4. 硬件连接情况

---

**版本**：v1.0 (C++ Edition)  
**更新日期**：2025-11-02  
**作者**：PS2 Hybrid Car Control System  
**基于**：Python版本 v1.0
