# PS2混合小车控制系统（万向轮版本）

## 项目简介

这是一个基于STM32F103C8T6的智能万向轮小车控制系统，支持PS2无线手柄控制和上位机串口控制两种模式。系统采用四个带编码器的JGB37-520直流电机配合万向轮（麦克纳姆轮或全向轮），通过两个L298N驱动模块进行驱动，可实现**全方位移动**（前后、左右平移、原地旋转）和自动寻路功能。

## 主要特性

- ✅ **双模式控制**：支持PS2手柄和上位机串口控制，可随时切换
- ✅ **PS2手柄控制**：
  - **左摇杆**：控制小车全方位平移（前后左右）
  - **JOYL按键**（左摇杆按下）：原地左转
  - **JOYR按键**（右摇杆按下）：原地右转
  - 实时响应，操作灵活
- ✅ **万向轮系统**：
  - 支持前进/后退
  - 支持左平移/右平移
  - 支持原地旋转
- ✅ **上位机控制**：
  - 支持手动控制命令
  - 支持自动寻路功能
  - 编码器反馈，精确定位
- ✅ **编码器反馈**：实时监测车轮转速和位移
- ✅ **PWM调速**：平滑的速度控制（150-900范围）

## 硬件配置

### 主控板
- **MCU**: STM32F103C8T6
  - 72MHz主频
  - 64KB Flash
  - 20KB RAM

### 电机系统
- **电机**: 4个 JGB37-520 带编码器直流电机
- **驱动**: 2个 L298N 电机驱动模块
- **PWM控制**: TIM1 CH1/CH2
- **编码器**: TIM2/TIM3（编码器模式）

### PS2手柄接口
| 引脚 | 功能 | STM32引脚 |
|------|------|-----------|
| DI   | 数据输入 | PB12 |
| CMD  | 命令   | PB13 |
| CS   | 片选   | PB14 |
| CLK  | 时钟   | PB15 |

### 电机驱动接口
| 功能 | STM32引脚 | 说明 |
|------|-----------|------|
| IN1  | PA0 | 左侧电机方向1 |
| IN2  | PA1 | 左侧电机方向2 |
| IN3  | PA4 | 右侧电机方向1 |
| IN4  | PA5 | 右侧电机方向2 |
| ENA  | PA8 (TIM1_CH1) | 左侧电机PWM |
| ENB  | PA9 (TIM1_CH2) | 右侧电机PWM |

### 串口通信
- **接口**: USART3
- **波特率**: 115200
- **引脚**: PB10(TX), PB11(RX)

## 软件架构

### 文件结构

```
PS2_Hybird/
├── Core/
│   ├── Inc/
│   │   ├── ax_ps2.h          # PS2手柄驱动头文件
│   │   ├── motor_control.h   # 电机控制头文件
│   │   ├── main.h
│   │   ├── tim.h
│   │   ├── usart.h
│   │   └── gpio.h
│   └── Src/
│       ├── ax_ps2.c          # PS2手柄驱动实现
│       ├── motor_control.c   # 电机控制实现
│       ├── main.c            # 主程序
│       ├── tim.c             # 定时器配置
│       ├── usart.c           # 串口配置和通信协议
│       └── gpio.c            # GPIO配置
├── car_controller.py         # Python上位机控制程序
├── 上位机使用说明.md         # 详细的上位机使用文档
└── README.md                 # 本文件
```

### 主要模块

#### 1. PS2手柄控制模块（ax_ps2.c）
- PS2手柄通信协议实现
- 摇杆和按键数据读取
- 使用DWT实现微秒级延时

#### 2. 电机控制模块（motor_control.c）
- 电机方向控制（前进/后退/停止）
- PWM速度调节
- 小车基本运动函数：
  - `Car_Forward()` - 前进
  - `Car_Backward()` - 后退
  - `Car_TurnLeft()` - 原地左转
  - `Car_TurnRight()` - 原地右转
  - `Car_Stop()` - 停止
- PS2手柄映射函数：`PS2_Control_Car()`
- 编码器读取和计数

#### 3. 串口通信模块（usart.c）
- UART接收中断处理
- 命令解析和执行
- 支持的命令：
  - `CMD:MOVE,<方向>,<速度>` - 移动控制
  - `CMD:STOP` - 停止
  - `CMD:STATUS` - 状态查询
  - `CMD:RESET_ENC` - 复位编码器
  - `CMD:MODE,<0|1>` - 模式切换

#### 4. 主程序（main.c）
- 系统初始化
- 模式管理（PS2/UART切换）
- 主控制循环

## 快速开始

### 1. 硬件连接

1. **电机连接**：
   - 左侧电机组连接到第一个L298N的OUT1/OUT2
   - 右侧电机组连接到第二个L298N的OUT3/OUT4
   - L298N的IN1-IN4连接到STM32的PA0,PA1,PA4,PA5
   - L298N的ENA,ENB连接到STM32的PA8,PA9

2. **编码器连接**：
   - 左侧编码器A/B相连接到PA0/PA1（TIM2）
   - 右侧编码器A/B相连接到PA6/PA7（TIM3）

3. **PS2手柄接口**：
   - 按照上述引脚表连接

4. **串口连接**：
   - USB转TTL模块连接到PB10(TX)/PB11(RX)

### 2. 编译和下载

1. 使用STM32CubeIDE或Keil MDK打开项目
2. 编译项目
3. 通过ST-Link或J-Link下载到STM32F103C8T6

### 3. PS2手柄使用

1. 上电后默认为PS2手柄模式
2. 打开PS2手柄电源
3. 使用左摇杆控制：
   - 前后推：前进/后退
   - 左右推：原地左转/右转
4. 摇杆有死区设置（±15），避免漂移

### 4. 上位机使用

#### 方式一：使用Python控制程序

```bash
# 安装依赖
pip install pyserial

# 运行控制程序
python car_controller.py
```

程序启动后会提示输入串口号（Windows默认COM3，Linux默认/dev/ttyUSB0），然后进入主菜单：
- 手动控制：使用键盘控制小车运动
- 自动寻路：执行预设路径（正方形、三角形、8字形等）
- 查询状态：读取编码器和模式信息
- 复位编码器：清零编码器计数

#### 方式二：使用串口调试工具

1. 打开串口调试工具（如PuTTY、串口助手等）
2. 配置：115200, 8N1
3. 发送命令（详见《上位机使用说明.md》）

示例命令：
```
CMD:MODE,1          # 切换到上位机模式
CMD:MOVE,F,500     # 前进，速度500
CMD:STOP           # 停止
CMD:STATUS         # 查询状态
```

## 使用示例

### Python自动寻路示例

```python
from car_controller import CarController, AutoNavigator

# 创建控制器
car = CarController('COM3')

# 创建导航器
nav = AutoNavigator(car)

# 执行正方形路径
square = nav.square_path(side_time=2.0, turn_time=0.8, speed=500)
nav.execute_path(square)

# 关闭连接
car.close()
```

### C语言嵌入式控制示例

```c
// PS2手柄控制
AX_PS2_ScanKey(&ps2_data);
PS2_Control_Car(ps2_data.LJoy_LR, ps2_data.LJoy_UD);

// 直接电机控制
Car_Forward(500);      // 前进
HAL_Delay(2000);       // 延时2秒
Car_TurnRight(300);    // 右转
HAL_Delay(800);        // 转向时间
Car_Stop();            // 停止
```

## 参数调整

### PWM速度范围
在 `motor_control.h` 中定义：
```c
#define MAX_PWM 900    // 最大占空比
#define MIN_PWM 150    // 最小占空比
#define DEAD_ZONE 15   // 摇杆死区
```

### 速度滤波
在 `motor_control.c` 中的 `Speed_Filter_Update()` 函数：
```c
*left_filtered = (int16_t)(0.3 * left_target + 0.7 * (*left_filtered));
```
调整系数可改变响应速度（0.3越大响应越快，但越不平滑）

### 转向标定
原地转90度所需时间需要实际测试确定，取决于：
- 电池电压
- 地面摩擦
- 小车重量
- 转向速度

建议测试方法：
1. 使用 `CMD:MOVE,R,300` 命令
2. 计时直到转90度
3. 记录时间，在自动寻路中使用该值

## 故障排除

### 小车不动
1. 检查电源是否充足（推荐7.4V锂电池）
2. 检查L298N使能跳帽是否插好
3. 检查电机线序是否正确
4. 用万用表测试PWM输出（PA8/PA9）

### 手柄无响应
1. 检查PS2接收器LED是否亮起
2. 检查手柄电池是否有电
3. 使用串口打印手柄数据，查看mode值（应为0x73或0x41）
4. 检查接线是否正确

### 串口通信失败
1. 确认波特率115200
2. 检查TX/RX是否接反
3. 确认已切换到上位机模式（`CMD:MODE,1`）
4. 检查串口是否被其他程序占用

### 编码器计数异常
1. 检查编码器供电（通常5V）
2. 确认TIM2/TIM3配置为编码器模式
3. 检查A/B相接线
4. 定期使用 `CMD:RESET_ENC` 清零

## 进阶功能

### 添加PID速度控制
可以在 `motor_control.c` 中添加PID控制器，实现恒速运行：

```c
typedef struct {
    float Kp, Ki, Kd;
    float error, integral, derivative, last_error;
} PID_Controller;

int16_t PID_Update(PID_Controller *pid, int16_t target, int16_t current) {
    pid->error = target - current;
    pid->integral += pid->error;
    pid->derivative = pid->error - pid->last_error;
    pid->last_error = pid->error;
    
    float output = pid->Kp * pid->error + 
                   pid->Ki * pid->integral + 
                   pid->Kd * pid->derivative;
    
    return (int16_t)output;
}
```

### 添加超声波避障
可以添加HC-SR04超声波模块实现自动避障。

### 添加陀螺仪
添加MPU6050可以实现更精确的转向角度控制。

## 文档

- [上位机使用说明](./上位机使用说明.md) - 详细的串口通信协议和编程接口文档
- [car_controller.py](./car_controller.py) - Python控制程序源码

## 许可证

本项目基于开源协议，可自由使用和修改。

## 技术支持

如有问题，请提供：
1. 硬件配置信息
2. 错误现象描述
3. 串口调试信息
4. 代码修改记录（如有）

## 更新日志

### v1.0 (2025-10-27)
- ✅ 完成PS2手柄控制功能
- ✅ 实现左摇杆前后控制和左右转向
- ✅ 添加串口通信协议
- ✅ 支持上位机自动寻路
- ✅ 提供Python控制程序
- ✅ 编写详细使用文档

## 致谢

- PS2手柄驱动参考自XTARK@塔克创新
- STM32 HAL库由STMicroelectronics提供

---

**项目名称**：PS2 Hybrid Car Control System  
**版本**：v1.0  
**日期**：2025-10-27
