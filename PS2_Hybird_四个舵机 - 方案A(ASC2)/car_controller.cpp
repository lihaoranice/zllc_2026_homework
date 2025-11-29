/**
 * PS2 Hybrid Car 上位机控制程序 (C++版本)
 * 支持手动控制和自动寻路功能
 * 
 * 编译方法:
 * Windows: g++ -std=c++17 car_controller.cpp -o car_controller.exe
 * Linux:   g++ -std=c++17 car_controller.cpp -o car_controller
 */

#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstring>

// Windows串口头文件
#ifdef _WIN32
#include <windows.h>
#else
// Linux串口头文件
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

using namespace std;

/**
 * 串口通信类
 */
class SerialPort {
private:
#ifdef _WIN32
    HANDLE hSerial;
    bool isOpen;
#else
    int fd;
#endif

public:
    /**
     * 构造函数
     * @param portName 串口名称 (Windows: "COM3", Linux: "/dev/ttyUSB0")
     * @param baudRate 波特率
     */
    SerialPort(const string& portName, int baudRate = 115200) {
#ifdef _WIN32
        isOpen = false;
        // 打开串口
        hSerial = CreateFileA(portName.c_str(),
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              NULL,
                              OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL,
                              NULL);
        
        if (hSerial == INVALID_HANDLE_VALUE) {
            throw runtime_error("无法打开串口: " + portName);
        }
        
        // 配置串口参数
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw runtime_error("获取串口状态失败");
        }
        
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw runtime_error("设置串口参数失败");
        }
        
        // 设置超时
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 50;
        timeouts.ReadTotalTimeoutConstant = 50;
        timeouts.ReadTotalTimeoutMultiplier = 10;
        timeouts.WriteTotalTimeoutConstant = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        
        if (!SetCommTimeouts(hSerial, &timeouts)) {
            CloseHandle(hSerial);
            throw runtime_error("设置串口超时失败");
        }
        
        isOpen = true;
        cout << "成功连接到 " << portName << "，波特率 " << baudRate << endl;
#else
        // Linux串口实现
        fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd == -1) {
            throw runtime_error("无法打开串口: " + portName);
        }
        
        struct termios options;
        tcgetattr(fd, &options);
        
        // 设置波特率
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        
        // 8N1
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        // 禁用硬件流控
        options.c_cflag &= ~CRTSCTS;
        options.c_cflag |= CREAD | CLOCAL;
        
        // 原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        
        // 设置超时
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 5;
        
        tcsetattr(fd, TCSANOW, &options);
        
        cout << "成功连接到 " << portName << "，波特率 " << baudRate << endl;
#endif
    }
    
    /**
     * 析构函数
     */
    ~SerialPort() {
        close();
    }
    
    /**
     * 写入数据
     * @param data 要写入的数据
     * @return 成功返回true
     */
    bool write(const string& data) {
#ifdef _WIN32
        if (!isOpen) return false;
        
        DWORD bytesWritten;
        string dataWithNewline = data + "\n";
        
        if (!WriteFile(hSerial, dataWithNewline.c_str(), 
                      dataWithNewline.length(), &bytesWritten, NULL)) {
            return false;
        }
        return true;
#else
        string dataWithNewline = data + "\n";
        ssize_t n = ::write(fd, dataWithNewline.c_str(), dataWithNewline.length());
        return n > 0;
#endif
    }
    
    /**
     * 读取数据
     * @return 读取到的字符串
     */
    string read() {
#ifdef _WIN32
        if (!isOpen) return "";
        
        char buffer[256];
        DWORD bytesRead;
        string result;
        
        this_thread::sleep_for(chrono::milliseconds(50));
        
        while (true) {
            if (!ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL)) {
                break;
            }
            if (bytesRead == 0) break;
            
            buffer[bytesRead] = '\0';
            result += buffer;
        }
        
        // 移除末尾的换行符
        while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
            result.pop_back();
        }
        
        return result.empty() ? "No response" : result;
#else
        char buffer[256];
        string result;
        
        this_thread::sleep_for(chrono::milliseconds(50));
        
        while (true) {
            ssize_t n = ::read(fd, buffer, sizeof(buffer) - 1);
            if (n <= 0) break;
            
            buffer[n] = '\0';
            result += buffer;
        }
        
        // 移除末尾的换行符
        while (!result.empty() && (result.back() == '\n' || result.back() == '\r')) {
            result.pop_back();
        }
        
        return result.empty() ? "No response" : result;
#endif
    }
    
    /**
     * 关闭串口
     */
    void close() {
#ifdef _WIN32
        if (isOpen) {
            CloseHandle(hSerial);
            isOpen = false;
            cout << "串口已关闭" << endl;
        }
#else
        if (fd != -1) {
            ::close(fd);
            fd = -1;
            cout << "串口已关闭" << endl;
        }
#endif
    }
};

/**
 * 小车控制器类
 */
class CarController {
private:
    SerialPort* serial;
    
public:
    /**
     * 构造函数
     * @param portName 串口名称
     * @param baudRate 波特率
     */
    CarController(const string& portName, int baudRate = 115200) {
        serial = new SerialPort(portName, baudRate);
        
        // 等待连接稳定
        this_thread::sleep_for(chrono::seconds(2));
        
        // 切换到上位机控制模式
        string response = setMode(1);
        cout << response << endl;
    }
    
    /**
     * 析构函数
     */
    ~CarController() {
        if (serial) {
            delete serial;
        }
    }
    
    /**
     * 发送命令并接收响应
     * @param command 命令字符串
     * @return 响应字符串
     */
    string sendCommand(const string& command) {
        if (!serial->write(command)) {
            return "Error: 发送命令失败";
        }
        return serial->read();
    }
    
    /**
     * 前进
     * @param speed 速度 (150-900)
     */
    string moveForward(int speed = 500) {
        return sendCommand("CMD:MOVE,F," + to_string(speed));
    }
    
    /**
     * 后退
     * @param speed 速度 (150-900)
     */
    string moveBackward(int speed = 500) {
        return sendCommand("CMD:MOVE,B," + to_string(speed));
    }
    
    /**
     * 原地左转
     * @param speed 速度 (150-900)
     */
    string turnLeft(int speed = 300) {
        return sendCommand("CMD:MOVE,L," + to_string(speed));
    }
    
    /**
     * 原地右转
     * @param speed 速度 (150-900)
     */
    string turnRight(int speed = 300) {
        return sendCommand("CMD:MOVE,R," + to_string(speed));
    }
    
    /**
     * 停止
     */
    string stop() {
        return sendCommand("CMD:STOP");
    }
    
    /**
     * 获取状态
     */
    string getStatus() {
        return sendCommand("CMD:STATUS");
    }
    
    /**
     * 复位编码器
     */
    string resetEncoder() {
        return sendCommand("CMD:RESET_ENC");
    }
    
    /**
     * 设置控制模式
     * @param mode 0=PS2, 1=UART
     */
    string setMode(int mode) {
        return sendCommand("CMD:MODE," + to_string(mode));
    }
};

/**
 * 路径步骤结构体
 */
struct PathStep {
    char direction;  // 方向: F, B, L, R
    int speed;       // 速度
    double duration; // 持续时间(秒)
    
    PathStep(char dir, int spd, double dur) 
        : direction(dir), speed(spd), duration(dur) {}
};

/**
 * 自动寻路类
 */
class AutoNavigator {
private:
    CarController* car;
    
public:
    /**
     * 构造函数
     * @param controller 小车控制器指针
     */
    AutoNavigator(CarController* controller) : car(controller) {}
    
    /**
     * 执行路径
     * @param path 路径步骤向量
     */
    void executePath(const vector<PathStep>& path) {
        cout << "\n开始执行自动寻路..." << endl;
        cout << string(50, '=') << endl;
        
        car->resetEncoder();
        this_thread::sleep_for(chrono::milliseconds(500));
        
        int step = 1;
        for (const auto& p : path) {
            cout << "\n步骤 " << step << ": ";
            
            string action;
            switch (p.direction) {
                case 'F':
                    cout << "前进，速度=" << p.speed 
                         << "，持续" << fixed << setprecision(1) 
                         << p.duration << "秒" << endl;
                    action = car->moveForward(p.speed);
                    break;
                case 'B':
                    cout << "后退，速度=" << p.speed 
                         << "，持续" << fixed << setprecision(1) 
                         << p.duration << "秒" << endl;
                    action = car->moveBackward(p.speed);
                    break;
                case 'L':
                    cout << "左转，速度=" << p.speed 
                         << "，持续" << fixed << setprecision(1) 
                         << p.duration << "秒" << endl;
                    action = car->turnLeft(p.speed);
                    break;
                case 'R':
                    cout << "右转，速度=" << p.speed 
                         << "，持续" << fixed << setprecision(1) 
                         << p.duration << "秒" << endl;
                    action = car->turnRight(p.speed);
                    break;
                default:
                    cout << "未知方向: " << p.direction << endl;
                    step++;
                    continue;
            }
            
            // 执行动作
            this_thread::sleep_for(chrono::milliseconds(
                static_cast<int>(p.duration * 1000)));
            car->stop();
            this_thread::sleep_for(chrono::milliseconds(500));
            
            // 查询状态
            string status = car->getStatus();
            cout << "  状态: " << status << endl;
            
            step++;
        }
        
        cout << "\n" << string(50, '=') << endl;
        cout << "自动寻路完成！" << endl;
    }
    
    /**
     * 生成正方形路径
     */
    vector<PathStep> squarePath(double sideTime = 2.0, 
                                double turnTime = 0.8, 
                                int speed = 500) {
        vector<PathStep> path;
        for (int i = 0; i < 4; i++) {
            path.push_back(PathStep('F', speed, sideTime));
            path.push_back(PathStep('R', 300, turnTime));
        }
        return path;
    }
    
    /**
     * 生成三角形路径
     */
    vector<PathStep> trianglePath(double sideTime = 2.0, 
                                   double turnTime = 1.0, 
                                   int speed = 500) {
        vector<PathStep> path;
        for (int i = 0; i < 3; i++) {
            path.push_back(PathStep('F', speed, sideTime));
            path.push_back(PathStep('R', 300, turnTime));
        }
        return path;
    }
    
    /**
     * 生成8字形路径
     */
    vector<PathStep> eightPath(double radiusTime = 1.5, int speed = 500) {
        vector<PathStep> path = {
            PathStep('F', speed, 1.0),
            PathStep('R', 250, radiusTime),
            PathStep('F', speed, 1.0),
            PathStep('L', 250, radiusTime)
        };
        return path;
    }
};

/**
 * 打印分隔线
 */
void printSeparator() {
    cout << string(50, '=') << endl;
}

/**
 * 手动控制菜单
 */
void manualControlMenu(CarController& car) {
    cout << "\n";
    printSeparator();
    cout << "手动控制模式" << endl;
    printSeparator();
    cout << "w - 前进    s - 后退" << endl;
    cout << "a - 左转    d - 右转" << endl;
    cout << "x - 停止" << endl;
    cout << "q - 退出手动模式" << endl;
    printSeparator();
    
    int speed = 500;
    string input;
    
    while (true) {
        cout << "\n请输入命令 (w/s/a/d/x/q): ";
        cin >> input;
        
        if (input.length() != 1) {
            cout << "无效命令！" << endl;
            continue;
        }
        
        char key = tolower(input[0]);
        
        switch (key) {
            case 'w':
                cout << car.moveForward(speed) << endl;
                break;
            case 's':
                cout << car.moveBackward(speed) << endl;
                break;
            case 'a':
                cout << car.turnLeft(300) << endl;
                break;
            case 'd':
                cout << car.turnRight(300) << endl;
                break;
            case 'x':
                cout << car.stop() << endl;
                break;
            case 'q':
                car.stop();
                cout << "退出手动控制模式" << endl;
                return;
            default:
                cout << "无效命令！" << endl;
        }
    }
}

/**
 * 自动寻路菜单
 */
void autoNavigationMenu(CarController& car) {
    AutoNavigator navigator(&car);
    string choice;
    
    while (true) {
        cout << "\n";
        printSeparator();
        cout << "自动寻路模式" << endl;
        printSeparator();
        cout << "1 - 正方形路径" << endl;
        cout << "2 - 三角形路径" << endl;
        cout << "3 - 8字形路径" << endl;
        cout << "4 - 自定义路径" << endl;
        cout << "0 - 返回主菜单" << endl;
        printSeparator();
        
        cout << "请选择: ";
        cin >> choice;
        
        if (choice == "1") {
            auto path = navigator.squarePath();
            navigator.executePath(path);
        }
        else if (choice == "2") {
            auto path = navigator.trianglePath();
            navigator.executePath(path);
        }
        else if (choice == "3") {
            auto path = navigator.eightPath();
            navigator.executePath(path);
        }
        else if (choice == "4") {
            cout << "\n自定义路径格式：方向,速度,时间（例如：F,500,2.0）" << endl;
            cout << "输入 'done' 完成路径输入" << endl;
            
            vector<PathStep> customPath;
            cin.ignore(); // 清除缓冲区
            
            while (true) {
                cout << "输入命令: ";
                string cmd;
                getline(cin, cmd);
                
                if (cmd == "done" || cmd == "DONE") {
                    break;
                }
                
                // 解析命令
                stringstream ss(cmd);
                string dirStr, speedStr, durationStr;
                
                if (getline(ss, dirStr, ',') && 
                    getline(ss, speedStr, ',') && 
                    getline(ss, durationStr)) {
                    
                    try {
                        char direction = toupper(dirStr[0]);
                        int speed = stoi(speedStr);
                        double duration = stod(durationStr);
                        
                        customPath.push_back(PathStep(direction, speed, duration));
                        cout << "已添加: " << direction << ", " 
                             << speed << ", " << duration << endl;
                    }
                    catch (...) {
                        cout << "格式错误，请重新输入" << endl;
                    }
                }
                else {
                    cout << "格式错误，请重新输入" << endl;
                }
            }
            
            if (!customPath.empty()) {
                navigator.executePath(customPath);
            }
        }
        else if (choice == "0") {
            break;
        }
        else {
            cout << "无效选择！" << endl;
        }
    }
}

/**
 * 主菜单
 */
void mainMenu(CarController& car) {
    string choice;
    
    while (true) {
        cout << "\n";
        printSeparator();
        cout << "PS2 Hybrid Car 上位机控制系统" << endl;
        printSeparator();
        cout << "1 - 手动控制" << endl;
        cout << "2 - 自动寻路" << endl;
        cout << "3 - 查询状态" << endl;
        cout << "4 - 复位编码器" << endl;
        cout << "5 - 切换到PS2模式" << endl;
        cout << "0 - 退出程序" << endl;
        printSeparator();
        
        cout << "请选择: ";
        cin >> choice;
        
        if (choice == "1") {
            manualControlMenu(car);
        }
        else if (choice == "2") {
            autoNavigationMenu(car);
        }
        else if (choice == "3") {
            string status = car.getStatus();
            cout << "\n当前状态: " << status << endl;
        }
        else if (choice == "4") {
            string result = car.resetEncoder();
            cout << "\n" << result << endl;
        }
        else if (choice == "5") {
            string result = car.setMode(0);
            cout << "\n" << result << endl;
            cout << "已切换到PS2手柄模式，程序将退出" << endl;
            return;
        }
        else if (choice == "0") {
            car.stop();
            cout << "\n正在退出..." << endl;
            return;
        }
        else {
            cout << "无效选择！" << endl;
        }
    }
}

/**
 * 主程序入口
 */
int main() {
    printSeparator();
    cout << "PS2 Hybrid Car 上位机控制程序 (C++版本)" << endl;
    printSeparator();
    
    // 获取串口号
    cout << "请输入串口号 (Windows默认: COM3, Linux默认: /dev/ttyUSB0): ";
    string port;
    getline(cin, port);
    
    if (port.empty()) {
#ifdef _WIN32
        port = "COM3";
#else
        port = "/dev/ttyUSB0";
#endif
    }
    
    try {
        // 创建控制器
        CarController car(port);
        
        // 进入主菜单
        mainMenu(car);
        
        car.stop();
    }
    catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        return 1;
    }
    
    cout << "程序已退出" << endl;
    return 0;
}
