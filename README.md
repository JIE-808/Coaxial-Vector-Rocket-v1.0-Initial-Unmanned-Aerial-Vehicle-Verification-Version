# 共轴矢量火箭 v1.1 - 初始无人机验证版本

## 项目简介

本项目是基于 STM32H723VGT6 微控制器的共轴矢量火箭/无人机飞行控制系统验证平台。该系统集成了姿态解算、推力矢量控制（TVC）、定高控制等核心飞控功能，用于验证共轴矢量推进技术在小型无人飞行器上的应用。

**GitHub 仓库**: [Coaxial-Vector-Rocket-v1.0-Initial-Unmanned-Aerial-Vehicle-Verification-Version](https://github.com/JIE-808/Coaxial-Vector-Rocket-v1.0-Initial-Unmanned-Aerial-Vehicle-Verification-Version.git)

---

## 硬件平台

### 主控芯片
- **MCU**: STM32H723VGT6
  - **内核**: ARM Cortex-M7 @ 550MHz
  - **Flash**: 1MB
  - **RAM**: 564KB (包括 DTCM/ITCM/SRAM)
  - **封装**: LQFP100

### 主要传感器与外设
| 外设模块 | 型号/接口 | 功能说明 |
|---------|----------|---------|
| **IMU** | BMI088 (SPI) | 6轴惯性测量单元（3轴加速度计 + 3轴陀螺仪） |
| **超声波** | UART5 | 定高测距传感器 |
| **LCD** | SPI2 | 实时状态显示屏 |
| **遥控接收** | SBUS (UART) | 遥控器信号接收 |
| **电调输出** | PWM (TIM) | 电机控制信号 |
| **电池监测** | ADC1 | 电池电压采集 |
| **USB CDC** | USB Device | 虚拟串口调试输出 |

---

## 软件架构

### 系统框架
```
├── Core/                    # STM32 HAL 库核心代码
│   ├── Inc/                 # 外设配置头文件
│   └── Src/                 # 主程序与外设初始化
├── BSP/                     # 板级支持包（业务逻辑层）
│   ├── BMI088/              # IMU 驱动
│   ├── attitude.c/h         # 姿态解算（四元数/欧拉角）
│   ├── tvc_pid.c/h          # 推力矢量控制 PID
│   ├── altitude_hold.c/h    # 定高控制算法
│   ├── esc_pwm.c/h          # 电调 PWM 输出
│   ├── ultrasonic.c/h       # 超声波测距
│   ├── vbat_adc.c/h         # 电池电压监测
│   ├── imu_heat.c/h         # IMU 温度控制
│   ├── lcd.c/h              # LCD 显示驱动
│   ├── usb_printf.c/h       # USB 调试输出
│   └── button.c/h           # 按键输入处理
├── Drivers/                 # STM32 HAL 驱动库
├── Middlewares/             # USB 中间件
├── USB_DEVICE/              # USB CDC 设备配置
└── cmake/                   # CMake 构建脚本
```

### 核心功能模块

#### 1. 姿态解算 (`attitude.c/h`)
- **功能**: 融合 BMI088 的加速度计和陀螺仪数据，计算飞行器实时姿态
- **算法**: 四元数姿态解算 + 欧拉角转换
- **输出**: Roll（横滚）、Pitch（俯仰）、Yaw（偏航）角度

#### 2. 推力矢量控制 (`tvc_pid.c/h`)
- **功能**: 通过 PID 控制器调节推力矢量方向，实现姿态稳定
- **控制对象**: 伺服舵机/矢量喷管
- **控制模式**: 串级 PID（角度环 + 角速度环）

#### 3. 定高控制 (`altitude_hold.c/h`)
- **功能**: 基于超声波传感器的高度保持算法
- **控制策略**: PID 高度闭环控制
- **应用场景**: 悬停、定高飞行

#### 4. IMU 驱动 (`BMI088/`)
- **通信接口**: SPI2 (DMA 加速)
- **数据采集**: 加速度计 ±24g、陀螺仪 ±2000°/s
- **中断支持**: 数据就绪中断（ACC_INT/GYRO_INT）

#### 5. 电调控制 (`esc_pwm.c/h`)
- **输出方式**: PWM 信号（50Hz / 1000-2000μs）
- **定时器**: TIM1/TIM8 高级定时器
- **功能**: 电机转速控制、解锁/上锁管理

#### 6. 遥控接收 (SBUS)
- **协议**: SBUS (100kbps 反相串口)
- **通道数**: 16 通道
- **接收方式**: UART DMA 接收
- **功能**: 遥控器指令解析、失控保护

#### 7. 电池监测 (`vbat_adc.c/h`)
- **采样方式**: ADC1 连续转换模式
- **监测参数**: 电池电压、电流（可选）
- **保护功能**: 低压报警

#### 8. LCD 显示 (`lcd.c/h`)
- **接口**: SPI2 (DMA)
- **刷新率**: 20Hz (50ms 周期)
- **显示内容**: 姿态角、高度、电压、遥控状态等

#### 9. USB 调试 (`usb_printf.c/h`)
- **接口**: USB CDC 虚拟串口
- **波特率**: 无限制（USB 全速）
- **功能**: 实时数据打印、参数调试

#### 10. IMU 加热 (`imu_heat.c/h`)
- **功能**: 恒温控制 IMU 工作温度，提高测量精度
- **控制方式**: PWM 加热 + 温度反馈

---

## 构建与编译

### 环境要求
- **CMake**: >= 3.22
- **工具链**: ARM GCC (`arm-none-eabi-gcc`) 或 LLVM Clang
- **调试器**: ST-Link / J-Link
- **IDE**: Visual Studio Code (推荐) / CLion / STM32CubeIDE

### 编译步骤

#### 方法 1: 使用 CMake (推荐)
```bash
# 1. 配置项目（选择预设）
cmake --preset=Debug

# 2. 编译
cmake --build --preset=Debug

# 3. 生成的固件位置
# build/Debug/TZ_wej_test1.elf
# build/Debug/TZ_wej_test1.bin
# build/Debug/TZ_wej_test1.hex
```

#### 方法 2: 使用 STM32CubeMX
1. 打开 `TZ_wej_test1.ioc` 文件
2. 点击 "Generate Code"
3. 使用 STM32CubeIDE 导入项目并编译

### 烧录固件
```bash
# 使用 ST-Link
st-flash write build/Debug/TZ_wej_test1.bin 0x08000000

# 或使用 OpenOCD
openocd -f interface/stlink.cfg -f target/stm32h7x.cfg \
        -c "program build/Debug/TZ_wej_test1.elf verify reset exit"
```

---

## 外设配置说明

### SPI 配置
| SPI | 设备 | CS 引脚 | 速率 | DMA |
|-----|------|---------|------|-----|
| SPI1 | BMI088 Gyro | PC3 | 10MHz | ✓ |
| SPI1 | BMI088 Accel | PC0 | 10MHz | ✓ |
| SPI2 | LCD | PE15 | 42MHz | ✓ |

### UART 配置
| UART | 功能 | 波特率 | DMA |
|------|------|--------|-----|
| UART5 | 超声波传感器 | 9600 | ✓ |
| USART1 | SBUS 接收 | 100000 | ✓ |

### ADC 配置
| ADC | 通道 | 采样率 | 用途 |
|-----|------|--------|------|
| ADC1 | CH4 | 连续转换 | 电池电压 |

### 定时器配置
| 定时器 | 功能 | 频率 |
|--------|------|------|
| TIM1 | 电调 PWM | 50Hz |
| TIM2 | 系统时基 | 1kHz |
| TIM8 | 伺服 PWM | 50Hz |

### DMA 配置
- **DMA1 Stream0**: SPI2_RX (LCD 数据接收)
- **DMA1 Stream1**: SPI2_TX (LCD 数据发送)
- **DMA1 Stream2**: UART5_RX (超声波数据)

---

## 系统运行逻辑

### 启动流程
1. **系统初始化**
   - MPU 配置（Cache 使能）
   - 时钟配置（550MHz）
   - 外设初始化（GPIO/SPI/UART/ADC/TIM/USB）

2. **模块初始化**
   - BMI088 自检与校准
   - 姿态解算模块初始化
   - PID 控制器参数加载
   - LCD 显示初始化
   - 电调解锁准备

3. **主循环运行**
   - IMU 数据采集（1kHz）
   - 姿态解算更新（1kHz）
   - PID 控制计算（500Hz）
   - 遥控指令解析（实时）
   - LCD 刷新（20Hz）
   - USB 状态打印（5Hz）

### 安全保护机制
- **姿态异常检测**: 超过阈值自动切断电机
- **遥控失控保护**: 信号丢失自动降落
- **低压保护**: 电池电压过低报警
- **看门狗**: 防止程序跑飞

---

## 调试与测试

### USB 虚拟串口调试
连接 USB 线后，系统会枚举为 CDC 设备，可使用串口工具查看实时数据：
```bash
# Windows: 查看 COM 端口
# Linux/Mac: /dev/ttyACM0

# 打开串口（波特率任意）
minicom -D /dev/ttyACM0
# 或
screen /dev/ttyACM0
```

### 输出数据格式
```
[STATUS] Roll: 1.23° | Pitch: -0.45° | Yaw: 89.12°
[ALTITUDE] Height: 1.25m | Target: 1.50m
[BATTERY] Voltage: 11.8V | Current: 2.3A
[RC] CH1: 1500 | CH2: 1500 | CH3: 1000 | CH4: 1500
```

---

## 开发路线图

- [x] 基础飞控框架搭建
- [x] IMU 数据采集与姿态解算
- [x] PID 控制器实现
- [x] 遥控接收与解析
- [x] LCD 实时显示
- [ ] 参数在线调试功能
- [ ] 黑匣子数据记录
- [ ] 自动起降算法
- [ ] GPS 定点悬停
- [ ] 航线规划功能

---

## 许可证

本项目遵循 MIT 许可证开源。

---

## 贡献者

- **JIE-808** - 项目创建者与主要开发者

---

## 联系方式

如有问题或建议，请通过以下方式联系：
- GitHub Issues: [提交问题](https://github.com/JIE-808/Coaxial-Vector-Rocket-v1.0-Initial-Unmanned-Aerial-Vehicle-Verification-Version/issues)
- Email: (请在 GitHub 个人主页查看)

---

## 致谢

- STMicroelectronics - STM32 HAL 库
- Bosch Sensortec - BMI088 IMU 传感器
- 开源社区 - 各类算法参考与技术支持

---

**⚠️ 安全提示**: 本项目涉及高速旋转部件和飞行器控制，测试时请务必采取安全措施，远离人群，佩戴护目镜，并在空旷场地进行。
