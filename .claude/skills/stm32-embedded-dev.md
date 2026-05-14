---
name: stm32-embedded-dev
description: STM32H723 embedded MCU development — BSP module pattern, CMake+Ninja build via STM32Cube toolchain, USB CDC communication, HAL usage, ANO protocol, Mahony AHRS. Use when writing or modifying C firmware for this STM32H723VGT6 flight controller project.
---

# STM32H723VGT6 嵌入式飞控开发规范

## 一、项目结构

```
TZ_wej_test1/
├── Core/
│   ├── Inc/                   # HAL 配置头 (main.h, gpio.h, spi.h, ...)
│   └── Src/                   # HAL 初始化 + main.c (应用逻辑)
├── BSP/                       # 板级支持包 — 自行开发的驱动模块
│   ├── BMI088/                # IMU 芯片驱动 (不可修改)
│   ├── attitude.c/.h          # Mahony AHRS 姿态解算
│   ├── imu_heat.c/.h          # IMU 恒温加热控制
│   ├── esc_pwm.c/.h           # 电调 PWM 输出
│   ├── tvc_pid.c/.h           # TVC 推力矢量 PID
│   ├── vbat_adc.c/.h          # 电池电压 ADC 采集
│   ├── ultrasonic.c/.h        # 超声波测距
│   ├── altitude_hold.c/.h     # 定高控制
│   ├── usb_printf.c/.h        # USB CDC 格式化输出
│   ├── ano_protocol.c/.h      # 匿名科创 V7 通信协议
│   ├── lcd.c/.h               # SPI LCD 驱动
│   └── button.c/.h            # 按键驱动
├── USB_DEVICE/
│   ├── App/                   # usbd_cdc_if.c, usb_device.c, usbd_desc.c
│   └── Target/                # usbd_conf.c (USB 硬件配置)
├── Drivers/                   # HAL 库 + CMSIS (CubeMX 生成)
├── Middlewares/               # USB Device Library
├── cmake/stm32cubemx/         # CubeMX 生成的 CMake 配置
├── CMakeLists.txt             # 用户源文件和 include 路径
└── .vscode/
    ├── c_cpp_properties.json  # IntelliSense 配置
    └── tasks.json             # 编译任务
```

**原则**：
- `Core/` 和 `Drivers/` 是 CubeMX 生成的，尽量少改；应用逻辑放 `BSP/` 和 `Core/Src/main.c`
- 新增 `.c` 文件必须加入 `CMakeLists.txt` 的 `target_sources`，`.h` 路径加入 `target_include_directories`
- 每个 BSP 模块遵循 `Init()` → 定时/事件驱动 `Update()` → `Get()` 获取数据的模式

## 二、编译流程

### 工具链位置（Windows + STM32CubeIDE 捆绑）

| 组件 | 路径 |
|------|------|
| CMake | `C:/Users/JIE-808/AppData/Local/stm32cube/bundles/cmake/4.3.1+st.1/bin/cmake` |
| Ninja | `C:/Users/JIE-808/AppData/Local/stm32cube/bundles/ninja/1.13.2+st.1/bin/ninja` |
| GCC | `C:/Users/JIE-808/AppData/Local/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc` |

### 编译命令

```bash
"C:/Users/JIE-808/AppData/Local/stm32cube/bundles/cmake/4.3.1+st.1/bin/cmake" \
  --build "d:/CNSA/SingleChip/STM32_Project/STM32H723VGT6/TZ_wej_test1/build/Debug" \
  -j8
```

### 修改代码后的必做步骤

1. **添加新 `.c` 文件** → 编辑根 `CMakeLists.txt`，在 `target_sources` 中添加
2. **添加新 `.h` 路径** → 编辑根 `CMakeLists.txt`，在 `target_include_directories` 中添加
3. **编译验证** → 执行上述 cmake --build 命令，确保无新增 warning
4. **IntelliSense 同步** → 如 IDE 报 include 找不到，在 VSCode 执行 `CMake: Delete Cache and Reconfigure`

### 编译警告处理

- 新增代码必须零警告
- 项目原有的 CubeMX 生成代码的 `unused-parameter` 警告忽略，不要修改
- `-Wsign-compare`：有符号/无符号比较必须显式转换或用 `U` 后缀

## 三、BSP 模块开发规范

### 头文件模板

```c
#ifndef __MODULE_NAME_H__
#define __MODULE_NAME_H__

#include <stdint.h>

/* 公开类型和常量 */

/* 初始化（在 main.c 的 InitApplicationModules 中调用一次） */
void Module_Init(void);

/* 获取数据 / 状态 */
float Module_GetValue(void);
uint8_t Module_IsReady(void);

#endif
```

### 源文件模式

```c
#include "module.h"
#include "stm32h7xx_hal.h"   /* 如需 HAL */

/* 静态全局变量，模块私有 */
static float internal_state;
static uint8_t initialized;

void Module_Init(void)
{
    /* 硬件初始化 + 变量赋初值 */
    initialized = 1;
}

float Module_GetValue(void)
{
    if (!initialized) return 0.0f;  /* 防御性检查 */
    return internal_state;
}
```

### 关键原则

- **不要动态内存分配** — 禁止 `malloc`/`free`，只用静态数组或栈变量
- **防御性检查** — `Init` 未完成时 `Get` 返回安全默认值（通常是 0）
- **无注释解释代码做什么** — 好的函数名和变量名已经说明了用途
- **全局变量最小化** — 模块内部状态用 `static`，跨模块共享通过公开的结构体指针（如 `Attitude_GetData()` 返回 `attitude_data_t *`）
- **`#include` 只包含真正需要的头文件** — 不要堆砌无用的 includes

## 四、USB CDC 通信规范

### 可用发送函数

| 函数 | 用途 | 上下文限制 |
|------|------|-----------|
| `usb_printf(fmt, ...)` | 格式化 ASCII 文本调试 | **仅限主循环**（内部有忙等待，ISR 中调用会死锁） |
| `CDC_Transmit_HS(buf, len)` | 原始二进制发送 | 主循环 + ISR 均可（但 ISR 中仅限单次调用） |

### USB ISR 约束（重要）

- `CDC_Receive_HS` 在 **NVIC 优先级 0（最高）** 的 USB 中断中执行
- **ISR 中不可以**：调用 `usb_printf`（会死锁）、调用多次 `CDC_Transmit_HS`（第二次会因 TxState 忙而静默丢弃）、做长时间处理
- **正确的 ISR 模式**：收到数据 → 解析 → 构建响应帧入缓冲区 → 设标志位 → 退出 ISR → **主循环检查标志并发送**

本项目已在 `ano_protocol.c` 中实现了此模式：`ANO_ProcessRx()`（ISR 调用）构建帧到 `ano_tx_buf` 并设 `ano_tx_pending=1`，主循环的 `ANO_Service_Deferred()` 负责实际发送。

### usb_printf 使用规则

- **初始化阶段**：可以自由使用，打印模块初始化状态
- **运行时**：禁止使用，ASCII 文本会干扰上位机二进制协议帧解析
- 运行时如需调试，通过匿名协议的自定义帧（0xF1~0xFA）发送

## 五、匿名科创 V7 通信协议

### 帧结构

```
HEAD  D_ADDR  ID   LEN   DATA[LEN]   SC   AC
0xAA  1B      1B   1B    N bytes     1B   1B
```

- SC = 从 HEAD 累加到 DATA 末尾，取低 8 位
- AC = 每次累加时将当前 SC 再次累加，取低 8 位
- 多字节数据 **小端序**
- 地址：`0xFF` 上位机/广播，`0x05` 飞控

### 本项目实现的帧

| ID | 名称 | 方向 | 说明 |
|----|------|------|------|
| 0x00 | ACK | FC→PC | 命令/写入确认 |
| 0x01 | SENSER | FC→PC | ACC+GYRO 原始数据 |
| 0x03 | EULER | FC→PC | 欧拉角 (ROL/PIT/YAW ×100 + FUSION_STA) |
| 0xE1 | PARAM_READ | 双向 | PC 请求参数 / FC 回复参数值(int32=0) |

### 上位机连接验证清单

1. 上位机连接后串口打开成功（"串口打开成功"）
2. 参数读取无超时（上位机发 0xE1 → FC 回 0xE1）
3. 飞控状态页 3D 模型随姿态转动

## 六、HAL 使用规范

- 初始化代码（MX_xxx_Init）由 CubeMX 生成在 `Core/Src/`，**不要手动修改**
- BSP 模块使用 HAL 外设句柄（如 `hspi2`, `htim1`）时，在模块内 `extern` 声明
- 中断回调（`HAL_xxx_Callback`）放在 `Core/Src/stm32h7xx_it.c` 或 `main.c`
- `HAL_Delay` 只在初始化阶段使用；运行时用 `HAL_GetTick()` 做非阻塞定时

### 外设分配（本项目）

| 外设 | 用途 |
|------|------|
| SPI2 | BMI088 IMU 通信 |
| TIM1 CH1/CH3 | 舵机 PWM 输出 |
| TIM2 CH1/CH2 | ESC 电调 PWM |
| TIM3 | 超声波 Trig/Echo |
| TIM4 | 500Hz 姿态控制中断 |
| ADC1 | 电池电压采样 |
| UART5 | SBUS 遥控器接收 |
| USB_OTG_HS | CDC 虚拟串口 |

## 七、主循环架构

```c
while (1) {
    ANO_Service_Deferred();   // 优先处理 USB 延迟响应（每次迭代）

    if (VBAT_IsShutdown()) {  // 致命低压 → 低功耗等待
        HAL_Delay(100);
        continue;
    }

    // 100Hz 超声波触发
    // 50Hz LCD 刷新
    // SBUS 遥控器解析
    // 100Hz ANO 协议发送 (0x03 姿态 + 0x01 传感器)
}
```

**定时规范**：
- 姿态数据发送：100Hz（10ms 间隔）— 上位机 3D 显示流畅
- LCD 刷新：50Hz — 人眼感知足够
- 超声波：10Hz — 声速物理限制
- 高实时控制（TVC、定高）：500Hz — 在 TIM4 中断中执行，不在主循环

## 八、常用操作速查

### 添加新的通信协议帧

1. 在 `ano_protocol.h` 添加 `#define ANO_ID_xxx`
2. 在 `ano_protocol.c` 实现发送函数（用 `ANO_Send_Frame` 直接发 或 `ANO_Send_Frame_Deferred` 延迟发）
3. 如需接收此帧，在 `ANO_ProcessRx` 的 `switch` 中添加 `case`
4. 在 `main.c` 中定时调用发送函数

### 添加新的 BSP 模块

1. 创建 `BSP/new_module.h` 和 `BSP/new_module.c`
2. 在根 `CMakeLists.txt` 的 `target_sources` 添加 `BSP/new_module.c`
3. 在 `main.c` 中 `#include "new_module.h"`，在 `InitApplicationModules` 中调用 `NewModule_Init()`
4. 编译验证

### 调试新代码

1. 初始化阶段可用 `usb_printf("[INIT] ...")` 打印状态
2. 运行时调试通过 ANO 自定义帧（0xF1~0xFA）或保持代码足够简单可预测
3. 逻辑分析仪 / 示波器检查 PWM 和时序信号
