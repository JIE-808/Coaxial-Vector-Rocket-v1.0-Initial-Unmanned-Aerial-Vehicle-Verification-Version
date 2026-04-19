/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "attitude.h"
#include "imu_heat.h"
#include "lcd.h"
#include "tvc_pid.h"
#include "altitude_hold.h"
#include "ultrasonic.h"
#include "usb_printf.h"
#include "vbat_adc.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DISPLAY_REFRESH_PERIOD_MS   50U   /* LCD 屏幕刷新周期：50ms = 20Hz */
#define STATUS_PRINT_PERIOD_MS     200U  /* USB 串口状态打印周期：200ms = 5Hz */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void InitApplicationModules(void);
static void RefreshRuntimeDisplay(void);
static void PrintRuntimeStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "esc_pwm.h"

#define SBUS_FRAME_SIZE 25

// H7芯片：DMA无法访问DTCMRAM，并且必须做32字节缓存行对齐
#if defined ( __ICCARM__ )
#pragma location=".dma_buffer"
#else
__attribute__((section(".dma_buffer")))
#endif
ALIGN_32BYTES(uint8_t sbus_rx_buf[64]);

uint16_t rc_channels[16];
uint8_t sbus_connected = 0; // 0=未连接或断开, 1=连接正常
uint16_t last_rx_size = 0;  // 用于调试接收到的字节数
volatile uint8_t is_crashed = 0;     // 姿态异常坠机保护状态（ISR读取，需volatile）
volatile uint8_t is_armed = 0;       // 系统解锁状态（ISR读取，需volatile）
uint32_t button_press_ms = 0; // 按键按下时间戳

uint32_t g_esc_arm_start_ms = 0; // 记录电调上电启动时间

void SBUS_Parse(uint8_t *buf, uint16_t *channels)
{
    if (buf[0] == 0x0F && buf[24] == 0x00)
    {
        channels[0]  = ((buf[1]       | buf[2] << 8)                   & 0x07FF);
        channels[1]  = ((buf[2] >> 3  | buf[3] << 5)                   & 0x07FF);
        channels[2]  = ((buf[3] >> 6  | buf[4] << 2  | buf[5] << 10)  & 0x07FF);
        channels[3]  = ((buf[5] >> 1  | buf[6] << 7)                   & 0x07FF);
        channels[4]  = ((buf[6] >> 4  | buf[7] << 4)                   & 0x07FF);
        channels[5]  = ((buf[7] >> 7  | buf[8] << 1  | buf[9] << 9)   & 0x07FF);
        channels[6]  = ((buf[9] >> 2  | buf[10] << 6)                  & 0x07FF);
        channels[7]  = ((buf[10] >> 5 | buf[11] << 3)                  & 0x07FF);
        channels[8]  = ((buf[12]      | buf[13] << 8)                  & 0x07FF);
        channels[9]  = ((buf[13] >> 3 | buf[14] << 5)                  & 0x07FF);
        channels[10] = ((buf[14] >> 6 | buf[15] << 2 | buf[16] << 10) & 0x07FF);
        channels[11] = ((buf[16] >> 1 | buf[17] << 7)                  & 0x07FF);
        channels[12] = ((buf[17] >> 4 | buf[18] << 4)                  & 0x07FF);
        channels[13] = ((buf[18] >> 7 | buf[19] << 1 | buf[20] << 9)  & 0x07FF);
        channels[14] = ((buf[20] >> 2 | buf[21] << 6)                  & 0x07FF);
        channels[15] = ((buf[21] >> 5 | buf[22] << 3)                  & 0x07FF);
        
        // 检查标志位 (buf[23]) 是否发生失控保护 (Failsafe) 或者是掉线 (Frame lost)
        if ((buf[23] & 0x08) || (buf[23] & 0x04)) {
            sbus_connected = 0; // 失控保护触发或者侦丢失
        } else {
            sbus_connected = 1; // 正常
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        // 关键：在读DMA内存前，先让CPU把Cache里的相关部分给无效化，否则读出来的全是老数据
        SCB_InvalidateDCache_by_Addr((uint32_t *)sbus_rx_buf, sizeof(sbus_rx_buf));

        last_rx_size = Size; // 记录最近一次接到的包长度
        if (Size >= SBUS_FRAME_SIZE)
        {
            // 简单查找帧头帧尾
            for (int i = 0; i <= Size - SBUS_FRAME_SIZE; i++)
            {
                if (sbus_rx_buf[i] == 0x0F && sbus_rx_buf[i + 24] == 0x00)
                {
                    SBUS_Parse(&sbus_rx_buf[i], rc_channels);
                    break;
                }
            }
        }
        // 重新开启空闲中断DMA接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, sizeof(sbus_rx_buf));
    }
}

// 添加串口错误回调函数（非常重要：防止上电瞬间接收机的杂波导致串口溢出死锁）
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        // 发生错误（如Overrun即ORE错误），强行重启DMA接收，防止永久死锁
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, sizeof(sbus_rx_buf));
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  /* 完成所有 BSP 模块初始化（USB / LCD / IMU / TVC / VBAT / ESC）
   * 此函数返回后，ESC 电调还在 3s 解锁中，按键暂时无效 */
  InitApplicationModules();
  
  // 启动UART5的SBUS DMA空闲中断接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx_buf, sizeof(sbus_rx_buf));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_display_ms = HAL_GetTick();   /* LCD 刷新时间戳 */
  uint32_t last_status_ms  = HAL_GetTick();   /* USB 打印时间戳 */
  uint32_t last_sonar_ms   = HAL_GetTick();   /* 超声波触发时间戳 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ── 致命低压保护：已切断输出，只做低功耗等待 ── */
    if (VBAT_IsShutdown()) {
      HAL_Delay(100);
      continue;
    }

    /* ── 姿态异常坠机保护（仅解锁后生效，侧倾或俯仰角 > 45度强制断电） ── */
    if (is_armed && Attitude_IsReady() && !is_crashed) {
      attitude_data_t *att = Attitude_GetData();
      if ((att->pitch > 20.0f || att->pitch < -20.0f) ||
          (att->roll > 20.0f || att->roll < -20.0f)) {
        
        is_crashed = 1; // 触发锁死（除非复位单片机否则不解锁）

        // 强行拉低电调用以停止无刷电机
        ESC_PWM_SetAllPulseUs(1000);
        TVC_SetBaseThrottle(1000);

        // 舵机归中（保持PWM通道运行，只是输出中位值）
        TVC_Center();
        
        usb_printf("\r\n[!!!] CRASH DETECTED, ALL POWER OFF [!!!]\r\n");
      }
    }

    /* ── 按键长按 3 秒解锁逻辑 ── */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) {
        if (button_press_ms == 0) {
            button_press_ms = HAL_GetTick(); // 记录起始按下时间
        } else if (!is_armed && (HAL_GetTick() - button_press_ms) >= 3000U) {
            is_armed = 1; // 满3秒后解锁
            TVC_Center();            // 清TVC积分，防止地面积分饱和导致起飞猛偏
            AltHold_ResetIntegral(); // 解锁时清积分，防止积分饱和
            Ultrasonic_Calibrate();  // 以当前距离为地面基准（高度归零）
        }
    } else {
        button_press_ms = 0; // 松开按键清零计时
    }

    /* ── ESC 油门控制：定高模式，CH3 → 目标高度 → PID → 油门 ── */
    if (is_crashed) {
      TVC_SetBaseThrottle(1000);
      ESC_PWM_SetAllPulseUs(1000);
    } else if (is_armed && sbus_connected) {
      /* 遥控器 CH3 杆量 → 目标高度（0~80cm） */
      AltHold_SetTargetFromStick(rc_channels[2]);
      /* 定高 PID 在超声波更新后（下方 100ms 块）触发 */
    } else {
      TVC_SetBaseThrottle(1000);
      ESC_PWM_SetAllPulseUs(1000);
    }

    /* ── 超声波：每 100ms 触发一次测量 + 实时检查超时 ── */
    Ultrasonic_CheckTimeout();
    if ((HAL_GetTick() - last_sonar_ms) >= 100U) {
      last_sonar_ms = HAL_GetTick();
      Ultrasonic_Trigger();
      /* 超声波更新后立即运行定高 PID */
      if (is_armed && !is_crashed && sbus_connected) {
        AltHold_Update(Ultrasonic_GetDistance_cm(), Ultrasonic_IsValid());
      }
    }

    /* ── 20Hz LCD 刷新：姿态 + 电池图标 + 距离 ── */
    if ((HAL_GetTick() - last_display_ms) >= DISPLAY_REFRESH_PERIOD_MS) {
      last_display_ms = HAL_GetTick();
      RefreshRuntimeDisplay();
    }

    /* ── 5Hz USB 状态打印 ── */
    if ((HAL_GetTick() - last_status_ms) >= STATUS_PRINT_PERIOD_MS) {
      last_status_ms = HAL_GetTick();
      PrintRuntimeStatus();
      
      int cur_throttle = rc_channels[2];
      int simulated_pwm = 1000 + ((cur_throttle - 300) * 1000) / (1700 - 300);
      if (simulated_pwm < 1000) simulated_pwm = 1000;
      if (simulated_pwm > 2000) simulated_pwm = 2000;
      
      if (is_crashed) {
          usb_printf(">> SYSTEM LOCKED: CRASH DETECTED! <<\r\n");
      } else if (!is_armed) {
          usb_printf(">> SYSTEM LOCKED: Hold KEY1 for 3s to ARM... <<\r\n");
      } else {
          usb_printf("SBUS[CH3]: %d, Target PWM: %d us, Connected: %d\r\n", cur_throttle, simulated_pwm, sbus_connected);
      }

      /* ── 诊断信息：舵机 + 状态 ── */
      usb_printf("[DBG] armed=%d crash=%d vbat_off=%d att_rdy=%d\r\n",
                 is_armed, is_crashed, VBAT_IsShutdown(), Attitude_IsReady());
      usb_printf("[DBG] servo_L=%lu servo_R=%lu base_thr=%u\r\n",
                 __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1),
                 __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3),
                 TVC_GetBaseThrottle());
      if (Attitude_IsReady()) {
          attitude_data_t *dbg = Attitude_GetData();
          usb_printf("[DBG] P:%.2f R:%.2f Y:%.2f dt:%.1fms\r\n",
                     dbg->pitch, dbg->roll, dbg->yaw, dbg->dt * 1000.0f);
      }
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
 * @brief  一次性初始化所有 BSP 模块
 *         调用顺序：USB → LCD → IMU → TVC → VBAT → ESC
 *         ESC 上电后需要 3 秒解锁，记录启动时间戳。
 */
static void InitApplicationModules(void)
{
  USB_Printf_Init();
  usb_printf("\r\n===== TZ_wej_test1 Boot =====\r\n");

  LCD_Init();
  LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);

  Attitude_Init();
  IMU_Heat_Init();
  TVC_Init();
  TVC_Center();

  VBAT_Init();

  ESC_PWM_Init();
  ESC_PWM_SetAllPulseUs(1000);

  Ultrasonic_Init();
  AltHold_Init();

  /* 启动 500Hz 姿态控制定时器 */
  HAL_TIM_Base_Start_IT(&htim4);

  usb_printf("[INIT] All modules ready.\r\n");
}

/**
 * @brief  20Hz LCD 运行时刷新：姿态 + 电池图标
 */
static void RefreshRuntimeDisplay(void)
{
  if (VBAT_GetState() == VBAT_STATE_CRITICAL) {
    VBAT_ShowCriticalWarning();
    return;
  }

  /* 防触摸静电干扰：每次刷新前重发 Display ON + Inversion ON，
     若 LCD 控制器被 ESD 打入 Sleep/Off 状态可立即恢复 */
  LCD_WR_REG(0x11);  /* Sleep Out  */
  LCD_WR_REG(0x21);  /* Inversion ON */
  LCD_WR_REG(0x29);  /* Display ON */

  Attitude_Display();
  VBAT_Display();
  Ultrasonic_Display();

  /* 在目标温度下一行(y=225)显示解锁状态 */
  LCD_Fill(10, 225, 230, 245, WHITE);
  if (is_crashed) {
    LCD_ShowString(10, 225, (const uint8_t *)"!! CRASH LOCKED !!", RED, WHITE, 16, 0);
  } else if (!is_armed) {
    uint32_t held = 0;
    if (button_press_ms != 0) {
      held = (HAL_GetTick() - button_press_ms) / 1000U;
      if (held > 3) held = 3;
    }
    char arm_buf[28];
    snprintf(arm_buf, sizeof(arm_buf), "Hold KEY1 3s: %lus/3s", held);
    LCD_ShowString(10, 225, (const uint8_t *)arm_buf, RED, WHITE, 16, 0);
  } else {
    if (sbus_connected) {
      /* 定高模式：显示目标高度 + 实际高度 */
      char alt_buf[32];
      snprintf(alt_buf, sizeof(alt_buf), "T:%2.0f A:%4.1f %4u",
               AltHold_GetTarget_cm(),
               Ultrasonic_IsValid() ? Ultrasonic_GetDistance_cm() : 0.0f,
               TVC_GetBaseThrottle());
      LCD_Fill(10, 225, 230, 245, WHITE);
      LCD_ShowString(10, 225, (const uint8_t *)alt_buf, GREEN, WHITE, 16, 0);
    } else {
      LCD_ShowString(10, 225, (const uint8_t *)"ARMED  SBUS:LOST", RED, WHITE, 16, 0);
    }
  }
}

/**
 * @brief  5Hz USB 串口状态打印：姿态 + 电池 + 温控
 */
static void PrintRuntimeStatus(void)
{
  if (!Attitude_IsReady()) {
    usb_printf("[WAIT] IMU not ready...\r\n");
    return;
  }

  attitude_data_t *att = Attitude_GetData();
  usb_printf("P:%6.2f R:%6.2f Y:%6.2f  T:%.1fC  VBAT:%.2fV(%.0f%%)  D:%.1fcm\r\n",
             att->pitch, att->roll, att->yaw,
             att->temp,
             VBAT_GetVoltage(), VBAT_GetPercent(),
             Ultrasonic_IsValid() ? Ultrasonic_GetDistance_cm() : -1.0f);
  /* 定高模式：显示目标/实际高度、PID输出、油门 */
  usb_printf("[ALT] tgt:%.0fcm act:%.1fcm pid:%+.0fus thr:%u\r\n",
             AltHold_GetTarget_cm(),
             Ultrasonic_IsValid() ? Ultrasonic_GetDistance_cm() : -1.0f,
             AltHold_GetPIDOutput(),
             TVC_GetBaseThrottle());
}

/**
 * @brief TIM4 中断回调 — 500Hz 姿态控制环
 *
 * 每 2ms 触发一次：
 *   1. Attitude_Update()   — 读 BMI088、Mahony 姿态解算
 *   2. IMU_Heat_Update()   — PID 恒温控制（根据当前芯片温度）
 *   3. TVC_Update()        — 推力矢量 PID，输出舵机脉宽
 *
 * 安全守卫：低压保护或 IMU 未就绪时直接跳过，不操作任何执行机构。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    if (VBAT_IsShutdown() || !Attitude_IsReady()) {
      return;
    }

    Attitude_Update();

    attitude_data_t *att = Attitude_GetData();
    IMU_Heat_Update(att->temp);

    /* 安全门限：
     *   舵机（pitch/roll）：IMU 就绪且未坠机时一直工作，
     *     这样上电后无需长按 3 秒也能手持观察舵机修正方向。
     *   电机油门：仍然由 is_armed 控制，未解锁时保持 1000us。
     *   电机差速（yaw）：仅在基础油门 >1050us 时叠加，避免地面自转。
     */
    if (!is_crashed) {
      TVC_Update(att->pitch, att->roll, att->gyro[1], att->gyro[0], att->gyro[2], att->dt);
    } else {
      TVC_Center();
    }
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
