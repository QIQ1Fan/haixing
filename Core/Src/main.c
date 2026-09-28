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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* ==========================================================================
 * 本文件在工程里的位置：全局唯一的"调度者"
 *
 *   main.c 负责两件事：
 *     1. 开机时按正确顺序初始化所有模块（顺序有讲究，见下面的说明）
 *     2. 跑一个 10ms 节拍的主循环，把周期性任务和串口命令串起来
 *
 *   它自己不做任何硬件操作，所有活儿都交给 bsp_* 模块。
 *   这样分工的好处是：以后加 PID 循迹逻辑，只改主循环里那一段，
 *   底层驱动一行都不用动。
 *
 *   主循环的数据流：
 *
 *     ┌─ 每 10ms ────────────────────────────────────────┐
 *     │   BSP_Encoder_Update()  采样一次编码器计数        │
 *     │   BSP_Key_Scan()        推进一次按键状态机        │
 *     │   BSP_Buzzer_Poll()     检查非阻塞鸣叫是否该关声  │
 *     └──────────────────────────────────────────────────┘
 *     ┌─ 每轮都查 ───────────────────────────────────────┐
 *     │   BSP_UART_GetChar()    收到字符就跑对应自检      │
 *     └──────────────────────────────────────────────────┘
 *
 *   为什么这三件事必须严格 10ms 一次：
 *     · 编码器测速是"增量 ÷ 采样周期"，周期不准转速就不准
 *     · 按键的长按 1 秒 = 100 次 × 10ms，周期不准时间阈值就错
 * ========================================================================== */
#include "bsp_gpio.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_encoder.h"
#include "bsp_track.h"
#include "bsp_key.h"
#include "bsp_buzzer.h"
#include "bsp_oled.h"
#include "bsp_uart.h"
#include "bsp_test.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --------------------------------------------------------------------------
 * 主循环节拍
 *
 * MAIN_TICK_MS 与 bsp_config.h 里的这些值必须一致，改动时要一起改：
 *     ENC_SAMPLE_PERIOD_MS   编码器测速的采样周期
 *     KEY_SCAN_PERIOD_MS     按键消抖与长按的计时基准
 * 三者不一致的话，转速会成比例地错，按键的时间阈值也会失准。
 *
 * s_next_tick 记录"上一拍是什么时候"，主循环拿当前时间去比。
 * -------------------------------------------------------------------------- */
static uint32_t s_next_tick = 0u;

#define MAIN_TICK_MS    10u
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  /* USER CODE BEGIN 2 */

  /* ==================================================================
   * BSP 初始化。顺序有讲究：
   *   1) 串口最先 —— 后面任何模块出问题，报错信息才打得出来
   *   2) 电机 PWM 通道先起、STBY 后使能 —— 避免上电瞬间电机乱冲
   *   3) OLED 放最后 —— 它自带 100ms 上电等待，不拖累前面
   * ================================================================== */
  BSP_UART_Init();
  BSP_UART_SendString("\r\n\r\n=================================\r\n");
  BSP_UART_SendString("  haixing line-follower  booting\r\n");
  BSP_UART_SendString("=================================\r\n");

  BSP_GPIO_Init();
  BSP_Buzzer_Init();
  BSP_Key_Init();
  BSP_Track_Init();
  BSP_Encoder_Init();
  BSP_Servo_Init();
  BSP_Motor_Init();
  BSP_OLED_Init();

  /* 舵机回中位，电机保持停止 */
  BSP_Servo_SetAngle(0.0f);
  BSP_Motor_Stop();

  BSP_UART_Printf("SYSCLK = %u Hz\r\n", (unsigned int)HAL_RCC_GetSysClockFreq());
  BSP_UART_SendString("所有外设初始化完成\r\n");

  BSP_Buzzer_Beep(120u);
  BSP_Test_ShowMenu();

  s_next_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 变量声明放在代码块最前面，是为了兼容 C90（ARMCC5 的默认标准）。
       C99 允许在语句中间声明变量，C90 不允许。 */
    uint32_t now;
    uint8_t  ch;

    /* ------------------------------------------------------------------
     * 10ms 周期任务
     *
     * 这是"软节拍器"：不依赖定时器中断，而是每轮循环看一眼时间够没够。
     * 好处是简单、不用碰中断优先级；代价是如果某一轮循环耗时超过 10ms
     * （比如某个阻塞操作），这一拍就会被拉长。
     *
     * ★ 用 (now - s_next_tick) 而不是 (now >= s_next_tick)：
     *   HAL_GetTick() 是 32 位毫秒，49.7 天会回绕归零。
     *   无符号相减能正确处理回绕，直接比大小则会在回绕那一刻卡死。
     *
     * 这里直接 s_next_tick = now 而不是 += 10，是刻意的：
     * 万一某轮超时了，让下一拍从"现在"重新起算，不会越积越多的追赶。
     * ------------------------------------------------------------------ */
    now = HAL_GetTick();
    if ((uint32_t)(now - s_next_tick) >= MAIN_TICK_MS) {
      s_next_tick = now;

      BSP_Encoder_Update();   /* 读编码器计数差 —— 速度反馈的来源 */
      BSP_Key_Scan();         /* 推进按键状态机，产生短按/长按/连发事件 */
      BSP_Buzzer_Poll();      /* 检查非阻塞鸣叫是否到点该关声 */
    }

    /* ------------------------------------------------------------------
     * 串口命令：收到一个字符就跑对应的底层自检
     * 非阻塞读，没数据时立刻返回，不会拖慢上面的 10ms 节拍
     * ------------------------------------------------------------------ */
    if (BSP_UART_GetChar(&ch) != 0u) {
      BSP_Test_RunCommand((char)ch);
    }

    /* ==================================================================
     * 底层全部验证通过后，把上面那段自检调用换掉，换成循迹逻辑：
     *
     *   float err = BSP_Track_GetErrorNorm();     // 1. 读位置误差
     *   float steer = pid_turn(err);              // 2. 转向环 PID
     *   int16_t base = 1800;                      // 3. 基础速度
     *
     *   BSP_Motor_SetSpeed(base - steer, base + steer);   // 差速转向
     *   // 或者 BSP_Servo_SetAngle(steer);                // 舵机转向
     *
     * 再往里加一层速度环：用 BSP_Encoder_GetRPM() 做反馈，
     * 把 base 换成"速度环的输出"，就是标准的双环控制了。
     * ================================================================== */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
