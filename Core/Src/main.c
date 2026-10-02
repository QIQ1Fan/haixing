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
 *   main.c 只做两件事：
 *     1. 开机时按正确顺序初始化所有模块（顺序有讲究，见下面每一步的注释）
 *     2. 跑一个 10ms 节拍的主循环，把周期性任务和串口命令串起来
 *
 *   它自己不做任何控制运算，也不碰任何寄存器 —— 所有活儿都交给
 *   下面的模块。这样分工的好处是：想改控制策略只动 app_car.c，
 *   想改界面只动 app_menu.c，底层驱动一行都不用动。
 *
 *   主循环（每 10ms 一拍，名义值；实际值由 bsp_encoder / app_car 自己量）：
 *
 *     ┌──────────────────────────────────────────────────────────┐
 *     │ ① BSP_Encoder_Update()  采样编码器计数 → 转速/里程        │
 *     │ ② BSP_Key_Scan()        推进按键状态机 → 短按/长按/连发   │
 *     │ ③ BSP_Buzzer_Poll()     到点关掉非阻塞鸣叫               │
 *     │ ④ APP_Car_Update()      ★ 循迹控制律：误差 → PID → 差速  │
 *     │ ⑤ APP_Leg_Update()      支撑脚角度斜坡（没在动就立刻返回）│
 *     │ ⑥ APP_Menu_Update()     按键处理 + OLED 刷新（自带节流） │
 *     └──────────────────────────────────────────────────────────┘
 *     ┌──────────────────── 每轮循环都查一下 ────────────────────┐
 *     │ BSP_UART_GetChar()  收到字符就跑对应的自检/命令           │
 *     └──────────────────────────────────────────────────────────┘
 *
 *   ★ 为什么这几件事必须挂在同一个节拍上：
 *     · 控制环的相位滞后直接由采样周期决定，周期不定就会画龙
 *     · 按键的长按 0.6 秒 = 60 次 × 10ms，周期不准时间阈值就错
 *     · 编码器测速是"增量 ÷ 时间"，周期不准转速就成比例地错
 *
 *   ★ 而"某一拍被 OLED 刷屏拖长了"并不致命：bsp_encoder 和 app_car
 *     各自用 HAL_GetTick() 量出真实的 dt，按真实时间换算。
 *     真正要避免的是"平均周期"被拖长，那才是真的控制变差 ——
 *     所以菜单刷新做了节流，而且只刷变化的那几页。
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
#include "bsp_pid.h"

#include "app_param.h"
#include "app_leg.h"
#include "app_car.h"
#include "app_menu.h"
#include "app_route.h"
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
 * ★ MAIN_TICK_MS 不在这里定义，它在 bsp_config.h 里 —— 因为它必须和
 *   CTRL_PERIOD_MS（控制周期）、ENC_SAMPLE_PERIOD_MS（测速名义周期）、
 *   KEY_SCAN_PERIOD_MS（按键计时基准）完全相等，而 bsp_config.h 里
 *   有一道 #if 编译期检查替我们盯着这件事，不相等直接编译报错。
 *   定义在这里的话那道检查就管不到了。
 *
 * s_next_tick 记录"上一拍是什么时候"，主循环拿当前时间去比。
 * -------------------------------------------------------------------------- */
static uint32_t s_next_tick = 0u;

/* --------------------------------------------------------------------------
 * ★ 已通过的路标（障碍）计数 —— 就是你说的 N
 *
 * 【为什么必须放在这里，不能放在 while 循环里面】
 *   原来它写在 while(1) 的花括号里：
 *       while (1) { uint8_t N = 0;  ... }
 *   循环体里的局部变量每一圈都会重新出生一次 —— 也就是说 N 每 10ms
 *   就被清零一次，自增过的东西立刻丢掉，当不了计数器。
 *   计数器必须活在循环外面（文件作用域，或者加 static）。
 *
 * 【它将来会怎么用】
 *   固定赛道上每个地形（十字路口 / 左转 / 右转 / 台阶 …）都有固定的
 *   探头图案。N 就是"已经过了几个路标"，用它在逻辑里对应"现在该期待
 *   哪一种图案"，等价于你 Arduino 代码里的 Z：
 *       if (图案 == 期待的图案 && N == k) { N++; ... 做动作 ... }
 *   ★ 等把 logic.ino 移植过来时，这个 N 会挪进 app_route.c 统一管理，
 *     到时候这里就不需要了。
 * -------------------------------------------------------------------------- */
uint8_t N = 0u;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  开机自报家门：把关键信息打到串口
  * @note   出问题时这一屏信息往往比任何调试手段都有用 ——
  *         比如 Clocks 不对说明晶振没起振（会退回内部 RC，PWM 频率全变），
  *         Params 显示 default 说明 Flash 里没有有效参数。
  */
static void main_boot_report(void)
{
    BSP_UART_SendString("\r\n\r\n=================================\r\n");
    BSP_UART_SendString("  haixing line-follower  booting\r\n");
    BSP_UART_SendString("=================================\r\n");

    BSP_UART_Printf("SYSCLK = %u Hz\r\n", (unsigned int)HAL_RCC_GetSysClockFreq());
    BSP_UART_Printf("Control period = %u ms\r\n", (unsigned int)MAIN_TICK_MS);
    BSP_UART_Printf("Param flash save = %s\r\n",
                    (PARAM_FLASH_ENABLE != 0) ? "enabled" : "disabled");

    /* 把当前生效的参数打一份出来。整定过程中对比"上次是什么值"
       靠的就是它 —— 屏幕上只能看一页，这里一次全打出来 */
    APP_Param_Print();

    BSP_UART_SendString("[boot] init done.\r\n");
    BSP_UART_SendString("[boot] OLED keys: OK=menu/enter  BACK=back/STOP  +/-=select\r\n");

    /* ★ 开机直接把自检菜单打出来，而不是"让用户敲 ? 才显示"。
       理由很实际：调试第一步就是怀疑"串口到底通不通"，
       如果菜单要靠收字符才出现，那么 RX 没接好的人和"命令敲错了"的人
       看到的现象一模一样 —— 都是一片沉默，没法往下走。
       打出来之后，即使 RX 完全没接，也能照着菜单继续验硬件。
       另外顺带把"键盘要半角、命令要 ASCII"这件事说清楚。 */
    BSP_UART_SendString("[boot] 菜单在下面。若敲命令没反应，先看 PA10(RX) 是否接了适配器的 TX。\r\n");
    BSP_UART_SendString("[boot] 注意：命令行里的 ? 和字母必须是【半角】ASCII，全角字符收不到。\r\n");
    BSP_Test_ShowMenu();
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
   * 初始化顺序，每一句的位置都有理由：
   *
   *   1) BSP_UART_Init()    —— 最先。后面任何模块出问题，报错才打得出来
   *   2) BSP_GPIO_Init()    —— 所有纯 GPIO 引脚的底座，电机方向/循迹/按键
   *                            都靠它，必须在电机和循迹之前
   *   1) BSP_UART_Init()    —— 最先。后面任何模块出问题，报错才打得出来
   *   2) APP_Param_Init()   —— 读 Flash 参数。★ 提到这里是有原因的：
   *                            它只读 Flash + 填内存，一个外设都不碰，
   *                            所以可以安全地跑在最前面，让后面每一个模块
   *                            一初始化就能拿到最终参数
   *   3) BSP_GPIO_Init()    —— 所有纯 GPIO 引脚的底座
   *   4) BSP_Buzzer_Init()  —— 上电先保证蜂鸣器是静音的
   *   5) BSP_Key_Init()     —— 按键状态清零
   *   6) BSP_Track_Init()   —— 循迹（引脚已在第 3 步配好）
   *   7) BSP_Encoder_Init() —— 编码器，给循迹控制提供转速
   *   8) BSP_Servo_Init()   —— 支撑脚舵机的软件 PWM 时基 + 中断
   *   9) APP_Leg_Init()     —— ★ 紧跟舵机初始化，见下面的说明
   *  10) BSP_Motor_Init()   —— 电机 PWM 起好、STBY 最后才使能，
   *                            避免上电瞬间电机乱冲
   *  11) BSP_OLED_Init()    —— 它自带 100ms 上电等待，放最后不拖累别人
   *  12) APP_Car_Init()     —— 建 PID、清状态（不动电机）
   *  13) APP_Menu_Init()    —— 界面，最后，它要往屏幕上画第一帧
   *
   *  ★★ 为什么 APP_Leg_Init() 要紧贴着 BSP_Servo_Init() ★★
   *   BSP_Servo_Init() 内部把初值写死成中位 1500us（90°），它并不知道
   *   支撑脚该停在哪个角度 —— 那是参数 LEGUP 决定的。
   *   如果按"舵机初始化 → 电机初始化 → OLED 初始化(内部 delay 100ms)
   *   → 再读参数 → 才设角度"的老顺序走，舵机在上电头 100 多毫秒里
   *   会一直收到 90° 的脉冲，然后才猛地甩到收起位 ——
   *   现象就是"每次上电支撑脚都先抖一下"。
   *   把参数提前读、把设角度紧跟舵机初始化之后，第一条脉冲就已经是
   *   LEGUP，机构全程一动不动。
   * ================================================================== */
  BSP_UART_Init();
  APP_Param_Init();
  BSP_GPIO_Init();
  BSP_Buzzer_Init();
  BSP_Key_Init();
  BSP_Track_Init();
  BSP_Encoder_Init();
  BSP_Servo_Init();
  APP_Leg_Init();
  BSP_Motor_Init();
  BSP_OLED_Init();

  APP_Car_Init();
  APP_Menu_Init();

  /* 确保上电是"停着、静音、收起支撑脚"的安全状态。
     参数里万一存了个离谱的值（比如 BASESPD 很大），也不会在上电瞬间动车 */
  APP_Car_Stop();

  BSP_Buzzer_Beep(120u);
  main_boot_report();

  /* ★ 固定地图模式：先问一句"跑路线还是进菜单"。
       选 K1 → 执行 app_route.c 里那条路线脚本（内部阻塞，跑完返回）；
       选 K2 → 立刻返回，下面照常进主循环，就是"调参 + 手动 START"那一套。
       所以调参和跑路线互不干扰：想调参就选 K2。
       30 秒不按键会自动进菜单，免得开了电就走开、车干等在询问画面。 */
  APP_Route_Enter();

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
     * （OLED 刷新就是这么个情况），这一拍就会被拉长。
     *
     * ★ 用 (now - s_next_tick) 而不是 (now >= s_next_tick)：
     *   HAL_GetTick() 是 32 位毫秒，49.7 天会回绕归零。
     *   无符号相减能正确处理回绕，直接比大小则会在回绕那一刻卡死。
     *
     * 这里直接 s_next_tick = now 而不是 += 10，是刻意的：
     * 万一某轮超时了，让下一拍从"现在"重新起算，不会越积越多的追赶。
     * 代价是平均周期会略大于 10ms —— 没关系，编码器测速和 PID
     * 都用实测 dt，按真实时间换算，读数依然准。
     * ------------------------------------------------------------------ */
    now = HAL_GetTick();
    if ((uint32_t)(now - s_next_tick) >= MAIN_TICK_MS) {
      s_next_tick = now;

      BSP_Encoder_Update();   /* ① 读编码器计数差 —— 速度反馈的来源 */
      BSP_Key_Scan();         /* ② 推进按键状态机，产生短按/长按/连发事件 */
      BSP_Buzzer_Poll();      /* ③ 检查非阻塞鸣叫是否到点该关声 */
      APP_Car_Update();       /* ④ ★ 循迹控制律：误差 → PID → 左右轮差速 */
      APP_Leg_Update();       /* ⑤ 支撑脚角度斜坡 */
      APP_Menu_Update();      /* ⑥ 按键处理 + OLED 刷新（自带节流） */
    }

    /* ------------------------------------------------------------------
     * 串口命令：收到一个字符就跑对应的自检 / 调参命令
     * 非阻塞读，没数据时立刻返回，不会拖慢上面的 10ms 节拍
     * ------------------------------------------------------------------ */
    if (BSP_UART_GetChar(&ch) != 0u) {
      BSP_Test_RunCommand((char)ch);
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
