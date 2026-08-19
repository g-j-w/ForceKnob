/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 力控旋钮（Force Knob）主程序
  *
  *  硬件：极客/WeAct STM32H743IIT6 核心板（V3）+ SimpleFOC Mini 驱动板
  *         + DFRobot 2804 无刷电机（自带 AS5600，7 对极）
  *
  *  引脚分配（按核心板 V3 原理图排针核对，杜邦线可直接接出）：
  *    ┌─────────────┬──────────────┬──────────────────────────────┐
  *    │ 功能         │ 引脚          │ 说明                          │
  *    ├─────────────┼──────────────┼──────────────────────────────┤
  *    │ PWM 三相     │ PC6/PC7/PC8  │ TIM8_CH1/CH2/CH3 → Mini IN1/2/3│
  *    │ 使能 EN      │ PB0          │ 驱动板 EN，拉高才有力          │
  *    │ 编码器 I2C1  │ PB8(SCL)/PB9(SDA) → AS5600                  │
  *    │ 串口  USART1 │ PB14(TX)/PB15(RX) → CH340                   │
  *    │ 按键(外接)   │ PA0          │ 面包板按键，按下接 GND          │
  *    └─────────────┴──────────────┴──────────────────────────────┘
  *  注意：本板 HSE 晶振用不了，时钟只用内部 HSI 64MHz → PLL → 240MHz。
  *
  * 用法：CubeMX 生成工程后，把各 USER CODE 段内容保留即可（本文件已完整）。
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

#include "encoder.h"        /* AS5600 编码器（I2C1，PB8/PB9） */
#include "force_feedback.h" /* 三种手感：段落/阻尼/弹簧 */
#include "button.h"         /* PA0 外接按键切模式 */
#include "foc_math.h"       /* 反 Park/反 Clarke/中心对齐等数学 */
#include <stdio.h>          /* printf / snprintf */
#include <string.h>
#include <math.h>
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

/* ============ 全局运行状态（volatile：中断里写，主循环里读） ============ */
volatile float g_angle    = 0.0f;  /* 机械角 [rad]，1kHz 中断里更新 */
volatile float g_velocity = 0.0f;  /* 角速度 [rad/s]（已 EMA 滤波） */
volatile float g_vq       = 0.0f;  /* 当前输出的 Vq 电压 [V]（显示/调试用） */
volatile int   g_mode     = 0;     /* 模式：0=SPIN自转 1=DETENT步进 2=DAMPING阻尼 */

/* ============ 关键参数 ============ */
#define VDC_VOLTAGE   12.0f    /* 母线电压 [V]，必须和实际 12V 供电一致 */
#define TIM8_ARR      5999.0f  /* TIM8 自动重装载值：
                                *  TIM8 时钟 = 120MHz，120M/6000 = 20kHz PWM
                                *  （改 TIM8 分频/周期时这里要跟着改） */
#define ALIGN_VOLTAGE 1.5f     /* 上电对齐电压 [V]，别超过 2V（2804 相阻 2.3Ω） */
#define ALIGN_TIME_MS 800      /* 对齐保持时间 [ms] */

/* ---- 模式 0 = SPIN 自转 参数 ----
 * 开路旋转：让磁场匀速旋转，电机跟着转（0.5 圈/秒）。
 * 用途：演示/测试驱动链路。 */
#define CONTROL_DT     0.001f                            /* 控制周期 1ms */
#define SPIN_TEST_VQ   2.5f                              /* 自转电压，别太大 */
#define SPIN_OMEGA_ELEC (2.0f * _PI * 0.5f * POLE_PAIRS) /* 电场转速 = 0.5转/秒 × 极对数 */
static float spin_theta = 0.0f;                          /* 电场旋转角 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_I2C1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM8_Init(void);
void MX_USART1_UART_Init(void);
static void MX_CORTEX_M7_Init(void);   /* 定义在 main.c 末尾，用 static 即可 */
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---------------------------------------------------------------
 * 1kHz 控制环中断回调
 * ---------------------------------------------------------------
 * CubeMX 在 stm32h7xx_it.c 里调用 HAL_TIM_IRQHandler(&htim2)，
 * 最终会走到这个回调。这里用强定义覆盖 HAL 的 __weak 版本。
 * 每 1ms 执行一次：
 *   读编码器 → 算速度 → 按模式算目标力矩 → 换算 Vq → FOC 变三相占空比
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        /* 1. 读 AS5600 角度（+内部做跨圈跟踪和速度 EMA 滤波） */
        encoder_update();
        g_angle    = encoder_get_angle();     /* 机械角 [rad] */
        g_velocity = encoder_get_velocity();  /* 角速度 [rad/s] */

        /* 2. 算 Vq + 反 Park：
         *    模式 0 = SPIN 自转：用固定 Vq 让磁场匀速旋转（spin_theta 一直加），
         *                        电机开路跟着转，不依赖编码器。
         *    模式 1/2 = DETENT/DAMPING：力反馈算 Vq，电气角 = 机械角×极对数。 */
        float valpha, vbeta;
        if (g_mode == 0)
        {
            spin_theta += SPIN_OMEGA_ELEC * CONTROL_DT;
            g_vq = SPIN_TEST_VQ;
            inv_park_transform(0.0f, g_vq, spin_theta, &valpha, &vbeta);
        }
        else
        {
            g_vq = force_feedback_compute(g_angle, g_velocity, g_mode);
            inv_park_transform(0.0f, g_vq, g_angle * POLE_PAIRS, &valpha, &vbeta);
        }

        float va, vb, vc;
        inv_clarke(valpha, vbeta, &va, &vb, &vc);
        center_align(&va, &vb, &vc, VDC_VOLTAGE);   /* 注入零序，钳位 ±VDC/2 */

        /* 3. 相电压 → 占空比（0~1）→ 直写 CCR（最快，不经过 HAL） */
        float da, db, dc;
        voltage_to_duty(va, vb, vc, VDC_VOLTAGE, &da, &db, &dc);

        TIM8->CCR1 = (uint32_t)(da * TIM8_ARR);
        TIM8->CCR2 = (uint32_t)(db * TIM8_ARR);
        TIM8->CCR3 = (uint32_t)(dc * TIM8_ARR);
    }
}

/* ---------------------------------------------------------------
 * 上电电机对齐（alignment）
 * ---------------------------------------------------------------
 * 没有这步，编码器零点和转子真实位置错位 → 力矩方向偏、电机发烫、手感乱。
 * 做法：A 相加 1.5V 保持 800ms，让转子被磁场吸到电气零度，
 *       然后把编码器清零。上电时电机"咔"地转一下是正常现象。
 */
static void motor_align(void)
{
    /* A 相占空比 = 0.5 + 1.5/12 = 0.625，B/C 相 = 0.5（零序，不产生净电压）
     * 这样 A 相对中性点电压 = (0.625-0.5)*12V = +1.5V */
    float duty = ALIGN_VOLTAGE / VDC_VOLTAGE + 0.5f;

    TIM8->CCR1 = (uint32_t)(duty   * TIM8_ARR);
    TIM8->CCR2 = (uint32_t)(0.5f   * TIM8_ARR);
    TIM8->CCR3 = (uint32_t)(0.5f   * TIM8_ARR);
    HAL_Delay(ALIGN_TIME_MS);

    encoder_reset_zero();          /* 当前位置记为机械角 0 */

    TIM8->CCR1 = 0; TIM8->CCR2 = 0; TIM8->CCR3 = 0;   /* 复位占空比 */
    HAL_Delay(50);
}

/* ---------------------------------------------------------------
 * printf 重定向到 USART1（CH340 串口）
 * ---------------------------------------------------------------
 * 新库链路：printf → _write(syscalls.c) → __io_putchar(本函数) → UART
 * 注意：本工程用的是 newlib-nano，默认不带浮点 %f！
 *       已在顶层 CMakeLists.txt 加了 -u _printf_float，别删。
 */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10);
    return ch;
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_CORTEX_M7_Init();
  /* USER CODE BEGIN 2 */
    /* ---- 用户外设初始化 ---- */
    encoder_init();         /* 编码器清零 */
    button_init();          /* 按键标志复位 */
    force_feedback_init();  /* 手感参数复位（弹簧零点清零） */

    /* 启动 TIM8 三路 PWM（初始占空比 0，安全）
     * 注意：TIM8 是高级定时器，HAL_TIM_PWM_Start 会自动打开主输出使能 MOE */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

    /* 关键！使能电机驱动：EN 拉高。
     * SimpleFOC Mini 的 DRV8313，EN 不拉高三相输出就是高阻，电机永远无力。
     * （PB0 上还并联了板上蓝色 LED，可当"驱动已使能"指示灯看） */
    HAL_GPIO_WritePin(EN_DRIVER_GPIO_Port, EN_DRIVER_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    motor_align();          /* 对齐 + 清编码器零点（上电"咔"一声正常） */

    /* 启动 TIM2 1kHz 控制环中断 */
    HAL_TIM_Base_Start_IT(&htim2);

    uint32_t debug_tick = 0;   /* 串口打印计时 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
        /* ---- 按键切换模式（0=SPIN自转 → 1=DETENT步进 → 2=DAMPING阻尼 → 0）---- */
        if (button_get_flag())
        {
            button_clear_flag();
            g_mode = (g_mode + 1) % 3;
            printf(">> BTN pressed, mode=%d\r\n", g_mode);   /* 调试：确认按键触发 */
        }

        /* ---- 串口调试输出（10Hz）---- */
        if (HAL_GetTick() - debug_tick >= 100)
        {
            debug_tick = HAL_GetTick();
            printf("M:%d A:%.3f V:%.3f Vq:%.3f\r\n",
                   g_mode, g_angle, g_velocity, g_vq);
        }
    /* USER CODE END 3 */
  }
  /* USER CODE END 4 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  *
  * 时钟树（本板 HSE 用不了，只能用内部 HSI 64MHz）：
  *   HSI 64M ─(DIVM1=4)→ 16M ─(DIVN1=30)→ 480M(VCO) ─(DIVP1=2)→ 240M 主频
  *   HPRE  /2 → HCLK 120MHz
  *   APB1/APB2 /1 → 120MHz
  * 注意：VOS 和 Flash 等待周期是 CubeMX 自动算的（VOS1 + 4WS @ 240MHz），
  *       不要手改。
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable（用内部 LDO） */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** 主稳压器电压档 VOS1（对应 240MHz，CubeMX 自动算） */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** 振荡器：只用 HSI，不开 HSE（板上晶振不工作） */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;      /* 64/4  = 16MHz  PLL 输入 */
  RCC_OscInitStruct.PLL.PLLN = 30;     /* 16*30 = 480MHz VCO */
  RCC_OscInitStruct.PLL.PLLP = 2;      /* 480/2 = 240MHz 系统主频 */
  RCC_OscInitStruct.PLL.PLLQ = 2;      /* 480/2 = 240MHz（外设时钟用） */
  RCC_OscInitStruct.PLL.PLLR = 2;      /* 480/2 = 240MHz */
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;  /* 8~16MHz 输入档 */
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;  /* VCO 480MHz 宽档 */
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** CPU/AHB/APB 总线分频 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;   /* 240/2 = 120MHz HCLK */
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;    /* 120MHz APB1 */
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;    /* 120MHz APB2 */
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Cortex M7 Cache 初始化（H7 默认全关，打开后 sinf/cosf 等更快）
  * @retval None
  * 注意：以后如果加 DMA/外设直接内存访问，要注意 D-Cache 一致性
  *       （CubeMX 生成内容，保持不动）。
  */
static void MX_CORTEX_M7_Init(void)
{
  /* Enable the I-Cache */
  SCB_EnableICache();

  /* Enable the D-Cache */
  SCB_EnableDCache();
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
