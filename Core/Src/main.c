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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "OLED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 6 字节数据包：AA | RUN | DIR | STEP_L | STEP_H | CHECK
   packed 保证不插入填充字节，sizeof 正好 6 */
typedef struct __attribute__((packed))
{
  uint8_t  header;    /* 固定 0xAA          */
  uint8_t  run;       /* 00 暂停 / 01 运行   */
  uint8_t  dir;       /* 00 正向 / 01 反向   */
  uint16_t step_ms;   /* 小端：低字节在前    */
  uint8_t  check;     /* 前 5 字节和，取低 8 位 */
} Packet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t  g_run  = 1;      /* 1 = 运行, 0 = 暂停 */
uint8_t  g_dir  = 0;      /* 0 = 正向, 1 = 反向 */
uint8_t  g_led  = 1;      /* 当前点亮第几颗：1~3 */
uint16_t g_step = 300;    /* 每颗亮多久：毫秒 */
uint32_t g_last = 0;      /* 上次换灯的时刻 */
uint8_t  g_dirty = 1;     /* 状态变了 → 需要刷新 OLED */

/* 串口接收：中断直接把字节写进这个结构体 */
static Packet          rx_pkt;
static volatile uint8_t pkt_ready = 0;

/* 串口中断发送：缓冲区必须是静态的，发送期间不能被改写 */
static uint8_t          fb_buf[64];
static volatile uint8_t fb_busy = 0;

/* 按键消抖用的状态（raw=原始电平, stable=消抖后电平, t=变化时刻） */
static uint8_t  key_dir_raw = 1, key_dir_stable = 1;
static uint32_t key_dir_t   = 0;
static uint8_t  key_run_raw = 1, key_run_stable = 1;
static uint32_t key_run_t   = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LED_Task(void);
static void LED_Show(uint8_t no);
void Key_Task(void);
void Packet_Task(void);
void Feedback_Send(void);
void UI_Task(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 屏幕方向测试：依次试 4 种组合，挑出显示正常的那一种 */
#define OLED_ORIENT_TEST 0

extern void OLED_WriteCommand(uint8_t Command);

/* 显示起始行测试：找出不出现“半行绕回”的那个值 */
#define OLED_OFFSET_TEST 0

#if OLED_OFFSET_TEST
void OLED_OffsetTest(void)
{
  static const uint8_t st[4] = { 0x40, 0x42, 0x44, 0x46 };  /* 起始行 0 / 2 / 4 / 6 */
  while (1) {
    for (int i = 0; i < 4; i++) {
      OLED_WriteCommand(st[i]);
      OLED_Clear();
      OLED_ShowString(1, 1, "STATE: RUN");
      OLED_ShowString(2, 1, "DIR: REV");
      OLED_ShowString(3, 1, "STEP: 300ms");
      OLED_ShowString(4, 1, "LED: 1");
      HAL_Delay(2500);
    }
  }
}
#endif


#if OLED_ORIENT_TEST
void OLED_OrientTest(void)
{
  static const uint8_t seg[2] = { 0xA0, 0xA1 };   /* 左右方向 */
  static const uint8_t com[2] = { 0xC0, 0xC8 };   /* 上下方向 */

  while (1) {
    for (int s = 0; s < 2; s++) {
      for (int c = 0; c < 2; c++) {

        OLED_WriteCommand(seg[s]);
        OLED_WriteCommand(com[c]);
        OLED_Clear();

        OLED_ShowString(1, 1, "STATE: RUN");
        OLED_ShowString(2, 1, "DIR: FWD");
        OLED_ShowString(3, 1, "STEP: 300ms");
        OLED_ShowString(4, 1, "LED: 1");

        HAL_Delay(2500);
      }
    }
  }
}
#endif
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  g_last = HAL_GetTick();
  LED_Show(g_led);              /* 上电立刻只亮第一颗 */

  OLED_Init();                 /* 屏初始化：内部会自己配好 PB8/PB9 */

#if OLED_ORIENT_TEST
  OLED_OrientTest();           /* 方向测试：死循环，2.5 秒换一种方向 */
#endif

#if OLED_OFFSET_TEST
  OLED_OffsetTest();           /* 起始行测试：2.5 秒换一个偏移 */
#endif
  UI_Task();                   /* 上电先把四行状态画出来 */

  /* 启动第一次串口中断接收：收满 6 字节才进回调 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_pkt, sizeof(rx_pkt));

  /* 上电先给电脑报一次当前状态 */
  Feedback_Send();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Key_Task();
    Packet_Task();
    LED_Task();
    UI_Task();
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
/* 只点亮第 no 颗，其余全灭 */
static void LED_Show(uint8_t no)
{
  /* 本实验的 LED 为共阳接法：低电平点亮，所以选中的那颗给 RESET */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, (no == 1) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, (no == 2) ? GPIO_PIN_RESET : GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, (no == 3) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 时间到了就跳到下一颗，不阻塞 */
void LED_Task(void)
{
  if (!g_run) return;                            /* 暂停：灯保持不动 */
  if (HAL_GetTick() - g_last < g_step) return;   /* 没到时间就退出 */
  g_last = HAL_GetTick();                        /* 记下这次换灯的时刻 */

  if (g_dir == 0)
    g_led = (g_led >= 3) ? 1 : g_led + 1;        /* 正向 1→2→3→1 */
  else
    g_led = (g_led <= 1) ? 3 : g_led - 1;        /* 反向 1→3→2→1 */

  LED_Show(g_led);
  g_dirty = 1;                                   /* 灯号变了，OLED 要跟着变 */
}

/* 消抖：返回 1 表示「刚刚按下」这一次，按住不放只返回一次 */
static uint8_t Key_Pressed(uint8_t *raw, uint8_t *stable, uint32_t *t,
                           GPIO_TypeDef *port, uint16_t pin)
{
  uint8_t now = (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 0 : 1;

  if (now != *raw) {          /* 电平变了，重新计时 */
    *raw = now;
    *t   = HAL_GetTick();
  }

  if (HAL_GetTick() - *t >= 15 && now != *stable) {
    *stable = now;            /* 稳定 15ms，承认这次变化 */
    if (*stable == 0) return 1;   /* 稳定为低 → 一次按下 */
  }
  return 0;
}

/* PB0 → 切方向；PB1 → 暂停/继续 */
void Key_Task(void)
{
  if (Key_Pressed(&key_dir_raw, &key_dir_stable, &key_dir_t, GPIOB, GPIO_PIN_0)) {
    g_dir   = !g_dir;
    g_dirty = 1;
    Feedback_Send();          /* 按键改了状态，也要回传电脑 */
  }

  if (Key_Pressed(&key_run_raw, &key_run_stable, &key_run_t, GPIOB, GPIO_PIN_1)) {
    g_run   = !g_run;
    g_last  = HAL_GetTick();   /* 恢复后从现在重新计时，避免立刻跳一格 */
    g_dirty = 1;
    Feedback_Send();
  }
}

/* 主循环里拆包：校验通过才改状态，不通过整包丢弃 */
void Packet_Task(void)
{
  if (!pkt_ready) return;

  Packet p = rx_pkt;             /* 先复制出来，避免处理期间被下一包覆盖 */
  pkt_ready = 0;

  /* 立刻重新挂上接收，为下一包做准备 */
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&rx_pkt, sizeof(rx_pkt));

  const uint8_t *b = (const uint8_t *)&p;
  uint8_t sum = 0;
  for (uint8_t i = 0; i < 5; i++) sum += b[i];   /* 前 5 字节累加 */

  if (p.header != 0xAA) return;                  /* 帧头不对，丢弃 */
  if (sum != p.check)   return;                  /* 校验不对，丢弃 */
  if (p.run > 1 || p.dir > 1) return;            /* 标志只允许 0/1 */
  if (p.step_ms < 100 || p.step_ms > 2000) return; /* 间隔越界，丢弃 */

  g_run   = p.run;
  g_dir   = p.dir;
  g_step  = p.step_ms;
  g_last  = HAL_GetTick();       /* 从这一刻重新计时 */
  g_dirty = 1;

  Feedback_Send();
}

/* 用中断发一行状态：STATE=RUN,DIR=FWD,STEP=300,LED=1 */
void Feedback_Send(void)
{
  if (fb_busy) return;           /* 上一次还没发完，本次跳过 */

  int n = snprintf((char *)fb_buf, sizeof(fb_buf),
                   "STATE=%s,DIR=%s,STEP=%u,LED=%u\r\n",
                   g_run ? "RUN" : "STOP",
                   g_dir ? "REV" : "FWD",
                   (unsigned int)g_step, (unsigned int)g_led);
  if (n <= 0 || n >= (int)sizeof(fb_buf)) return;

  fb_busy = 1;
  if (HAL_UART_Transmit_IT(&huart1, fb_buf, (uint16_t)n) != HAL_OK)
    fb_busy = 0;                 /* 启动失败就立刻清标志，下次还能发 */
}

/* 收满 6 字节触发：回调里只置标志，其余交给主循环 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    pkt_ready = 1;
  }
}

/* 发送完成触发：清忙标志 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    fb_busy = 0;
  }
}

/* 状态变了才刷屏：每行末尾补空格，用来擦掉上一次留下的旧字符 */
void UI_Task(void)
{
  char line[17];

  if (!g_dirty) return;
  g_dirty = 0;

  snprintf(line, sizeof(line), "STATE: %s   ", g_run ? "RUN" : "STOP");
  OLED_ShowString(1, 1, line);

  snprintf(line, sizeof(line), "DIR: %s   ", g_dir ? "REV" : "FWD");
  OLED_ShowString(2, 1, line);

  snprintf(line, sizeof(line), "STEP: %ums   ", (unsigned int)g_step);
  OLED_ShowString(3, 1, line);

  snprintf(line, sizeof(line), "LED: %u   ", (unsigned int)g_led);
  OLED_ShowString(4, 1, line);
}
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
