/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <ST7789.h>

#include "bitmap.h"
#include "fonts.h"

#include "lvgl.h"
#include "stdio.h"
#include "stdlib.h"  // Thêm cho malloc/free

// Include LVGL examples
#include "examples/lv_examples.h"
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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

osThreadId blinkLEDTaskHandle;
osThreadId lvglTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
void StartBlinkTask(void const * argument);
void StartLVGLTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// LVGL display buffer và flush callback - minimal version
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[240 * 1];  // Buffer cho chỉ 1 dòng pixel để tiết kiệm RAM

// LVGL flush callback function - FIXED for direct drawing
void my_disp_flush(lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p)
{
    int32_t x, y;
    for (y = area->y1; y <= area->y2; y++) {
        for (x = area->x1; x <= area->x2; x++) {
            ST7789_DrawPixel(x, y, color_p->full);
            color_p++;
        }
    }
    // Không cần ST7789_Update() vì vẽ trực tiếp
    lv_disp_flush_ready(disp_drv);
}

// Button definitions (PC0-PC4)
#define BTN_UP    0  // PC0
#define BTN_DOWN  1  // PC1  
#define BTN_LEFT  2  // PC2
#define BTN_RIGHT 3  // PC3
#define BTN_ENTER 4  // PC4

// Button state variables
static uint32_t last_key = 0;
static lv_indev_state_t last_state = LV_INDEV_STATE_REL;

// Function to read button states
uint32_t button_read(void)
{
    // Check each button (active LOW với pull-up)
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) return BTN_UP;
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) return BTN_DOWN;
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) return BTN_LEFT;
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) return BTN_RIGHT;
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == GPIO_PIN_RESET) return BTN_ENTER;
    
    return 0xFF; // No button pressed
}

// LVGL input device read callback
void keypad_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t last_btn = 0xFF;
    uint32_t btn_pressed = button_read();
    
    if(btn_pressed != 0xFF) {
        // Button is pressed
        data->state = LV_INDEV_STATE_PR;
        last_btn = btn_pressed;
        
        // Convert to LVGL keys
        switch(btn_pressed) {
            case BTN_UP:    data->key = LV_KEY_UP;    break;
            case BTN_DOWN:  data->key = LV_KEY_DOWN;  break;
            case BTN_LEFT:  data->key = LV_KEY_LEFT;  break;
            case BTN_RIGHT: data->key = LV_KEY_RIGHT; break;
            case BTN_ENTER: data->key = LV_KEY_ENTER; break;
            default:        data->key = 0;           break;
        }
    } else {
        // No button pressed
        data->state = LV_INDEV_STATE_REL;
        data->key = 0;
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
  MX_SPI2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	// ST7789 display initialization procedure
	ST7789_Init();
	// Setting the display rotation
	ST7789_rotation( 1 );

	// // Simple test để kiểm tra màn hình hoạt động
	// ST7789_FillScreen(ST7789_RED);
	// HAL_Delay(500);
	// ST7789_FillScreen(ST7789_GREEN);
	// HAL_Delay(500);
	// ST7789_FillScreen(ST7789_BLUE);
	// HAL_Delay(500);
	// ST7789_FillScreen(ST7789_BLACK);
	
	// // Vẽ text test
	// ST7789_print(10, 10, ST7789_WHITE, ST7789_BLACK, 0, &Font_11x18, 1, "ST7789 Ready!");

	/*
	// LVGL initialization - MOVED TO TASK TO AVOID DUPLICATE
	lv_init();
	
	// Display buffer initialization  
	lv_disp_draw_buf_init(&draw_buf, buf, NULL, 240 * 1);
	
	// Display driver initialization
	static lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);
	disp_drv.hor_res = 240;
	disp_drv.ver_res = 240;
	disp_drv.flush_cb = my_disp_flush;
	disp_drv.draw_buf = &draw_buf;
	lv_disp_drv_register(&disp_drv);
	*/
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of blinkLEDTask */
  osThreadDef(blinkLEDTask, StartBlinkTask, osPriorityNormal, 0, 64);  // Giảm từ 128->64
  blinkLEDTaskHandle = osThreadCreate(osThread(blinkLEDTask), NULL);

  /* definition and creation of lvglTask */
  osThreadDef(lvglTask, StartLVGLTask, osPriorityNormal, 0, 512);  // Tăng từ 256->512
  lvglTaskHandle = osThreadCreate(osThread(lvglTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* Configure Button pins as inputs */
  /*UP button: PC0, DOWN button: PC1, LEFT: PC2, RIGHT: PC3, ENTER: PC4*/
  __HAL_RCC_GPIOC_CLK_ENABLE();  // Thêm clock enable cho GPIOC
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up, button press = LOW
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlinkTask */
/**
  * @brief  Function implementing the blinkLEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlinkTask */
void StartBlinkTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    // HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLVGLTask */
/**
  * @brief  Function implementing the lvglTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLVGLTask */
void StartLVGLTask(void const * argument)
{
  /* USER CODE BEGIN StartLVGLTask */
  
  // Bước 1: Init LVGL
  lv_init();

  // Bước 2: Init display driver  
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, 240 * 1);

  // Bước 3: Init display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);         // Khởi tạo với default values
  disp_drv.hor_res = 240;              // Độ phân giải ngang
  disp_drv.ver_res = 240;              // Độ phân giải dọc
  disp_drv.flush_cb = my_disp_flush;   // Callback để vẽ lên LCD
  disp_drv.draw_buf = &draw_buf;       // Gán buffer đã tạo
  lv_disp_drv_register(&disp_drv);     // Đăng ký driver với LVGL

  // Initialize default theme
  lv_theme_t * theme = lv_theme_default_init(
      lv_disp_get_default(),           // Display mặc định
      lv_palette_main(LV_PALETTE_BLUE), // Primary color
      lv_palette_main(LV_PALETTE_RED),  // Secondary color  
      LV_THEME_DEFAULT_DARK,           // Dark/Light mode
      LV_FONT_DEFAULT                  // Font mặc định
  );
  lv_disp_set_theme(lv_disp_get_default(), theme);

  // Setup input device (keypad/buttons)  
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);       // Khởi tạo input driver
  indev_drv.type = LV_INDEV_TYPE_KEYPAD; // Loại input: keypad
  indev_drv.read_cb = keypad_read;     // Callback đọc button
  lv_indev_t * indev = lv_indev_drv_register(&indev_drv);

  // Tạo group cho input navigation
  lv_group_t * g = lv_group_create();  // Tạo group để navigate
  lv_indev_set_group(indev, g);        // Gán group cho input device

  
  // Test button với màu explicit trước
  lv_obj_t * btn = lv_btn_create(lv_scr_act());

  lv_obj_set_size(btn, 120, 50);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, -30);
  lv_obj_set_style_bg_color(btn, lv_color_hex(0x0080ff), LV_PART_MAIN); // Xanh dương
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
  
  lv_obj_t * label = lv_label_create(btn);
  lv_label_set_text(label, "Test Button");
  lv_obj_center(label);
  
  // Thêm vào group để có thể focus
  lv_group_add_obj(g, btn);

  // Chạy example button
  lv_example_btn_1();
  
  /* Infinite loop */
  for(;;)
  {
    // Chạy LVGL timer handler
    lv_timer_handler();
    osDelay(10);  // LVGL cần chạy thường xuyên
  }
  /* USER CODE END StartLVGLTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
