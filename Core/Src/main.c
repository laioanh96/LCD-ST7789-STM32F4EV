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


#include "stdio.h"
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
SPI_HandleTypeDef hspi2;

osThreadId blinkLEDTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
void StartBlinkTask(void const * argument);

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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
	// включаем под�?ветку ди�?пле�? BLK
//	HAL_GPIO_WritePin( BLK_GPIO_Port, BLK_Pin, GPIO_PIN_SET );

	// ST7789 display initialization procedure
	ST7789_Init();

	// Setting the display rotation is optional because mode 1 is set by default (there are 4 modes in total: 1, 2, 3, 4)
	ST7789_rotation( 1 );

	int hour = 12, minute = 34, second = 56; // Khởi tạo giờ phút giây ban đầu

	// Vẽ nền và mặt đồng hồ chỉ 1 lần
	ST7789_DrawImage(0, 0, 240, 240, logoRGB);
	ST7789_DrawRectangleFilled(40, 40, 200, 200, RGB565(30, 30, 30));
	ST7789_DrawRectangle(40, 40, 200, 200, ST7789_WHITE);

	// Vẽ các vạch giờ chỉ 1 lần
	int cx = (40 + 200) / 2;
	int cy = (40 + 200) / 2;
	for (int i = 0; i < 12; i++) {
	    float angle = (i * 30 - 90) * 3.14159f / 180.0f;
	    int x1 = cx + (int)(60 * cosf(angle));
	    int y1 = cy + (int)(60 * sinf(angle));
	    int x2 = cx + (int)(70 * cosf(angle));
	    int y2 = cy + (int)(70 * sinf(angle));
	    ST7789_DrawLine(x1, y1, x2, y2, ST7789_WHITE);
	}

	char timeStrOld[16] = "";
	while (1) {
	    // Xóa kim cũ bằng màu nền đồng hồ (hoặc vẽ đè)
	    // (Có thể lưu lại tọa độ kim cũ để xóa chính xác hơn)
	    // Ở đây đơn giản là vẽ lại vùng đồng hồ nhỏ quanh tâm
	    ST7789_DrawRectangleFilled(cx-71, cy-71, cx+71, cy+71, RGB565(30, 30, 30));
	    ST7789_DrawRectangle(40, 40, 200, 200, ST7789_WHITE);
	    for (int i = 0; i < 12; i++) {
	        float angle = (i * 30 - 90) * 3.14159f / 180.0f;
	        int x1 = cx + (int)(60 * cosf(angle));
	        int y1 = cy + (int)(60 * sinf(angle));
	        int x2 = cx + (int)(70 * cosf(angle));
	        int y2 = cy + (int)(70 * sinf(angle));
	        ST7789_DrawLine(x1, y1, x2, y2, ST7789_WHITE);
	    }

	    // Vẽ kim giờ
	    float angle_h = ((hour % 12) + minute / 60.0f) * 30.0f - 90.0f;
	    angle_h = angle_h * 3.14159f / 180.0f;
	    int hx = cx + (int)(40 * cosf(angle_h));
	    int hy = cy + (int)(40 * sinf(angle_h));
	    ST7789_DrawLine(cx, cy, hx, hy, RGB565(255, 0, 0));

	    // Vẽ kim phút
	    float angle_m = (minute + second / 60.0f) * 6.0f - 90.0f;
	    angle_m = angle_m * 3.14159f / 180.0f;
	    int mx = cx + (int)(55 * cosf(angle_m));
	    int my = cy + (int)(55 * sinf(angle_m));
	    ST7789_DrawLine(cx, cy, mx, my, RGB565(0, 255, 0));

	    // Vẽ kim giây
	    float angle_s = second * 6.0f - 90.0f;
	    angle_s = angle_s * 3.14159f / 180.0f;
	    int sx = cx + (int)(65 * cosf(angle_s));
	    int sy = cy + (int)(65 * sinf(angle_s));
	    ST7789_DrawLine(cx, cy, sx, sy, RGB565(0, 200, 255));

	    // Vẽ tâm đồng hồ
	    ST7789_DrawCircleFilled(cx, cy, 4, ST7789_WHITE);

	    // Xóa số cũ (vẽ đè bằng màu nền)
	    ST7789_DrawRectangleFilled(70, 210, 170, 230, RGB565(30,30,30));
	    // Hiển thị số giờ/phút/giây dạng số ở dưới
	    char timeStr[16];
	    sprintf(timeStr, "%02d:%02d:%02d", hour, minute, second);
	    ST7789_print(70, 210, ST7789_CYAN, RGB565(30,30,30), 1, &Font_11x18, 1, timeStr);

	    // Tăng thời gian (giả lập, nếu không có RTC)
	    HAL_Delay(1);
	    second++;
	    if (second >= 60) { second = 0; minute++; }
	    if (minute >= 60) { minute = 0; hour++; }
	    if (hour >= 24)   { hour = 0; }
	}

	ST7789_DrawImage( 0, 0, 240, 240, logoRGB );

	// ST7789_print( 20, 220, RGB565(180, 0, 0) , RGB565(0, 10, 120) , 1, &Font_11x18, 1, "Oanh Love Giang" );
//				// // очи�?тка только буфера кадра  ( при етом �?ам �?кран не очищаеть�?�? )
//				// //	#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //			ST7789_ClearFrameBuffer();
//				// //	#endif
//
//				// // закрашиваем ве�?ь �?кран указаным цветом
//				// ST7789_FillScreen( RGB565(255, 0, 0) );
//				// //#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //		ST7789_Update();
//				// //#endif
//				// HAL_Delay (2000);
//				// // закрашиваем ве�?ь �?кран указаным цветом
//				// ST7789_FillScreen( RGB565(0, 255, 0) );
//				// //#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //		ST7789_Update();
//				// //#endif
//				// HAL_Delay (2000);
//				// // закрашиваем ве�?ь �?кран указаным цветом
//				// ST7789_FillScreen( RGB565(0, 0, 255) );
//				// //#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //		ST7789_Update();
//				// //#endif
//				// HAL_Delay (2000);
//
//				// // пр�?моугольник закрашеный ( координата X и Y ( начина�? �? 0 ) ширина и вы�?ота в пик�?ел�?х )
//				// ST7789_DrawRectangleFilled(0, 0, 240, 240, RGB565(255, 255, 255)) ;
//				// //#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //		ST7789_Update();
//				// //#endif
//				// HAL_Delay (1000);
//
//				// for( uint8_t i = 0; i< 240; i+=3){
//				// 	// пр�?моугольник закрашеный ( координата X и Y ( начина�? �? 0 ) ширина и вы�?ота в пик�?ел�?х )
//				// 	ST7789_DrawRectangleFilled(i, i, 240-i, 240-i, RGB565(i/2, 255-i, 0+i)) ;
//				// }
//				// //#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //		ST7789_Update();
//				// //#endif
//
//				// for( uint8_t i = 0; i< 120; i+=3){
//				// 	// пр�?моугольник пу�?тотелый ( координата X и Y ( начина�? �? 0 ) ширина и вы�?ота в пик�?ел�?х )
//				// 	ST7789_DrawRectangle(i, i, 240-i, 240-i, RGB565(255, 0, 0)) ;
//				// }
//				// //#if FRAME_BUFFER	// е�?ли включен буфер кадра
//				// //		ST7789_Update();
//				// //#endif
//
//				// HAL_Delay (2000);
//
//
//		// ри�?уем цветную иконку. параметры координаты х и у ( начина�? �? 0 ), размер иконки шир и вы�?, им�? иконки ( ма�?�?ив )
//		//	ST7789_DrawImage( 80, 80, 85, 53, logoRGB	);
//		//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//		//		ST7789_Update();
//		//#endif
//
//	// закрашиваем ве�?ь �?кран указаным цветом
//	ST7789_FillScreen( RGB565(0, 10, 100) );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// печатаем �?имвол ( один ) параметры: х,  у, ( начина�? �? 0 ),  цвет �?имвола, цвет фона, вкл/выкл фон, размер шрифта, множитель шрифта (увеличивает в х раз шрифт ), �?ам �?имвол ( поддерживает кириллицу )
//	// ST7789_DrawChar( 20, 20, RGB565( 255, 255, 255 ) , RGB565( 0, 10, 10 ) , 0, &Font_16x26, 3, 'F' );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// печатаем �?троку параметры: х,  у, ( начина�? �? 0 ), цвет �?троки, цвет фона, вкл/выкл фон, размер шрифта, множитель шрифта (увеличивает в х раз шрифт ), �?ама �?трока ( поддерживает кириллицу )
//	ST7789_print( 50, 20, RGB565(255, 255, 255) , RGB565(0, 10, 100) , 1, &Font_16x26, 1, "STM32 TFT" );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// печатаем �?троку параметры: х,  у, ( начина�? �? 0 ),  цвет �?троки, цвет фона, вкл/выкл фон, размер шрифта, множитель шрифта (увеличивает в х раз шрифт ), �?ама �?трока ( поддерживает кириллицу )
//	ST7789_print( 10, 160, RGB565(255, 0, 0) , RGB565(0, 10, 100) , 1, &Font_11x18, 1, "Tôi là tên là Oanh" );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// печатаем �?троку параметры: х,  у, ( начина�? �? 0 ),  цвет �?троки, цвет фона, вкл/выкл фон, размер шрифта, множитель шрифта (увеличивает в х раз шрифт ), �?ама �?трока ( поддерживает кириллицу )
//	ST7789_print( 8, 200, RGB565(0, 255, 0) , RGB565(0, 10, 100) , 1, &Font_7x9, 2, "ST7789 : 240x320" );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// печатаем �?имвол �? указаным углом, параметры: х,  у, ( начина�? �? 0 ), цвет �?троки, цвет фона, вкл/выкл фон, размер шрифта, множитель шрифта (увеличивает в х раз шрифт ), угол поворота (0.0 - 360.0), �?ам�?имвол ( поддерживает кириллицу )
//	ST7789_DrawCharWithAngle( 50, 50, RGB565(255, 255, 255) , RGB565(0, 10, 100) , 1, &Font_11x18, 1, 90.0, 'R' );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// печатаем �?троку �? указаным углом, параметры: х,  у, ( начина�? �? 0 ), цвет �?троки, цвет фона, вкл/выкл фон, размер шрифта, множитель шрифта (увеличивает в х раз шрифт ), угол поворота (0.0 - 360.0), �?ама �?трока ( поддерживает кириллицу )
//	ST7789_printWithAngle( 100, 100, RGB565(255, 255, 255) , RGB565(0, 10, 100) , 1, &Font_11x18, 1, 180.0, "STM32 TFT" );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// ри�?уем цветную иконку. параметры координаты х и у ( начина�? �? 0 ), размер иконки шир и вы�?, им�? иконки ( ма�?�?ив )
//	ST7789_DrawImage( 0, 0, 240, 240, logoRGB );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// очи�?тка �?крана - закрашивает �?кран цветом черный
//	//ST7789_Clear();
//	// очи�?тка только буфера кадра  ( при етом �?ам �?кран не очищаеть�?�? )
//	//	#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//			ST7789_ClearFrameBuffer();
//	//	#endif
//
//	// ри�?уем монохромную иконку. параметры координаты х и у ( начина�? �? 0 ), им�? иконки ( ма�?�?ив ), размер иконки шир и вы�?, цвет отображени�?
//	//ST7789_DrawBitmap( 60, 200, logo, 128, 27, RGB565(255, 0, 0) );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// ри�?уем монохромную иконку �? указаным углом поворота, параметры координаты х и у ( начина�? �? 0 ), им�? иконки ( ма�?�?ив ), размер иконки шир и вы�?, цвет отображени�?, угол поворота (0-360)
//	ST7789_DrawBitmapWithAngle( 60, 150, logo, 128, 27, RGB565(255, 255, 255), 10.0 );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// очи�?тка �?крана - закрашивает �?кран цветом черный
//	//ST7789_Clear();
//
//	// круг пу�?тотелый
//	//ST7789_DrawCircle(50, 100, 50, RGB565(255, 0, 255));
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// круг закрашеный
//	//ST7789_DrawCircleFilled(50, 290, 20, RGB565(255, 40, 255)) ;
//
//	// ри�?уем �?лип�? ( координаты центра, радиу�? по Х радиу�? по У, цвет )
//	ST7789_DrawEllipse( 150, 150, 20, 80, RGB565(0, 0, 255) );
//
//	// ри�?уем �?лип�? закрашенный ( координаты центра, радиу�? по Х радиу�? по У, цвет )
//	ST7789_DrawEllipseFilled( 150, 150, 80, 20, RGB565(0, 0, 255) );
//
//	// ри�?уем �?лип�? под указаным углом наклона ( координаты центра, радиу�? по Х радиу�? по У, угол поворота (0-360), цвет )
//	ST7789_DrawEllipseWithAngle( 150, 150, 80, 20, 45.0, RGB565(0, 0, 255) );
//
//	// ри�?уем �?лип�? закрашенный под указаным углом наклона ( координаты центра, радиу�? по Х радиу�? по У, угол поворота (0-360), цвет )
//	ST7789_DrawEllipseFilledWithAngle( 150, 150, 80, 20, 200.0, RGB565(0, 255, 0) );
//
//	// лини�?
//	//ST7789_DrawLine(1, 319, 239, 319, RGB565(255, 255, 0));
//
//	// ри�?уем линию �? указаным углом и длиной ( начальные координаты, длина линии, угол поворота (0-360), и цвет линии )
//	ST7789_DrawLineWithAngle(100, 100, 50, 45.0, RGB565(255, 255, 255));
//
//	// пр�?моугольник закрашеный
//	//ST7789_DrawRectangleFilled(90, 265, 140, 310, RGB565(0, 255, 0)) ;
//
//	// пр�?моугольник пу�?тотелый
//	//ST7789_DrawRectangle(160, 265, 220, 309, RGB565(255, 255, 255)) ;
//
//	// ри�?уем треугольник пу�?тотелый
//	//ST7789_DrawTriangle(60, 40, 150, 100, 200, 200, RGB565(100, 255, 150) );
//
//	// ри�?уем треугольник закрашеный
//	//ST7789_DrawFilledTriangle(20, 40, 150, 100, 200, 200, RGB565(100, 255, 150) );
//
//	// ри�?уем пр�?моугольник �? закругленными кра�?ми ( закрашенный )
//	//ST7789_DrawFillRoundRect(10, 10, 50, 50, 10, RGB565(100, 255, 150));
//
//	// ри�?уем пр�?моугольник �? закругленными кра�?ми ( пу�?тотелый )
//	//ST7789_DrawRoundRect(10, 10, 50, 50, 10, RGB565(100, 255, 150));
//
//	// ри�?уем полукруг ( правое или левое полушарие (параметр 1 или 2) ) закрашенный
//	//ST7789_DrawFillCircleHelper(30, 30, 20 , 1, 0, RGB565(100, 255, 150));
//
//	// ри�?уем дугу ( четверть круга (параметр 1, 2, 4, 8) ) шириной 1 пик�?ель
//	//ST7789_DrawCircleHelper(30, 30, 20 , 1, RGB565(100, 255, 150));
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// переход в �?п�?щий режим
//	//ST7789_SleepModeEnter();
//
//	// выход их �?п�?щего режима
//	//ST7789_SleepModeExit();
//
//	// вкл/выкл ди�?пле�? 0-выкл 1- вкл
//	//ST7789_DisplayPower( 1 );
//
//	// инвер�?и�? цветов 0-вкл  1-выкл
//	//ST7789_InversionMode(1);
//
//	// ри�?ованиe дуга тол�?та�? ( ча�?ть круга ) ( координаты центра, радиу�?, начальный и конечный угол (0-360), цвет линии, толщина линии)
//	// е�?ли нужно нари�?овать наоборот другую ча�?ть то мен�?ем начальный угол и конечный ме�?тами
//	ST7789_DrawArc(100, 100, 50, 320, 220, RGB565(255, 255, 0), 5);
//	ST7789_DrawArc(100, 100, 50, 220, 320, RGB565(255, 0, 255), 5);
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// лини�? тол�?та�? ( по�?ледний параметр толшина )
//	ST7789_DrawLineThick(10, 120, 200, 140, RGB565(255, 255, 0), 5);
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
//
//	// лини�? тол�?та�? нужной длины и указаным углом поворота (0-360) ( по�?ледний параметр толшина )
//	ST7789_DrawLineThickWithAngle( 100, 100, 80, 90.0, RGB565(255, 255, 0), 10 );
//	//#if FRAME_BUFFER	// е�?ли включен буфер кадра
//	//		ST7789_Update();
//	//#endif
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
  osThreadDef(blinkLEDTask, StartBlinkTask, osPriorityNormal, 0, 128);
  blinkLEDTaskHandle = osThreadCreate(osThread(blinkLEDTask), NULL);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
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
