/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : input.h
  * @brief          : Header for input.c file - Analog Joystick Input Handler
  ******************************************************************************
  * @attention
  *
  * This header provides interface for analog joystick input handling
  * Compatible with LVGL input device system
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INPUT_H__
#define __INPUT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lvgl.h"

/* Exported types ------------------------------------------------------------*/
// Input modes
typedef enum {
    INPUT_MODE_KEYPAD = 0,    // Navigation keys (default)
    INPUT_MODE_MOUSE = 1      // Mouse cursor control
} input_mode_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  LVGL input device read callback for analog joystick
  * @param  indev_drv: Input device driver
  * @param  data: Input data structure to fill
  * @retval None
  * @note   Supports both KEYPAD and MOUSE modes automatically
  */
void joystick_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);

/**
  * @brief  Initialize GPIO pins for joystick input
  * @note   Call this function in MX_GPIO_Init() or similar
  * @retval None
  */
void joystick_gpio_init(void);

/**
  * @brief  Get current joystick raw values for debugging
  * @param  x_raw: Pointer to store X raw value (0-4095)
  * @param  y_raw: Pointer to store Y raw value (0-4095)  
  * @param  btn: Pointer to store button state (0=pressed, 1=released)
  * @retval None
  */
void joystick_get_raw_values(uint16_t* x_raw, uint16_t* y_raw, uint8_t* btn);

/**
  * @brief  Calibrate joystick center position
  * @note   Call this when joystick is in center position
  * @retval None
  */
void joystick_calibrate_center(void);

/**
  * @brief  Set input mode (Keypad or Mouse)
  * @param  mode: INPUT_MODE_KEYPAD or INPUT_MODE_MOUSE
  * @retval None
  */
void joystick_set_input_mode(input_mode_t mode);

/**
  * @brief  Get current input mode
  * @retval Current input mode (INPUT_MODE_KEYPAD or INPUT_MODE_MOUSE)
  */
input_mode_t joystick_get_input_mode(void);

/**
  * @brief  Toggle between keypad and mouse modes
  * @retval None
  * @note   Useful for runtime mode switching
  */
void joystick_toggle_input_mode(void);

/**
  * @brief  Get current mouse cursor position
  * @param  x: Pointer to store X coordinate
  * @param  y: Pointer to store Y coordinate
  * @retval None
  */
void joystick_get_cursor_position(lv_coord_t* x, lv_coord_t* y);

/**
  * @brief  Set mouse cursor position
  * @param  x: X coordinate (0-239)
  * @param  y: Y coordinate (0-239)
  * @retval None
  */
void joystick_set_cursor_position(lv_coord_t x, lv_coord_t y);

/**
  * @brief  Test function - read only digital button safely
  * @retval Button state: 0=pressed, 1=released
  */
uint8_t joystick_test_button(void);

/**
  * @brief  Test function - read single ADC channel safely
  * @param  channel: ADC channel to read (0 or 1)
  * @retval ADC value (0-4095) or 2048 on error
  */
uint16_t joystick_test_adc_channel(uint8_t channel);

/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __INPUT_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
