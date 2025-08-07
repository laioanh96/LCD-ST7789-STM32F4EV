/**
 * @file input_handler.h
 * @brief Input handler for button navigation - Xử lý đầu vào cho điều hướng nút
 */

#ifndef INPUT_HANDLER_H
#define INPUT_HANDLER_H

#include "lvgl.h"
#include "stm32f4xx_hal.h"

// Button definitions - Định nghĩa các nút
#define BTN_UP_PIN      GPIO_PIN_11    // PC0 - Nút lên
#define BTN_DOWN_PIN    GPIO_PIN_12    // PC1 - Nút xuống  
#define BTN_OK_PIN      GPIO_PIN_0    // PC2 - Nút OK

#define BTN_PORT        GPIOC

// Button states - Trạng thái nút
typedef struct {
    uint8_t up_pressed;     // Nút lên được nhấn
    uint8_t down_pressed;   // Nút xuống được nhấn
    uint8_t ok_pressed;     // Nút OK được nhấn
    uint32_t last_press_time; // Thời gian nhấn cuối
} button_state_t;

// Function prototypes - Khai báo hàm
void input_handler_init(void);
void input_read_cb(lv_indev_drv_t * indev_drv, lv_indev_data_t * data);
void input_handler_task(void);
lv_indev_t * get_keyboard_indev(void);  // Get input device - Lấy input device

#endif /* INPUT_HANDLER_H */
