
/**
 * @file input_handler.c
 * @brief Input handler implementation - Triển khai xử lý đầu vào
 */

#include "input_handler.h"
#include "home_display.h"
#include "cmsis_os.h"

// Global button state - Trạng thái nút toàn cục
static button_state_t btn_state = {0};
static lv_indev_t * keyboard_indev = NULL;

// Button debounce time in ms - Thời gian chống dội nút (ms)
#define DEBOUNCE_TIME 200  // Tăng lên 200ms cho joystick SW

/**
 * @brief Initialize input handler - Khởi tạo xử lý đầu vào
 */
void input_handler_init(void)
{
    // Initialize button GPIO pins - Khởi tạo các chân GPIO cho nút
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIOC clock for buttons - Bật clock cho GPIOC cho nút
    __HAL_RCC_GPIOC_CLK_ENABLE();
    // Enable GPIOA clock for LED - Bật clock cho GPIOA cho LED
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure button pins as input with pull-up - Cấu hình chân nút như đầu vào với pull-up
    GPIO_InitStruct.Pin = BTN_UP_PIN | BTN_DOWN_PIN | BTN_OK_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BTN_PORT, &GPIO_InitStruct);
    
    // Register input device with LVGL - Đăng ký thiết bị đầu vào với LVGL
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_KEYPAD;  // Dùng KEYPAD cho buttons
    indev_drv.read_cb = input_read_cb;
    keyboard_indev = lv_indev_drv_register(&indev_drv);
}

/**
 * @brief Read input callback for LVGL - Callback đọc đầu vào cho LVGL
 */
void input_read_cb(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static uint32_t last_key = 0;
    
    // Read button states (active low with pull-up) - Đọc trạng thái nút (active low với pull-up)
    uint8_t btn_up = !HAL_GPIO_ReadPin(BTN_PORT, BTN_UP_PIN);
    uint8_t btn_down = !HAL_GPIO_ReadPin(BTN_PORT, BTN_DOWN_PIN);
    uint8_t btn_ok = !HAL_GPIO_ReadPin(BTN_PORT, BTN_OK_PIN);
    
    // Default state - Trạng thái mặc định
    data->state = LV_INDEV_STATE_REL;
    data->key = last_key;
    
    // Check for button presses with debounce - Kiểm tra nhấn nút với chống dội
    uint32_t current_time = HAL_GetTick();
    
    if (current_time - btn_state.last_press_time > DEBOUNCE_TIME) {
        if (btn_up && !btn_state.up_pressed) {
            // Direct group navigation - Điều hướng group trực tiếp (PC0 - next)
            if(main_group) {
                lv_group_focus_next(main_group);  // Move focus to next icon
            }
            data->state = LV_INDEV_STATE_PR;
            btn_state.up_pressed = 1;
            btn_state.last_press_time = current_time;
        }
        else if (btn_down && !btn_state.down_pressed) {
            // Direct group navigation - Điều hướng group trực tiếp (PC1 - previous) 
            if(main_group) {
                lv_group_focus_prev(main_group);   // Move focus to previous icon
            }
            data->state = LV_INDEV_STATE_PR;
            btn_state.down_pressed = 1;
            btn_state.last_press_time = current_time;
        }
        else if (btn_ok && !btn_state.ok_pressed) {
            // Send ENTER key for selection - Gửi phím ENTER để chọn (PC2)
            if(main_group) {
                lv_obj_t * focused_obj = lv_group_get_focused(main_group);
                if(focused_obj) {
                    lv_event_send(focused_obj, LV_EVENT_CLICKED, NULL);  // Simulate click
                }
            }
            data->state = LV_INDEV_STATE_PR;
            btn_state.ok_pressed = 1;
            btn_state.last_press_time = current_time;
        }
    }
    
    // Reset button states when released - Reset trạng thái nút khi thả
    if (!btn_up) btn_state.up_pressed = 0;
    if (!btn_down) btn_state.down_pressed = 0;
    if (!btn_ok) btn_state.ok_pressed = 0;
}

/**
 * @brief Input handler task - Task xử lý đầu vào
 */
void input_handler_task(void)
{
    // This function can be called periodically to handle input
    // Hàm này có thể được gọi định kỳ để xử lý đầu vào
    // Currently handled by LVGL timer system
    // Hiện tại được xử lý bởi hệ thống timer LVGL
}

/**
 * @brief Get keyboard input device - Lấy keyboard input device
 */
lv_indev_t * get_keyboard_indev(void)
{
    return keyboard_indev;
}
