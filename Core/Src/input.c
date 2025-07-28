/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : input.c
  * @brief          : Analog Joystick Input Handler for LVGL
  ******************************************************************************
  * @attention
  *
  * Analog Joystick Module Implementation:
  * - VRX (PA0): Analog X-axis input via ADC1_IN0
  * - VRY (PA1): Analog Y-axis input via ADC1_IN1  
  * - SW (PC0):  Digital button input with pull-up
  *
  * Joystick behavior:
  * - Center position: VRX ≈ 1.65V, VRY ≈ 1.65V
  * - Left: VRX → 0V, Right: VRX → 3.3V
  * - Up: VRY → 0V, Down: VRY → 3.3V (may need inversion)
  * - Button: SW = LOW when pressed
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lvgl.h"
#include "stm32f4xx_hal.h"  // For ADC and GPIO HAL functions

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint16_t x_raw;      // ADC value 0-4095 for X-axis
    uint16_t y_raw;      // ADC value 0-4095 for Y-axis
    uint8_t button;      // Button state: 0=pressed, 1=released
} joystick_data_t;

typedef enum {
    JOY_CENTER = 0,
    JOY_UP,
    JOY_DOWN, 
    JOY_LEFT,
    JOY_RIGHT
} joystick_direction_t;

/* Private defines -----------------------------------------------------------*/
// ADC threshold values for direction detection
#define JOY_THRESHOLD_LOW   1500   // Below this = LOW side
#define JOY_THRESHOLD_HIGH  2500   // Above this = HIGH side  
#define JOY_CENTER_VAL      2048   // Center position (~1.65V)

// Debouncing timing
#define DEBOUNCE_MS         50     // Minimum time between readings

// ADC channels
#define ADC_CHANNEL_VRX     ADC_CHANNEL_0  // PA0
#define ADC_CHANNEL_VRY     ADC_CHANNEL_1  // PA1

// Mouse control settings
#define MOUSE_SPEED_MIN     1      // Minimum cursor speed
#define MOUSE_SPEED_MAX     8      // Maximum cursor speed
#define MOUSE_DEADZONE      200    // Deadzone around center for mouse mode
#define MOUSE_SENSITIVITY   150    // Lower = more sensitive

// Input modes
typedef enum {
    INPUT_MODE_KEYPAD = 0,    // Navigation keys (current default)
    INPUT_MODE_MOUSE = 1      // Mouse cursor control
} input_mode_t;

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;  // ADC handle declared in main.c

// Global input mode control
static input_mode_t current_input_mode = INPUT_MODE_KEYPAD;  // Default to keypad

// Mouse cursor position (for mouse mode)
static lv_coord_t cursor_x = 120;  // Start at center (240/2)
static lv_coord_t cursor_y = 120;  // Start at center (240/2)

// Mouse movement smoothing
static float velocity_x = 0.0f;
static float velocity_y = 0.0f;

/* Private function prototypes -----------------------------------------------*/
static uint16_t ADC_ReadChannel(uint32_t channel);
static joystick_data_t read_joystick(void);
static joystick_direction_t get_joystick_direction(joystick_data_t* joy);
static int16_t calculate_mouse_movement_speed(uint16_t raw_value);
static void update_mouse_position(joystick_data_t* joy);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Read ADC channel value
  * @param  channel: ADC channel to read (ADC_CHANNEL_0, ADC_CHANNEL_1, etc.)
  * @retval ADC value (0-4095)
  */
static uint16_t ADC_ReadChannel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Safety check: ensure ADC is initialized
    if (hadc1.Instance == NULL) {
        return 2048; // Return center value if ADC not ready
    }
    
    // Configure ADC channel
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        return 2048; // Return center value on config error
    }
    
    // Start ADC conversion
    if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        return 2048; // Return center value on start error
    }
    
    // Wait for conversion to complete with timeout
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint16_t result = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
        return result;
    }
    
    HAL_ADC_Stop(&hadc1);
    return 2048; // Return center value on timeout
}

/**
  * @brief  Read complete joystick data (X, Y, Button)
  * @retval joystick_data_t structure with current values
  */
static joystick_data_t read_joystick(void)
{
    joystick_data_t joy = {0};
    
    // TEMPORARILY: Return safe values to avoid HardFault
    // TODO: Re-enable ADC reading once system is stable
    joy.x_raw = 2048;  // Center value
    joy.y_raw = 2048;  // Center value
    joy.button = 1;    // Released
    
    return joy;
    
    /* ORIGINAL CODE - TEMPORARILY DISABLED
    // Read analog channels
    joy.x_raw = ADC_ReadChannel(ADC_CHANNEL_VRX);  // 0-4095
    joy.y_raw = ADC_ReadChannel(ADC_CHANNEL_VRY);  // 0-4095
    
    // Read digital button (active LOW with pull-up)
    joy.button = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
    
    return joy;
    */
}

/**
  * @brief  Convert analog joystick values to direction
  * @param  joy: Pointer to joystick data structure
  * @retval joystick_direction_t: Direction enum
  */
static joystick_direction_t get_joystick_direction(joystick_data_t* joy)
{
    // Check X axis first (Left/Right have priority)
    if (joy->x_raw < JOY_THRESHOLD_LOW) {
        return JOY_LEFT;
    } else if (joy->x_raw > JOY_THRESHOLD_HIGH) {
        return JOY_RIGHT;
    }
    
    // Check Y axis (Up/Down)
    if (joy->y_raw < JOY_THRESHOLD_LOW) {
        return JOY_UP;      // May need inversion based on mounting
    } else if (joy->y_raw > JOY_THRESHOLD_HIGH) {
        return JOY_DOWN;    // May need inversion based on mounting
    }
    
    return JOY_CENTER;
}

/**
  * @brief  Calculate mouse movement speed based on joystick deflection
  * @param  raw_value: ADC raw value (0-4095)
  * @retval Movement speed (-MOUSE_SPEED_MAX to +MOUSE_SPEED_MAX)
  */
static int16_t calculate_mouse_movement_speed(uint16_t raw_value)
{
    // Calculate distance from center
    int16_t distance_from_center = (int16_t)raw_value - JOY_CENTER_VAL;
    
    // Apply deadzone
    if (abs(distance_from_center) < MOUSE_DEADZONE) {
        return 0;  // No movement in deadzone
    }
    
    // Calculate speed based on deflection
    // More deflection = faster movement
    int16_t speed = abs(distance_from_center) / MOUSE_SENSITIVITY;
    if (speed < MOUSE_SPEED_MIN) speed = MOUSE_SPEED_MIN;
    if (speed > MOUSE_SPEED_MAX) speed = MOUSE_SPEED_MAX;
    
    // Apply direction
    return (distance_from_center > 0) ? speed : -speed;
}

/**
  * @brief  Update mouse cursor position with smooth movement
  * @param  joy: Pointer to joystick data structure
  * @retval None
  */
static void update_mouse_position(joystick_data_t* joy)
{
    // Calculate movement deltas
    int16_t delta_x = calculate_mouse_movement_speed(joy->x_raw);
    int16_t delta_y = calculate_mouse_movement_speed(joy->y_raw);
    
    // Apply smoothing/acceleration
    velocity_x = velocity_x * 0.7f + delta_x * 0.3f;  // Smooth movement
    velocity_y = velocity_y * 0.7f + delta_y * 0.3f;
    
    // Update cursor position
    cursor_x += (int16_t)velocity_x;
    cursor_y += (int16_t)velocity_y;
    
    // Clamp to screen boundaries
    if (cursor_x < 0) cursor_x = 0;
    if (cursor_x >= 240) cursor_x = 239;
    if (cursor_y < 0) cursor_y = 0;
    if (cursor_y >= 240) cursor_y = 239;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  LVGL input device read callback for analog joystick (Dual Mode)
  * @param  indev_drv: Input device driver
  * @param  data: Input data structure to fill
  * @retval None
  * @note   Supports both KEYPAD and MOUSE modes
  */
void joystick_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    static joystick_direction_t last_dir = JOY_CENTER;
    static uint8_t last_btn_state = 1;  // Released
    static uint32_t last_read_time = 0;
    
    // Debouncing: Skip if too soon since last read
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_read_time < DEBOUNCE_MS) {
        // Keep previous state based on current mode
        if (current_input_mode == INPUT_MODE_KEYPAD) {
            // Keypad mode: Keep previous key state
            if (last_dir != JOY_CENTER) {
                data->state = LV_INDEV_STATE_PR;
                switch(last_dir) {
                    case JOY_UP:    data->key = LV_KEY_UP;    break;
                    case JOY_DOWN:  data->key = LV_KEY_DOWN;  break;
                    case JOY_LEFT:  data->key = LV_KEY_LEFT;  break;
                    case JOY_RIGHT: data->key = LV_KEY_RIGHT; break;
                    default:        data->key = 0;           break;
                }
            } else if (last_btn_state == 0) {
                data->state = LV_INDEV_STATE_PR;
                data->key = LV_KEY_ENTER;
            } else {
                data->state = LV_INDEV_STATE_REL;
                data->key = 0;
            }
        } else {
            // Mouse mode: Keep previous cursor position
            data->point.x = cursor_x;
            data->point.y = cursor_y;
            data->state = (last_btn_state == 0) ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
        }
        return;
    }
    
    // Update read time
    last_read_time = current_time;
    
    // Read joystick
    joystick_data_t joy = read_joystick();
    
    // Update state variables
    last_btn_state = joy.button;
    
    // Handle input based on current mode
    if (current_input_mode == INPUT_MODE_KEYPAD) {
        // ========== KEYPAD MODE ==========
        joystick_direction_t current_dir = get_joystick_direction(&joy);
        last_dir = current_dir;
        
        // Handle directional movement
        if (current_dir != JOY_CENTER) {
            data->state = LV_INDEV_STATE_PR;
            
            switch(current_dir) {
                case JOY_UP:    data->key = LV_KEY_UP;    break;
                case JOY_DOWN:  data->key = LV_KEY_DOWN;  break;
                case JOY_LEFT:  data->key = LV_KEY_LEFT;  break;
                case JOY_RIGHT: data->key = LV_KEY_RIGHT; break;
                default:        data->key = 0;           break;
            }
        } 
        // Handle button press (active LOW)
        else if (joy.button == 0) {  
            data->state = LV_INDEV_STATE_PR;
            data->key = LV_KEY_ENTER;
        }
        // Nothing pressed
        else {
            data->state = LV_INDEV_STATE_REL;
            data->key = 0;
        }
    } else {
        // ========== MOUSE MODE ==========
        // Update cursor position based on joystick input
        update_mouse_position(&joy);
        
        // Fill LVGL mouse data
        data->point.x = cursor_x;
        data->point.y = cursor_y;
        
        // Handle button press (active LOW)
        if (joy.button == 0) {
            data->state = LV_INDEV_STATE_PR;  // Mouse clicked
        } else {
            data->state = LV_INDEV_STATE_REL; // Mouse released
        }
    }
}

/**
  * @brief  Initialize GPIO pins for joystick input
  * @note   Call this function in MX_GPIO_Init() or similar
  * @retval None
  */
void joystick_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();  // For ADC pins PA0, PA1
    __HAL_RCC_GPIOC_CLK_ENABLE();  // For button pin PC0
    
    // Configure analog pins for ADC (PA0, PA1)
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;  // VRX, VRY
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure digital button pin (PC0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;  // SW button
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;  // Pull-up, button press = LOW
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief  Get current joystick raw values for debugging
  * @param  x_raw: Pointer to store X raw value
  * @param  y_raw: Pointer to store Y raw value
  * @param  btn: Pointer to store button state
  * @retval None
  */
void joystick_get_raw_values(uint16_t* x_raw, uint16_t* y_raw, uint8_t* btn)
{
    joystick_data_t joy = read_joystick();
    if (x_raw) *x_raw = joy.x_raw;
    if (y_raw) *y_raw = joy.y_raw;
    if (btn) *btn = joy.button;
}

/**
  * @brief  Calibrate joystick center position
  * @note   Call this when joystick is in center position
  * @retval None
  */
void joystick_calibrate_center(void)
{
    // Read current position as center
    joystick_data_t joy = read_joystick();
    
    // You can store these values and adjust thresholds accordingly
    // For now, just use default center value (2048)
    // In future, you could implement dynamic threshold adjustment
}

/**
  * @brief  Set input mode (Keypad or Mouse)
  * @param  mode: INPUT_MODE_KEYPAD or INPUT_MODE_MOUSE
  * @retval None
  */
void joystick_set_input_mode(input_mode_t mode)
{
    current_input_mode = mode;
    
    // Reset cursor to center when switching to mouse mode
    if (mode == INPUT_MODE_MOUSE) {
        cursor_x = 120;  // Center X
        cursor_y = 120;  // Center Y
        velocity_x = 0.0f;
        velocity_y = 0.0f;
    }
}

/**
  * @brief  Get current input mode
  * @retval Current input mode (INPUT_MODE_KEYPAD or INPUT_MODE_MOUSE)
  */
input_mode_t joystick_get_input_mode(void)
{
    return current_input_mode;
}

/**
  * @brief  Toggle between keypad and mouse modes
  * @retval None
  */
void joystick_toggle_input_mode(void)
{
    if (current_input_mode == INPUT_MODE_KEYPAD) {
        joystick_set_input_mode(INPUT_MODE_MOUSE);
    } else {
        joystick_set_input_mode(INPUT_MODE_KEYPAD);
    }
}

/**
  * @brief  Get current mouse cursor position
  * @param  x: Pointer to store X coordinate
  * @param  y: Pointer to store Y coordinate
  * @retval None
  */
void joystick_get_cursor_position(lv_coord_t* x, lv_coord_t* y)
{
    if (x) *x = cursor_x;
    if (y) *y = cursor_y;
}

/**
  * @brief  Set mouse cursor position
  * @param  x: X coordinate (0-239)
  * @param  y: Y coordinate (0-239)
  * @retval None
  */
void joystick_set_cursor_position(lv_coord_t x, lv_coord_t y)
{
    // Clamp to screen boundaries
    if (x < 0) x = 0;
    if (x >= 240) x = 239;
    if (y < 0) y = 0;
    if (y >= 240) y = 239;
    
    cursor_x = x;
    cursor_y = y;
}

/**
  * @brief  Test function - read only digital button safely
  * @retval Button state: 0=pressed, 1=released
  */
uint8_t joystick_test_button(void)
{
    // Read digital button (active LOW with pull-up)
    return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
}

/**
  * @brief  Test function - read single ADC channel safely
  * @param  channel: ADC channel to read (0 or 1)
  * @retval ADC value (0-4095) or 2048 on error
  */
uint16_t joystick_test_adc_channel(uint8_t channel)
{
    if (channel > 1) return 2048;  // Invalid channel
    
    uint32_t adc_channel = (channel == 0) ? ADC_CHANNEL_0 : ADC_CHANNEL_1;
    return ADC_ReadChannel(adc_channel);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
