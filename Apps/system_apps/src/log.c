#include "log.h"

// Private variables - Biến riêng tư
static UART_HandleTypeDef log_uart;
static log_level_t current_log_level = LOG_LEVEL_INFO;
static char log_buffer[LOG_BUFFER_SIZE];
static log_status_t log_stats = {0};
static uint8_t log_initialized = 0;

// Private function prototypes - Khai báo hàm riêng tư
static void log_uart_init(void);
static void log_gpio_init(void);
static const char* log_level_to_string(log_level_t level);
static const char* log_level_to_color(log_level_t level);
static uint32_t log_get_timestamp(void);

/**
 * @brief Initialize logging system - Khởi tạo hệ thống logging
 */
void log_init(void)
{
    if (log_initialized) {
        return;
    }
    
    // Initialize GPIO for UART - Khởi tạo GPIO cho UART
    log_gpio_init();
    
    // Initialize UART - Khởi tạo UART
   log_uart_init();
    
    // Reset statistics - Reset thống kê
    log_reset_stats();
    
    log_initialized = 1;
    
    // Send initialization message - Gửi tin nhắn khởi tạo
    LOGI("Log system initialized at %d baud", LOG_UART_BAUDRATE);
    log_print_separator('=', 50);
}

/**
 * @brief Deinitialize logging system - Hủy khởi tạo hệ thống logging
 */
void log_deinit(void)
{
    if (!log_initialized) {
        return;
    }
    
    LOGI("Log system shutting down...");
    HAL_UART_DeInit(&log_uart);
    log_initialized = 0;
}

/**
 * @brief Initialize GPIO pins for UART - Khởi tạo chân GPIO cho UART
 */
static void log_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clock - Bật clock cho GPIO
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure UART pins - Cấu hình chân UART
    // PA9 -> USART1_TX, PA10 -> USART1_RX
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Initialize UART peripheral - Khởi tạo ngoại vi UART
 */
static void log_uart_init(void)
{
   // Enable UART clock - Bật clock cho UART
   __HAL_RCC_USART1_CLK_ENABLE();

   // Configure UART - Cấu hình UART
   log_uart.Instance = USART1;
   log_uart.Init.BaudRate = LOG_UART_BAUDRATE;
   log_uart.Init.WordLength = UART_WORDLENGTH_8B;
   log_uart.Init.StopBits = UART_STOPBITS_1;
   log_uart.Init.Parity = UART_PARITY_NONE;
   log_uart.Init.Mode = UART_MODE_TX_RX;
   log_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   log_uart.Init.OverSampling = UART_OVERSAMPLING_16;

   if (HAL_UART_Init(&log_uart) != HAL_OK) {
       // Error handling - Xử lý lỗi
       Error_Handler();
   }
}

/**
 * @brief Set log level - Đặt mức độ log
 */
void log_set_level(log_level_t level)
{
    current_log_level = level;
}

/**
 * @brief Get current log level - Lấy mức độ log hiện tại
 */
log_level_t log_get_level(void)
{
    return current_log_level;
}

/**
 * @brief Main logging function - Hàm logging chính
 */
void log_printf(log_level_t level, const char* tag, const char* format, ...)
{
    if (!log_initialized || level > current_log_level) {
        return;
    }
    
    va_list args;
    int written = 0;
    
    // Clear buffer - Xóa buffer
    memset(log_buffer, 0, LOG_BUFFER_SIZE);
    
    // Add timestamp if enabled - Thêm timestamp nếu được bật
    #if LOG_ENABLE_TIMESTAMP
    uint32_t timestamp = log_get_timestamp();
    written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written,
                       "[%08lu] ", timestamp);
    #endif
    
    // Add color and level - Thêm màu và mức độ
    written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written,
                       "%s[%s]%s ", 
                       log_level_to_color(level),
                       log_level_to_string(level),
                       LOG_COLOR_RESET);
    
    // Add tag if provided - Thêm tag nếu có
    if (tag && strlen(tag) > 0) {
        written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written,
                           "[%s] ", tag);
    }
    
    // Add formatted message - Thêm tin nhắn đã định dạng
    va_start(args, format);
    written += vsnprintf(log_buffer + written, LOG_BUFFER_SIZE - written, format, args);
    va_end(args);
    
    // Add newline - Thêm xuống dòng
    if (written < LOG_BUFFER_SIZE - 2) {
        strcat(log_buffer, "\r\n");
        written += 2;
    }
    
    // Send via UART - Gửi qua UART
    log_raw(log_buffer, written);
}

/**
 * @brief Send raw data via UART - Gửi dữ liệu thô qua UART
 */
void log_raw(const char* data, uint16_t length)
{
    if (!log_initialized || !data || length == 0) {
        return;
    }
    
    // Update statistics - Cập nhật thống kê
    if (length > LOG_BUFFER_SIZE) {
        log_stats.overflow_count++;
        length = LOG_BUFFER_SIZE;
    }
    
    log_stats.total_bytes_sent += length;
    
    // Send data with timeout - Gửi dữ liệu với timeout
    HAL_UART_Transmit(&log_uart, (uint8_t*)data, length, HAL_MAX_DELAY);
}

/**
 * @brief Send raw string via UART - Gửi chuỗi thô qua UART
 */
void log_raw_string(const char* string)
{
    if (string) {
        log_raw(string, strlen(string));
    }
}

/**
 * @brief Print hex dump - In dump hex
 */
void log_print_hex(const uint8_t* data, uint16_t length, const char* title)
{
    if (!data || length == 0) {
        return;
    }
    
    if (title) {
        LOGI("=== %s ===", title);
    }
    
    for (uint16_t i = 0; i < length; i += 16) {
        char hex_line[80] = {0};
        char ascii_line[17] = {0};
        int hex_pos = 0;
        
        // Address - Địa chỉ
        hex_pos += sprintf(hex_line, "%04X: ", i);
        
        // Hex bytes - Byte hex
        for (uint8_t j = 0; j < 16 && (i + j) < length; j++) {
            uint8_t byte = data[i + j];
            hex_pos += sprintf(hex_line + hex_pos, "%02X ", byte);
            ascii_line[j] = (byte >= 32 && byte <= 126) ? byte : '.';
        }
        
        // Padding - Đệm
        for (uint8_t j = (length - i > 16) ? 16 : (length - i); j < 16; j++) {
            hex_pos += sprintf(hex_line + hex_pos, "   ");
        }
        
        // ASCII representation - Biểu diễn ASCII
        sprintf(hex_line + hex_pos, " |%s|", ascii_line);
        
        LOGI("%s", hex_line);
    }
}

/**
 * @brief Print system information - In thông tin hệ thống
 */
void log_print_system_info(void)
{
    LOGI("=== System Information ===");
    LOGI("MCU: STM32F407VET6");
    LOGI("SYSCLK: %lu Hz", HAL_RCC_GetSysClockFreq());
    LOGI("HCLK: %lu Hz", HAL_RCC_GetHCLKFreq());
    LOGI("PCLK1: %lu Hz", HAL_RCC_GetPCLK1Freq());
    LOGI("PCLK2: %lu Hz", HAL_RCC_GetPCLK2Freq());
    LOGI("HAL Version: %lu", HAL_GetHalVersion());
    LOGI("UID: %08lX-%08lX-%08lX", 
         HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2());
}

/**
 * @brief Print separator line - In dòng phân cách
 */
void log_print_separator(char character, uint8_t length)
{
    char separator[81] = {0}; // Max 80 characters + null terminator
    
    if (length > 80) {
        length = 80;
    }
    
    memset(separator, character, length);
    LOGI("%s", separator);
}

/**
 * @brief Get logging statistics - Lấy thống kê logging
 */
void log_get_status(log_status_t* status)
{
    if (status) {
        status->total_size = LOG_BUFFER_SIZE;
        status->used_size = 0; // Not applicable for synchronous logging
        status->free_size = LOG_BUFFER_SIZE;
        status->overflow_count = log_stats.overflow_count;
        status->total_bytes_sent = log_stats.total_bytes_sent;
    }
}

/**
 * @brief Reset logging statistics - Reset thống kê logging
 */
void log_reset_stats(void)
{
    memset(&log_stats, 0, sizeof(log_stats));
    log_stats.total_size = LOG_BUFFER_SIZE;
    log_stats.free_size = LOG_BUFFER_SIZE;
}

// Private helper functions - Hàm trợ giúp riêng tư

/**
 * @brief Convert log level to string - Chuyển mức độ log thành chuỗi
 */
static const char* log_level_to_string(log_level_t level)
{
    switch (level) {
        case LOG_LEVEL_ERROR: return "E";
        case LOG_LEVEL_WARN:  return "W";
        case LOG_LEVEL_INFO:  return "I";
        case LOG_LEVEL_DEBUG: return "D";
        case LOG_LEVEL_TRACE: return "T";
        default: return "?";
    }
}

/**
 * @brief Convert log level to color - Chuyển mức độ log thành màu
 */
static const char* log_level_to_color(log_level_t level)
{
    switch (level) {
        case LOG_LEVEL_ERROR: return LOG_COLOR_RED;
        case LOG_LEVEL_WARN:  return LOG_COLOR_YELLOW;
        case LOG_LEVEL_INFO:  return LOG_COLOR_GREEN;
        case LOG_LEVEL_DEBUG: return LOG_COLOR_BLUE;
        case LOG_LEVEL_TRACE: return LOG_COLOR_CYAN;
        default: return LOG_COLOR_RESET;
    }
}

/**
 * @brief Get timestamp in milliseconds - Lấy timestamp tính bằng millisecond
 */
static uint32_t log_get_timestamp(void)
{
    return HAL_GetTick();
}

// Weak function for error handling - Hàm yếu cho xử lý lỗi
__weak void Error_Handler(void)
{
    // User can override this function - Người dùng có thể ghi đè hàm này
    while(1) {
        // Infinite loop - Vòng lặp vô hạn
    }
}
