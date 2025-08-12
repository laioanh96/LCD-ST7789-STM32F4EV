#ifndef LOG_H
#define LOG_H

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Log levels - Mức độ log
typedef enum {
    LOG_LEVEL_ERROR = 0,
    LOG_LEVEL_WARN  = 1,
    LOG_LEVEL_INFO  = 2,
    LOG_LEVEL_DEBUG = 3,
    LOG_LEVEL_TRACE = 4
} log_level_t;

// Forward declaration - Khai báo trước
extern UART_HandleTypeDef huart1;

// Configuration - Cấu hình
#define LOG_UART_INSTANCE       huart1
#define LOG_UART_BAUDRATE      115200
#define LOG_BUFFER_SIZE        512    // Increased from 256
#define LOG_MAX_LINE_LENGTH    256    // Increased from 128

// Enable/Disable logging - Bật/tắt logging
#define LOG_ENABLE             1
#define LOG_ENABLE_TIMESTAMP   1
#define LOG_ENABLE_COLORS      1

// ANSI Color codes for terminal - Mã màu ANSI cho terminal
#if LOG_ENABLE_COLORS
#define LOG_COLOR_RED     "\033[31m"
#define LOG_COLOR_YELLOW  "\033[33m"
#define LOG_COLOR_GREEN   "\033[32m"
#define LOG_COLOR_BLUE    "\033[34m"
#define LOG_COLOR_CYAN    "\033[36m"
#define LOG_COLOR_RESET   "\033[0m"
#else
#define LOG_COLOR_RED     ""
#define LOG_COLOR_YELLOW  ""
#define LOG_COLOR_GREEN   ""
#define LOG_COLOR_BLUE    ""
#define LOG_COLOR_CYAN    ""
#define LOG_COLOR_RESET   ""
#endif

// Core functions - Hàm cốt lõi
void log_init(void);
void log_deinit(void);
void log_set_level(log_level_t level);
log_level_t log_get_level(void);

// Main logging function - Hàm logging chính
void log_printf(log_level_t level, const char* tag, const char* format, ...);

// Raw data transmission - Truyền dữ liệu thô
void log_raw(const char* data, uint16_t length);
void log_raw_string(const char* string);

// Convenience macros - Macro tiện lợi
#if LOG_ENABLE
#define LOG_E(tag, fmt, ...) log_printf(LOG_LEVEL_ERROR, tag, fmt, ##__VA_ARGS__)
#define LOG_W(tag, fmt, ...) log_printf(LOG_LEVEL_WARN,  tag, fmt, ##__VA_ARGS__)
#define LOG_I(tag, fmt, ...) log_printf(LOG_LEVEL_INFO,  tag, fmt, ##__VA_ARGS__)
#define LOG_D(tag, fmt, ...) log_printf(LOG_LEVEL_DEBUG, tag, fmt, ##__VA_ARGS__)
#define LOG_T(tag, fmt, ...) log_printf(LOG_LEVEL_TRACE, tag, fmt, ##__VA_ARGS__)

// Quick macros without tag - Macro nhanh không cần tag
#define LOGE(fmt, ...) LOG_E("", fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) LOG_W("", fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) LOG_I("", fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) LOG_D("", fmt, ##__VA_ARGS__)
#define LOGT(fmt, ...) LOG_T("", fmt, ##__VA_ARGS__)

// Function entry/exit tracing - Theo dõi vào/ra hàm
#define LOG_FUNC_ENTRY() LOG_T("FUNC", ">> %s", __FUNCTION__)
#define LOG_FUNC_EXIT()  LOG_T("FUNC", "<< %s", __FUNCTION__)

#else
// Disabled logging - Vô hiệu hóa logging
#define LOG_E(tag, fmt, ...)
#define LOG_W(tag, fmt, ...)
#define LOG_I(tag, fmt, ...)
#define LOG_D(tag, fmt, ...)
#define LOG_T(tag, fmt, ...)
#define LOGE(fmt, ...)
#define LOGW(fmt, ...)
#define LOGI(fmt, ...)
#define LOGD(fmt, ...)
#define LOGT(fmt, ...)
#define LOG_FUNC_ENTRY()
#define LOG_FUNC_EXIT()
#endif

// Utility functions - Hàm tiện ích
void log_print_hex(const uint8_t* data, uint16_t length, const char* title);
void log_print_system_info(void);
void log_print_separator(char character, uint8_t length);

// Buffer status - Trạng thái buffer
typedef struct {
    uint16_t total_size;
    uint16_t used_size;
    uint16_t free_size;
    uint32_t overflow_count;
    uint32_t error_count;
    uint32_t total_bytes_sent;
} log_status_t;

void log_get_status(log_status_t* status);
void log_reset_stats(void);
void log_print_stats(void);
void log_test_messages(void);
void log_debug_message_assembly(void);

#ifdef __cplusplus
}
#endif

#endif //LOG_H