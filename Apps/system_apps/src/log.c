#include "log.h"

// External UART handle from main.c
extern UART_HandleTypeDef huart1;

// Private variables
static log_level_t current_log_level = LOG_LEVEL_INFO;
static char log_buffer[LOG_BUFFER_SIZE];
static log_status_t log_stats = {0};
static uint8_t log_initialized = 0;

// Private function prototypes
static const char* log_level_to_string(log_level_t level);
static const char* log_level_to_color(log_level_t level);
static unsigned long log_get_timestamp(void);

/**
 * @brief Initialize logging system
 */
void log_init(void)
{
    if (log_initialized) {
        return;
    }
    
    log_reset_stats();
    log_initialized = 1;
    
    // Send simple test message
    const char* init_msg = "\r\n=== LOG SYSTEM STARTED ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)init_msg, strlen(init_msg), 1000);
}

/**
 * @brief Deinitialize logging system
 */
void log_deinit(void)
{
    if (!log_initialized) {
        return;
    }
    
    const char* shutdown_msg = "=== LOG SYSTEM SHUTDOWN ===\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t*)shutdown_msg, strlen(shutdown_msg), 1000);
    log_initialized = 0;
}

/**
 * @brief Set log level
 */
void log_set_level(log_level_t level)
{
    current_log_level = level;
}

/**
 * @brief Get current log level
 */
log_level_t log_get_level(void)
{
    return current_log_level;
}

/**
 * @brief Main logging function
 */
void log_printf(log_level_t level, const char* tag, const char* format, ...)
{
    if (!log_initialized || level > current_log_level) {
        return;
    }
    
    va_list args;
    int written = 0;
    
    // Clear buffer
    memset(log_buffer, 0, LOG_BUFFER_SIZE);
    
    // Add timestamp if enabled
    #if LOG_ENABLE_TIMESTAMP
    unsigned long timestamp = log_get_timestamp();
    written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written,
                       "[%08lu] ", timestamp);
    #endif
    
    // Add color and level
    written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written,
                       "%s[%s]%s ", 
                       log_level_to_color(level),
                       log_level_to_string(level),
                       LOG_COLOR_RESET);
    
    // Add tag if provided
    if (tag && strlen(tag) > 0) {
        written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written,
                           "%s ", tag);
    }
    
    // Check if we have format string and arguments
    if (format && strlen(format) > 0) {
        // Calculate remaining space for message
        int remaining_space = LOG_BUFFER_SIZE - written - 3; // Reserve space for \r\n\0
        if (remaining_space > 0) {
            // Add formatted message
            va_start(args, format);
            int message_len = vsnprintf(log_buffer + written, remaining_space, format, args);
            va_end(args);
            
            // Check if message was written successfully
            if (message_len > 0 && message_len < remaining_space) {
                written += message_len;
            } else if (message_len >= remaining_space) {
                // Message was truncated, add truncation indicator
                written += remaining_space - 15; // Leave space for indicator
                written += snprintf(log_buffer + written, 15, "...[TRUNCATED]");
            }
        }
    }
    
    // Add newline
    if (written < LOG_BUFFER_SIZE - 2) {
        written += snprintf(log_buffer + written, LOG_BUFFER_SIZE - written, "\r\n");
    }
    
    // Send via UART
    log_raw(log_buffer, written);
}

/**
 * @brief Send raw data via UART
 */
void log_raw(const char* data, uint16_t length)
{
    if (!log_initialized || !data || length == 0) {
        return;
    }
    
    // Update statistics
    if (length > LOG_BUFFER_SIZE) {
        log_stats.overflow_count++;
        length = LOG_BUFFER_SIZE;
    }
    
    log_stats.total_bytes_sent += length;
    
    // Send data with timeout and error checking
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, length, 5000);
    
    if (status != HAL_OK) {
        log_stats.error_count++;
        // Try to send error indicator if possible
        const char* error_msg = "[LOG_ERR]\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)error_msg, strlen(error_msg), 1000);
    }
}

/**
 * @brief Send raw string via UART
 */
void log_raw_string(const char* string)
{
    if (string) {
        log_raw(string, strlen(string));
    }
}

/**
 * @brief Print hex dump
 */
void log_print_hex(const uint8_t* data, uint16_t length, const char* title)
{
    if (!log_initialized || !data) {
        return;
    }
    
    // Print title
    if (title) {
        LOG_I("HEX", "%s:", title);
    }
    
    // Print hex data in lines of 16 bytes
    char hex_line[64];
    for (uint16_t i = 0; i < length; i += 16) {
        int line_written = 0;
        
        // Print offset
        line_written += snprintf(hex_line + line_written, sizeof(hex_line) - line_written,
                                "%04X: ", i);
        
        // Print hex bytes
        for (uint16_t j = 0; j < 16 && (i + j) < length; j++) {
            line_written += snprintf(hex_line + line_written, sizeof(hex_line) - line_written,
                                    "%02X ", data[i + j]);
        }
        
        LOG_I("HEX", "%s", hex_line);
    }
}

/**
 * @brief Print system information
 */
void log_print_system_info(void)
{
    if (!log_initialized) {
        return;
    }
    
    log_print_separator('=', 50);
    LOG_I("SYS", "System Information:");
    LOG_I("SYS", "MCU: STM32F407VET6");
    LOG_I("SYS", "Clock: %lu MHz", HAL_RCC_GetHCLKFreq() / 1000000);
    LOG_I("SYS", "UART Baudrate: %d", LOG_UART_BAUDRATE);
    LOG_I("SYS", "Log Buffer Size: %d bytes", LOG_BUFFER_SIZE);
    LOG_I("SYS", "Tick: %lu ms", HAL_GetTick());
    log_print_separator('=', 50);
}

/**
 * @brief Print separator line
 */
void log_print_separator(char character, uint8_t length)
{
    if (!log_initialized) {
        return;
    }
    
    char separator[64] = {0};
    
    if (length > sizeof(separator) - 3) {
        length = sizeof(separator) - 3;
    }
    
    memset(separator, character, length);
    strcat(separator, "\r\n");
    
    log_raw_string(separator);
}

/**
 * @brief Get log statistics
 */
void log_get_status(log_status_t* status)
{
    if (status) {
        *status = log_stats;
        status->total_size = LOG_BUFFER_SIZE;
        status->used_size = 0; // Not applicable in this implementation
        status->free_size = LOG_BUFFER_SIZE;
    }
}

/**
 * @brief Reset log statistics
 */
void log_reset_stats(void)
{
    memset(&log_stats, 0, sizeof(log_stats));
}

/**
 * @brief Print log statistics
 */
void log_print_stats(void)
{
    if (!log_initialized) {
        return;
    }
    
    log_print_separator('-', 40);
    log_raw_string("LOG STATISTICS:\r\n");
    
    char stats_buffer[128];
    snprintf(stats_buffer, sizeof(stats_buffer), 
             "Buffer Size: %d bytes\r\n", LOG_BUFFER_SIZE);
    log_raw_string(stats_buffer);
    
    snprintf(stats_buffer, sizeof(stats_buffer), 
             "Total Bytes Sent: %lu\r\n", log_stats.total_bytes_sent);
    log_raw_string(stats_buffer);
    
    snprintf(stats_buffer, sizeof(stats_buffer), 
             "Overflow Count: %lu\r\n", log_stats.overflow_count);
    log_raw_string(stats_buffer);
    
    snprintf(stats_buffer, sizeof(stats_buffer), 
             "Error Count: %lu\r\n", log_stats.error_count);
    log_raw_string(stats_buffer);
    
    log_print_separator('-', 40);
}

/**
 * @brief Test log messages with different lengths
 */
void log_test_messages(void)
{
    if (!log_initialized) {
        return;
    }
    
    // Test simple raw transmission first
    log_raw_string("=== RAW TEST START ===\r\n");
    
    // Test 1: Very simple message
    LOG_I("TEST", "A");
    HAL_Delay(50);
    
    // Test 2: Short message
    LOG_I("TEST", "Hello");
    HAL_Delay(50);
    
    // Test 3: Medium message  
    LOG_I("TEST", "This is a test message");
    HAL_Delay(50);
    
    // Test 4: Check what's in the buffer by sending it raw
    memset(log_buffer, 0, LOG_BUFFER_SIZE);
    snprintf(log_buffer, LOG_BUFFER_SIZE, "[DEBUG] Buffer test message\r\n");
    log_raw_string(log_buffer);
    HAL_Delay(50);
    
    log_raw_string("=== RAW TEST END ===\r\n");
    
    // Print statistics after test
    HAL_Delay(100);
    log_print_stats();
}

/**
 * @brief Simple debug function to test message assembly
 */
void log_debug_message_assembly(void)
{
    if (!log_initialized) {
        return;
    }
    
    // Test manual message assembly
    char test_buffer[256];
    memset(test_buffer, 0, sizeof(test_buffer));
    
    // Add timestamp
    unsigned long timestamp = HAL_GetTick();
    int written = snprintf(test_buffer, sizeof(test_buffer), "[%08lu] ", timestamp);
    
    // Add level
    written += snprintf(test_buffer + written, sizeof(test_buffer) - written, "[I] ");
    
    // Add tag  
    written += snprintf(test_buffer + written, sizeof(test_buffer) - written, "DEBUG ");
    
    // Add message
    written += snprintf(test_buffer + written, sizeof(test_buffer) - written, "Manual assembly test");
    
    // Add newline
    written += snprintf(test_buffer + written, sizeof(test_buffer) - written, "\r\n");
    
    // Send via raw UART
    HAL_UART_Transmit(&huart1, (uint8_t*)test_buffer, written, 1000);
    
    // Test direct call to log_printf
    HAL_Delay(100);
    log_printf(LOG_LEVEL_INFO, "DIRECT", "Direct call test");
    
    // Test with different message types
    HAL_Delay(100);
    log_printf(LOG_LEVEL_INFO, "TEST", "Hello World");
}

// Private helper functions

/**
 * @brief Convert log level to string
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
 * @brief Convert log level to color
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
 * @brief Get timestamp in milliseconds
 */
static unsigned long log_get_timestamp(void)
{
    return HAL_GetTick();
}
