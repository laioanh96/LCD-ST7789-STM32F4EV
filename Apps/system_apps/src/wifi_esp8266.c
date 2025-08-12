#include "wifi_esp8266.h"
#include "log.h"  // Include logging system
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Private variables - Biến riêng tư
static UART_HandleTypeDef esp8266_uart;
static char esp8266_buffer[ESP8266_BUFFER_SIZE];
static uint16_t buffer_index = 0;
static wifi_status_t current_wifi_status = WIFI_STATUS_IDLE;
static bool esp8266_initialized = false;

// Private function prototypes - Khai báo hàm riêng tư
static void esp8266_uart_init(void);
static void esp8266_gpio_init(void);
static bool esp8266_wait_response(const char* expected, uint32_t timeout);
static void esp8266_clear_buffer(void);
static bool esp8266_parse_ip_info(const char* response, wifi_connection_t* info);
static bool esp8266_send_web_response(uint8_t conn_id);

/**
 * @brief Initialize ESP8266 module - Khởi tạo module ESP8266
 */
bool esp8266_init(void)
{
    if (esp8266_initialized) {
        LOG_I("ESP8266", "ESP8266 already initialized");
        return true;
    }
    
    LOG_I("ESP8266", "Initializing ESP8266 module...");
    
    // Initialize GPIO and UART - Khởi tạo GPIO và UART
    LOG_I("ESP8266", "Initializing GPIO...");
    esp8266_gpio_init();
    
    LOG_I("ESP8266", "Initializing UART2...");
    esp8266_uart_init();
    
    LOG_I("ESP8266", "UART2 initialization complete. Baudrate: %d", ESP8266_BAUDRATE);
    
    // Wait for module startup - Chờ module khởi động
    LOG_I("ESP8266", "Waiting 2 seconds for ESP8266 to boot...");
    HAL_Delay(2000);
    
    // Test module communication - Test giao tiếp module
    LOG_I("ESP8266", "Testing AT communication...");
    if (!esp8266_test()) {
        LOG_E("ESP8266", "Failed to communicate with ESP8266");
        LOG_E("ESP8266", "Please check:");
        LOG_E("ESP8266", "  1. ESP8266 power supply (3.3V)");
        LOG_E("ESP8266", "  2. UART connections (PA2=TX, PA3=RX)");
        LOG_E("ESP8266", "  3. Baudrate (%d)", ESP8266_BAUDRATE);
        LOG_E("ESP8266", "  4. ESP8266 module is functioning");
        return false;
    }
    
    LOG_I("ESP8266", "AT communication successful");
    
    // Set station mode - Đặt chế độ station
    LOG_I("ESP8266", "Setting WiFi mode to Station...");
    if (!esp8266_set_mode(WIFI_MODE_STATION)) {
        LOG_E("ESP8266", "Failed to set station mode");
        return false;
    }
    
    esp8266_initialized = true;
    current_wifi_status = WIFI_STATUS_READY;
    
    LOG_I("ESP8266", "ESP8266 initialized successfully");
    return true;
}

/**
 * @brief Initialize GPIO pins for ESP8266 UART - Khởi tạo chân GPIO cho UART ESP8266
 */
static void esp8266_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Enable GPIO clock - Bật clock cho GPIO
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // Configure UART pins for USART2 (PA2=TX, PA3=RX)
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief Initialize UART for ESP8266 - Khởi tạo UART cho ESP8266
 */
static void esp8266_uart_init(void)
{
    // Enable UART clock - Bật clock cho UART
    __HAL_RCC_USART2_CLK_ENABLE();
    
    // Configure UART - Cấu hình UART
    esp8266_uart.Instance = ESP8266_UART_INSTANCE;
    esp8266_uart.Init.BaudRate = ESP8266_BAUDRATE;
    esp8266_uart.Init.WordLength = UART_WORDLENGTH_8B;
    esp8266_uart.Init.StopBits = UART_STOPBITS_1;
    esp8266_uart.Init.Parity = UART_PARITY_NONE;
    esp8266_uart.Init.Mode = UART_MODE_TX_RX;
    esp8266_uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    esp8266_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    
    if (HAL_UART_Init(&esp8266_uart) != HAL_OK) {
        LOG_E("ESP8266", "UART initialization failed");
        Error_Handler();
    }
}

/**
 * @brief Test multiple baudrates - Test nhiều baudrate
 */
/**
 * @brief Test multiple baudrates - Test nhiều baudrate
 */
/**
 * @brief Test UART connection with simple transmission - Test kết nối UART đơn giản
 */
bool esp8266_test_uart_basic(void)
{
    LOG_I("ESP8266", "Testing basic UART transmission...");
    
    // Clear buffer first
    esp8266_clear_buffer();
    
    // Send simple AT command
    const char* test_cmd = "AT";
    LOG_I("ESP8266", "Sending: '%s'", test_cmd);
    
    HAL_StatusTypeDef uart_status = HAL_UART_Transmit(&esp8266_uart, (uint8_t*)test_cmd, strlen(test_cmd), 1000);
    if (uart_status != HAL_OK) {
        LOG_E("ESP8266", "UART transmit failed! Status: %d", uart_status);
        return false;
    }
    
    uart_status = HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"\r\n", 2, 1000);
    if (uart_status != HAL_OK) {
        LOG_E("ESP8266", "UART transmit CRLF failed! Status: %d", uart_status);
        return false;
    }
    
    LOG_I("ESP8266", "UART transmission successful, checking for response...");
    
    // Try to receive any data
    uint8_t received_char;
    uint32_t start_time = HAL_GetTick();
    bool received_any_data = false;
    
    while ((HAL_GetTick() - start_time) < 2000) { // 2 second timeout
        if (HAL_UART_Receive(&esp8266_uart, &received_char, 1, 10) == HAL_OK) {
            if (!received_any_data) {
                LOG_I("ESP8266", "First byte received: 0x%02X ('%c')", received_char, 
                      (received_char >= 32 && received_char < 127) ? received_char : '?');
                received_any_data = true;
            }
            
            if (buffer_index < ESP8266_BUFFER_SIZE - 1) {
                esp8266_buffer[buffer_index++] = received_char;
                esp8266_buffer[buffer_index] = '\0';
            }
        }
    }
    
    if (received_any_data) {
        LOG_I("ESP8266", "Received %d bytes: '%s'", buffer_index, esp8266_buffer);
        return true;
    } else {
        LOG_W("ESP8266", "No response received from ESP8266");
        return false;
    }
}

/**
 * @brief Test ESP8266 communication - Test giao tiếp ESP8266
 */
bool esp8266_test(void)
{
    LOG_D("ESP8266", "Testing ESP8266 communication...");
    
    // First test basic UART transmission
    if (!esp8266_test_uart_basic()) {
        LOG_E("ESP8266", "Basic UART test failed");
        
        // Try a few common baudrates quickly
        LOG_I("ESP8266", "Trying common baudrates...");
        
        // Try 9600 baud
        HAL_UART_DeInit(&esp8266_uart);
        esp8266_uart.Init.BaudRate = 9600;
        if (HAL_UART_Init(&esp8266_uart) == HAL_OK) {
            LOG_I("ESP8266", "Testing 9600 baud...");
            esp8266_clear_buffer();
            HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"AT\r\n", 4, 1000);
            HAL_Delay(500);
            if (esp8266_test_uart_basic()) {
                LOG_I("ESP8266", "Found response at 9600 baud!");
                return true;
            }
        }
        
        // Try 57600 baud
        HAL_UART_DeInit(&esp8266_uart);
        esp8266_uart.Init.BaudRate = 57600;
        if (HAL_UART_Init(&esp8266_uart) == HAL_OK) {
            LOG_I("ESP8266", "Testing 57600 baud...");
            esp8266_clear_buffer();
            HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"AT\r\n", 4, 1000);
            HAL_Delay(500);
            if (esp8266_test_uart_basic()) {
                LOG_I("ESP8266", "Found response at 57600 baud!");
                return true;
            }
        }
        
        // Restore original baudrate
        HAL_UART_DeInit(&esp8266_uart);
        esp8266_uart.Init.BaudRate = ESP8266_BAUDRATE;
        HAL_UART_Init(&esp8266_uart);
        
        return false;
    }
    
    // Then try proper AT command with OK response
    return esp8266_send_command(ESP8266_CMD_TEST, ESP8266_RESP_OK, 1000);
}

/**
 * @brief Reset ESP8266 module - Reset module ESP8266
 */
bool esp8266_reset(void)
{
    LOG_I("ESP8266", "Resetting ESP8266 module...");
    
    if (!esp8266_send_command(ESP8266_CMD_RESET, ESP8266_RESP_READY, 5000)) {
        LOG_E("ESP8266", "Reset failed");
        return false;
    }
    
    current_wifi_status = WIFI_STATUS_READY;
    HAL_Delay(2000); // Wait for module to fully restart
    
    LOG_I("ESP8266", "Reset successful");
    return true;
}

/**
 * @brief Set WiFi mode - Đặt chế độ WiFi
 */
bool esp8266_set_mode(wifi_mode_t mode)
{
    char command[32];
    snprintf(command, sizeof(command), "%s=%d", ESP8266_CMD_MODE, mode);
    
    LOG_D("ESP8266", "Setting WiFi mode to %d", mode);
    return esp8266_send_command(command, ESP8266_RESP_OK, 2000);
}

/**
 * @brief Connect to WiFi network - Kết nối mạng WiFi
 */
bool esp8266_connect(const char* ssid, const char* password)
{
    if (!ssid || strlen(ssid) == 0) {
        LOG_E("ESP8266", "Invalid SSID");
        return false;
    }
    
    char command[128];
    if (password && strlen(password) > 0) {
        snprintf(command, sizeof(command), "%s=\"%s\",\"%s\"", ESP8266_CMD_CONNECT, ssid, password);
    } else {
        snprintf(command, sizeof(command), "%s=\"%s\"", ESP8266_CMD_CONNECT, ssid);
    }
    
    LOG_I("ESP8266", "Connecting to WiFi: %s", ssid);
    current_wifi_status = WIFI_STATUS_CONNECTING;
    
    if (esp8266_send_command(command, ESP8266_RESP_OK, 15000)) {
        // Wait for "WIFI GOT IP" message
        if (esp8266_wait_response(ESP8266_RESP_GOT_IP, 10000)) {
            current_wifi_status = WIFI_STATUS_CONNECTED;
            LOG_I("ESP8266", "WiFi connected successfully");
            return true;
        }
    }
    
    current_wifi_status = WIFI_STATUS_ERROR;
    LOG_E("ESP8266", "WiFi connection failed");
    return false;
}

/**
 * @brief Disconnect from WiFi - Ngắt kết nối WiFi
 */
bool esp8266_disconnect(void)
{
    LOG_I("ESP8266", "Disconnecting from WiFi...");
    
    if (esp8266_send_command(ESP8266_CMD_DISCONNECT, ESP8266_RESP_OK, 5000)) {
        current_wifi_status = WIFI_STATUS_DISCONNECTED;
        LOG_I("ESP8266", "WiFi disconnected");
        return true;
    }
    
    return false;
}

/**
 * @brief Get current WiFi status - Lấy trạng thái WiFi hiện tại
 */
wifi_status_t esp8266_get_status(void)
{
    return current_wifi_status;
}

/**
 * @brief Scan available WiFi networks - Quét mạng WiFi có sẵn
 */
bool esp8266_scan_networks(wifi_network_t* networks, uint8_t* count, uint8_t max_networks)
{
    if (!networks || !count) {
        return false;
    }
    
    *count = 0;
    
    LOG_I("ESP8266", "Scanning WiFi networks...");
    
    // Send scan command - Gửi lệnh quét
    esp8266_clear_buffer();
    if (!esp8266_send_command(ESP8266_CMD_SCAN, ESP8266_RESP_OK, 15000)) {
        LOG_E("ESP8266", "WiFi scan failed");
        return false;
    }
    
    // Parse scan results from buffer - Phân tích kết quả quét từ buffer
    char* line = strtok(esp8266_buffer, "\r\n");
    while (line && *count < max_networks) {
        // Look for lines starting with "+CWLAP:"
        if (strncmp(line, "+CWLAP:", 7) == 0) {
            // Format: +CWLAP:(security),(ssid),(rssi),(mac),(channel)
            // Example: +CWLAP:(3,"MyWiFi",-45,"aa:bb:cc:dd:ee:ff",6)
            
            char* data = line + 7; // Skip "+CWLAP:"
            if (*data == '(') {
                data++; // Skip opening parenthesis
                
                // Parse security
                int security = atoi(data);
                networks[*count].security = (wifi_security_t)security;
                
                // Find SSID (between quotes)
                char* ssid_start = strchr(data, '"');
                if (ssid_start) {
                    ssid_start++; // Skip opening quote
                    char* ssid_end = strchr(ssid_start, '"');
                    if (ssid_end) {
                        size_t ssid_len = ssid_end - ssid_start;
                        if (ssid_len < ESP8266_MAX_SSID_LEN) {
                            strncpy(networks[*count].ssid, ssid_start, ssid_len);
                            networks[*count].ssid[ssid_len] = '\0';
                            
                            // Parse RSSI (after second comma)
                            char* rssi_start = strchr(ssid_end + 1, ',');
                            if (rssi_start) {
                                networks[*count].rssi = atoi(rssi_start + 1);
                                
                                // Parse channel (after MAC address)
                                char* mac_start = strchr(rssi_start + 1, '"');
                                if (mac_start) {
                                    char* mac_end = strchr(mac_start + 1, '"');
                                    if (mac_end) {
                                        char* channel_start = strchr(mac_end + 1, ',');
                                        if (channel_start) {
                                            networks[*count].channel = atoi(channel_start + 1);
                                        }
                                    }
                                }
                                
                                LOG_D("ESP8266", "Found: %s (RSSI: %d, Ch: %d)", 
                                     networks[*count].ssid, 
                                     networks[*count].rssi,
                                     networks[*count].channel);
                                (*count)++;
                            }
                        }
                    }
                }
            }
        }
        line = strtok(NULL, "\r\n");
    }
    
    LOG_I("ESP8266", "Found %d WiFi networks", *count);
    return true;
}

/**
 * @brief Get current WiFi connection information - Lấy thông tin kết nối WiFi hiện tại
 */
bool esp8266_get_connection_info(wifi_connection_t* info)
{
    if (!info) {
        return false;
    }
    
    memset(info, 0, sizeof(wifi_connection_t));
    info->status = current_wifi_status;
    
    if (current_wifi_status != WIFI_STATUS_CONNECTED) {
        return false;
    }
    
    // Get IP information - Lấy thông tin IP
    if (!esp8266_send_command("AT+CIFSR", ESP8266_RESP_OK, 3000)) {
        return false;
    }
    
    // Parse IP information from buffer
    esp8266_parse_ip_info(esp8266_buffer, info);
    
    // Get current AP information - Lấy thông tin AP hiện tại
    if (esp8266_send_command("AT+CWJAP?", ESP8266_RESP_OK, 3000)) {
        // Parse current AP info
        char* cwjap_line = strstr(esp8266_buffer, "+CWJAP:");
        if (cwjap_line) {
            // Format: +CWJAP:"ssid","bssid",channel,rssi
            char* ssid_start = strchr(cwjap_line, '"');
            if (ssid_start) {
                ssid_start++; // Skip opening quote
                char* ssid_end = strchr(ssid_start, '"');
                if (ssid_end) {
                    size_t ssid_len = ssid_end - ssid_start;
                    if (ssid_len < ESP8266_MAX_SSID_LEN) {
                        strncpy(info->ssid, ssid_start, ssid_len);
                        info->ssid[ssid_len] = '\0';
                        
                        // Parse RSSI (after last comma)
                        char* rssi_start = strrchr(ssid_end, ',');
                        if (rssi_start) {
                            info->rssi = atoi(rssi_start + 1);
                        }
                    }
                }
            }
        }
    }
    
    return true;
}

/**
 * @brief Start TCP server - Khởi động server TCP
 */
bool esp8266_start_tcp_server(uint16_t port)
{
    char command[32];
    snprintf(command, sizeof(command), "%s=1,%d", ESP8266_CMD_START_SERVER, port);
    
    LOG_I("ESP8266", "Starting TCP server on port %d", port);
    return esp8266_send_command(command, ESP8266_RESP_OK, 3000);
}

/**
 * @brief Connect to TCP server - Kết nối server TCP
 */
bool esp8266_connect_tcp(const char* host, uint16_t port, uint8_t* connection_id)
{
    if (!host || !connection_id) {
        return false;
    }
    
    char command[128];
    snprintf(command, sizeof(command), "%s=\"TCP\",\"%s\",%d", ESP8266_CMD_CONNECT_TCP, host, port);
    
    LOG_I("ESP8266", "Connecting to TCP server %s:%d", host, port);
    
    if (esp8266_send_command(command, ESP8266_RESP_OK, 10000)) {
        *connection_id = 0; // Single connection mode
        return true;
    }
    
    return false;
}

/**
 * @brief Send data through connection - Gửi dữ liệu qua kết nối
 */
bool esp8266_send_data(uint8_t connection_id, const uint8_t* data, uint16_t length)
{
    if (!data || length == 0) {
        return false;
    }
    
    char command[32];
    snprintf(command, sizeof(command), "%s=%d", ESP8266_CMD_SEND_DATA, length);
    
    // Send length command - Gửi lệnh độ dài
    esp8266_clear_buffer();
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)command, strlen(command), 1000);
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"\r\n", 2, 1000);
    
    // Wait for ">" prompt - Chờ dấu nhắc ">"
    if (!esp8266_wait_response(">", 2000)) {
        LOG_E("ESP8266", "No prompt received for data transmission");
        return false;
    }
    
    // Send actual data - Gửi dữ liệu thực tế
    HAL_UART_Transmit(&esp8266_uart, data, length, 5000);
    
    // Wait for SEND OK - Chờ SEND OK
    return esp8266_wait_response("SEND OK", 5000);
}

/**
 * @brief Send string through connection - Gửi chuỗi qua kết nối
 */
bool esp8266_send_string(uint8_t connection_id, const char* string)
{
    if (!string) {
        return false;
    }
    
    return esp8266_send_data(connection_id, (const uint8_t*)string, strlen(string));
}

/**
 * @brief Close connection - Đóng kết nối
 */
bool esp8266_close_connection(uint8_t connection_id)
{
    char command[32];
    snprintf(command, sizeof(command), "%s=%d", ESP8266_CMD_CLOSE, connection_id);
    
    return esp8266_send_command(command, ESP8266_RESP_OK, 3000);
}

/**
 * @brief HTTP GET request - Yêu cầu HTTP GET
 */
bool esp8266_http_get(const char* url, http_response_t* response)
{
    if (!url || !response) {
        return false;
    }
    
    // Extract host and path from URL - Trích xuất host và path từ URL
    char host[64] = {0};
    char path[128] = "/";
    uint16_t port = 80;
    
    // Simple URL parsing (http://host:port/path)
    const char* host_start = strstr(url, "://");
    if (host_start) {
        host_start += 3; // Skip "://"
        const char* path_start = strchr(host_start, '/');
        if (path_start) {
            strncpy(host, host_start, path_start - host_start);
            strncpy(path, path_start, sizeof(path) - 1);
        } else {
            strncpy(host, host_start, sizeof(host) - 1);
        }
    }
    
    // Connect to server - Kết nối server
    uint8_t conn_id;
    if (!esp8266_connect_tcp(host, port, &conn_id)) {
        LOG_E("ESP8266", "Failed to connect to %s", host);
        return false;
    }
    
    // Send HTTP GET request - Gửi yêu cầu HTTP GET
    char http_request[256];
    snprintf(http_request, sizeof(http_request),
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n\r\n",
        path, host);
    
    bool result = esp8266_send_string(conn_id, http_request);
    
    // Close connection - Đóng kết nối
    esp8266_close_connection(conn_id);
    
    if (result) {
        // Parse response (simplified) - Phân tích phản hồi (đơn giản hóa)
        response->status_code = 200;
        response->content_length = 0;
        strcpy(response->content_type, "text/html");
        response->body = NULL;
    }
    
    return result;
}

/**
 * @brief HTTP POST request - Yêu cầu HTTP POST
 */
bool esp8266_http_post(const char* url, const char* content_type, const char* body, http_response_t* response)
{
    // Implementation similar to GET but with POST method
    // TODO: Implement full HTTP POST functionality
    return false;
}

/**
 * @brief Free HTTP response memory - Giải phóng bộ nhớ phản hồi HTTP
 */
void esp8266_http_free_response(http_response_t* response)
{
    if (response && response->body) {
        free(response->body);
        response->body = NULL;
    }
}

/**
 * @brief Ping host - Ping host
 */
bool esp8266_ping(const char* host, uint16_t* response_time)
{
    char command[64];
    snprintf(command, sizeof(command), "AT+PING=\"%s\"", host);
    
    return esp8266_send_command(command, ESP8266_RESP_OK, 5000);
}

/**
 * @brief Get module information - Lấy thông tin module
 */
void esp8266_get_module_info(char* info, uint8_t max_length)
{
    if (!info) return;
    
    if (esp8266_send_command(ESP8266_CMD_VERSION, ESP8266_RESP_OK, 2000)) {
        // Parse version info from buffer
        strncpy(info, "ESP8266 Module", max_length - 1);
        info[max_length - 1] = '\0';
    }
}

/**
 * @brief Print current status - In trạng thái hiện tại
 */
void esp8266_print_status(void)
{
    const char* status_strings[] = {
        "IDLE", "READY", "CONNECTING", "CONNECTED", "DISCONNECTED", "ERROR"
    };
    
    LOG_I("ESP8266", "Status: %s", status_strings[current_wifi_status]);
    
    // If connected, get and print connection info - Nếu đã kết nối, lấy và in thông tin kết nối
    if (current_wifi_status == WIFI_STATUS_CONNECTED) {
        wifi_connection_t info;
        if (esp8266_get_connection_info(&info)) {
            LOG_I("ESP8266", "Connected to SSID: %s", info.ssid);
            LOG_I("ESP8266", "IP Address: %s", info.ip);
            LOG_I("ESP8266", "MAC Address: %s", info.mac);
            LOG_I("ESP8266", "Signal Strength: %d dBm", info.rssi);
        }
    }
}

/**
 * @brief Get current connected SSID - Lấy SSID hiện tại đang kết nối
 */
bool esp8266_get_current_ssid(char* ssid, uint8_t max_length)
{
    if (!ssid || max_length == 0) {
        return false;
    }
    
    if (current_wifi_status != WIFI_STATUS_CONNECTED) {
        LOG_W("ESP8266", "Not connected to any WiFi network");
        return false;
    }
    
    wifi_connection_t info;
    if (esp8266_get_connection_info(&info)) {
        if (strlen(info.ssid) > 0) {
            strncpy(ssid, info.ssid, max_length - 1);
            ssid[max_length - 1] = '\0';
            LOG_I("ESP8266", "Current SSID: %s", ssid);
            return true;
        }
    }
    
    return false;
}

/**
 * @brief Print available WiFi networks - In danh sách mạng WiFi có sẵn
 */
void esp8266_print_available_networks(void)
{
    wifi_network_t networks[10]; // Maximum 10 networks
    uint8_t count = 0;
    
    LOG_I("ESP8266", "Scanning for available networks...");
    
    if (esp8266_scan_networks(networks, &count, 10)) {
        if (count > 0) {
            LOG_I("ESP8266", "Found %d networks:", count);
            for (uint8_t i = 0; i < count; i++) {
                const char* security_str = "Unknown";
                switch (networks[i].security) {
                    case WIFI_SECURITY_OPEN: security_str = "Open"; break;
                    case WIFI_SECURITY_WEP: security_str = "WEP"; break;
                    case WIFI_SECURITY_WPA_PSK: security_str = "WPA"; break;
                    case WIFI_SECURITY_WPA2_PSK: security_str = "WPA2"; break;
                    case WIFI_SECURITY_WPA_WPA2_PSK: security_str = "WPA/WPA2"; break;
                }
                
                LOG_I("ESP8266", "  %d. SSID: %-20s | RSSI: %3d dBm | Security: %-8s | Channel: %2d", 
                     i + 1, networks[i].ssid, networks[i].rssi, security_str, networks[i].channel);
            }
        } else {
            LOG_W("ESP8266", "No networks found");
        }
    } else {
        LOG_E("ESP8266", "Network scan failed");
    }
}

/**
 * @brief Send AT command and wait for response - Gửi lệnh AT và chờ phản hồi
 */
bool esp8266_send_command(const char* command, const char* expected_response, uint32_t timeout)
{
    if (!command || !expected_response) {
        return false;
    }
    
    LOG_D("ESP8266", "Sending: %s", command);
    
    // Clear buffer - Xóa buffer
    esp8266_clear_buffer();
    
    // Send command - Gửi lệnh
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)command, strlen(command), 1000);
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"\r\n", 2, 1000);
    
    // Wait for response - Chờ phản hồi
    return esp8266_wait_response(expected_response, timeout);
}

/**
 * @brief Wait for specific response - Chờ phản hồi cụ thể
 */
static bool esp8266_wait_response(const char* expected, uint32_t timeout)
{
    uint32_t start_time = HAL_GetTick();
    uint8_t received_char;
    uint32_t last_debug_time = start_time;
    
    LOG_D("ESP8266", "Waiting for response: '%s' (timeout: %lu ms)", expected, timeout);
    
    while ((HAL_GetTick() - start_time) < timeout) {
        if (HAL_UART_Receive(&esp8266_uart, &received_char, 1, 10) == HAL_OK) {
            if (buffer_index < ESP8266_BUFFER_SIZE - 1) {
                esp8266_buffer[buffer_index++] = received_char;
                esp8266_buffer[buffer_index] = '\0';
                
                // Debug: Print received chars every 100ms
                uint32_t current_time = HAL_GetTick();
                if (current_time - last_debug_time >= 100) {
                    LOG_D("ESP8266", "Received so far (%d chars): '%s'", buffer_index, esp8266_buffer);
                    last_debug_time = current_time;
                }
                
                // Check if expected response is found - Kiểm tra phản hồi mong đợi
                if (strstr(esp8266_buffer, expected)) {
                    LOG_D("ESP8266", "✓ Found expected response: '%s'", expected);
                    LOG_D("ESP8266", "Full buffer: '%s'", esp8266_buffer);
                    return true;
                }
                
                // Check for error responses - Kiểm tra phản hồi lỗi
                if (strstr(esp8266_buffer, ESP8266_RESP_ERROR) || 
                    strstr(esp8266_buffer, "FAIL")) {
                    LOG_W("ESP8266", "✗ Error response received");
                    LOG_W("ESP8266", "Buffer content: '%s'", esp8266_buffer);
                    return false;
                }
            } else {
                LOG_W("ESP8266", "Buffer overflow! Received data truncated. Consider increasing ESP8266_BUFFER_SIZE");
                // Don't clear buffer immediately, try to parse what we have
                break; // Exit the receive loop to parse current data
            }
        }
    }
    
    LOG_W("ESP8266", "✗ Timeout waiting for: '%s'", expected);
    LOG_W("ESP8266", "Final buffer content (%d chars): '%s'", buffer_index, esp8266_buffer);
    return false;
}

/**
 * @brief Parse IP information from response - Phân tích thông tin IP từ phản hồi
 */
static bool esp8266_parse_ip_info(const char* response, wifi_connection_t* info)
{
    if (!response || !info) {
        return false;
    }
    
    // Look for STAIP line - Tìm dòng STAIP
    char* staip_line = strstr(response, "+CIFSR:STAIP,");
    if (staip_line) {
        char* ip_start = strchr(staip_line, '"');
        if (ip_start) {
            ip_start++; // Skip opening quote
            char* ip_end = strchr(ip_start, '"');
            if (ip_end) {
                size_t ip_len = ip_end - ip_start;
                if (ip_len < sizeof(info->ip)) {
                    strncpy(info->ip, ip_start, ip_len);
                    info->ip[ip_len] = '\0';
                }
            }
        }
    }
    
    // Look for STAMAC line - Tìm dòng STAMAC
    char* stamac_line = strstr(response, "+CIFSR:STAMAC,");
    if (stamac_line) {
        char* mac_start = strchr(stamac_line, '"');
        if (mac_start) {
            mac_start++; // Skip opening quote
            char* mac_end = strchr(mac_start, '"');
            if (mac_end) {
                size_t mac_len = mac_end - mac_start;
                if (mac_len < sizeof(info->mac)) {
                    strncpy(info->mac, mac_start, mac_len);
                    info->mac[mac_len] = '\0';
                }
            }
        }
    }
    
    return true;
}

/**
 * @brief Clear receive buffer - Xóa buffer nhận
 */
static void esp8266_clear_buffer(void)
{
    memset(esp8266_buffer, 0, ESP8266_BUFFER_SIZE);
    buffer_index = 0;
}

/**
 * @brief Process received data - Xử lý dữ liệu nhận được
 */
void esp8266_process_received_data(void)
{
    // This function can be called from main loop to process incoming data
    // Implementation depends on specific requirements
}

/**
 * @brief Start Web Server - Khởi động Web Server
 */
bool esp8266_start_web_server(uint16_t port)
{
    LOG_I("ESP8266", "🚀 Starting Mobile-Compatible Web Server on port %d...", port);
    
    // Set timeout for server connections (important for mobile)
    if (!esp8266_send_command("AT+CIPSTO=180", ESP8266_RESP_OK, 2000)) {
        LOG_W("ESP8266", "Failed to set server timeout (continuing anyway)");
    }
    
    // Enable multiple connections first
    if (!esp8266_send_command("AT+CIPMUX=1", ESP8266_RESP_OK, 3000)) {
        LOG_E("ESP8266", "❌ Failed to enable multiple connections");
        return false;
    }
    LOG_I("ESP8266", "✅ Multiple connections enabled");
    
    // Start server
    char command[32];
    snprintf(command, sizeof(command), "AT+CIPSERVER=1,%d", port);
    
    if (esp8266_send_command(command, ESP8266_RESP_OK, 3000)) {
        LOG_I("ESP8266", "🎉 Web Server started successfully on port %d", port);
        LOG_I("ESP8266", "🌍 Access from any device at: http://192.168.1.8:%d", port);
        LOG_I("ESP8266", "📱 iPhone users: Make sure you're on WiFi 'Thu Giang' network");
        LOG_I("ESP8266", "💡 Try accessing multiple times if first attempt fails");
        return true;
    } else {
        LOG_E("ESP8266", "❌ Failed to start web server");
        return false;
    }
}

/**
 * @brief Stop Web Server - Dừng Web Server
 */
bool esp8266_stop_web_server(void)
{
    LOG_I("ESP8266", "Stopping Web Server...");
    
    if (esp8266_send_command("AT+CIPSERVER=0", ESP8266_RESP_OK, 3000)) {
        LOG_I("ESP8266", "Web Server stopped");
        return true;
    } else {
        LOG_E("ESP8266", "Failed to stop web server");
        return false;
    }
}

/**
 * @brief Handle Web Requests - Xử lý yêu cầu Web
 */
void esp8266_handle_web_requests(void)
{
    // Clear buffer and check for incoming data
    esp8266_clear_buffer();
    
    // Check for data with longer timeout for mobile compatibility
    uint8_t received_char;
    uint32_t start_time = HAL_GetTick();
    bool has_data = false;
    
    // Increased timeout for mobile browsers that may be slower
    while ((HAL_GetTick() - start_time) < 20) { // 20ms check instead of 10ms
        if (HAL_UART_Receive(&esp8266_uart, &received_char, 1, 1) == HAL_OK) {
            if (buffer_index < ESP8266_BUFFER_SIZE - 1) {
                esp8266_buffer[buffer_index++] = received_char;
                esp8266_buffer[buffer_index] = '\0';
                has_data = true;
                start_time = HAL_GetTick(); // Reset timeout when receiving data
            }
        }
    }
    
    if (has_data && buffer_index > 0) {
        // Only log if significant data received (reduce log spam)
        if (buffer_index > 10) {
            LOG_D("ESP8266", "📡 Received %d bytes from client", buffer_index);
        }
        
        // Check for new connection with better parsing
        if (strstr(esp8266_buffer, "+IPD,")) {
            LOG_I("ESP8266", "🔗 New client connection detected");
            
            // Parse connection ID
            char* ipd_start = strstr(esp8266_buffer, "+IPD,");
            if (ipd_start) {
                uint8_t conn_id = ipd_start[5] - '0';
                LOG_I("ESP8266", "📊 Connection ID: %d", conn_id);
                
                // Check for HTTP request (more comprehensive check for mobile browsers)
                bool is_http_request = false;
                if (strstr(esp8266_buffer, "GET ") || 
                    strstr(esp8266_buffer, "POST ") ||
                    strstr(esp8266_buffer, "HTTP/1.") ||
                    strstr(esp8266_buffer, "User-Agent:") ||
                    strstr(esp8266_buffer, "Host:")) {
                    is_http_request = true;
                }
                
                if (is_http_request) {
                    LOG_I("ESP8266", "✅ HTTP request detected from mobile/browser, sending response...");
                    
                    // Log the type of client if we can detect it
                    if (strstr(esp8266_buffer, "iPhone") || strstr(esp8266_buffer, "Safari")) {
                        LOG_I("ESP8266", "📱 iPhone/Safari client detected");
                    } else if (strstr(esp8266_buffer, "Android")) {
                        LOG_I("ESP8266", "🤖 Android client detected");
                    } else if (strstr(esp8266_buffer, "Chrome")) {
                        LOG_I("ESP8266", "🌐 Chrome browser detected");
                    } else {
                        LOG_I("ESP8266", "🖥️ Other browser/client detected");
                    }
                    
                    esp8266_send_web_response(conn_id);
                } else {
                    LOG_W("ESP8266", "⚠️ Non-HTTP data received: %s", esp8266_buffer);
                }
            }
        }
    }
}

/**
 * @brief Send Web Response - Gửi phản hồi Web
 */
static bool esp8266_send_web_response(uint8_t conn_id)
{
    LOG_I("ESP8266", "Preparing mobile-friendly web response for connection %d", conn_id);
    
    // Mobile-friendly HTML content with proper headers
    const char* html_body = 
        "<!DOCTYPE html>"
        "<html lang=\"vi\">"
        "<head>"
        "<meta charset=\"UTF-8\">"
        "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">"
        "<meta http-equiv=\"Cache-Control\" content=\"no-cache, no-store, must-revalidate\">"
        "<meta http-equiv=\"Pragma\" content=\"no-cache\">"
        "<meta http-equiv=\"Expires\" content=\"0\">"
        "<title>ESP8266 WiFi của GIANG</title>"
        "<style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Arial,sans-serif;"
        "text-align:center;padding:20px;margin:0;background:linear-gradient(135deg,#667eea 0%,#764ba2 100%);"
        "color:white;min-height:100vh;display:flex;flex-direction:column;justify-content:center;}"
        "h1{font-size:2.5em;margin:20px 0;text-shadow:2px 2px 4px rgba(0,0,0,0.3);}"
        "p{font-size:1.3em;margin:15px 0;}"
        ".highlight{background:rgba(255,255,255,0.2);padding:15px;border-radius:10px;margin:20px 0;}"
        ".info{font-size:1.1em;opacity:0.9;}"
        "@media (max-width: 480px){h1{font-size:2em;}p{font-size:1.1em;}}"
        "</style>"
        "</head>"
        "<body>"
        "<h1>🌐 ESP8266 Server</h1>"
        "<div class=\"highlight\">"
        "<p><strong>🎉 Đây là WiFi của GIANG 🎉</strong></p>"
        "</div>"
        "<p class=\"info\">📱 IP Address: 192.168.1.8</p>"
        "<p class=\"info\">✅ Kết nối thành công từ iPhone!</p>"
        "<p class=\"info\">⏰ " __DATE__ " " __TIME__ "</p>"
        "</body>"
        "</html>";
    
    // Calculate content length
    uint16_t body_length = strlen(html_body);
    
    // Prepare HTTP headers - more compatible with mobile browsers
    char http_headers[300];
    snprintf(http_headers, sizeof(http_headers),
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=UTF-8\r\n"
        "Content-Length: %d\r\n"
        "Connection: close\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Pragma: no-cache\r\n"
        "Expires: 0\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "\r\n", body_length);
    
    uint16_t headers_length = strlen(http_headers);
    uint16_t total_length = headers_length + body_length;
    char send_cmd[64];
    
    LOG_I("ESP8266", "Sending %d bytes total (%d headers + %d body) to connection %d", 
          total_length, headers_length, body_length, conn_id);
    
    // Prepare send command
    snprintf(send_cmd, sizeof(send_cmd), "AT+CIPSEND=%d,%d", conn_id, total_length);
    
    // Clear buffer before sending
    esp8266_clear_buffer();
    
    // Send the CIPSEND command
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)send_cmd, strlen(send_cmd), 1000);
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"\r\n", 2, 1000);
    
    LOG_I("ESP8266", "CIPSEND command sent, waiting for '>' prompt...");
    
    // Wait longer for ESP8266 to respond with ">" - mobile requests may need more time
    HAL_Delay(200);
    
    // Send HTTP headers first
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)http_headers, headers_length, 3000);
    LOG_I("ESP8266", "HTTP headers sent");
    
    // Small delay between headers and body
    HAL_Delay(50);
    
    // Send HTML body
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)html_body, body_length, 5000);
    LOG_I("ESP8266", "HTML body sent - mobile-friendly response complete");
    
    // Wait a bit longer for mobile browsers to process
    HAL_Delay(800);
    
    // Close the connection
    memset(send_cmd, 0, sizeof(send_cmd));
    snprintf(send_cmd, sizeof(send_cmd), "AT+CIPCLOSE=%d", conn_id);
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)send_cmd, strlen(send_cmd), 1000);
    HAL_UART_Transmit(&esp8266_uart, (uint8_t*)"\r\n", 2, 1000);
    
    LOG_I("ESP8266", "Connection %d closed", conn_id);
    return true;
}

/**
 * @brief Deinitialize ESP8266 - Hủy khởi tạo ESP8266
 */
void esp8266_deinit(void)
{
    if (esp8266_initialized) {
        HAL_UART_DeInit(&esp8266_uart);
        esp8266_initialized = false;
        current_wifi_status = WIFI_STATUS_IDLE;
        LOG_I("ESP8266", "ESP8266 deinitialized");
    }
}

// Weak function for error handling - Hàm yếu cho xử lý lỗi
__weak void Error_Handler(void)
{
    while(1) {
        // Infinite loop
    }
}