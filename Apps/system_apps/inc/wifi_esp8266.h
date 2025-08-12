#ifndef WIFI_ESP8266_H
#define WIFI_ESP8266_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ESP8266 Configuration - Cấu hình ESP8266
#define ESP8266_UART_INSTANCE     USART2    // Change this to your UART
#define ESP8266_BAUDRATE         115200
#define ESP8266_BUFFER_SIZE      1024
#define ESP8266_TIMEOUT          5000       // 5 seconds
#define ESP8266_MAX_CONNECTIONS  5
#define ESP8266_MAX_SSID_LEN     32
#define ESP8266_MAX_PASSWORD_LEN 64

// WiFi Status - Trạng thái WiFi
typedef enum {
    WIFI_STATUS_IDLE = 0,
    WIFI_STATUS_READY,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_DISCONNECTED,
    WIFI_STATUS_ERROR
} wifi_status_t;

// WiFi Security - Bảo mật WiFi
typedef enum {
    WIFI_SECURITY_OPEN = 0,
    WIFI_SECURITY_WEP,
    WIFI_SECURITY_WPA_PSK,
    WIFI_SECURITY_WPA2_PSK,
    WIFI_SECURITY_WPA_WPA2_PSK
} wifi_security_t;

// WiFi Mode - Chế độ WiFi
typedef enum {
    WIFI_MODE_STATION = 1,      // STA mode
    WIFI_MODE_AP = 2,           // AP mode  
    WIFI_MODE_BOTH = 3          // STA+AP mode
} wifi_mode_t;

// Connection Type - Loại kết nối
typedef enum {
    CONNECTION_TCP = 0,
    CONNECTION_UDP = 1
} connection_type_t;

// WiFi Network Info - Thông tin mạng WiFi
typedef struct {
    char ssid[ESP8266_MAX_SSID_LEN];
    int8_t rssi;                // Signal strength
    wifi_security_t security;
    uint8_t channel;
} wifi_network_t;

// WiFi Connection Info - Thông tin kết nối WiFi
typedef struct {
    char ssid[ESP8266_MAX_SSID_LEN];
    char password[ESP8266_MAX_PASSWORD_LEN];
    char ip[16];                // xxx.xxx.xxx.xxx
    char gateway[16];
    char netmask[16];
    char mac[18];               // xx:xx:xx:xx:xx:xx
    wifi_status_t status;
    int8_t rssi;
} wifi_connection_t;

// Core Functions - Hàm cốt lõi
bool esp8266_init(void);
bool esp8266_reset(void);
bool esp8266_test(void);
void esp8266_deinit(void);

// WiFi Station Mode Functions - Hàm chế độ Station
bool esp8266_set_mode(wifi_mode_t mode);
bool esp8266_scan_networks(wifi_network_t* networks, uint8_t* count, uint8_t max_networks);
bool esp8266_connect(const char* ssid, const char* password);
bool esp8266_disconnect(void);
bool esp8266_get_connection_info(wifi_connection_t* info);
wifi_status_t esp8266_get_status(void);

// Network Functions - Hàm mạng
bool esp8266_start_tcp_server(uint16_t port);
bool esp8266_connect_tcp(const char* host, uint16_t port, uint8_t* connection_id);
bool esp8266_send_data(uint8_t connection_id, const uint8_t* data, uint16_t length);
bool esp8266_send_string(uint8_t connection_id, const char* string);
bool esp8266_close_connection(uint8_t connection_id);

// HTTP Client Functions - Hàm HTTP Client  
typedef struct {
    uint16_t status_code;
    uint16_t content_length;
    char content_type[64];
    char* body;
} http_response_t;

bool esp8266_http_get(const char* url, http_response_t* response);
bool esp8266_http_post(const char* url, const char* content_type, const char* body, http_response_t* response);
void esp8266_http_free_response(http_response_t* response);

// Web Server Functions - Hàm Web Server
bool esp8266_start_web_server(uint16_t port);
bool esp8266_stop_web_server(void);
void esp8266_handle_web_requests(void);

// Utility Functions - Hàm tiện ích
bool esp8266_ping(const char* host, uint16_t* response_time);
void esp8266_get_module_info(char* info, uint8_t max_length);
void esp8266_print_status(void);
bool esp8266_get_current_ssid(char* ssid, uint8_t max_length);
void esp8266_print_available_networks(void);

// Low-level Communication - Giao tiếp mức thấp
bool esp8266_send_command(const char* command, const char* expected_response, uint32_t timeout);
void esp8266_process_received_data(void);

// AT Commands - Lệnh AT
#define ESP8266_CMD_TEST        "AT"
#define ESP8266_CMD_RESET       "AT+RST"
#define ESP8266_CMD_VERSION     "AT+GMR"
#define ESP8266_CMD_MODE        "AT+CWMODE"
#define ESP8266_CMD_SCAN        "AT+CWLAP"
#define ESP8266_CMD_CONNECT     "AT+CWJAP"
#define ESP8266_CMD_DISCONNECT  "AT+CWQAP"
#define ESP8266_CMD_START_SERVER "AT+CIPSERVER"
#define ESP8266_CMD_CONNECT_TCP "AT+CIPSTART"
#define ESP8266_CMD_SEND_DATA   "AT+CIPSEND"
#define ESP8266_CMD_CLOSE       "AT+CIPCLOSE"

// Response Strings - Chuỗi phản hồi
#define ESP8266_RESP_OK         "OK"
#define ESP8266_RESP_ERROR      "ERROR"
#define ESP8266_RESP_READY      "ready"
#define ESP8266_RESP_CONNECTED  "WIFI CONNECTED"
#define ESP8266_RESP_GOT_IP     "WIFI GOT IP"

#ifdef __cplusplus
}
#endif

#endif // WIFI_ESP8266_H