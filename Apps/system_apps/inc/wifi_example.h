#ifndef WIFI_EXAMPLE_H
#define WIFI_EXAMPLE_H

#include "wifi_esp8266.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief WiFi Example Functions - Các hàm ví dụ WiFi
 * 
 * These functions demonstrate how to use the ESP8266 WiFi library
 * to check SSID, scan networks, and monitor connections.
 * 
 * Các hàm này minh họa cách sử dụng thư viện WiFi ESP8266
 * để kiểm tra SSID, quét mạng và giám sát kết nối.
 */

// Basic WiFi status checking - Kiểm tra trạng thái WiFi cơ bản
void wifi_check_current_connection(void);
void wifi_simple_status_check(void);

// Network scanning - Quét mạng
void wifi_scan_example(void);

// Connection management - Quản lý kết nối
void wifi_connect_and_check_example(const char* ssid, const char* password);

// Continuous monitoring - Giám sát liên tục
void wifi_monitor_example(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_EXAMPLE_H
