#include "wifi_esp8266.h"
#include "log.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * @brief Example: Check current WiFi status and SSID - Ví dụ: Kiểm tra trạng thái WiFi và SSID hiện tại
 */
void wifi_check_current_connection(void)
{
    LOG_I("WIFI_EXAMPLE", "=== Checking Current WiFi Connection ===");
    
    // Get current status - Lấy trạng thái hiện tại
    wifi_status_t status = esp8266_get_status();
    
    if (status == WIFI_STATUS_CONNECTED) {
        // Method 1: Get just the SSID - Phương pháp 1: Chỉ lấy SSID
        char current_ssid[ESP8266_MAX_SSID_LEN];
        if (esp8266_get_current_ssid(current_ssid, sizeof(current_ssid))) {
            LOG_I("WIFI_EXAMPLE", "Connected to: %s", current_ssid);
        }
        
        // Method 2: Get full connection info - Phương pháp 2: Lấy đầy đủ thông tin kết nối
        wifi_connection_t info;
        if (esp8266_get_connection_info(&info)) {
            LOG_I("WIFI_EXAMPLE", "Full Connection Info:");
            LOG_I("WIFI_EXAMPLE", "  SSID: %s", info.ssid);
            LOG_I("WIFI_EXAMPLE", "  IP: %s", info.ip);
            LOG_I("WIFI_EXAMPLE", "  MAC: %s", info.mac);
            LOG_I("WIFI_EXAMPLE", "  Signal: %d dBm", info.rssi);
        }
    } else {
        LOG_W("WIFI_EXAMPLE", "Not connected to WiFi");
    }
}

/**
 * @brief Example: Scan and display available networks - Ví dụ: Quét và hiển thị mạng có sẵn
 */
void wifi_scan_example(void)
{
    LOG_I("WIFI_EXAMPLE", "=== WiFi Network Scan ===");
    
    // Method 1: Use built-in print function - Phương pháp 1: Dùng hàm in có sẵn
    esp8266_print_available_networks();
    
    // Method 2: Manual scan and process - Phương pháp 2: Quét thủ công và xử lý
    wifi_network_t networks[5]; // Scan maximum 5 networks
    uint8_t count = 0;
    
    if (esp8266_scan_networks(networks, &count, 5)) {
        LOG_I("WIFI_EXAMPLE", "Manual scan found %d networks:", count);
        
        // Find strongest signal - Tìm tín hiệu mạnh nhất
        int8_t strongest_rssi = -100;
        uint8_t strongest_index = 0;
        
        for (uint8_t i = 0; i < count; i++) {
            if (networks[i].rssi > strongest_rssi) {
                strongest_rssi = networks[i].rssi;
                strongest_index = i;
            }
        }
        
        if (count > 0) {
            LOG_I("WIFI_EXAMPLE", "Strongest signal: %s (%d dBm)", 
                 networks[strongest_index].ssid, strongest_rssi);
        }
        
        // Find open networks - Tìm mạng mở
        LOG_I("WIFI_EXAMPLE", "Open networks:");
        bool found_open = false;
        for (uint8_t i = 0; i < count; i++) {
            if (networks[i].security == WIFI_SECURITY_OPEN) {
                LOG_I("WIFI_EXAMPLE", "  - %s (Channel %d, RSSI %d dBm)", 
                     networks[i].ssid, networks[i].channel, networks[i].rssi);
                found_open = true;
            }
        }
        if (!found_open) {
            LOG_I("WIFI_EXAMPLE", "  No open networks found");
        }
    }
}

/**
 * @brief Example: Connect to WiFi and check status - Ví dụ: Kết nối WiFi và kiểm tra trạng thái
 */
void wifi_connect_and_check_example(const char* ssid, const char* password)
{
    LOG_I("WIFI_EXAMPLE", "=== Connect and Check Example ===");
    
    if (!ssid) {
        LOG_E("WIFI_EXAMPLE", "SSID cannot be NULL");
        return;
    }
    
    LOG_I("WIFI_EXAMPLE", "Attempting to connect to: %s", ssid);
    
    // Try to connect - Thử kết nối
    if (esp8266_connect(ssid, password)) {
        LOG_I("WIFI_EXAMPLE", "Connection successful!");
        
        // Wait a moment for IP assignment - Chờ một chút để được cấp IP
        HAL_Delay(2000);
        
        // Check connection status - Kiểm tra trạng thái kết nối
        wifi_check_current_connection();
        
        // Print detailed status - In trạng thái chi tiết
        esp8266_print_status();
        
    } else {
        LOG_E("WIFI_EXAMPLE", "Connection failed!");
        LOG_I("WIFI_EXAMPLE", "Current status: %d", esp8266_get_status());
    }
}

/**
 * @brief Example: Monitor WiFi connection continuously - Ví dụ: Giám sát kết nối WiFi liên tục
 */
void wifi_monitor_example(void)
{
    static uint32_t last_check_time = 0;
    static char last_ssid[ESP8266_MAX_SSID_LEN] = {0};
    static wifi_status_t last_status = WIFI_STATUS_IDLE;
    
    uint32_t current_time = HAL_GetTick();
    
    // Check every 5 seconds - Kiểm tra mỗi 5 giây
    if (current_time - last_check_time >= 5000) {
        last_check_time = current_time;
        
        wifi_status_t current_status = esp8266_get_status();
        
        // Check if status changed - Kiểm tra nếu trạng thái thay đổi
        if (current_status != last_status) {
            LOG_I("WIFI_MONITOR", "Status changed: %d -> %d", last_status, current_status);
            last_status = current_status;
        }
        
        // If connected, check if SSID changed - Nếu đã kết nối, kiểm tra SSID có thay đổi không
        if (current_status == WIFI_STATUS_CONNECTED) {
            char current_ssid[ESP8266_MAX_SSID_LEN];
            if (esp8266_get_current_ssid(current_ssid, sizeof(current_ssid))) {
                if (strcmp(current_ssid, last_ssid) != 0) {
                    LOG_I("WIFI_MONITOR", "Connected to new network: %s", current_ssid);
                    strcpy(last_ssid, current_ssid);
                }
            }
        } else {
            // Clear last SSID if not connected - Xóa SSID cuối nếu không kết nối
            if (strlen(last_ssid) > 0) {
                LOG_I("WIFI_MONITOR", "Disconnected from: %s", last_ssid);
                last_ssid[0] = '\0';
            }
        }
    }
}

/**
 * @brief Example: Simple WiFi status check function for main loop - Ví dụ: Hàm kiểm tra trạng thái WiFi đơn giản cho vòng lặp chính
 */
void wifi_simple_status_check(void)
{
    char ssid[ESP8266_MAX_SSID_LEN];
    
    if (esp8266_get_current_ssid(ssid, sizeof(ssid))) {
        LOG_I("WIFI_STATUS", "📶 Connected to: %s", ssid);
    } else {
        wifi_status_t status = esp8266_get_status();
        switch (status) {
            case WIFI_STATUS_CONNECTING:
                LOG_I("WIFI_STATUS", "🔄 Connecting...");
                break;
            case WIFI_STATUS_DISCONNECTED:
                LOG_I("WIFI_STATUS", "❌ Disconnected");
                break;
            case WIFI_STATUS_ERROR:
                LOG_I("WIFI_STATUS", "⚠️  Error");
                break;
            default:
                LOG_I("WIFI_STATUS", "⏸️  Ready/Idle");
                break;
        }
    }
}
