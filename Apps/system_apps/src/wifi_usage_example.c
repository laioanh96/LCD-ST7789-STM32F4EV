#include "wifi_esp8266.h"
#include "wifi_example.h"
#include "log.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Function prototypes
void log_test_messages(void);

void wifi_test_demo(void)
{
    LOG_I("MAIN", "=== Starting WiFi Test Demo ===");
    
    // Test log system first - Test hệ thống log trước
    LOG_I("LOG_TEST", "Testing log system integrity...");
    log_test_messages();
    
    // Add delay between tests - Thêm delay giữa các test
    HAL_Delay(500);
    
    LOG_I("MAIN", "WiFi Test Demo initialized - proceeding with ESP8266 tests");
    
    // 1. Initialize ESP8266 - Khởi tạo ESP8266
    LOG_I("MAIN", "Step 1: Initializing ESP8266 module...");
    if (!esp8266_init()) {
        LOG_E("MAIN", "ESP8266 initialization failed! Check connections and power supply");
        return;
    }
    LOG_I("MAIN", "ESP8266 initialization completed successfully");
    
    HAL_Delay(500);
    
    // 2. Test basic communication - Test giao tiếp cơ bản
    LOG_I("MAIN", "Step 2: Testing ESP8266 communication...");
    if (esp8266_test()) {
        LOG_I("MAIN", "ESP8266 communication test passed - AT commands working");
    } else {
        LOG_E("MAIN", "ESP8266 communication test failed - check UART connections");
        return;
    }
    
    HAL_Delay(500);
    
    // 3. Scan available networks - Quét mạng có sẵn
    // LOG_I("MAIN", "Step 3: Scanning for available WiFi networks...");
    // wifi_scan_example();
    
    HAL_Delay(1000);
    
    // 4. Try to connect to your WiFi - Thử kết nối WiFi của bạn
    // Replace "YourWiFiName" and "YourPassword" with actual values
    // Thay "YourWiFiName" và "YourPassword" bằng giá trị thực tế
    const char* ssid = "Thu Giang";        // ✅ 2.4GHz network  
    const char* password = "0333338454";   // ✅ Correct password
    
    LOG_I("MAIN", "Step 4: Attempting to connect to WiFi network: %s", ssid);
    wifi_connect_and_check_example(ssid, password);
    
    HAL_Delay(2000);
    
    // 5. Check final connection status - Kiểm tra trạng thái kết nối cuối cùng
    LOG_I("MAIN", "Step 5: Final connection status check...");
    wifi_check_current_connection();
    
    // 6. Start Web Server if connected - Khởi động Web Server nếu đã kết nối
    wifi_status_t status = esp8266_get_status();
    if (status == WIFI_STATUS_CONNECTED) {
        LOG_I("MAIN", "Step 6: Starting Web Server...");
        if (esp8266_start_web_server(80)) {
            LOG_I("MAIN", "🌐 Web Server started successfully!");
            LOG_I("MAIN", "📱 Visit: http://192.168.1.8");
            LOG_I("MAIN", "💡 You can now open a web browser and visit the IP address!");
            LOG_I("MAIN", "🔄 Web server is now running continuously...");
            LOG_I("MAIN", "💻 Try visiting the website multiple times!");
            
            // Run web server continuously - just demo for 60 seconds
            LOG_I("MAIN", "Handling web requests for 60 seconds...");
            for (int i = 0; i < 600; i++) { // 60 seconds
                esp8266_handle_web_requests();
                HAL_Delay(100);
                
                // Print status every 10 seconds
                if (i % 100 == 0) {
                    LOG_I("MAIN", "⏰ Web server running... (%d seconds)", i/10);
                }
            }
            
            LOG_I("MAIN", "Demo period completed. Add web server to main loop for continuous operation.");
        } else {
            LOG_E("MAIN", "Failed to start web server");
        }
    } else {
        LOG_W("MAIN", "WiFi not connected - cannot start web server");
    }
    
    LOG_I("MAIN", "=== WiFi Test Demo Completed ===");
}

/*
 * Add this to your main() function's while(1) loop for continuous web server:
 * Thêm vào vòng lặp while(1) trong hàm main() để chạy web server liên tục:
 * 
 * while (1)
 * {
 *     // Your existing code here...
 *     
 *     // Handle web requests continuously (IMPORTANT!)
 *     esp8266_handle_web_requests();
 *     
 *     // WiFi monitoring (call every few seconds)
 *     static uint32_t last_monitor = 0;
 *     if (HAL_GetTick() - last_monitor > 5000) {
 *         wifi_monitor_example();
 *         last_monitor = HAL_GetTick();
 *     }
 *     
 *     HAL_Delay(50); // Small delay for web responsiveness
 * }
 * 
 * IMPORTANT: Visit http://192.168.1.8 in your browser after WiFi connects!
 * QUAN TRỌNG: Truy cập http://192.168.1.8 trên trình duyệt sau khi WiFi kết nối!
 * 
 * The webpage will show: "Đây là WiFi của OANH"
 * Trang web sẽ hiển thị: "Đây là WiFi của OANH"
 */

/*
 * Functions you can use to check WiFi SSID:
 * Các hàm bạn có thể dùng để check SSID WiFi:
 * 
 * 1. esp8266_get_current_ssid() - Get just the SSID string
 *    Chỉ lấy chuỗi SSID
 * 
 * 2. esp8266_get_connection_info() - Get full connection details
 *    Lấy đầy đủ thông tin kết nối
 * 
 * 3. esp8266_print_status() - Print complete status to log
 *    In trạng thái đầy đủ ra log
 * 
 * 4. esp8266_scan_networks() - Scan and get list of available networks
 *    Quét và lấy danh sách mạng có sẵn
 * 
 * 5. esp8266_print_available_networks() - Print scanned networks
 *    In danh sách mạng đã quét
 */

/*
 * Example usage in your code:
 * Ví dụ sử dụng trong code của bạn:
 * 
 * // Check current SSID
 * char current_ssid[32];
 * if (esp8266_get_current_ssid(current_ssid, sizeof(current_ssid))) {
 *     printf("Connected to: %s\n", current_ssid);
 * } else {
 *     printf("Not connected\n");
 * }
 * 
 * // Get full info
 * wifi_connection_t info;
 * if (esp8266_get_connection_info(&info)) {
 *     printf("SSID: %s\n", info.ssid);
 *     printf("IP: %s\n", info.ip);
 *     printf("Signal: %d dBm\n", info.rssi);
 * }
 * 
 * // Scan networks
 * wifi_network_t networks[10];
 * uint8_t count;
 * if (esp8266_scan_networks(networks, &count, 10)) {
 *     for (int i = 0; i < count; i++) {
 *         printf("Network %d: %s (%d dBm)\n", i+1, networks[i].ssid, networks[i].rssi);
 *     }
 * }
 */
