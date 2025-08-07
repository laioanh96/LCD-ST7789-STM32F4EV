#include <ST7789.h>

#include "bitmap.h"
#include "fonts.h"
#include "cmsis_os.h"  // For osDelay
#include <math.h>      // For cos, sin functions
#include "stm32f4xx_hal.h"  // For HAL_GPIO functions

#include "lvgl.h"
#include "home_display.h"
#include "input_handler.h"  // For button navigation - Cho điều hướng nút

 // Forward declarations for iPhone-like interface
 void create_home_screen(void);
 void create_app_screen(uint8_t app_id);
 void home_button_event_cb(lv_event_t * e);
 void back_button_event_cb(lv_event_t * e);
 void create_settings_app(lv_obj_t * parent, uint8_t app_id);
 void create_clock_app(lv_obj_t * parent, uint8_t app_id);
 
 // Clock functions - Hàm đồng hồ
 void update_clock_time(void);
 void update_clock_hands(void);
 void clock_timer_callback(lv_timer_t * timer);
 void rotate_hand(lv_obj_t * hand, lv_point_t * points, float angle, int hand_length);
 
 // Public clock control functions - Hàm điều khiển đồng hồ công khai
 void set_clock_time(uint8_t hour, uint8_t minute, uint8_t second);
 void start_clock_timer(void);
 void stop_clock_timer(void);

// Global objects for iPhone interface
static lv_obj_t * home_screen;
static lv_obj_t * app_screen;
static lv_obj_t * current_screen;
lv_group_t * main_group;  // Remove static to make it accessible from input_handler
static lv_indev_t * keyboard_indev;

// App pool - Keep all apps in memory for fast switching
static lv_obj_t * app_screens[9] = {NULL};

// Global clock variables - Biến thời gian toàn cục
static uint8_t current_hour = 12;
static uint8_t current_minute = 0;
static uint8_t current_second = 0;

// Clock hand objects - Đối tượng kim đồng hồ (using lv_line)
static lv_obj_t * hour_hand_obj = NULL;
static lv_obj_t * minute_hand_obj = NULL;
static lv_obj_t * second_hand_obj = NULL;

// Clock timer for updates - Timer để cập nhật đồng hồ
static lv_timer_t * clock_timer = NULL;

// Hand line points - Điểm cho kim đồng hồ
static lv_point_t hour_hand_points[2];
static lv_point_t minute_hand_points[2];
static lv_point_t second_hand_points[2];

// App names for 9 icons (3x3 grid)
const char* app_names[9] = {
    "Settings", "Clock", "Camera",
    "Music", "Photos", "Games", 
    "Weather", "Maps", "Phone"
};
typedef enum
{
  SETTINGS = 0U,
  CLOCK,
  CAMERA,
  MUSIC,
  PHOTOS,
  GAMES,
  WEATHER,
  MAPS,
  PHONE,
  APP_TOTAL
} Application;

// App colors for icons
const uint32_t app_colors[9] = {
    0x808080, 0x00BFFF, 0x32CD32,  // Settings, Clock, Camera
    0xFF69B4, 0xFF8C00, 0x9932CC,  // Music, Photos, Games
    0x87CEEB, 0x228B22, 0x00FF00   // Weather, Maps, Phone
};

// Event callback for home screen icons
void home_button_event_cb(lv_event_t * e)
{
    lv_obj_t * btn = lv_event_get_target(e);
    uint8_t app_id = (uint8_t)(uintptr_t)lv_obj_get_user_data(btn);
    
    // Clear current group
    lv_group_remove_all_objs(main_group);

    if(app_screen != NULL) {
        lv_obj_del(app_screen);
        app_screen = NULL;
    }
    // Create and show app screen
    create_app_screen(app_id);
    
    // Load the app screen
    lv_scr_load(app_screen);
    current_screen = app_screen;
    
}

void back_button_event_cb(lv_event_t * e)
{
    // Stop clock timer when leaving any app - Dừng timer khi thoát app
    stop_clock_timer();
    
    // Clear hand references - Xóa references kim đồng hồ
    hour_hand_obj = NULL;
    minute_hand_obj = NULL;
    second_hand_obj = NULL;
    
    // Clear current group
    lv_group_remove_all_objs(main_group);
    
    // Recreate home screen completely
    if(home_screen != NULL) {
        lv_obj_del(home_screen);
        home_screen = NULL;
    }
    create_home_screen();
    
    // home_screen đã được load trong create_home_screen()
    current_screen = home_screen;
}

void create_settings_app(lv_obj_t * parent, uint8_t app_id)
{
    // Create main content - Simple big text để dễ thấy
    lv_obj_t * content_label = lv_label_create(parent);
    
    // Create content text
    char content_text[200];  // Increase buffer size để đủ chỗ cho string dài
    snprintf(content_text, sizeof(content_text), "%s App\n\nThis is the %s application.\n\nUse buttons to navigate:\nUP/DOWN: Select\nENTER: Confirm\nLEFT: Back", 
            app_names[app_id], app_names[app_id]);
    
    lv_label_set_text(content_label, content_text);
    lv_obj_set_style_text_color(content_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_text_font(content_label, &lv_font_montserrat_12, LV_PART_MAIN);
    lv_obj_set_style_text_align(content_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_obj_align(content_label, LV_ALIGN_CENTER, 0, 10);
    
    // Add spinner animation for some apps
    if(app_id == 1 || app_id == 6) {  // Clock or Weather
        lv_obj_t * spinner = lv_spinner_create(parent, 1000, 60);
        lv_obj_set_size(spinner, 50, 50);
        lv_obj_align(spinner, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_arc_color(spinner, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    }
}

void create_clock_app(lv_obj_t * parent, uint8_t app_id)
{
    //TODO
    // // Create clock container
    // lv_obj_t * clock_container = lv_obj_create(parent);
    // lv_obj_set_size(clock_container, 240, 240);
    // lv_obj_align(clock_container, LV_ALIGN_CENTER, 0, 0);
    // lv_obj_set_style_bg_color(clock_container, lv_color_hex(0x000000), LV_PART_MAIN);
    // lv_obj_set_style_border_color(clock_container, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // lv_obj_set_style_border_width(clock_container, 2, LV_PART_MAIN);
    // lv_obj_set_style_radius(clock_container, 120, LV_PART_MAIN); // Circular
    // lv_obj_set_style_pad_all(clock_container, 0, LV_PART_MAIN);  // Remove all padding
    
    // // Create hour markers (12 dots)
    // for(int i = 0; i < 12; i++) {
    //     lv_obj_t * marker = lv_obj_create(clock_container);
    //     lv_obj_set_size(marker, 4, 4);
    //     lv_obj_set_style_bg_color(marker, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    //     lv_obj_set_style_radius(marker, 2, LV_PART_MAIN);
    //     lv_obj_set_style_border_width(marker, 0, LV_PART_MAIN);
    //     lv_obj_set_style_outline_width(marker, 0, LV_PART_MAIN);  // Remove outline
    //     lv_obj_set_style_pad_all(marker, 0, LV_PART_MAIN);        // Remove padding
        
    //     // Calculate marker position (radius = 75px from center)
    //     float angle = (i * 30 - 120) * 3.14159 / 180; // Convert to radians, -90 to start at 12
    //     int x = 120 + (int)(100 * cos(angle)) - 2; // Center + radius - half marker size
    //     int y = 120 + (int)(100 * sin(angle)) - 2;
    //     lv_obj_set_pos(marker, x, y);
    // }
    
    // // Create center dot - TRONG CÙNG clock_container với kim đồng hồ
    // lv_obj_t * center = lv_obj_create(clock_container);
    // lv_obj_set_size(center, 6, 6);  // Bigger để dễ thấy
    // lv_obj_set_pos(center, 120 - 3, 120 - 3); // Center at (120,120) minus half size
    // lv_obj_set_style_bg_color(center, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red center
    // lv_obj_set_style_radius(center, 3, LV_PART_MAIN);
    // lv_obj_set_style_border_width(center, 0, LV_PART_MAIN);
    // lv_obj_set_style_outline_width(center, 0, LV_PART_MAIN);  // Remove outline
    // lv_obj_set_style_pad_all(center, 0, LV_PART_MAIN);        // Remove padding
    
    // // Initialize hand points (all pointing to 12 o'clock initially)
    // // Hour hand - 45px long
    // hour_hand_points[0].x = 120;  // Center
    // hour_hand_points[0].y = 120;
    // hour_hand_points[1].x = 120;  // Point up initially
    // hour_hand_points[1].y = 75;   // 120 - 45
    
    // // Minute hand - 65px long
    // minute_hand_points[0].x = 120;  // Center
    // minute_hand_points[0].y = 120;
    // minute_hand_points[1].x = 120;  // Point up initially
    // minute_hand_points[1].y = 55;   // 120 - 65
    
    // // Second hand - 80px long
    // second_hand_points[0].x = 120;  // Center
    // second_hand_points[0].y = 120;
    // second_hand_points[1].x = 120;  // Point up initially
    // second_hand_points[1].y = 40;   // 120 - 80
    
    // // Create hour hand line (thick, short, white)
    // hour_hand_obj = lv_line_create(clock_container);
    // lv_line_set_points(hour_hand_obj, hour_hand_points, 2);
    // lv_obj_set_style_line_width(hour_hand_obj, 4, LV_PART_MAIN);
    // lv_obj_set_style_line_color(hour_hand_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // lv_obj_set_style_line_rounded(hour_hand_obj, true, LV_PART_MAIN);
    
    // // Create minute hand line (medium, longer, white)
    // minute_hand_obj = lv_line_create(clock_container);
    // lv_line_set_points(minute_hand_obj, minute_hand_points, 2);
    // lv_obj_set_style_line_width(minute_hand_obj, 3, LV_PART_MAIN);
    // lv_obj_set_style_line_color(minute_hand_obj, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // lv_obj_set_style_line_rounded(minute_hand_obj, true, LV_PART_MAIN);
    
    // // Create second hand line (thin, longest, red)
    // second_hand_obj = lv_line_create(clock_container);
    // lv_line_set_points(second_hand_obj, second_hand_points, 2);
    // lv_obj_set_style_line_width(second_hand_obj, 2, LV_PART_MAIN);
    // lv_obj_set_style_line_color(second_hand_obj, lv_color_hex(0xFF0000), LV_PART_MAIN); // Red
    // lv_obj_set_style_line_rounded(second_hand_obj, true, LV_PART_MAIN);
    
    // // Set initial time and start clock - Đặt thời gian ban đầu và khởi động đồng hồ
    // set_clock_time(12, 0, 0);  // Start at 12:00:00
    // start_clock_timer();       // Start the 1-second timer
    
    // // // Create digital time display
    // // lv_obj_t * time_label = lv_label_create(parent);
    // // lv_label_set_text(time_label, "12:00:00");
    // // lv_obj_set_style_text_color(time_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // // lv_obj_set_style_text_font(time_label, &lv_font_montserrat_12, LV_PART_MAIN);
    // // lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    // // lv_obj_align(time_label, LV_ALIGN_CENTER, 0, -80);
    
    // // TODO: Add timer to update clock hands and digital time
    // // For now, shows static 12:00:00
}

void create_app_screen(uint8_t app_id)
{ 
    // Create new app screen - Make sure parent is NULL for root object
    app_screen = lv_obj_create(NULL);
    
    // Set background color based on app - Bright color để dễ thấy
    lv_obj_set_style_bg_color(app_screen, lv_color_hex(0x1111FF), LV_PART_MAIN);  // Red để debug
    lv_obj_set_style_bg_opa(app_screen, LV_OPA_COVER, LV_PART_MAIN);

    // Create back button
    lv_obj_t * back_btn = lv_btn_create(app_screen);
    lv_obj_set_size(back_btn, 45, 30);  // Bigger button để dễ thấy
    lv_obj_set_pos(back_btn, 180, 10);  // Top-left corner
    
    lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x444444), LV_PART_MAIN);
    lv_obj_set_style_radius(back_btn, 8, LV_PART_MAIN);
    
    lv_obj_t * back_label = lv_label_create(back_btn);
    lv_label_set_text(back_label, "< Back");
    lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_center(back_label);
    
    lv_obj_add_event_cb(back_btn, back_button_event_cb, LV_EVENT_CLICKED, NULL);
    
    lv_obj_set_style_outline_width(back_btn, 3, LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_color(back_btn, lv_color_hex(0xFFFFFF), LV_STATE_FOCUS_KEY);
    lv_obj_set_style_outline_opa(back_btn, LV_OPA_80, LV_STATE_FOCUS_KEY);

    switch (app_id)
    {
    case SETTINGS:
        create_settings_app(app_screen, app_id);
        /* code */
        break;
    case CLOCK:
        create_clock_app(app_screen, app_id);
        /* code */
        break;
    case CAMERA:
        /* code */
        break;
    case MUSIC:
        /* code */
        break;
    case PHOTOS:
        /* code */
        break;
    case GAMES:
        /* code */
        break;
    case WEATHER:
        /* code */
        break;
    case MAPS:
        /* code */
        break;
    case PHONE:
        /* code */
        break;
    default:
        break;
    }
}

// Clock management functions - Hàm quản lý đồng hồ
void update_clock_time(void)
{
    // Increment time - Tăng thời gian
    current_second++;
    if(current_second >= 60) {
        current_second = 0;
        current_minute++;
        if(current_minute >= 60) {
            current_minute = 0;
            current_hour++;
            if(current_hour >= 12) {  // 12-hour format
                current_hour = 0;
            }
        }
    }
}

void rotate_hand(lv_obj_t * hand, lv_point_t * points, float angle, int hand_length)
{
    if(hand == NULL || points == NULL) return;
    
    // Convert angle to radians - Chuyển góc sang radian
    float angle_rad = (angle - 90) * 3.14159 / 180; // -90 để bắt đầu từ 12 giờ
    
    // Clock center coordinates - Tọa độ tâm đồng hồ
    int center_x = 120;
    int center_y = 120;
    
    // Calculate end point of hand - Tính điểm cuối kim
    points[0].x = center_x;  // Start at center
    points[0].y = center_y;
    points[1].x = center_x + (int)(hand_length * cos(angle_rad));  // End point X
    points[1].y = center_y + (int)(hand_length * sin(angle_rad));  // End point Y
    
    // Update the line with new points - Cập nhật line với điểm mới
    lv_line_set_points(hand, points, 2);
}

void update_clock_hands(void)
{
    if(hour_hand_obj == NULL || minute_hand_obj == NULL || second_hand_obj == NULL) return;
    
    // Calculate angles - Tính góc cho từng kim
    float second_angle = current_second * 6.0;      // 360/60 = 6 degrees per second
    float minute_angle = current_minute * 6.0 + current_second * 0.1;  // 6 degrees per minute + smooth movement
    float hour_angle = current_hour * 30.0 + current_minute * 0.5;     // 30 degrees per hour + smooth movement
    
    // Update hand positions with proper lengths - Cập nhật vị trí kim với độ dài đúng
    rotate_hand(second_hand_obj, second_hand_points, second_angle, 80);  // 80px long
    rotate_hand(minute_hand_obj, minute_hand_points, minute_angle, 65);  // 65px long
    rotate_hand(hour_hand_obj, hour_hand_points, hour_angle, 45);        // 45px long
}

void clock_timer_callback(lv_timer_t * timer)
{
    // Update time every second - Cập nhật thời gian mỗi giây
    update_clock_time();
    update_clock_hands();
    
    // Debug: Flash LED to show timer is working - Debug: nhấp nháy LED để thấy timer hoạt động
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);  // Toggle debug LED
}

// Public functions to control clock - Hàm công khai điều khiển đồng hồ
void set_clock_time(uint8_t hour, uint8_t minute, uint8_t second)
{
    current_hour = hour % 12;    // 12-hour format
    current_minute = minute % 60;
    current_second = second % 60;
    update_clock_hands();
}

void start_clock_timer(void)
{
    if(clock_timer == NULL) {
        // Create timer that runs every 1000ms (1 second)
        clock_timer = lv_timer_create(clock_timer_callback, 1000, NULL);
    }
}

void stop_clock_timer(void)
{
    if(clock_timer != NULL) {
        lv_timer_del(clock_timer);
        clock_timer = NULL;
    }
}


void create_home_screen(void)
{    
    // Create navigation group first - Tạo group điều hướng trước
    main_group = lv_group_create();
    lv_group_set_default(main_group);
    
    // Create home screen container
    home_screen = lv_obj_create(NULL);
    
    // Set BLACK background like iPhone
    lv_obj_set_style_bg_color(home_screen, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(home_screen, LV_OPA_COVER, LV_PART_MAIN);
    
    // Load the screen immediately
    lv_scr_load(home_screen);
    
    // Create 3x3 grid of app icons using for loop
    for(int i = 0; i < 9; i++) {
        lv_obj_t * btn = lv_btn_create(home_screen);
        
        // Calculate position in 3x3 grid
        int row = i / 3;
        int col = i % 3;
        
        // Icon size and spacing
        int icon_size = 60;
        int spacing = 20;
        int start_x = (240 - 3 * icon_size - 2 * spacing) / 2;  // Center horizontally
        int start_y = (240 - 3 * icon_size - 2 * spacing) / 2;  // Center vertically
        
        int x = start_x + col * (icon_size + spacing);
        int y = start_y + row * (icon_size + spacing);
        
        lv_obj_set_size(btn, icon_size, icon_size);
        lv_obj_set_pos(btn, x, y);
        
        // Set app color
        lv_obj_set_style_bg_color(btn, lv_color_hex(app_colors[i]), LV_PART_MAIN);
        // lv_obj_set_style_bg_color(btn,  lv_color_hex(0x000000), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN);
        
        // Make it round like iPhone icons - more curved
        lv_obj_set_style_radius(btn, 30, LV_PART_MAIN);  // Half of 60px = perfect circle
        
        // Remove border
        // lv_obj_set_style_border_width(btn, 0, LV_PART_MAIN);
        
        // Add moonlight shadow effect - Hiệu ứng bóng ánh trăng
        lv_obj_set_style_shadow_width(btn, 14, LV_PART_MAIN);           // Bóng rộng 12px
        lv_obj_set_style_shadow_color(btn, lv_color_hex(0xC0C0C0), LV_PART_MAIN);  // Màu bạc ánh trăng
        lv_obj_set_style_shadow_opa(btn, LV_OPA_40, LV_PART_MAIN);      // Độ mờ 40% để tạo hiệu ứng nhẹ nhàng
        lv_obj_set_style_shadow_ofs_x(btn, 0, LV_PART_MAIN);         // Dịch bóng sang phải 2px
        lv_obj_set_style_shadow_ofs_y(btn, 0, LV_PART_MAIN);         // Dịch bóng xuống dưới 2px
        lv_obj_set_style_shadow_spread(btn, 3, LV_PART_MAIN);           // Lan rộng bóng 2px
        
        // Create label for app name with crisp text
        lv_obj_t * label = lv_label_create(btn);
        lv_label_set_text(label, app_names[i]);
        lv_obj_set_style_text_color(label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
        
        // Use font 16 for crisp text on 60px icons
        lv_obj_set_style_text_font(label, &lv_font_montserrat_12, LV_PART_MAIN);
        
        // Optimize text rendering
        lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
        lv_obj_set_style_text_opa(label, LV_OPA_COVER, LV_PART_MAIN);
        
        // Set label size to prevent wrapping unless necessary
        lv_obj_set_width(label, icon_size - 6);  // Slightly more space
        lv_obj_set_height(label, LV_SIZE_CONTENT);
        
        // Center the label precisely
        lv_obj_center(label);
        
        // Store app ID in user data
        lv_obj_set_user_data(btn, (void*)(uintptr_t)i);
        
        // Add click event
        lv_obj_add_event_cb(btn, home_button_event_cb, LV_EVENT_CLICKED, NULL);
        
        // Add to navigation group - Thêm vào group điều hướng
        lv_group_add_obj(main_group, btn);
        
        // Set focus style for selected icon - Đặt style focus cho icon được chọn
        lv_obj_set_style_outline_width(btn, 3, LV_STATE_FOCUS_KEY);
        lv_obj_set_style_outline_color(btn, lv_color_hex(0xFFFFFF), LV_STATE_FOCUS_KEY);
        lv_obj_set_style_outline_opa(btn, LV_OPA_80, LV_STATE_FOCUS_KEY);
    }
    // Set focus to first icon - Đặt focus vào icon đầu tiên
    if (main_group && lv_group_get_obj_count(main_group) > 0) {
        lv_group_focus_next(main_group);  // Enable initial focus - ENABLE!
    }
}
