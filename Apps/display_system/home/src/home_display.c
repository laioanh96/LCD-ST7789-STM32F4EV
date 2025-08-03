#include <ST7789.h>

#include "bitmap.h"
#include "fonts.h"
#include "cmsis_os.h"  // For osDelay

#include "lvgl.h"
#include "home_display.h"

 // Forward declarations for iPhone-like interface
 void create_home_screen(void);
 void create_app_screen(uint8_t app_id);
 void home_button_event_cb(lv_event_t * e);
 void back_button_event_cb(lv_event_t * e);

// Global objects for iPhone interface
static lv_obj_t * home_screen;
static lv_obj_t * app_screen;
static lv_obj_t * current_screen;
static lv_group_t * main_group;

// App names for 9 icons (3x3 grid)
const char* app_names[9] = {
    "Settings", "Clock", "Camera",
    "Music", "Photos", "Games", 
    "Weather", "Maps", "Phone"
};

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
    
    // Create and show app screen
    create_app_screen(app_id);
    lv_scr_load(app_screen);
    current_screen = app_screen;
}

void back_button_event_cb(lv_event_t * e)
{
    // lv_obj_t * btn = lv_event_get_target(e);
    // uint8_t app_id = (uint8_t)(uintptr_t)lv_obj_get_user_data(btn);
    
    // // Clear current group
    // lv_group_remove_all_objs(main_group);
    
    // // Create and show app screen
    // create_app_screen(app_id);
    // lv_scr_load(app_screen);
    // current_screen = app_screen;
}


void create_app_screen(uint8_t app_id)
{
    // // Create app screen
    // app_screen = lv_obj_create(NULL);
    
    // // Set background color based on app
    // lv_obj_set_style_bg_color(app_screen, lv_color_hex(app_colors[app_id]), LV_PART_MAIN);
    // lv_obj_set_style_bg_opa(app_screen, LV_OPA_COVER, LV_PART_MAIN);
    
    // // Create back button
    // lv_obj_t * back_btn = lv_btn_create(app_screen);
    // lv_obj_set_size(back_btn, 80, 35);
    // lv_obj_set_pos(back_btn, 10, 10);  // Top-left corner
    
    // lv_obj_set_style_bg_color(back_btn, lv_color_hex(0x444444), LV_PART_MAIN);
    // lv_obj_set_style_radius(back_btn, 8, LV_PART_MAIN);
    
    // lv_obj_t * back_label = lv_label_create(back_btn);
    // lv_label_set_text(back_label, "< Back");
    // lv_obj_set_style_text_color(back_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // lv_obj_center(back_label);
    
    // lv_obj_add_event_cb(back_btn, back_button_event_cb, LV_EVENT_CLICKED, NULL);
    
    // // Create main content
    // lv_obj_t * content_label = lv_label_create(app_screen);
    
    // // Create content text
    // char content_text[100];
    // sprintf(content_text, "%s App\n\nThis is the %s application.\n\nUse buttons to navigate:\nUP/DOWN: Select\nENTER: Confirm\nLEFT: Back", 
    //         app_names[app_id], app_names[app_id]);
    
    // lv_label_set_text(content_label, content_text);
    // lv_obj_set_style_text_color(content_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    // lv_obj_set_style_text_align(content_label, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    // lv_obj_align(content_label, LV_ALIGN_CENTER, 0, 10);
    
    // // Add spinner animation for some apps
    // if(app_id == 1 || app_id == 6) {  // Clock or Weather
    //     lv_obj_t * spinner = lv_spinner_create(app_screen, 1000, 60);
    //     lv_obj_set_size(spinner, 50, 50);
    //     lv_obj_align(spinner, LV_ALIGN_CENTER, 0, -60);
    //     lv_obj_set_style_arc_color(spinner, lv_color_hex(0xFFFFFF), LV_PART_INDICATOR);
    // }
    
    // // Add objects to navigation group
    // lv_group_add_obj(main_group, back_btn);
}


void create_home_screen(void)
{
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
        
        // Disable shadow temporarily to focus on text quality
        // lv_obj_set_style_shadow_width(btn, 8, LV_PART_MAIN);
        // lv_obj_set_style_shadow_color(btn, lv_color_hex(0x000000), LV_PART_MAIN);
        // lv_obj_set_style_shadow_opa(btn, LV_OPA_30, LV_PART_MAIN);
        
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
        // lv_obj_add_event_cb(btn, home_button_event_cb, LV_EVENT_CLICKED, NULL);
        
        // Add to navigation group
        // lv_group_add_obj(main_group, btn);
    }
}
