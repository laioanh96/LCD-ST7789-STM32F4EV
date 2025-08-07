#ifndef HOME_DISPLAY_H
#define HOME_DISPLAY_H

#include "lvgl.h"

// External group declaration for input handler access
extern lv_group_t * main_group;

// Function prototypes - Khai báo hàm
void create_home_screen(void);
void create_app_screen(uint8_t app_id);

#endif //HOME_DISPLAY_H
