/**
 * @file lv_conf.h
 * Configuration file for v8.3.11 - File cấu hình cho LVGL v8.3.11
 */

/*
 * Copy this file as `lv_conf.h` - Sao chép file này thành `lv_conf.h`
 * 1. simply next to the `lvgl` folder - đơn giản đặt cạnh thư mục `lvgl`
 * 2. or any other places and - hoặc bất kỳ nơi nào khác và
 *    - define `LV_CONF_INCLUDE_SIMPLE` - định nghĩa `LV_CONF_INCLUDE_SIMPLE`
 *    - add the path as include path - thêm đường dẫn như include path
 */

/* clang-format off */
#if 1 /*Set it to "1" to enable content - Đặt thành "1" để kích hoạt nội dung*/

#ifndef LV_CONF_H
#define LV_CONF_H

#include <stdint.h>

/*====================
   COLOR SETTINGS - CÀI ĐẶT MÀU SẮC
 *====================*/

/*Color depth: 1 (1 byte per pixel), 8 (RGB332), 16 (RGB565), 32 (ARGB8888)*/
/*Độ sâu màu: 1 (1 byte/pixel), 8 (RGB332), 16 (RGB565), 32 (ARGB8888)*/
#define LV_COLOR_DEPTH 16

/*Swap the 2 bytes of RGB565 color. Useful if the display has an 8-bit interface (e.g. SPI)*/
/*Hoán đổi 2 byte của màu RGB565. Hữu ích nếu màn hình có giao diện 8-bit (ví dụ: SPI)*/
#define LV_COLOR_16_SWAP 1

/*Enable features to draw on transparent background.*/
/*Kích hoạt tính năng vẽ trên nền trong suốt.*/
/*It's required if opa, and transform_* style properties are used.*/
/*Cần thiết nếu sử dụng thuộc tính opa và transform_* style.*/
/*Can be also used if the UI is above another layer (e.g. an OSD menu or video player)*/
/*Cũng có thể dùng nếu UI nằm trên lớp khác (ví dụ: menu OSD hoặc video player)*/
#define LV_COLOR_SCREEN_TRANSP 0

/* Adjust color mix functions rounding. GPUs might calculate color mix (blending) differently.*/
/* Điều chỉnh làm tròn hàm trộn màu. GPU có thể tính toán trộn màu khác nhau.*/
/* 0: round down, 64: round up from x.75, 128: round up from half, 192: round up from x.25, 254: round up */
/* 0: làm tròn xuống, 64: làm tròn lên từ x.75, 128: làm tròn lên từ nửa, 192: làm tròn lên từ x.25, 254: làm tròn lên */
#define LV_COLOR_MIX_ROUND_OFS 0

/*Images pixels with this color will not be drawn if they are chroma keyed)*/
/*Pixel hình ảnh có màu này sẽ không được vẽ nếu chúng được chroma key*/
#define LV_COLOR_CHROMA_KEY lv_color_hex(0x00ff00)

/*=========================
   MEMORY SETTINGS - CÀI ĐẶT BỘ NHỚ
 *=========================*/

/*1: use custom malloc/free, 0: use the built-in `lv_mem_alloc()` and `lv_mem_free()`*/
/*1: dùng malloc/free tùy chỉnh, 0: dùng `lv_mem_alloc()` và `lv_mem_free()` có sẵn*/
#define LV_MEM_CUSTOM 0
#if LV_MEM_CUSTOM == 0
    /*Size of the memory available for `lv_mem_alloc()` in bytes (>= 2kB)*/
    /*Kích thước bộ nhớ có sẵn cho `lv_mem_alloc()` tính bằng byte (>= 2kB)*/
    #define LV_MEM_SIZE (16U * 1024U)          /*[bytes]*/

    /*Set an address for the memory pool instead of allocating it as a normal array. Can be in external SRAM too.*/
    /*Đặt địa chỉ cho pool bộ nhớ thay vì cấp phát như mảng bình thường. Có thể ở SRAM ngoài.*/
    #define LV_MEM_ADR 0     /*0: unused - không dùng*/
#else       /*LV_MEM_CUSTOM*/
    #define LV_MEM_CUSTOM_INCLUDE <stdlib.h>   /*Header for the dynamic memory function - Header cho hàm bộ nhớ động*/
    #define LV_MEM_CUSTOM_ALLOC   malloc
    #define LV_MEM_CUSTOM_FREE    free
    #define LV_MEM_CUSTOM_REALLOC realloc
#endif     /*LV_MEM_CUSTOM*/

/*Number of the intermediate memory buffer used during rendering and other internal processing.*/
/*Số lượng buffer bộ nhớ trung gian dùng trong quá trình render và xử lý nội bộ khác.*/
/*You will see an error log message if there wasn't enough buffers. */
/*Bạn sẽ thấy thông báo lỗi nếu không đủ buffer.*/
#define LV_MEM_BUF_MAX_NUM 16

/*Use the standard `memcpy` and `memset` instead of LVGL's own functions. (Might or might not be faster).*/
/*Dùng `memcpy` và `memset` chuẩn thay vì hàm riêng của LVGL. (Có thể nhanh hơn hoặc không).*/
#define LV_MEMCPY_MEMSET_STD 0

/*====================
   HAL SETTINGS - CÀI ĐẶT HAL
 *====================*/

/*Default display refresh period. LVG will redraw changed areas with this period time*/
/*Chu kỳ làm mới màn hình mặc định. LVGL sẽ vẽ lại vùng thay đổi theo chu kỳ này*/
#define LV_DISP_DEF_REFR_PERIOD 30      /*[ms]*/

/*Input device read period in milliseconds*/
/*Chu kỳ đọc thiết bị đầu vào tính bằng millisecond*/
#define LV_INDEV_DEF_READ_PERIOD 30     /*[ms]*/

/*Use a custom tick source that tells the elapsed time in milliseconds.*/
/*Dùng nguồn tick tùy chỉnh cho biết thời gian đã trôi qua tính bằng millisecond.*/
/*It removes the need to manually update the tick with `lv_tick_inc()`) */
/*Điều này loại bỏ việc phải cập nhật tick thủ công bằng `lv_tick_inc()`*/
#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
    #define LV_TICK_CUSTOM_INCLUDE "main.h"         /*Header for the system time function - Header cho hàm thời gian hệ thống*/
    #define LV_TICK_CUSTOM_SYS_TIME_EXPR (HAL_GetTick())    /*Expression evaluating to current system time in ms - Biểu thức tính thời gian hệ thống hiện tại (ms)*/
#endif   /*LV_TICK_CUSTOM*/

/*Default Dot Per Inch. Used to initialize default sizes such as widgets sized, style paddings.*/
/*Dot Per Inch mặc định. Dùng để khởi tạo kích thước mặc định như kích thước widget, padding style.*/
/*(Not so important, you can adjust it to modify default sizes and spaces)*/
/*(Không quan trọng lắm, bạn có thể điều chỉnh để thay đổi kích thước và khoảng cách mặc định)*/
#define LV_DPI_DEF 130     /*[px/inch]*/

/*=======================
 * FEATURE CONFIGURATION - CẤU HÌNH TÍNH NĂNG
 *=======================*/

/*-------------
 * Drawing - VẼ
 *-----------*/

/*Enable complex draw engine.*/
/*Kích hoạt engine vẽ phức tạp.*/
/*Required to draw shadow, gradient, rounded corners, circles, arc, skew, image transformations or any masks*/
/*Cần thiết để vẽ bóng, gradient, góc bo tròn, hình tròn, cung, nghiêng, biến đổi hình ảnh hoặc mask*/
#define LV_DRAW_COMPLEX 1
#if LV_DRAW_COMPLEX != 0

    /*Allow buffering some shadow calculation.*/
    /*Cho phép buffer một số tính toán bóng.*/
    /*LV_DRAW_COMPLEX should be 1 also to enable shadow drawing*/
    /*LV_DRAW_COMPLEX cũng phải là 1 để kích hoạt vẽ bóng*/
    #define LV_DRAW_SW_SHADOW_CACHE_SIZE 4

    /*Set number of maximally cached circle data.*/
    /*Đặt số lượng tối đa dữ liệu hình tròn được cache.*/
    /*The circumference of 1/4 circle are saved for anti-aliasing*/
    /*Chu vi của 1/4 hình tròn được lưu để chống răng cưa*/
    /*radius * 4 bytes are used per circle (the most often used sizes are saved)*/
    /*radius * 4 byte được dùng cho mỗi hình tròn (các kích thước thường dùng nhất được lưu)*/
    /*0: to disable caching*/
    /*0: để vô hiệu hóa cache*/
    #define LV_DRAW_SW_CIRCLE_CACHE_SIZE 4

#endif /*LV_DRAW_COMPLEX*/

/*Default image cache size. Image caching keeps the images opened.*/
/*Kích thước cache hình ảnh mặc định. Cache hình ảnh giữ các hình ảnh đang mở.*/
/*If only the built-in image formats are used there is no real advantage of caching. (I.e. if no new image decoder is added)*/
/*Nếu chỉ dùng các định dạng hình ảnh có sẵn thì không có lợi ích thực sự từ cache. (Tức là nếu không thêm decoder hình ảnh mới)*/
/*With complex image decoders (e.g. PNG or JPG) caching can save the continuous open/decode of images.*/
/*Với các decoder hình ảnh phức tạp (ví dụ: PNG hoặc JPG) cache có thể tiết kiệm việc mở/decode hình ảnh liên tục.*/
/*However the opened images might consume additional RAM.*/
/*Tuy nhiên các hình ảnh đang mở có thể tiêu thụ thêm RAM.*/
/*0: to disable caching*/
/*0: để vô hiệu hóa cache*/
#define LV_IMG_CACHE_DEF_SIZE 0

/*Maximum buffer size to allocate for rotation. Only used if software rotation is used in the display driver.*/
/*Kích thước buffer tối đa để cấp phát cho xoay. Chỉ dùng nếu xoay bằng phần mềm được sử dụng trong driver màn hình.*/
#define LV_DISP_ROT_MAX_BUF (10*1024)

/*-------------
 * GPU - BỘ XỬ LÝ ĐỒ HỌA
 *-----------*/

/*Use STM32's DMA2D (aka Chrom Art) GPU*/
/*Sử dụng GPU DMA2D (hay Chrom Art) của STM32*/
#define LV_USE_GPU_STM32_DMA2D 0
#if LV_USE_GPU_STM32_DMA2D
    /*Must be defined to include path of CMSIS header of target processor*/
    /*Phải được định nghĩa để include đường dẫn header CMSIS của bộ xử lý đích*/
    /*e.g. "stm32f769xx.h" or "stm32f429xx.h"*/
    /*ví dụ: "stm32f769xx.h" hoặc "stm32f429xx.h"*/
    #define LV_GPU_DMA2D_CMSIS_INCLUDE
#endif

/*Use NXP's PXP GPU iMX RTxxx platforms*/
/*Sử dụng GPU PXP của NXP cho nền tảng iMX RTxxx*/
#define LV_USE_GPU_NXP_PXP 0
#if LV_USE_GPU_NXP_PXP
    /*1: Add default bare metal and FreeRTOS interrupt handling routines for PXP (lv_gpu_nxp_pxp_osa.c)*/
    /*1: Thêm routine xử lý ngắt bare metal và FreeRTOS mặc định cho PXP (lv_gpu_nxp_pxp_osa.c)*/
    /*   and call lv_gpu_nxp_pxp_init() automatically during lv_init(). Note that symbol SDK_OS_FREE_RTOS*/
    /*   và gọi lv_gpu_nxp_pxp_init() tự động trong lv_init(). Lưu ý rằng symbol SDK_OS_FREE_RTOS*/
    /*   has to be defined in order to use FreeRTOS OSA, otherwise bare-metal implementation is selected.*/
    /*   phải được định nghĩa để sử dụng FreeRTOS OSA, nếu không thì implementation bare-metal được chọn.*/
    /*0: User will call lv_gpu_nxp_pxp_init() manually before lv_init()*/
    /*0: User sẽ gọi lv_gpu_nxp_pxp_init() thủ công trước lv_init()*/
    #define LV_USE_GPU_NXP_PXP_AUTO_INIT 0
#endif

/*Use NXP's VG-Lite GPU iMX RTxxx platforms*/
/*Sử dụng GPU VG-Lite của NXP cho nền tảng iMX RTxxx*/
#define LV_USE_GPU_NXP_VG_LITE 0

/*Use SDL renderer API*/
/*Sử dụng API renderer SDL*/
#define LV_USE_GPU_SDL 0
#if LV_USE_GPU_SDL
    #define LV_GPU_SDL_INCLUDE_PATH <SDL2/SDL.h>
    /*Texture cache size, 8MB by default*/
    /*Kích thước cache texture, mặc định 8MB*/
    #define LV_GPU_SDL_LRU_SIZE (1024 * 1024 * 8)
    /*Custom blend mode for mask drawing, disable if you need to link with older SDL2 lib*/
    /*Chế độ blend tùy chỉnh để vẽ mask, vô hiệu hóa nếu cần link với thư viện SDL2 cũ hơn*/
    #define LV_GPU_SDL_CUSTOM_BLEND_MODE (SDL_VERSION_ATLEAST(2, 0, 6))
#endif

/*-------------
 * Logging - GHI LOG
 *-----------*/

/*Enable the log module*/
/*Kích hoạt module log*/
#define LV_USE_LOG 0
#if LV_USE_LOG

    /*How important log should be added:*/
    /*Mức độ quan trọng của log cần được thêm:*/
    /*LV_LOG_LEVEL_TRACE       A lot of logs to give detailed information*/
    /*LV_LOG_LEVEL_TRACE       Rất nhiều log để cung cấp thông tin chi tiết*/
    /*LV_LOG_LEVEL_INFO        Log important events*/
    /*LV_LOG_LEVEL_INFO        Log các sự kiện quan trọng*/
    /*LV_LOG_LEVEL_WARN        Log if something unwanted happened but didn't cause a problem*/
    /*LV_LOG_LEVEL_WARN        Log nếu điều gì đó không mong muốn xảy ra nhưng không gây vấn đề*/
    /*LV_LOG_LEVEL_ERROR       Only critical issue, when the system may fail*/
    /*LV_LOG_LEVEL_ERROR       Chỉ vấn đề nghiêm trọng, khi hệ thống có thể lỗi*/
    /*LV_LOG_LEVEL_USER        Only logs added by the user*/
    /*LV_LOG_LEVEL_USER        Chỉ các log được thêm bởi user*/
    /*LV_LOG_LEVEL_NONE        Do not log anything*/
    /*LV_LOG_LEVEL_NONE        Không log gì cả*/
    #define LV_LOG_LEVEL LV_LOG_LEVEL_WARN

    /*1: Print the log with 'printf';*/
    /*1: In log bằng 'printf';*/
    /*0: User need to register a callback with `lv_log_register_print_cb()`*/
    /*0: User cần đăng ký callback với `lv_log_register_print_cb()`*/
    #define LV_LOG_PRINTF 0

    /*Enable/disable LV_LOG_TRACE in modules that produces a huge number of logs*/
    /*Bật/tắt LV_LOG_TRACE trong các module tạo ra rất nhiều log*/
    #define LV_LOG_TRACE_MEM        1
    #define LV_LOG_TRACE_TIMER      1
    #define LV_LOG_TRACE_INDEV      1
    #define LV_LOG_TRACE_DISP_REFR  1
    #define LV_LOG_TRACE_EVENT      1
    #define LV_LOG_TRACE_OBJ_CREATE 1
    #define LV_LOG_TRACE_LAYOUT     1
    #define LV_LOG_TRACE_ANIM       1

#endif  /*LV_USE_LOG*/

/*-------------
 * Asserts - KHẲNG ĐỊNH
 *-----------*/

/*Enable asserts if an operation is failed or an invalid data is found.*/
/*Kích hoạt assert nếu một thao tác thất bại hoặc tìm thấy dữ liệu không hợp lệ.*/
/*If LV_USE_LOG is enabled an error message will be printed on failure*/
/*Nếu LV_USE_LOG được bật, thông báo lỗi sẽ được in khi thất bại*/
#define LV_USE_ASSERT_NULL          1   /*Check if the parameter is NULL. (Very fast, recommended)*/
                                        /*Kiểm tra nếu tham số là NULL. (Rất nhanh, khuyên dùng)*/
#define LV_USE_ASSERT_MALLOC        1   /*Checks is the memory is successfully allocated or no. (Very fast, recommended)*/
                                        /*Kiểm tra bộ nhớ có được cấp phát thành công hay không. (Rất nhanh, khuyên dùng)*/
#define LV_USE_ASSERT_STYLE         0   /*Check if the used styles are properly initialized. (Very fast, recommended)*/
                                        /*Kiểm tra các style được sử dụng có được khởi tạo đúng cách hay không. (Rất nhanh, khuyên dùng)*/
#define LV_USE_ASSERT_MEM_INTEGRITY 0   /*Check the integrity of `lv_mem` after critical operations. (Slow)*/
                                        /*Kiểm tra tính toàn vẹn của `lv_mem` sau các thao tác quan trọng. (Chậm)*/
#define LV_USE_ASSERT_OBJ           0   /*Check the object's type and existence (e.g. not deleted). (Slow)*/
                                        /*Kiểm tra loại và sự tồn tại của object (ví dụ: chưa bị xóa). (Chậm)*/

/*Add a custom handler when assert happens e.g. to restart the MCU*/
/*Thêm handler tùy chỉnh khi assert xảy ra ví dụ để khởi động lại MCU*/
#define LV_ASSERT_HANDLER_INCLUDE <stdint.h>
#define LV_ASSERT_HANDLER while(1);   /*Halt by default - Dừng theo mặc định*/

/*-------------
 * Others - KHÁC
 *-----------*/

/*1: Show CPU usage and FPS count*/
/*1: Hiển thị mức sử dụng CPU và số FPS*/
#define LV_USE_PERF_MONITOR 0
#if LV_USE_PERF_MONITOR
    #define LV_USE_PERF_MONITOR_POS LV_ALIGN_TOP_LEFT
#endif

/*1: Show the used memory and the memory fragmentation*/
/*1: Hiển thị bộ nhớ đã sử dụng và phân mảnh bộ nhớ*/
/* Requires LV_MEM_CUSTOM = 0*/
/* Yêu cầu LV_MEM_CUSTOM = 0*/
#define LV_USE_MEM_MONITOR 0
#if LV_USE_MEM_MONITOR
    #define LV_USE_MEM_MONITOR_POS LV_ALIGN_TOP_RIGHT
#endif

/*1: Draw random colored rectangles over the redrawn areas*/
/*1: Vẽ các hình chữ nhật màu ngẫu nhiên trên các vùng được vẽ lại*/
#define LV_USE_REFR_DEBUG 0

/*Change the built in (v)snprintf functions*/
/*Thay đổi các hàm (v)snprintf có sẵn*/
#define LV_SPRINTF_CUSTOM 0
#if LV_SPRINTF_CUSTOM
    #define LV_SPRINTF_INCLUDE <stdio.h>
    #define lv_snprintf  snprintf
    #define lv_vsnprintf vsnprintf
#else   /*LV_SPRINTF_CUSTOM*/
    #define LV_SPRINTF_USE_FLOAT 0
#endif  /*LV_SPRINTF_CUSTOM*/

#define LV_USE_USER_DATA 1

/*Garbage Collector settings*/
/*Cài đặt Garbage Collector*/
/*Used if lvgl is bound to higher level language and the memory is managed by that language*/
/*Được sử dụng nếu lvgl được liên kết với ngôn ngữ cấp cao hơn và bộ nhớ được quản lý bởi ngôn ngữ đó*/
#define LV_ENABLE_GC 0
#if LV_ENABLE_GC != 0
    #define LV_GC_INCLUDE "gc.h"                           /*Include Garbage Collector related things - Include các thứ liên quan đến Garbage Collector*/
#endif /*LV_ENABLE_GC*/

/*=====================
 *  COMPILER SETTINGS - CÀI ĐẶT COMPILER
 *====================*/

/*For big endian systems set to 1*/
/*Đối với hệ thống big endian đặt thành 1*/
#define LV_BIG_ENDIAN_SYSTEM 0

/*Define a custom attribute to `lv_tick_inc` function*/
/*Định nghĩa thuộc tính tùy chỉnh cho hàm `lv_tick_inc`*/
#define LV_ATTRIBUTE_TICK_INC

/*Define a custom attribute to `lv_timer_handler` function*/
/*Định nghĩa thuộc tính tùy chỉnh cho hàm `lv_timer_handler`*/
#define LV_ATTRIBUTE_TIMER_HANDLER

/*Define a custom attribute to `lv_disp_flush_ready` function*/
/*Định nghĩa thuộc tính tùy chỉnh cho hàm `lv_disp_flush_ready`*/
#define LV_ATTRIBUTE_FLUSH_READY

/*Required alignment size for buffers*/
/*Kích thước căn chỉnh yêu cầu cho buffer*/
#define LV_ATTRIBUTE_MEM_ALIGN_SIZE 1

/*Will be added where memories needs to be aligned (with -Os data might not be aligned to boundary by default).*/
/*Sẽ được thêm nơi bộ nhớ cần được căn chỉnh (với -Os dữ liệu có thể không được căn chỉnh theo boundary theo mặc định).*/
/* E.g. __attribute__((aligned(4)))*/
/* Ví dụ: __attribute__((aligned(4)))*/
#define LV_ATTRIBUTE_MEM_ALIGN

/*Attribute to mark large constant arrays for example font's bitmaps*/
/*Thuộc tính để đánh dấu các mảng hằng số lớn ví dụ như bitmap của font*/
#define LV_ATTRIBUTE_LARGE_CONST

/*Complier prefix for a big array declaration in RAM*/
/*Tiền tố compiler cho khai báo mảng lớn trong RAM*/
#define LV_ATTRIBUTE_LARGE_RAM_ARRAY

/*Place performance critical functions into a faster memory (e.g RAM)*/
/*Đặt các hàm quan trọng về hiệu suất vào bộ nhớ nhanh hơn (ví dụ: RAM)*/
#define LV_ATTRIBUTE_FAST_MEM

/*Prefix variables that are used in GPU accelerated operations, often these need to be placed in RAM sections that are DMA accessible*/
/*Tiền tố các biến được sử dụng trong các thao tác tăng tốc GPU, thường những biến này cần được đặt trong các phần RAM có thể truy cập DMA*/
#define LV_ATTRIBUTE_DMA

/*Export integer constant to binding. This macro is used with constants in the form of LV_<CONST> that*/
/*Xuất hằng số nguyên để binding. Macro này được sử dụng với các hằng số có dạng LV_<CONST> mà*/
/*should also appear on LVGL binding API such as Micropython or C++.*/
/*cũng nên xuất hiện trên API binding LVGL như Micropython hoặc C++.*/
#define LV_EXPORT_CONST_INT(int_value) struct _silence_gcc_warning /*The default value just prevents GCC warning - Giá trị mặc định chỉ để ngăn cảnh báo GCC*/

/*Extend the default -32k..32k coordinate range to -4M..4M by using int32_t for coordinates instead of int16_t*/
/*Mở rộng phạm vi tọa độ mặc định -32k..32k thành -4M..4M bằng cách sử dụng int32_t cho tọa độ thay vì int16_t*/
#define LV_USE_LARGE_COORD 0

/*==================
 *   FONT USAGE - SỬ DỤNG FONT
 *=================*/

/*Montserrat fonts with various styles and sizes.*/
/*Font Montserrat với nhiều style và kích thước khác nhau.*/
/*The fonts are compressed by https://github.com/lvgl/lv_font_compress*/
/*Các font được nén bởi https://github.com/lvgl/lv_font_compress*/
/*Set LV_FONT_DEFAULT in all sub-theme to point the font used by default. */
/*Đặt LV_FONT_DEFAULT trong tất cả sub-theme để trỏ đến font được sử dụng mặc định.*/
#define LV_FONT_MONTSERRAT_8  0
#define LV_FONT_MONTSERRAT_10 1
#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_16 1
#define LV_FONT_MONTSERRAT_18 0
#define LV_FONT_MONTSERRAT_20 0
#define LV_FONT_MONTSERRAT_22 0
#define LV_FONT_MONTSERRAT_24 0
#define LV_FONT_MONTSERRAT_26 0
#define LV_FONT_MONTSERRAT_28 0
#define LV_FONT_MONTSERRAT_30 0
#define LV_FONT_MONTSERRAT_32 0
#define LV_FONT_MONTSERRAT_34 0
#define LV_FONT_MONTSERRAT_36 0
#define LV_FONT_MONTSERRAT_38 0
#define LV_FONT_MONTSERRAT_40 0
#define LV_FONT_MONTSERRAT_42 0
#define LV_FONT_MONTSERRAT_44 0
#define LV_FONT_MONTSERRAT_46 0
#define LV_FONT_MONTSERRAT_48 0

/*Demonstrate special features*/
/*Thể hiện các tính năng đặc biệt*/
#define LV_FONT_MONTSERRAT_12_SUBPX      0
#define LV_FONT_MONTSERRAT_28_COMPRESSED 0  /*bpp = 3*/
#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW 0  /*Hebrew, Arabic, Persian letters and all their forms - Chữ cái Hebrew, Arabic, Persian và tất cả các dạng của chúng*/
#define LV_FONT_SIMSUN_16_CJK            0  /*1000 most common CJK radicals - 1000 bộ thủ CJK phổ biến nhất*/

/*Pixel perfect monospace fonts*/
/*Font monospace hoàn hảo từng pixel*/
#define LV_FONT_UNSCII_8  0
#define LV_FONT_UNSCII_16 0

/*Optionally declare custom fonts here.*/
/*Tùy chọn khai báo font tùy chỉnh ở đây.*/
/*You can use these fonts as default font too and they will be available globally.*/
/*Bạn cũng có thể sử dụng các font này làm font mặc định và chúng sẽ có sẵn toàn cục.*/
/*E.g. #define LV_FONT_CUSTOM_DECLARE   LV_FONT_DECLARE(my_font_1) LV_FONT_DECLARE(my_font_2)*/
/*Ví dụ: #define LV_FONT_CUSTOM_DECLARE   LV_FONT_DECLARE(my_font_1) LV_FONT_DECLARE(my_font_2)*/
#define LV_FONT_CUSTOM_DECLARE

/*Always set a default font*/
/*Luôn đặt một font mặc định*/
#define LV_FONT_DEFAULT &lv_font_montserrat_10

/*Enable handling large font and/or fonts with a lot of characters.
 *The limit depends on the font size, font face and bpp.
 *Compiler error will be triggered if a font needs it.*/
#define LV_FONT_FMT_TXT_LARGE 0

/*Enables/disables support for compressed fonts.*/
#define LV_USE_FONT_COMPRESSED 0

/*Enable subpixel rendering*/
#define LV_USE_FONT_SUBPX 0
#if LV_USE_FONT_SUBPX
    /*Set the pixel order of the display. Physical order of RGB channels. Doesn't matter with "normal" fonts.*/
    #define LV_FONT_SUBPX_BGR 0  /*0: RGB; 1:BGR order*/
#endif

/*Enable drawing placeholders when glyph dsc is not found*/
#define LV_USE_FONT_PLACEHOLDER 1

/*=================
 *  TEXT SETTINGS - CÀI ĐẶT VĂN BẢN
 *=================*/

/**
 * Select a character encoding for strings.
 * Your IDE or editor should have the same character encoding
 * Chọn một mã hóa ký tự cho chuỗi.
 * IDE hoặc trình soạn thảo của bạn nên có cùng mã hóa ký tự
 * - LV_TXT_ENC_UTF8
 * - LV_TXT_ENC_ASCII
 */
#define LV_TXT_ENC LV_TXT_ENC_UTF8

/*Can break (wrap) texts on these chars*/
/*Có thể ngắt (xuống dòng) văn bản tại các ký tự này*/
#define LV_TXT_BREAK_CHARS " ,.;:-_"

/*If a word is at least this long, will break wherever "prettiest"
 *To disable, set to a value <= 0*/
/*Nếu một từ dài ít nhất như thế này, sẽ ngắt ở vị trí "đẹp nhất"
 *Để tắt, đặt giá trị <= 0*/
#define LV_TXT_LINE_BREAK_LONG_LEN 0

/*Minimum number of characters in a long word to put on a line before a break.
 *Depends on LV_TXT_LINE_BREAK_LONG_LEN.*/
/*Số ký tự tối thiểu trong một từ dài để đặt trên một dòng trước khi ngắt.
 *Phụ thuộc vào LV_TXT_LINE_BREAK_LONG_LEN.*/
#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN 3

/*Minimum number of characters in a long word to put on a line after a break.
 *Depends on LV_TXT_LINE_BREAK_LONG_LEN.*/
/*Số ký tự tối thiểu trong một từ dài để đặt trên một dòng sau khi ngắt.
 *Phụ thuộc vào LV_TXT_LINE_BREAK_LONG_LEN.*/
#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN 3

/*The control character to use for signalling text recoloring.*/
/*Ký tự điều khiển sử dụng để báo hiệu đổi màu văn bản.*/
#define LV_TXT_COLOR_CMD "#"

/*Support bidirectional texts.
 *Allows mixing Left-to-Right and Right-to-Left texts.
 *The direction will be processed according to the Unicode Bidirectional Algorithm:
 *https://www.unicode.org/reports/tr9/*/
#define LV_USE_BIDI 0
#if LV_USE_BIDI
    /*Set the default direction. Supported values:
    *`LV_BASE_DIR_LTR` Left-to-Right
    *`LV_BASE_DIR_RTL` Right-to-Left
    *`LV_BASE_DIR_AUTO` detect texts base direction*/
    #define LV_BIDI_BASE_DIR_DEF LV_BASE_DIR_AUTO
#endif

/*Enable Arabic/Persian processing
 *In these languages characters should be replaced with an other form based on their position in the text*/
#define LV_USE_ARABIC_PERSIAN_CHARS 0

/*==================
 *  WIDGET USAGE - SỬ DỤNG WIDGET
 *================*/

/*Documentation of the widgets: https://docs.lvgl.io/latest/en/html/widgets/index.html*/
/*Tài liệu về các widget: https://docs.lvgl.io/latest/en/html/widgets/index.html*/

#define LV_USE_ARC        1
#define LV_USE_ANIMIMG    0
#define LV_USE_BAR        0
#define LV_USE_BTN        1
#define LV_USE_BTNMATRIX  0
#define LV_USE_CANVAS     0
#define LV_USE_CHECKBOX   0
#define LV_USE_DROPDOWN   0   /*Requires: lv_label - Yêu cầu: lv_label*/
#define LV_USE_IMG        0   /*Requires: lv_label - Yêu cầu: lv_label*/
#define LV_USE_LABEL      1
#if LV_USE_LABEL
    #define LV_LABEL_TEXT_SELECTION 0 /*Enable selecting text of the label - Bật chọn văn bản của label*/
    #define LV_LABEL_LONG_TXT_HINT 0  /*Store some extra info in labels to speed up drawing of very long texts - Lưu thông tin thêm trong label để tăng tốc vẽ văn bản rất dài*/
#endif
#define LV_USE_LINE       0
#define LV_USE_ROLLER     0   /*Requires: lv_label - Yêu cầu: lv_label*/
#if LV_USE_ROLLER
    #define LV_ROLLER_INF_PAGES 7 /*Number of extra "pages" when the roller is infinite - Số "trang" thêm khi roller vô hạn*/
#endif
#define LV_USE_SLIDER     0   /*Requires: lv_bar - Yêu cầu: lv_bar*/
#define LV_USE_SWITCH     0
#define LV_USE_TEXTAREA   0   /*Requires: lv_label - Yêu cầu: lv_label*/
#if LV_USE_TEXTAREA != 0
    #define LV_TEXTAREA_DEF_PWD_SHOW_TIME 1500    /*ms*/
#endif

#define LV_USE_TABLE      0

/*==================
 * EXTRA COMPONENTS - THÀNH PHẦN THÊM
 *=================*/

/*-----------
 * Widgets - Widget
 *----------*/
#define LV_USE_CALENDAR   0
#if LV_USE_CALENDAR
    #define LV_CALENDAR_WEEK_STARTS_MONDAY 0
    #if LV_CALENDAR_WEEK_STARTS_MONDAY
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Mo", "Tu", "We", "Th", "Fr", "Sa", "Su"}
    #else
        #define LV_CALENDAR_DEFAULT_DAY_NAMES {"Su", "Mo", "Tu", "We", "Th", "Fr", "Sa"}
    #endif
    #define LV_CALENDAR_DEFAULT_MONTH_NAMES {"January", "February", "March",  "April", "May",  "June", "July", "August", "September", "October", "November", "December"}
    #define LV_USE_CALENDAR_HEADER_ARROW 1
    #define LV_USE_CALENDAR_HEADER_DROPDOWN 1
#endif  /*LV_USE_CALENDAR*/

#define LV_USE_CHART      0
#if LV_USE_CHART
    #define LV_CHART_AXIS_TICK_LABEL_MAX_LEN 256
#endif

#define LV_USE_COLORWHEEL 0

#define LV_USE_IMGBTN     0

#define LV_USE_KEYBOARD   0
#define LV_USE_LED        0
#define LV_USE_LIST       0
#define LV_USE_MENU       0
#define LV_USE_METER      0
#define LV_USE_MSGBOX     0
#define LV_USE_SPINBOX    0
#define LV_USE_SPINNER    1
#define LV_USE_TABVIEW    0
#define LV_USE_TILEVIEW   0
#define LV_USE_WIN        0
#define LV_USE_SPAN       0
#if LV_USE_SPAN
    /*A line text can contain maximum num of span descriptor */
    /*Một dòng văn bản có thể chứa tối đa số descriptor span */
    #define LV_SPAN_SNIPPET_STACK_SIZE 64
#endif

/*-----------
 * Themes - Chủ đề
 *----------*/

/*A simple, impressive and very complete theme*/
/*Một chủ đề đơn giản, ấn tượng và rất hoàn chỉnh*/
#define LV_USE_THEME_DEFAULT 1
#if LV_USE_THEME_DEFAULT

    /*0: Light mode; 1: Dark mode*/
    /*0: Chế độ sáng; 1: Chế độ tối*/
    #define LV_THEME_DEFAULT_DARK 0

    /*1: Enable grow on press*/
    /*1: Bật hiệu ứng phóng to khi nhấn*/
    #define LV_THEME_DEFAULT_GROW 1

    /*Default transition time in [ms]*/
    /*Thời gian chuyển đổi mặc định [ms]*/
    #define LV_THEME_DEFAULT_TRANSITION_TIME 80
#endif /*LV_USE_THEME_DEFAULT*/

/*A very simple theme that is a good starting point for a custom theme*/
/*Một chủ đề rất đơn giản là điểm khởi đầu tốt cho chủ đề tùy chỉnh*/
#define LV_USE_THEME_BASIC 0

/*A theme designed for monochrome displays*/
/*Một chủ đề thiết kế cho màn hình đơn sắc*/
#define LV_USE_THEME_MONO 0

/*-----------
 * Layouts - Bố cục
 *----------*/

/*A layout similar to Flexbox in CSS.*/
/*Một bố cục tương tự Flexbox trong CSS.*/
#define LV_USE_FLEX 0

/*A layout similar to Grid in CSS.*/
/*Một bố cục tương tự Grid trong CSS.*/
#define LV_USE_GRID 0

/*---------------------
 * 3rd party libraries - Thư viện bên thứ ba
 *--------------------*/

/*File system interfaces for common APIs */
/*Giao diện hệ thống file cho các API phổ biến */

/*API for fopen, fread, etc*/
/*API cho fopen, fread, v.v.*/
#define LV_USE_FS_STDIO 0
#if LV_USE_FS_STDIO
    #define LV_FS_STDIO_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A') - Đặt một chữ cái viết hoa mà ổ đĩa sẽ truy cập được (ví dụ: 'A')*/
    #define LV_FS_STDIO_PATH ""         /*Set the working directory. File/directory paths will be appended to it. - Đặt thư mục làm việc. Đường dẫn file/thư mục sẽ được nối vào nó.*/
    #define LV_FS_STDIO_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read() - >0 để cache số byte này trong lv_fs_read()*/
#endif

/*API for open, read, etc*/
/*API cho open, read, v.v.*/
#define LV_USE_FS_POSIX 0
#if LV_USE_FS_POSIX
    #define LV_FS_POSIX_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A') - Đặt một chữ cái viết hoa mà ổ đĩa sẽ truy cập được (ví dụ: 'A')*/
    #define LV_FS_POSIX_PATH ""         /*Set the working directory. File/directory paths will be appended to it. - Đặt thư mục làm việc. Đường dẫn file/thư mục sẽ được nối vào nó.*/
    #define LV_FS_POSIX_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read() - >0 để cache số byte này trong lv_fs_read()*/
#endif

/*API for CreateFile, ReadFile, etc*/
/*API cho CreateFile, ReadFile, v.v.*/
#define LV_USE_FS_WIN32 0
#if LV_USE_FS_WIN32
    #define LV_FS_WIN32_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A') - Đặt một chữ cái viết hoa mà ổ đĩa sẽ truy cập được (ví dụ: 'A')*/
    #define LV_FS_WIN32_PATH ""         /*Set the working directory. File/directory paths will be appended to it. - Đặt thư mục làm việc. Đường dẫn file/thư mục sẽ được nối vào nó.*/
    #define LV_FS_WIN32_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read() - >0 để cache số byte này trong lv_fs_read()*/
#endif

/*API for FATFS (needs to be added separately). Uses f_open, f_read, etc*/
/*API cho FATFS (cần được thêm riêng). Sử dụng f_open, f_read, v.v.*/
#define LV_USE_FS_FATFS 0
#if LV_USE_FS_FATFS
    #define LV_FS_FATFS_LETTER '\0'     /*Set an upper cased letter on which the drive will accessible (e.g. 'A') - Đặt một chữ cái viết hoa mà ổ đĩa sẽ truy cập được (ví dụ: 'A')*/
    #define LV_FS_FATFS_CACHE_SIZE 0    /*>0 to cache this number of bytes in lv_fs_read() - >0 để cache số byte này trong lv_fs_read()*/
#endif

/*PNG decoder library*/
/*Thư viện giải mã PNG*/
#define LV_USE_PNG 0

/*BMP decoder library*/
/*Thư viện giải mã BMP*/
#define LV_USE_BMP 0

/*JPG + split JPG decoder library.
 *Split JPG is a custom format optimized for embedded systems. */
/*Thư viện giải mã JPG + split JPG.
 *Split JPG là định dạng tùy chỉnh được tối ưu hóa cho hệ thống nhúng. */
#define LV_USE_SJPG 0

/*GIF decoder library*/
/*Thư viện giải mã GIF*/
#define LV_USE_GIF 0

/*QR code library*/
/*Thư viện mã QR*/
#define LV_USE_QRCODE 0

/*FreeType library*/
/*Thư viện FreeType*/
#define LV_USE_FREETYPE 0
#if LV_USE_FREETYPE
    /*Memory used by FreeType to cache characters [bytes] (-1: no caching)*/
    /*Bộ nhớ được FreeType sử dụng để cache ký tự [bytes] (-1: không cache)*/
    #define LV_FREETYPE_CACHE_SIZE (16 * 1024)
    #if LV_FREETYPE_CACHE_SIZE >= 0
        /* 1: bitmap cache use the sbit cache, 0:bitmap cache use the image cache. */
        /* sbit cache:it is much more memory efficient for small bitmaps(font size < 256) */
        /* if font size >= 256, must use image cache */
        /* 1: bitmap cache sử dụng sbit cache, 0: bitmap cache sử dụng image cache. */
        /* sbit cache: hiệu quả bộ nhớ hơn nhiều cho bitmap nhỏ (kích thước font < 256) */
        /* nếu kích thước font >= 256, phải sử dụng image cache */
        #define LV_FREETYPE_SBIT_CACHE 0
        /* Maximum number of opened FT_Face/FT_Size objects managed by this cache instance. */
        /* (0:use system defaults) */
        /* Số lượng tối đa các đối tượng FT_Face/FT_Size đã mở được quản lý bởi instance cache này. */
        /* (0: sử dụng mặc định hệ thống) */
        #define LV_FREETYPE_CACHE_FT_FACES 0
        #define LV_FREETYPE_CACHE_FT_SIZES 0
    #endif
#endif

/*Rlottie library*/
/*Thư viện Rlottie*/
#define LV_USE_RLOTTIE 0

/*FFmpeg library for image decoding and playing videos
 *Supports all major image formats so do not enable other image decoder with it*/
#define LV_USE_FFMPEG 0
#if LV_USE_FFMPEG
    /*Dump input information to stderr*/
    #define LV_FFMPEG_DUMP_FORMAT 0
#endif

/*-----------
 * Others
 *----------*/

/*1: Enable API to take snapshot for object*/
#define LV_USE_SNAPSHOT 0

/*1: Enable Monkey test*/
/*1: Bật kiểm thử Monkey*/
#define LV_USE_MONKEY 0

/*1: Enable grid navigation*/
/*1: Bật điều hướng lưới*/
#define LV_USE_GRIDNAV 0

/*1: Enable lv_obj fragment*/
/*1: Bật lv_obj fragment*/
#define LV_USE_FRAGMENT 0

/*1: Support using images as font in label or span widgets */
/*1: Hỗ trợ sử dụng hình ảnh làm font trong widget label hoặc span */
#define LV_USE_IMGFONT 0

/*1: Enable a published subscriber based messaging system */
/*1: Bật hệ thống tin nhắn dựa trên published subscriber */
#define LV_USE_MSG 0

/*1: Enable Pinyin input method*/
/*Requires: lv_keyboard*/
/*1: Bật phương thức nhập Pinyin*/
/*Yêu cầu: lv_keyboard*/
#define LV_USE_IME_PINYIN 0
#if LV_USE_IME_PINYIN
    /*1: Use default thesaurus*/
    /*If you do not use the default thesaurus, be sure to use `lv_ime_pinyin` after setting the thesaurus*/
    /*1: Sử dụng từ điển mặc định*/
    /*Nếu bạn không sử dụng từ điển mặc định, hãy chắc chắn sử dụng `lv_ime_pinyin` sau khi đặt từ điển*/
    #define LV_IME_PINYIN_USE_DEFAULT_DICT 1
    /*Set the maximum number of candidate panels that can be displayed*/
    /*This needs to be adjusted according to the size of the screen*/
    /*Đặt số lượng tối đa các panel ứng viên có thể được hiển thị*/
    /*Điều này cần được điều chỉnh theo kích thước màn hình*/
    #define LV_IME_PINYIN_CAND_TEXT_NUM 6

    /*Use 9 key input(k9)*/
    /*Sử dụng nhập 9 phím (k9)*/
    #define LV_IME_PINYIN_USE_K9_MODE      1
    #if LV_IME_PINYIN_USE_K9_MODE == 1
        #define LV_IME_PINYIN_K9_CAND_TEXT_NUM 3
    #endif // LV_IME_PINYIN_USE_K9_MODE
#endif

/*==================
* EXAMPLES - VÍ DỤ
*==================*/

/*Enable the examples to be built with the library*/
/*Bật các ví dụ để được xây dựng với thư viện*/
#define LV_BUILD_EXAMPLES 0

/*===================
 * DEMO USAGE - SỬ DỤNG DEMO
 ====================*/

/*Show some widget. It might be required to increase `LV_MEM_SIZE` */
/*Hiển thị một số widget. Có thể cần tăng `LV_MEM_SIZE` */
#define LV_USE_DEMO_WIDGETS 0
#if LV_USE_DEMO_WIDGETS
#define LV_DEMO_WIDGETS_SLIDESHOW 0
#endif

/*Demonstrate the usage of encoder and keyboard*/
/*Thể hiện cách sử dụng encoder và bàn phím*/
#define LV_USE_DEMO_KEYPAD_AND_ENCODER 0

/*Benchmark your system*/
/*Đánh giá hiệu năng hệ thống của bạn*/
#define LV_USE_DEMO_BENCHMARK 0

/*Stress test for LVGL*/
/*Kiểm thử stress cho LVGL*/
#define LV_USE_DEMO_STRESS 0

/*Music player demo*/
/*Demo trình phát nhạc*/
#define LV_USE_DEMO_MUSIC 0
#if LV_USE_DEMO_MUSIC
    #define LV_DEMO_MUSIC_SQUARE    0
    #define LV_DEMO_MUSIC_LANDSCAPE 0
    #define LV_DEMO_MUSIC_ROUND     0
    #define LV_DEMO_MUSIC_LARGE     0
    #define LV_DEMO_MUSIC_AUTO_PLAY 0
#endif

/*--END OF LV_CONF_H--*/

#endif /*LV_CONF_H*/

#endif /*End of "Content enable"*/
