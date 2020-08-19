// SPDX-License-Identifier: MIT
/*
 * MIT License
 * Copyright (c) 2019 Renesas Electronics Corporation
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "mbed.h"
#include "EasyAttach_CameraAndLCD.h"
#include "r_dk2_if.h"
#include "r_drp_simple_isp.h"
#include "r_drp_image_rotate.h"
#include "r_drp_resize_bilinear.h"
#include "r_drp_cropping.h"
#include "D6T_44L_06.h"
#include "SHT30_DIS_B.h"
#include "BARO_2SMPB_02E.h"
#include "dcache-control.h"
#include "AsciiFont.h"

#define M_PI 3.1415926535897932384626433832795

/*! Frame buffer stride: Frame buffer stride should be set to a multiple of 32 or 128
    in accordance with the frame buffer burst transfer mode. */
//#define VIDEO_PIXEL_HW         LCD_PIXEL_WIDTH
//#define VIDEO_PIXEL_VW         LCD_PIXEL_HEIGHT

#define VIDEO_PIXEL_HW         (1280)       /* HD */
#define VIDEO_PIXEL_VW         (720)        /* HD */
#define CROP_PIXEL_HW          (960)
#define CROP_PIXEL_VW          (672)
#define LCD_PIXEL_HW           (480)
#define LCD_PIXEL_VW           (272)

//#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * 1) + 63u) & ~63u)
#define FRAME_BUFFER_STRIDE    (((VIDEO_PIXEL_HW * 1) + 31u) & ~31u)
#define FRAME_BUFFER_STRIDE_0  (((CROP_PIXEL_HW * 1) + 31u) & ~31u)
#define FRAME_BUFFER_STRIDE_2  (((LCD_PIXEL_HW) + 31u) & ~31u)
#define FRAME_BUFFER_HEIGHT    (VIDEO_PIXEL_VW)
#define FRAME_BUFFER_HEIGHT_0  (CROP_PIXEL_VW)
#define FRAME_BUFFER_HEIGHT_2  (LCD_PIXEL_VW)

#define DRP_FLG_TILE_ALL       (R_DK2_TILE_0 | R_DK2_TILE_1 | R_DK2_TILE_2 | R_DK2_TILE_3 | R_DK2_TILE_4 | R_DK2_TILE_5)
#define DRP_FLG_CAMER_IN       (0x00000100)

/* ASCII BUFFER Parameter GRAPHICS_LAYER_3 */
#define ASCII_BUFFER_BYTE_PER_PIXEL   (2)
#define ASCII_BUFFER_STRIDE           (((LCD_PIXEL_HW * ASCII_BUFFER_BYTE_PER_PIXEL) + 31u) & ~31u)
#define ASCII_COLOR_WHITE             (0xFFFF)
#define ASCII_COLOR_BLACK             (0x00F0)
#define ASCII_FONT_SIZE               (2)
#define ASCII_FONT_SIZE_S             (1)
#define TILE_ALPHA_MAX      (0x0F)
#define TILE_ALPHA_SWITCH2  (0x0A)
#define TILE_ALPHA_SWITCH1  (0x06)
#define TILE_ALPHA_DEFAULT  (0x03)

#define TILE_TEMP_MARGIN_UPPER (120)         // original 20   
#define TILE_TEMP_MARGIN_UNDER (40)         // original 70
#define TILE_RESO_4         (4)
#define TILE_RESO_68        (68)
#define TILE_RESO_120       (120)

#define TILE_SIZE_HW_4x4    (LCD_PIXEL_HW/TILE_RESO_4)
#define TILE_SIZE_VW_4x4    (LCD_PIXEL_VW/TILE_RESO_4)
#define TILE_SIZE_HW_120x68  (LCD_PIXEL_HW/TILE_RESO_120)
#define TILE_SIZE_VW_120x68  (LCD_PIXEL_VW/TILE_RESO_68)

#define SUB_PHASE_MAX       (10)
#define SUB_PHASE_DEMO1     SUB_PHASE_MAX*2
#define SUB_PHASE_DEMO2     SUB_PHASE_MAX*3
#define PHASE_DELAY         (200)


static DisplayBase Display;

static uint8_t fbuf_bayer[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((aligned(128)));
static uint8_t fbuf_gray[FRAME_BUFFER_STRIDE * FRAME_BUFFER_HEIGHT]__attribute((section("NC_BSS")));
static uint8_t fbuf_gray0[FRAME_BUFFER_STRIDE_0 * FRAME_BUFFER_HEIGHT_0]__attribute((section("NC_BSS")));
static uint8_t fbuf_gray1[FRAME_BUFFER_STRIDE_2 * FRAME_BUFFER_HEIGHT_2]__attribute((section("NC_BSS")));
static uint8_t fbuf_gray2[FRAME_BUFFER_STRIDE_2 * FRAME_BUFFER_HEIGHT_2]__attribute((section("NC_BSS")));
static uint8_t fbuf_ascii0[ASCII_BUFFER_STRIDE * LCD_PIXEL_VW]__attribute((aligned(32)));
static uint8_t fbuf_ascii1[ASCII_BUFFER_STRIDE * LCD_PIXEL_VW]__attribute((aligned(32)));

AsciiFont* p_af0;
AsciiFont* p_af1;
int screen = 0;

/* thermal data array[y][x] */
static float array4x4[TILE_RESO_4][TILE_RESO_4];
static float array120x68[TILE_RESO_120][TILE_RESO_68];

static r_drp_simple_isp_t param_isp __attribute((section("NC_BSS")));
static r_drp_resize_bilinear_t param_resize_bilinear __attribute((section("NC_BSS")));
static r_drp_image_rotate_t param_image_rotate __attribute((section("NC_BSS")));
static r_drp_cropping_t param_image_crop __attribute((section("NC_BSS")));
static uint8_t drp_lib_id[R_DK2_TILE_NUM] = {0};
static Thread drpTask(osPriorityHigh, 1024*8);
static D6T_44L_06 d6t_44l(I2C_SDA, I2C_SCL);
// 2JCIE-EV01-RP1
static SHT30_DIS_B sht30(I2C_SDA, I2C_SCL);                    // [SHT30-DIS-B] : Temperature / humidity sensor
static BARO_2SMPB_02E baro_2smpb(I2C_SDA, I2C_SCL);            // [2SMPB-02E]   : MEMS digital barometric pressure sensor
static uint8_t display_alpha = 0x08;
static int16_t temp_margin = TILE_TEMP_MARGIN_UPPER;
static InterruptIn button0(USER_BUTTON0);
static InterruptIn button1(USER_BUTTON1);
static int16_t buf[16];

static char title_str[] =   "GR-MANGO THERMOGRAPHY DEMO (OMRON 2JCIE-EV01-RP1+D6T-44L-06)";

/*******************************************************************************
* Function Name: normalize0to1
* Description  : Normalize thermal data of a reception packet to the range of 0-1.
*                Also the value outside min, max is saturated.
* Arguments    : data     - input tempetue data
*                min      - smallest threshold tempature value
*                           (The integer which set a centigrade to 10 times)
*                max      - highest threshold  tempature value
*                           (The integer which set a centigrade to 10 times)
* Return Value : normalized output data
*******************************************************************************/
float normalize0to1(int16_t data, int min, int max)
{
    float normalized;

    if (data <= min) {
        normalized = 0.0;
    }
    else if (max <= data) {
        normalized = 1.0;
    }
    else {
        normalized = ((float)(data - min)) / ((float)(max - min));
    }
    return normalized;
}

/*******************************************************************************
* Function Name: conv_normalize_to_color
* Description  : Change a floating-point data of 0.0-1.0 to the pixel color of
*                ARGB4444(truth order:GBAR).
*                Low temperature changes blue and high temperature to red.
* Arguments    : data        - Normalized thermal data from 0.0 to 1.0.
* Return Value : ARGB4444 pixel color data.
*******************************************************************************/
static uint16_t conv_normalize_to_color(uint8_t alpha, float data) {
    uint8_t green;
    uint8_t blue;
    uint8_t red;

    if (0.0 == data) {
        /* Display blue when the temperature is below the minimum. */
        blue  = 0x0F;
        green = 0x00;
        red   = 0x00;
    }
    else if (1.0 == data) {
        /* Display red when the maximum temperature is exceeded. */
        blue  = 0x00;
        green = 0x00;
        red   = 0x0F;
    }
    else {
        float cosval   = cos( 4 * M_PI * data);
        int    color    = (int)((((-cosval)/2) + 0.5) * 15);
        if (data < 0.25) {
            blue  = 0xF;
            green = color;
            red   = 0x00;
        }
        else if (data < 0.50) {
            blue  = color;
            green = 0x0F;
            red   = 0x00;
        }
        else if (data < 0.75) {
            blue  = 0x00;
            green = 0x0F;
            red   = color;
        }
        else {
            blue  = 0x00;
            green = color;
            red   = 0x0F;
        }
    }
    return ((green << 12) | (blue << 8) | (alpha << 4) | red);
}

/*******************************************************************************
* Function Name: liner_interpolation
* Description  : Expand float data array from array[y_in_size][x_in_size] to array[y_out_size][x_out_size]
*                Linear complementation is used for expansion algorithm.
* Arguments    : p_in_array   - pointer of input data
*                p_out_array  - pointer of output data
*                x_in_size    - input array x size
*                y_in_size    - input array y size
*                x_out_size   - output array x size
*                y_out_size   - output array y size
* Return Value : none
*******************************************************************************/
static void liner_interpolation(float* p_in_array, float* p_out_array, int x_in_size, int y_in_size, int x_out_size, int y_out_size)
{
    int   x_in;
    int   y_in;
    int   x_out_start;
    int   x_out_goal;
    float x_delta;
    int   x_w_pos;
    int   y_out_start;
    int   y_out_goal;
    float y_delta;
    int   y_w_pos;
    float data_start;
    float data_goal;
    float data_w_pos;

    /* expand x direction */
    for (y_in = 0; y_in < y_in_size; y_in++) {
        y_out_goal = (y_out_size - 1) * y_in / (y_in_size - 1);
        for (x_in = 1; x_in < x_in_size; x_in++) {
            x_out_start  = (x_out_size - 1) * (x_in - 1) / (x_in_size - 1);
            x_out_goal   = (x_out_size - 1) * (x_in    ) / (x_in_size - 1);
            x_delta   = x_out_goal - x_out_start;

            data_start = p_in_array[(x_in - 1) + (x_in_size * y_in)];
            data_goal  = p_in_array[(x_in    ) + (x_in_size * y_in)];

            for (x_w_pos = x_out_start; x_w_pos <= x_out_goal; x_w_pos++) {
                data_w_pos = (data_start * ((x_out_goal - x_w_pos ) / x_delta))
                           + (data_goal  * ((x_w_pos - x_out_start) / x_delta));
                p_out_array[x_w_pos + (x_out_size * y_out_goal)] = data_w_pos;
            }
        }
    }

    /* expand y direction */
    for (x_w_pos = 0; x_w_pos < x_out_size; x_w_pos++) {
        for (y_in = 1; y_in < y_in_size; y_in++) {
            y_out_start  = (y_out_size - 1) * (y_in - 1) / (y_in_size - 1);
            y_out_goal   = (y_out_size - 1) * (y_in    ) / (y_in_size - 1);
            y_delta   = y_out_goal - y_out_start;

            data_start = p_out_array[x_w_pos + (x_out_size * y_out_start)];
            data_goal  = p_out_array[x_w_pos + (x_out_size * y_out_goal)];

            for (y_w_pos = y_out_start+1; y_w_pos < y_out_goal; y_w_pos++) {
                data_w_pos = (data_start * ((y_out_goal - y_w_pos ) / y_delta))
                           + (data_goal  * ((y_w_pos - y_out_start) / y_delta));
                p_out_array[x_w_pos + (x_out_size * y_w_pos)] = data_w_pos;
            }
        }
    }
}

/*******************************************************************************
* Function Name: update_thermograph
* Description  : Update display thermograph.
* Arguments    : reso_x    - input array x size
*                reso_y    - input array y size
*                tile_hw   - tile width pixel size
*                tile_vw   - tile height pixel size
*                alpha     - alpha pixel value of thermograph
*                p_array   - pointer of thermal data array
*                title_str - title string
* Return Value : none
*******************************************************************************/
void update_thermograph(int reso_x, int reso_y, int tile_hw, int tile_vw, uint8_t alpha, float* p_array, const char* head_str, const char* foot_str)
{
    AsciiFont* p_af;
    uint8_t*   p_fbuf;
    int x, y;
    uint16_t color;
    char str_buf[64];

    if (0 == screen)
    {
        p_af = p_af0;
        p_fbuf = &fbuf_ascii0[0];
    }
    else
    {
        p_af = p_af1;
        p_fbuf = &fbuf_ascii1[0];
    }

    for (y = 0; y < reso_y; y++)
    {
        for (x = 0; x < reso_x; x++)
        {
            color = conv_normalize_to_color(alpha, p_array[(y * reso_x)  + x]);
            p_af->Erase(color, (tile_hw * x), (tile_vw * y), tile_hw, tile_vw);
        }
    }
    //p_af->Erase(ASCII_COLOR_WHITE, 0, 0, 30, 20);
    p_af->DrawStr(head_str, 0, 0, ASCII_COLOR_WHITE, ASCII_FONT_SIZE, 40);
    p_af->DrawStr(foot_str, 0, 264, ASCII_COLOR_WHITE, ASCII_FONT_SIZE_S, 80);
    for (int y = 0; y < 4; y++) {
        for (int x = 0;x < 4; x++) {
            int px = LCD_PIXEL_HW / 4 * (x + 1) - LCD_PIXEL_HW / 6;
            int py = LCD_PIXEL_VW / 4 * (y + 1) - LCD_PIXEL_VW / 7;

            sprintf(str_buf, "%4.1f   ", buf[y*4+x] / 10.0);
            p_af->DrawStr(str_buf, px, py, ASCII_COLOR_WHITE, 1, 4);
        }
    }

    dcache_clean(p_fbuf, sizeof(fbuf_ascii0));
    Display.Graphics_Read_Change(DisplayBase::GRAPHICS_LAYER_3, (void *)p_fbuf);

    if (0 == screen)
    {
        screen = 1;
    }
    else
    {
        screen = 0;
    }

    return;
}

/*******************************************************************************
* Function Name: IntCallbackFunc_Vfield
*******************************************************************************/
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type) {
    drpTask.flags_set(DRP_FLG_CAMER_IN);
}

/*******************************************************************************
* Function Name: cb_drp_finish
*******************************************************************************/
static void cb_drp_finish(uint8_t id) {
    uint32_t tile_no;
    uint32_t set_flgs = 0;

    // Change the operation state of the DRP library notified by the argument to finish
    for (tile_no = 0; tile_no < R_DK2_TILE_NUM; tile_no++) {
        if (drp_lib_id[tile_no] == id) {
            set_flgs |= (1 << tile_no);
        }
    }
    drpTask.flags_set(set_flgs);
}

/*******************************************************************************
* Function Name: Start_Video_Camera
*******************************************************************************/
static void Start_Video_Camera(void) {
    // Video capture setting (progressive form fixed)
    Display.Video_Write_Setting(
        DisplayBase::VIDEO_INPUT_CHANNEL_0,
        DisplayBase::COL_SYS_NTSC_358,
        (void *)fbuf_bayer,
        FRAME_BUFFER_STRIDE,
        DisplayBase::VIDEO_FORMAT_RAW8,
        DisplayBase::WR_RD_WRSWA_NON,
        VIDEO_PIXEL_VW,
        VIDEO_PIXEL_HW
    );
    EasyAttach_CameraStart(Display, DisplayBase::VIDEO_INPUT_CHANNEL_0);
}

/*******************************************************************************
* Function Name: Start_LCD_Display
*******************************************************************************/
#if MBED_CONF_APP_LCD
static void Start_LCD_Display(void) {
    DisplayBase::rect_t rect;

    rect.vs = 0;
    rect.vw = LCD_PIXEL_VW;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_0,
        (void *)fbuf_gray2,
        FRAME_BUFFER_STRIDE_2,
        DisplayBase::GRAPHICS_FORMAT_CLUT8,
        DisplayBase::WR_RD_WRSWA_32_16_8BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_0);

    ThisThread::sleep_for(50);
    EasyAttach_LcdBacklight(true);
}
#endif

/*******************************************************************************
* Function Name: Start_Thermo_Display
*******************************************************************************/
static void Start_Thermo_Display(void) {
    DisplayBase::rect_t rect;

    memset(fbuf_ascii1, 0, sizeof(fbuf_ascii1));

    rect.vs = 0;
    rect.vw = LCD_PIXEL_VW;
    rect.hs = 0;
    rect.hw = LCD_PIXEL_HW;
    Display.Graphics_Read_Setting(
        DisplayBase::GRAPHICS_LAYER_3,
        (void *)fbuf_ascii0,
        ASCII_BUFFER_STRIDE,
        DisplayBase::GRAPHICS_FORMAT_ARGB4444,
        DisplayBase::WR_RD_WRSWA_32_16BIT,
        &rect
    );
    Display.Graphics_Start(DisplayBase::GRAPHICS_LAYER_3);

}

/*******************************************************************************
* Function Name: drp_task
*******************************************************************************/
static void drp_task(void) {
    //EasyAttach_Init(Display, 480, 272);
    EasyAttach_Init(Display, 640, 480);
    //EasyAttach_Init(Display);
    // Interrupt callback function setting (Field end signal for recording function in scaler 0)
    Display.Graphics_Irq_Handler_Set(DisplayBase::INT_TYPE_S0_VFIELD, 0, IntCallbackFunc_Vfield);
    Start_Video_Camera();
#if MBED_CONF_APP_LCD
    Start_LCD_Display();
#endif
    Start_Thermo_Display();

    R_DK2_Initialize();

    while (true) {
        ThisThread::flags_wait_all(DRP_FLG_CAMER_IN);
        R_DK2_Load(g_drp_lib_simple_isp_bayer2grayscale_6,
        //    g_drp_lib_simple_isp_bayer2yuv_6,
                   R_DK2_TILE_0,
                   R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);

        memset(&param_isp, 0, sizeof(param_isp));
        param_isp.src    = (uint32_t)fbuf_bayer;
        param_isp.dst    = (uint32_t)fbuf_gray;
        param_isp.width  = VIDEO_PIXEL_HW;
        param_isp.height = VIDEO_PIXEL_VW;
        param_isp.gain_r = 0x1800;
        param_isp.gain_g = 0x1000;
        param_isp.gain_b = 0x1C00;
        param_isp.bias_r = -16;
        param_isp.bias_g = -16;
        param_isp.bias_b = -16;
        R_DK2_Start(drp_lib_id[0], (void *)&param_isp, sizeof(r_drp_simple_isp_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        /* clop */
        R_DK2_Load(g_drp_lib_cropping,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_resize_bilinear, 0, sizeof(param_resize_bilinear));
        param_image_crop.src    = (uint32_t)fbuf_gray;
        param_image_crop.dst    = (uint32_t)fbuf_gray0;
        param_image_crop.src_width  = VIDEO_PIXEL_HW;
        param_image_crop.src_height = VIDEO_PIXEL_VW;
        param_image_crop.offset_x   = (VIDEO_PIXEL_HW - CROP_PIXEL_HW) / 2;
        param_image_crop.offset_y   = 0; //(VIDEO_PIXEL_VW - CROP_PIXEL_VW) / 2;
        param_image_crop.dst_width  = CROP_PIXEL_HW;
        param_image_crop.dst_height = CROP_PIXEL_VW;
        R_DK2_Start(drp_lib_id[0], (void *)&param_image_crop, sizeof(r_drp_cropping_t));
        ThisThread::flags_wait_all(R_DK2_TILE_0); 
        R_DK2_Unload(0, drp_lib_id);

        /* resize bilinear */
        R_DK2_Load(g_drp_lib_resize_bilinear,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_6, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_resize_bilinear, 0, sizeof(param_resize_bilinear));
        param_resize_bilinear.src    = (uint32_t)fbuf_gray0;
        param_resize_bilinear.dst    = (uint32_t)fbuf_gray1;
        param_resize_bilinear.src_width  = CROP_PIXEL_HW;
        param_resize_bilinear.src_height = CROP_PIXEL_VW;
        param_resize_bilinear.dst_width  = LCD_PIXEL_HW;
        param_resize_bilinear.dst_height = LCD_PIXEL_VW;
        R_DK2_Start(drp_lib_id[0], (void *)&param_resize_bilinear, sizeof(r_drp_resize_bilinear_t));
        ThisThread::flags_wait_all(DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

        /* image rotate (flip) */

        R_DK2_Load(g_drp_lib_image_rotate,
            R_DK2_TILE_0,
            R_DK2_TILE_PATTERN_1_1_1_1_1_1, NULL, &cb_drp_finish, drp_lib_id);
        R_DK2_Activate(0, 0);
        memset(&param_image_rotate, 0, sizeof(param_image_rotate));
        param_image_rotate.src    = (uint32_t)fbuf_gray1;
        param_image_rotate.dst    = (uint32_t)fbuf_gray2;
        param_image_rotate.src_width  = LCD_PIXEL_HW;
        param_image_rotate.src_height = LCD_PIXEL_VW;
        param_image_rotate.dst_stride  = FRAME_BUFFER_STRIDE_2;
        param_image_rotate.mode = 4;            // image flip
        R_DK2_Start(drp_lib_id[0], (void *)&param_image_rotate, sizeof(r_drp_image_rotate_t));
        ThisThread::flags_wait_all(R_DK2_TILE_0);   //DRP_FLG_TILE_ALL);
        R_DK2_Unload(0, drp_lib_id);

    }
}
/*******************************************************************************
* Function Name: User button callback
*******************************************************************************/
static void button_fall0(void) {
    display_alpha++;
    display_alpha &= 0x0f;
}

static void button_fall1(void) {
    temp_margin += 10;
    if (temp_margin > 140) temp_margin = 20;
}

/*******************************************************************************
* Function Name: flip : mirror sensord data
*******************************************************************************/
static int16_t flip_buf(int16_t *buf) {
    int16_t x, y, t, max;

    max = -32768;
    for (y = 0; y < 4; y++) {
        for (x = 0; x < 2; x++) {
            t = buf[y * 4 + x];
            buf[y * 4 + x] = buf[y * 4 + 3 - x];
            buf[y * 4 + 3 - x] = t;
        }
        for (x = 0; x < 4; x++) {
            if (buf[y * 4 + x] > max) max = buf[y * 4 + x];
        }
    }
    return max;
}
/*******************************************************************************
* Function Name: main function
*******************************************************************************/
int main(void) {
    int16_t pdta;
    char    head_str[100];
    char    foot_str[100];
    static int16_t max_temp;
    int32_t sht30_humi, sht30_temp;
    uint32_t press, dp, dt;
    int16_t temp16;

    // Set up button
    button0.fall(&button_fall0);
    button1.fall(&button_fall1);

    // Start DRP task
    drpTask.start(callback(drp_task));

    printf("\x1b[2J");  // Clear screen

    // setup sensors
    d6t_44l.setup();
    baro_2smpb.setup();
    sht30.setup();

    ThisThread::sleep_for(150);

    AsciiFont ascii_font0(fbuf_ascii0, LCD_PIXEL_HW, LCD_PIXEL_VW, ASCII_BUFFER_STRIDE, ASCII_BUFFER_BYTE_PER_PIXEL);
    AsciiFont ascii_font1(fbuf_ascii1, LCD_PIXEL_HW, LCD_PIXEL_VW, ASCII_BUFFER_STRIDE, ASCII_BUFFER_BYTE_PER_PIXEL);

    p_af0 = &ascii_font0;
    p_af1 = &ascii_font1;

    while (1) {
        int x, y;

        printf("\x1b[%d;%dH", 0, 0);  // Move cursor (y , x)
        // get ir data
        while (d6t_44l.read(&pdta, &buf[0]) == false) {
            ThisThread::sleep_for(10);
        }
        max_temp = flip_buf(buf);        
        for (y = 0; y < TILE_RESO_4; y++)
        {
            for (x = 0; x < TILE_RESO_4; x++)
            {
                array4x4[y][x] = normalize0to1(buf[x + (TILE_RESO_4*y)], pdta - TILE_TEMP_MARGIN_UNDER,  pdta + temp_margin);
            }
        }
        printf("PTAT: %6.1f[degC]\r\n", pdta / 10.0);
        for (int i = 0; i < 16; i++) {
            printf("%4.1f, ", buf[i] / 10.0);

            if ((i % 4) == 4 - 1) {
                printf("\r\n");
            }
        }

        // get temp & humidity
        sht30.read(&sht30_humi, &sht30_temp);
        // get pressure
        baro_2smpb.read(&press, &temp16, &dp, &dt);
        // disp thermograph
        sprintf(head_str, "MAX:%5.1fC(T:%5.1fC H:%5.1f%% P:%6.1fHP)" , max_temp / 10.0, sht30_temp / 100.0, sht30_humi / 100.0, press / 1000.0 );
        sprintf(foot_str, "%s PTAT:%5.1fC Alfa:%1XH", title_str, pdta / 10.0, display_alpha);
        liner_interpolation(&array4x4[0][0], &array120x68[0][0], TILE_RESO_4, TILE_RESO_4, TILE_RESO_120, TILE_RESO_68);
        update_thermograph(TILE_RESO_120, TILE_RESO_68, TILE_SIZE_HW_120x68, TILE_SIZE_VW_120x68, display_alpha, &array120x68[0][0], head_str, foot_str);

        ThisThread::sleep_for(PHASE_DELAY);
    }
}

