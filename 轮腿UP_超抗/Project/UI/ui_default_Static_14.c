//
// Created by RM UI Designer
//

#include "ui_default_Static_14.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 14

ui_string_frame_t ui_default_Static_14;

ui_interface_string_t* ui_default_Static_m_s = &ui_default_Static_14.option;

void _ui_init_default_Static_14() {
    ui_default_Static_14.option.figure_name[0] = FRAME_ID;
    ui_default_Static_14.option.figure_name[1] = GROUP_ID;
    ui_default_Static_14.option.figure_name[2] = START_ID;
    ui_default_Static_14.option.operate_tpyel = 1;
    ui_default_Static_14.option.figure_tpye = 7;
    ui_default_Static_14.option.layer = 0;
    ui_default_Static_14.option.font_size = 18;
    ui_default_Static_14.option.start_x = 1805;
    ui_default_Static_14.option.start_y = 408;
    ui_default_Static_14.option.color = 8;
    ui_default_Static_14.option.str_length = 3;
    ui_default_Static_14.option.width = 2;
    strcpy(ui_default_Static_m_s->string, "m/s");

    ui_proc_string_frame(&ui_default_Static_14);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_14, sizeof(ui_default_Static_14));
}

void _ui_update_default_Static_14() {
    ui_default_Static_14.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_default_Static_14);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_14, sizeof(ui_default_Static_14));
}

void _ui_remove_default_Static_14() {
    ui_default_Static_14.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_default_Static_14);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_14, sizeof(ui_default_Static_14));
}