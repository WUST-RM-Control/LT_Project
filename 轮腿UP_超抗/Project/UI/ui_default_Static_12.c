//
// Created by RM UI Designer
//

#include "ui_default_Static_12.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 12

ui_string_frame_t ui_default_Static_12;

ui_interface_string_t* ui_default_Static_cm = &ui_default_Static_12.option;

void _ui_init_default_Static_12() {
    ui_default_Static_12.option.figure_name[0] = FRAME_ID;
    ui_default_Static_12.option.figure_name[1] = GROUP_ID;
    ui_default_Static_12.option.figure_name[2] = START_ID;
    ui_default_Static_12.option.operate_tpyel = 1;
    ui_default_Static_12.option.figure_tpye = 7;
    ui_default_Static_12.option.layer = 0;
    ui_default_Static_12.option.font_size = 18;
    ui_default_Static_12.option.start_x = 1828;
    ui_default_Static_12.option.start_y = 348;
    ui_default_Static_12.option.color = 8;
    ui_default_Static_12.option.str_length = 2;
    ui_default_Static_12.option.width = 2;
    strcpy(ui_default_Static_cm->string, "cm");

    ui_proc_string_frame(&ui_default_Static_12);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_12, sizeof(ui_default_Static_12));
}

void _ui_update_default_Static_12() {
    ui_default_Static_12.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_default_Static_12);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_12, sizeof(ui_default_Static_12));
}

void _ui_remove_default_Static_12() {
    ui_default_Static_12.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_default_Static_12);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_12, sizeof(ui_default_Static_12));
}