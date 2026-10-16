//
// Created by RM UI Designer
//

#include "ui_default_Static_13.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 13

ui_string_frame_t ui_default_Static_13;

ui_interface_string_t* ui_default_Static_Length = &ui_default_Static_13.option;

void _ui_init_default_Static_13() {
    ui_default_Static_13.option.figure_name[0] = FRAME_ID;
    ui_default_Static_13.option.figure_name[1] = GROUP_ID;
    ui_default_Static_13.option.figure_name[2] = START_ID;
    ui_default_Static_13.option.operate_tpyel = 1;
    ui_default_Static_13.option.figure_tpye = 7;
    ui_default_Static_13.option.layer = 0;
    ui_default_Static_13.option.font_size = 15;
    ui_default_Static_13.option.start_x = 1596;
    ui_default_Static_13.option.start_y = 351;
    ui_default_Static_13.option.color = 8;
    ui_default_Static_13.option.str_length = 7;
    ui_default_Static_13.option.width = 2;
    strcpy(ui_default_Static_Length->string, "Length:");

    ui_proc_string_frame(&ui_default_Static_13);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_13, sizeof(ui_default_Static_13));
}

void _ui_update_default_Static_13() {
    ui_default_Static_13.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_default_Static_13);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_13, sizeof(ui_default_Static_13));
}

void _ui_remove_default_Static_13() {
    ui_default_Static_13.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_default_Static_13);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_13, sizeof(ui_default_Static_13));
}