//
// Created by RM UI Designer
//

#include "ui_default_Static_10.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 28
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Static_10;
ui_interface_line_t *ui_default_Static_L4 = (ui_interface_line_t *)&(ui_default_Static_10.data[0]);
ui_interface_line_t *ui_default_Static_L5 = (ui_interface_line_t *)&(ui_default_Static_10.data[1]);
ui_interface_line_t *ui_default_Static_L6 = (ui_interface_line_t *)&(ui_default_Static_10.data[2]);
ui_interface_round_t *ui_default_Static_R2 = (ui_interface_round_t *)&(ui_default_Static_10.data[3]);
ui_interface_round_t *ui_default_Static_R1 = (ui_interface_round_t *)&(ui_default_Static_10.data[4]);

void _ui_init_default_Static_10() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Static_10.data[i].figure_name[0] = FRAME_ID;
        ui_default_Static_10.data[i].figure_name[1] = GROUP_ID;
        ui_default_Static_10.data[i].figure_name[2] = i + START_ID;
        ui_default_Static_10.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Static_10.data[i].operate_tpyel = 0;
    }

    ui_default_Static_L4->figure_tpye = 0;
    ui_default_Static_L4->layer = 0;
    ui_default_Static_L4->start_x = 1633;
    ui_default_Static_L4->start_y = 705;
    ui_default_Static_L4->end_x = 1595;
    ui_default_Static_L4->end_y = 725;
    ui_default_Static_L4->color = 8;
    ui_default_Static_L4->width = 5;

    ui_default_Static_L5->figure_tpye = 0;
    ui_default_Static_L5->layer = 0;
    ui_default_Static_L5->start_x = 1635;
    ui_default_Static_L5->start_y = 705;
    ui_default_Static_L5->end_x = 1673;
    ui_default_Static_L5->end_y = 722;
    ui_default_Static_L5->color = 8;
    ui_default_Static_L5->width = 5;

    ui_default_Static_L6->figure_tpye = 0;
    ui_default_Static_L6->layer = 0;
    ui_default_Static_L6->start_x = 1633;
    ui_default_Static_L6->start_y = 662;
    ui_default_Static_L6->end_x = 1634;
    ui_default_Static_L6->end_y = 700;
    ui_default_Static_L6->color = 8;
    ui_default_Static_L6->width = 5;

    ui_default_Static_R2->figure_tpye = 2;
    ui_default_Static_R2->layer = 0;
    ui_default_Static_R2->r = 84;
    ui_default_Static_R2->start_x = 1633;
    ui_default_Static_R2->start_y = 703;
    ui_default_Static_R2->color = 8;
    ui_default_Static_R2->width = 5;

    ui_default_Static_R1->figure_tpye = 2;
    ui_default_Static_R1->layer = 0;
    ui_default_Static_R1->r = 140;
    ui_default_Static_R1->start_x = 1633;
    ui_default_Static_R1->start_y = 703;
    ui_default_Static_R1->color = 8;
    ui_default_Static_R1->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_10, sizeof(ui_default_Static_10));
}

void _ui_update_default_Static_10() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Static_10.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_10, sizeof(ui_default_Static_10));
}

void _ui_remove_default_Static_10() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Static_10.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_10, sizeof(ui_default_Static_10));
}
