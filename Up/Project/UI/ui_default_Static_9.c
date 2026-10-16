//
// Created by RM UI Designer
//

#include "ui_default_Static_9.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 21
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Static_9;
ui_interface_arc_t *ui_default_Static_Pitch_3 = (ui_interface_arc_t *)&(ui_default_Static_9.data[0]);
ui_interface_arc_t *ui_default_Static_Pitch_2 = (ui_interface_arc_t *)&(ui_default_Static_9.data[1]);
ui_interface_arc_t *ui_default_Static_Pitch_1 = (ui_interface_arc_t *)&(ui_default_Static_9.data[2]);
ui_interface_round_t *ui_default_Static_Star_Round = (ui_interface_round_t *)&(ui_default_Static_9.data[3]);
ui_interface_line_t *ui_default_Static_L1 = (ui_interface_line_t *)&(ui_default_Static_9.data[4]);
ui_interface_line_t *ui_default_Static_L2 = (ui_interface_line_t *)&(ui_default_Static_9.data[5]);
ui_interface_line_t *ui_default_Static_L3 = (ui_interface_line_t *)&(ui_default_Static_9.data[6]);

void _ui_init_default_Static_9() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Static_9.data[i].figure_name[0] = FRAME_ID;
        ui_default_Static_9.data[i].figure_name[1] = GROUP_ID;
        ui_default_Static_9.data[i].figure_name[2] = i + START_ID;
        ui_default_Static_9.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Static_9.data[i].operate_tpyel = 0;
    }

    ui_default_Static_Pitch_3->figure_tpye = 4;
    ui_default_Static_Pitch_3->layer = 0;
    ui_default_Static_Pitch_3->rx = 360;
    ui_default_Static_Pitch_3->ry = 360;
    ui_default_Static_Pitch_3->start_x = 960;
    ui_default_Static_Pitch_3->start_y = 540;
    ui_default_Static_Pitch_3->color = 8;
    ui_default_Static_Pitch_3->width = 16;
    ui_default_Static_Pitch_3->start_angle = 270;
    ui_default_Static_Pitch_3->end_angle = 271;

    ui_default_Static_Pitch_2->figure_tpye = 4;
    ui_default_Static_Pitch_2->layer = 0;
    ui_default_Static_Pitch_2->rx = 380;
    ui_default_Static_Pitch_2->ry = 380;
    ui_default_Static_Pitch_2->start_x = 960;
    ui_default_Static_Pitch_2->start_y = 540;
    ui_default_Static_Pitch_2->color = 8;
    ui_default_Static_Pitch_2->width = 16;
    ui_default_Static_Pitch_2->start_angle = 310;
    ui_default_Static_Pitch_2->end_angle = 311;

    ui_default_Static_Pitch_1->figure_tpye = 4;
    ui_default_Static_Pitch_1->layer = 0;
    ui_default_Static_Pitch_1->rx = 380;
    ui_default_Static_Pitch_1->ry = 380;
    ui_default_Static_Pitch_1->start_x = 960;
    ui_default_Static_Pitch_1->start_y = 540;
    ui_default_Static_Pitch_1->color = 8;
    ui_default_Static_Pitch_1->width = 16;
    ui_default_Static_Pitch_1->start_angle = 229;
    ui_default_Static_Pitch_1->end_angle = 230;

    ui_default_Static_Star_Round->figure_tpye = 2;
    ui_default_Static_Star_Round->layer = 0;
    ui_default_Static_Star_Round->r = 20;
    ui_default_Static_Star_Round->start_x = 959;
    ui_default_Static_Star_Round->start_y = 541;
    ui_default_Static_Star_Round->color = 8;
    ui_default_Static_Star_Round->width = 2;

    ui_default_Static_L1->figure_tpye = 0;
    ui_default_Static_L1->layer = 0;
    ui_default_Static_L1->start_x = 1586;
    ui_default_Static_L1->start_y = 678;
    ui_default_Static_L1->end_x = 1634;
    ui_default_Static_L1->end_y = 705;
    ui_default_Static_L1->color = 8;
    ui_default_Static_L1->width = 5;

    ui_default_Static_L2->figure_tpye = 0;
    ui_default_Static_L2->layer = 0;
    ui_default_Static_L2->start_x = 1682;
    ui_default_Static_L2->start_y = 673;
    ui_default_Static_L2->end_x = 1636;
    ui_default_Static_L2->end_y = 704;
    ui_default_Static_L2->color = 8;
    ui_default_Static_L2->width = 5;

    ui_default_Static_L3->figure_tpye = 0;
    ui_default_Static_L3->layer = 0;
    ui_default_Static_L3->start_x = 1634;
    ui_default_Static_L3->start_y = 696;
    ui_default_Static_L3->end_x = 1635;
    ui_default_Static_L3->end_y = 758;
    ui_default_Static_L3->color = 8;
    ui_default_Static_L3->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_9, sizeof(ui_default_Static_9));
}

void _ui_update_default_Static_9() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Static_9.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_9, sizeof(ui_default_Static_9));
}

void _ui_remove_default_Static_9() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Static_9.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_default_Static_9, sizeof(ui_default_Static_9));
}
