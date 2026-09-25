//
// Created by RM UI Designer
//

#include "ui_default_Dynamic_0.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Dynamic_0;
ui_interface_arc_t *ui_default_Dynamic_SuperCap = (ui_interface_arc_t *)&(ui_default_Dynamic_0.data[0]);
ui_interface_number_t *ui_default_Dynamic_Pitch_Num = (ui_interface_number_t *)&(ui_default_Dynamic_0.data[1]);
ui_interface_arc_t *ui_default_Dynamic_Pitch = (ui_interface_arc_t *)&(ui_default_Dynamic_0.data[2]);
ui_interface_arc_t *ui_default_Dynamic_Fric_First_M = (ui_interface_arc_t *)&(ui_default_Dynamic_0.data[3]);
ui_interface_arc_t *ui_default_Dynamic_Fric_First_L = (ui_interface_arc_t *)&(ui_default_Dynamic_0.data[4]);
ui_interface_arc_t *ui_default_Dynamic_Fric_First_R = (ui_interface_arc_t *)&(ui_default_Dynamic_0.data[5]);
ui_interface_arc_t *ui_default_Dynamic_Fric_Second_R = (ui_interface_arc_t *)&(ui_default_Dynamic_0.data[6]);

void _ui_init_default_Dynamic_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_Dynamic_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_Dynamic_0.data[i].figure_name[2] = i + START_ID;
        ui_default_Dynamic_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Dynamic_0.data[i].operate_tpyel = 0;
    }

    ui_default_Dynamic_SuperCap->figure_tpye = 4;
    ui_default_Dynamic_SuperCap->layer = 0;
    ui_default_Dynamic_SuperCap->rx = 380;
    ui_default_Dynamic_SuperCap->ry = 380;
    ui_default_Dynamic_SuperCap->start_x = 960;
    ui_default_Dynamic_SuperCap->start_y = 540;
    ui_default_Dynamic_SuperCap->color = 6;
    ui_default_Dynamic_SuperCap->width = 16;
    ui_default_Dynamic_SuperCap->start_angle = 50;
    ui_default_Dynamic_SuperCap->end_angle = 130;

    ui_default_Dynamic_Pitch_Num->figure_tpye = 6;
    ui_default_Dynamic_Pitch_Num->layer = 0;
    ui_default_Dynamic_Pitch_Num->font_size = 30;
    ui_default_Dynamic_Pitch_Num->start_x = 482;
    ui_default_Dynamic_Pitch_Num->start_y = 640;
    ui_default_Dynamic_Pitch_Num->color = 6;
    ui_default_Dynamic_Pitch_Num->number = 45;
    ui_default_Dynamic_Pitch_Num->width = 3;

    ui_default_Dynamic_Pitch->figure_tpye = 4;
    ui_default_Dynamic_Pitch->layer = 0;
    ui_default_Dynamic_Pitch->rx = 380;
    ui_default_Dynamic_Pitch->ry = 380;
    ui_default_Dynamic_Pitch->start_x = 962;
    ui_default_Dynamic_Pitch->start_y = 542;
    ui_default_Dynamic_Pitch->color = 6;
    ui_default_Dynamic_Pitch->width = 16;
    ui_default_Dynamic_Pitch->start_angle = 269;
    ui_default_Dynamic_Pitch->end_angle = 271;

    ui_default_Dynamic_Fric_First_M->figure_tpye = 4;
    ui_default_Dynamic_Fric_First_M->layer = 0;
    ui_default_Dynamic_Fric_First_M->rx = 21;
    ui_default_Dynamic_Fric_First_M->ry = 21;
    ui_default_Dynamic_Fric_First_M->start_x = 1635;
    ui_default_Dynamic_Fric_First_M->start_y = 756;
    ui_default_Dynamic_Fric_First_M->color = 6;
    ui_default_Dynamic_Fric_First_M->width = 5;
    ui_default_Dynamic_Fric_First_M->start_angle = 30;
    ui_default_Dynamic_Fric_First_M->end_angle = 330;

    ui_default_Dynamic_Fric_First_L->figure_tpye = 4;
    ui_default_Dynamic_Fric_First_L->layer = 0;
    ui_default_Dynamic_Fric_First_L->rx = 21;
    ui_default_Dynamic_Fric_First_L->ry = 21;
    ui_default_Dynamic_Fric_First_L->start_x = 1586;
    ui_default_Dynamic_Fric_First_L->start_y = 677;
    ui_default_Dynamic_Fric_First_L->color = 6;
    ui_default_Dynamic_Fric_First_L->width = 5;
    ui_default_Dynamic_Fric_First_L->start_angle = 30;
    ui_default_Dynamic_Fric_First_L->end_angle = 330;

    ui_default_Dynamic_Fric_First_R->figure_tpye = 4;
    ui_default_Dynamic_Fric_First_R->layer = 0;
    ui_default_Dynamic_Fric_First_R->rx = 21;
    ui_default_Dynamic_Fric_First_R->ry = 21;
    ui_default_Dynamic_Fric_First_R->start_x = 1682;
    ui_default_Dynamic_Fric_First_R->start_y = 672;
    ui_default_Dynamic_Fric_First_R->color = 6;
    ui_default_Dynamic_Fric_First_R->width = 5;
    ui_default_Dynamic_Fric_First_R->start_angle = 30;
    ui_default_Dynamic_Fric_First_R->end_angle = 330;

    ui_default_Dynamic_Fric_Second_R->figure_tpye = 4;
    ui_default_Dynamic_Fric_Second_R->layer = 0;
    ui_default_Dynamic_Fric_Second_R->rx = 21;
    ui_default_Dynamic_Fric_Second_R->ry = 21;
    ui_default_Dynamic_Fric_Second_R->start_x = 1671;
    ui_default_Dynamic_Fric_Second_R->start_y = 720;
    ui_default_Dynamic_Fric_Second_R->color = 6;
    ui_default_Dynamic_Fric_Second_R->width = 5;
    ui_default_Dynamic_Fric_Second_R->start_angle = 30;
    ui_default_Dynamic_Fric_Second_R->end_angle = 330;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_0, sizeof(ui_default_Dynamic_0));
}

void _ui_update_default_Dynamic_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_0, sizeof(ui_default_Dynamic_0));
}

void _ui_remove_default_Dynamic_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_0, sizeof(ui_default_Dynamic_0));
}
