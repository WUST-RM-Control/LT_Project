//
// Created by RM UI Designer
//

#include "ui_default_Dynamic_1.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 7
#define OBJ_NUM 7
#define FRAME_OBJ_NUM 7

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Dynamic_1;
ui_interface_arc_t *ui_default_Dynamic_Fric_Second_M = (ui_interface_arc_t *)&(ui_default_Dynamic_1.data[0]);
ui_interface_arc_t *ui_default_Dynamic_Fric_Second_L = (ui_interface_arc_t *)&(ui_default_Dynamic_1.data[1]);
ui_interface_arc_t *ui_default_Dynamic_Chassis_1 = (ui_interface_arc_t *)&(ui_default_Dynamic_1.data[2]);
ui_interface_arc_t *ui_default_Dynamic_Chassis_3 = (ui_interface_arc_t *)&(ui_default_Dynamic_1.data[3]);
ui_interface_arc_t *ui_default_Dynamic_Chassis_2 = (ui_interface_arc_t *)&(ui_default_Dynamic_1.data[4]);
ui_interface_arc_t *ui_default_Dynamic_Chassis_4 = (ui_interface_arc_t *)&(ui_default_Dynamic_1.data[5]);
ui_interface_line_t *ui_default_Dynamic_Armor = (ui_interface_line_t *)&(ui_default_Dynamic_1.data[6]);

void _ui_init_default_Dynamic_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_1.data[i].figure_name[0] = FRAME_ID;
        ui_default_Dynamic_1.data[i].figure_name[1] = GROUP_ID;
        ui_default_Dynamic_1.data[i].figure_name[2] = i + START_ID;
        ui_default_Dynamic_1.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Dynamic_1.data[i].operate_tpyel = 0;
    }

    ui_default_Dynamic_Fric_Second_M->figure_tpye = 4;
    ui_default_Dynamic_Fric_Second_M->layer = 0;
    ui_default_Dynamic_Fric_Second_M->rx = 21;
    ui_default_Dynamic_Fric_Second_M->ry = 21;
    ui_default_Dynamic_Fric_Second_M->start_x = 1633;
    ui_default_Dynamic_Fric_Second_M->start_y = 660;
    ui_default_Dynamic_Fric_Second_M->color = 6;
    ui_default_Dynamic_Fric_Second_M->width = 5;
    ui_default_Dynamic_Fric_Second_M->start_angle = 30;
    ui_default_Dynamic_Fric_Second_M->end_angle = 330;

    ui_default_Dynamic_Fric_Second_L->figure_tpye = 4;
    ui_default_Dynamic_Fric_Second_L->layer = 0;
    ui_default_Dynamic_Fric_Second_L->rx = 21;
    ui_default_Dynamic_Fric_Second_L->ry = 21;
    ui_default_Dynamic_Fric_Second_L->start_x = 1596;
    ui_default_Dynamic_Fric_Second_L->start_y = 724;
    ui_default_Dynamic_Fric_Second_L->color = 6;
    ui_default_Dynamic_Fric_Second_L->width = 5;
    ui_default_Dynamic_Fric_Second_L->start_angle = 30;
    ui_default_Dynamic_Fric_Second_L->end_angle = 330;

    ui_default_Dynamic_Chassis_1->figure_tpye = 4;
    ui_default_Dynamic_Chassis_1->layer = 0;
    ui_default_Dynamic_Chassis_1->rx = 21;
    ui_default_Dynamic_Chassis_1->ry = 21;
    ui_default_Dynamic_Chassis_1->start_x = 1544;
    ui_default_Dynamic_Chassis_1->start_y = 771;
    ui_default_Dynamic_Chassis_1->color = 6;
    ui_default_Dynamic_Chassis_1->width = 5;
    ui_default_Dynamic_Chassis_1->start_angle = 30;
    ui_default_Dynamic_Chassis_1->end_angle = 330;

    ui_default_Dynamic_Chassis_3->figure_tpye = 4;
    ui_default_Dynamic_Chassis_3->layer = 0;
    ui_default_Dynamic_Chassis_3->rx = 21;
    ui_default_Dynamic_Chassis_3->ry = 21;
    ui_default_Dynamic_Chassis_3->start_x = 1544;
    ui_default_Dynamic_Chassis_3->start_y = 633;
    ui_default_Dynamic_Chassis_3->color = 6;
    ui_default_Dynamic_Chassis_3->width = 5;
    ui_default_Dynamic_Chassis_3->start_angle = 30;
    ui_default_Dynamic_Chassis_3->end_angle = 330;

    ui_default_Dynamic_Chassis_2->figure_tpye = 4;
    ui_default_Dynamic_Chassis_2->layer = 0;
    ui_default_Dynamic_Chassis_2->rx = 21;
    ui_default_Dynamic_Chassis_2->ry = 21;
    ui_default_Dynamic_Chassis_2->start_x = 1720;
    ui_default_Dynamic_Chassis_2->start_y = 770;
    ui_default_Dynamic_Chassis_2->color = 6;
    ui_default_Dynamic_Chassis_2->width = 5;
    ui_default_Dynamic_Chassis_2->start_angle = 30;
    ui_default_Dynamic_Chassis_2->end_angle = 330;

    ui_default_Dynamic_Chassis_4->figure_tpye = 4;
    ui_default_Dynamic_Chassis_4->layer = 0;
    ui_default_Dynamic_Chassis_4->rx = 21;
    ui_default_Dynamic_Chassis_4->ry = 21;
    ui_default_Dynamic_Chassis_4->start_x = 1720;
    ui_default_Dynamic_Chassis_4->start_y = 633;
    ui_default_Dynamic_Chassis_4->color = 6;
    ui_default_Dynamic_Chassis_4->width = 5;
    ui_default_Dynamic_Chassis_4->start_angle = 30;
    ui_default_Dynamic_Chassis_4->end_angle = 330;

    ui_default_Dynamic_Armor->figure_tpye = 0;
    ui_default_Dynamic_Armor->layer = 0;
    ui_default_Dynamic_Armor->start_x = 1600;
    ui_default_Dynamic_Armor->start_y = 848;
    ui_default_Dynamic_Armor->end_x = 1675;
    ui_default_Dynamic_Armor->end_y = 848;
    ui_default_Dynamic_Armor->color = 6;
    ui_default_Dynamic_Armor->width = 5;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_1, sizeof(ui_default_Dynamic_1));
}

void _ui_update_default_Dynamic_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_1.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_1, sizeof(ui_default_Dynamic_1));
}

void _ui_remove_default_Dynamic_1() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_1.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_1, sizeof(ui_default_Dynamic_1));
}
