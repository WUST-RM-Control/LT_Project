//
// Created by RM UI Designer
//

#include "ui_default_Dynamic_2.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 14
#define OBJ_NUM 4
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_Dynamic_2;
ui_interface_number_t *ui_default_Dynamic_Speed = (ui_interface_number_t *)&(ui_default_Dynamic_2.data[0]);
ui_interface_rect_t *ui_default_Dynamic_Aim = (ui_interface_rect_t *)&(ui_default_Dynamic_2.data[1]);
ui_interface_number_t *ui_default_Dynamic_SpeedNum = (ui_interface_number_t *)&(ui_default_Dynamic_2.data[2]);
ui_interface_number_t *ui_default_Dynamic_LegLength = (ui_interface_number_t *)&(ui_default_Dynamic_2.data[3]);

void _ui_init_default_Dynamic_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_2.data[i].figure_name[0] = FRAME_ID;
        ui_default_Dynamic_2.data[i].figure_name[1] = GROUP_ID;
        ui_default_Dynamic_2.data[i].figure_name[2] = i + START_ID;
        ui_default_Dynamic_2.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_Dynamic_2.data[i].operate_tpyel = 0;
    }

    ui_default_Dynamic_Speed->figure_tpye = 6;
    ui_default_Dynamic_Speed->layer = 0;
    ui_default_Dynamic_Speed->font_size = 30;
    ui_default_Dynamic_Speed->start_x = 1622;
    ui_default_Dynamic_Speed->start_y = 610;
    ui_default_Dynamic_Speed->color = 6;
    ui_default_Dynamic_Speed->number = 1;
    ui_default_Dynamic_Speed->width = 3;

    ui_default_Dynamic_Aim->figure_tpye = 1;
    ui_default_Dynamic_Aim->layer = 0;
    ui_default_Dynamic_Aim->start_x = 911;
    ui_default_Dynamic_Aim->start_y = 686;
    ui_default_Dynamic_Aim->color = 5;
    ui_default_Dynamic_Aim->width = 2;
    ui_default_Dynamic_Aim->end_x = 1011;
    ui_default_Dynamic_Aim->end_y = 786;

	ui_default_Dynamic_SpeedNum->figure_tpye = 6;
	ui_default_Dynamic_SpeedNum->layer = 0;
	ui_default_Dynamic_SpeedNum->font_size = 20;
	ui_default_Dynamic_SpeedNum->start_x = 1745;
	ui_default_Dynamic_SpeedNum->start_y = 412;
	ui_default_Dynamic_SpeedNum->color = 6;
	ui_default_Dynamic_SpeedNum->number = 10;
	ui_default_Dynamic_SpeedNum->width = 2;
	
	ui_default_Dynamic_LegLength->figure_tpye = 6;
	ui_default_Dynamic_LegLength->layer = 0;
	ui_default_Dynamic_LegLength->font_size = 20;
	ui_default_Dynamic_LegLength->start_x = 1746;
	ui_default_Dynamic_LegLength->start_y = 351;
	ui_default_Dynamic_LegLength->color = 6;
	ui_default_Dynamic_LegLength->number = 20;
	ui_default_Dynamic_LegLength->width = 2;

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_2, sizeof(ui_default_Dynamic_2));
}

void _ui_update_default_Dynamic_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_2.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_2, sizeof(ui_default_Dynamic_2));
}

void _ui_remove_default_Dynamic_2() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_Dynamic_2.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_default_Dynamic_2, sizeof(ui_default_Dynamic_2));
}
