//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_Dynamic_0;

ui_interface_arc_t *ui_g_Dynamic_SuperCap = (ui_interface_arc_t*)&(ui_g_Dynamic_0.data[0]);
ui_interface_arc_t *ui_g_Dynamic_Pitch = (ui_interface_arc_t*)&(ui_g_Dynamic_0.data[1]);
ui_interface_number_t *ui_g_Dynamic_Pitch_Num = (ui_interface_number_t*)&(ui_g_Dynamic_0.data[2]);
ui_interface_arc_t *ui_g_Dynamic_Fric_First_L = (ui_interface_arc_t*)&(ui_g_Dynamic_0.data[3]);
ui_interface_arc_t *ui_g_Dynamic_Fric_First_R = (ui_interface_arc_t*)&(ui_g_Dynamic_0.data[4]);
ui_interface_arc_t *ui_g_Dynamic_Chassis_1 = (ui_interface_arc_t*)&(ui_g_Dynamic_0.data[5]);
ui_interface_arc_t *ui_g_Dynamic_Chassis_3 = (ui_interface_arc_t*)&(ui_g_Dynamic_0.data[6]);

void _ui_init_g_Dynamic_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_0.data[i].figure_name[0] = 0;
        ui_g_Dynamic_0.data[i].figure_name[1] = 0;
        ui_g_Dynamic_0.data[i].figure_name[2] = i + 0;
        ui_g_Dynamic_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 0;
    }

    ui_g_Dynamic_SuperCap->figure_type = 4;
    ui_g_Dynamic_SuperCap->operate_type = 1;
    ui_g_Dynamic_SuperCap->layer = 0;
    ui_g_Dynamic_SuperCap->color = 6;
    ui_g_Dynamic_SuperCap->start_x = 960;
    ui_g_Dynamic_SuperCap->start_y = 540;
    ui_g_Dynamic_SuperCap->width = 16;
    ui_g_Dynamic_SuperCap->start_angle = 50;
    ui_g_Dynamic_SuperCap->end_angle = 130;
    ui_g_Dynamic_SuperCap->rx = 380;
    ui_g_Dynamic_SuperCap->ry = 380;

    ui_g_Dynamic_Pitch->figure_type = 4;
    ui_g_Dynamic_Pitch->operate_type = 1;
    ui_g_Dynamic_Pitch->layer = 0;
    ui_g_Dynamic_Pitch->color = 6;
    ui_g_Dynamic_Pitch->start_x = 960;
    ui_g_Dynamic_Pitch->start_y = 540;
    ui_g_Dynamic_Pitch->width = 16;
    ui_g_Dynamic_Pitch->start_angle = 269;
    ui_g_Dynamic_Pitch->end_angle = 271;
    ui_g_Dynamic_Pitch->rx = 380;
    ui_g_Dynamic_Pitch->ry = 380;

    ui_g_Dynamic_Pitch_Num->figure_type = 6;
    ui_g_Dynamic_Pitch_Num->operate_type = 1;
    ui_g_Dynamic_Pitch_Num->layer = 0;
    ui_g_Dynamic_Pitch_Num->color = 6;
    ui_g_Dynamic_Pitch_Num->start_x = 482;
    ui_g_Dynamic_Pitch_Num->start_y = 640;
    ui_g_Dynamic_Pitch_Num->width = 3;
    ui_g_Dynamic_Pitch_Num->font_size = 30;
    ui_g_Dynamic_Pitch_Num->number = 45;

    ui_g_Dynamic_Fric_First_L->figure_type = 4;
    ui_g_Dynamic_Fric_First_L->operate_type = 1;
    ui_g_Dynamic_Fric_First_L->layer = 0;
    ui_g_Dynamic_Fric_First_L->color = 6;
    ui_g_Dynamic_Fric_First_L->start_x = 1586;
    ui_g_Dynamic_Fric_First_L->start_y = 677;
    ui_g_Dynamic_Fric_First_L->width = 5;
    ui_g_Dynamic_Fric_First_L->start_angle = 30;
    ui_g_Dynamic_Fric_First_L->end_angle = 330;
    ui_g_Dynamic_Fric_First_L->rx = 21;
    ui_g_Dynamic_Fric_First_L->ry = 21;

    ui_g_Dynamic_Fric_First_R->figure_type = 4;
    ui_g_Dynamic_Fric_First_R->operate_type = 1;
    ui_g_Dynamic_Fric_First_R->layer = 0;
    ui_g_Dynamic_Fric_First_R->color = 6;
    ui_g_Dynamic_Fric_First_R->start_x = 1682;
    ui_g_Dynamic_Fric_First_R->start_y = 672;
    ui_g_Dynamic_Fric_First_R->width = 5;
    ui_g_Dynamic_Fric_First_R->start_angle = 30;
    ui_g_Dynamic_Fric_First_R->end_angle = 330;
    ui_g_Dynamic_Fric_First_R->rx = 21;
    ui_g_Dynamic_Fric_First_R->ry = 21;

    ui_g_Dynamic_Chassis_1->figure_type = 4;
    ui_g_Dynamic_Chassis_1->operate_type = 1;
    ui_g_Dynamic_Chassis_1->layer = 0;
    ui_g_Dynamic_Chassis_1->color = 6;
    ui_g_Dynamic_Chassis_1->start_x = 1544;
    ui_g_Dynamic_Chassis_1->start_y = 771;
    ui_g_Dynamic_Chassis_1->width = 5;
    ui_g_Dynamic_Chassis_1->start_angle = 30;
    ui_g_Dynamic_Chassis_1->end_angle = 330;
    ui_g_Dynamic_Chassis_1->rx = 21;
    ui_g_Dynamic_Chassis_1->ry = 21;

    ui_g_Dynamic_Chassis_3->figure_type = 4;
    ui_g_Dynamic_Chassis_3->operate_type = 1;
    ui_g_Dynamic_Chassis_3->layer = 0;
    ui_g_Dynamic_Chassis_3->color = 6;
    ui_g_Dynamic_Chassis_3->start_x = 1544;
    ui_g_Dynamic_Chassis_3->start_y = 633;
    ui_g_Dynamic_Chassis_3->width = 5;
    ui_g_Dynamic_Chassis_3->start_angle = 30;
    ui_g_Dynamic_Chassis_3->end_angle = 330;
    ui_g_Dynamic_Chassis_3->rx = 21;
    ui_g_Dynamic_Chassis_3->ry = 21;


    ui_proc_7_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}

void _ui_update_g_Dynamic_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}

void _ui_remove_g_Dynamic_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Dynamic_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_0, sizeof(ui_g_Dynamic_0));
}
ui_7_frame_t ui_g_Dynamic_1;

ui_interface_arc_t *ui_g_Dynamic_Chassis_2 = (ui_interface_arc_t*)&(ui_g_Dynamic_1.data[0]);
ui_interface_arc_t *ui_g_Dynamic_Chassis_4 = (ui_interface_arc_t*)&(ui_g_Dynamic_1.data[1]);
ui_interface_line_t *ui_g_Dynamic_Armor = (ui_interface_line_t*)&(ui_g_Dynamic_1.data[2]);
ui_interface_number_t *ui_g_Dynamic_Speed = (ui_interface_number_t*)&(ui_g_Dynamic_1.data[3]);
ui_interface_rect_t *ui_g_Dynamic_Aim = (ui_interface_rect_t*)&(ui_g_Dynamic_1.data[4]);
ui_interface_line_t *ui_g_Dynamic_Gimbal1 = (ui_interface_line_t*)&(ui_g_Dynamic_1.data[5]);
ui_interface_line_t *ui_g_Dynamic_Gimbal2 = (ui_interface_line_t*)&(ui_g_Dynamic_1.data[6]);

void _ui_init_g_Dynamic_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_1.data[i].figure_name[0] = 0;
        ui_g_Dynamic_1.data[i].figure_name[1] = 0;
        ui_g_Dynamic_1.data[i].figure_name[2] = i + 7;
        ui_g_Dynamic_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Dynamic_1.data[i].operate_type = 0;
    }

    ui_g_Dynamic_Chassis_2->figure_type = 4;
    ui_g_Dynamic_Chassis_2->operate_type = 1;
    ui_g_Dynamic_Chassis_2->layer = 0;
    ui_g_Dynamic_Chassis_2->color = 6;
    ui_g_Dynamic_Chassis_2->start_x = 1720;
    ui_g_Dynamic_Chassis_2->start_y = 770;
    ui_g_Dynamic_Chassis_2->width = 5;
    ui_g_Dynamic_Chassis_2->start_angle = 30;
    ui_g_Dynamic_Chassis_2->end_angle = 330;
    ui_g_Dynamic_Chassis_2->rx = 21;
    ui_g_Dynamic_Chassis_2->ry = 21;

    ui_g_Dynamic_Chassis_4->figure_type = 4;
    ui_g_Dynamic_Chassis_4->operate_type = 1;
    ui_g_Dynamic_Chassis_4->layer = 0;
    ui_g_Dynamic_Chassis_4->color = 6;
    ui_g_Dynamic_Chassis_4->start_x = 1720;
    ui_g_Dynamic_Chassis_4->start_y = 633;
    ui_g_Dynamic_Chassis_4->width = 5;
    ui_g_Dynamic_Chassis_4->start_angle = 30;
    ui_g_Dynamic_Chassis_4->end_angle = 330;
    ui_g_Dynamic_Chassis_4->rx = 21;
    ui_g_Dynamic_Chassis_4->ry = 21;

    ui_g_Dynamic_Armor->figure_type = 0;
    ui_g_Dynamic_Armor->operate_type = 1;
    ui_g_Dynamic_Armor->layer = 0;
    ui_g_Dynamic_Armor->color = 6;
    ui_g_Dynamic_Armor->start_x = 1600;
    ui_g_Dynamic_Armor->start_y = 848;
    ui_g_Dynamic_Armor->width = 5;
    ui_g_Dynamic_Armor->end_x = 1675;
    ui_g_Dynamic_Armor->end_y = 848;

    ui_g_Dynamic_Speed->figure_type = 6;
    ui_g_Dynamic_Speed->operate_type = 1;
    ui_g_Dynamic_Speed->layer = 0;
    ui_g_Dynamic_Speed->color = 6;
    ui_g_Dynamic_Speed->start_x = 1622;
    ui_g_Dynamic_Speed->start_y = 610;
    ui_g_Dynamic_Speed->width = 3;
    ui_g_Dynamic_Speed->font_size = 30;
    ui_g_Dynamic_Speed->number = 1;

    ui_g_Dynamic_Aim->figure_type = 1;
    ui_g_Dynamic_Aim->operate_type = 1;
    ui_g_Dynamic_Aim->layer = 0;
    ui_g_Dynamic_Aim->color = 5;
    ui_g_Dynamic_Aim->start_x = 911;
    ui_g_Dynamic_Aim->start_y = 686;
    ui_g_Dynamic_Aim->width = 5;
    ui_g_Dynamic_Aim->end_x = 1011;
    ui_g_Dynamic_Aim->end_y = 786;

    ui_g_Dynamic_Gimbal1->figure_type = 0;
    ui_g_Dynamic_Gimbal1->operate_type = 1;
    ui_g_Dynamic_Gimbal1->layer = 0;
    ui_g_Dynamic_Gimbal1->color = 6;
    ui_g_Dynamic_Gimbal1->start_x = 238;
    ui_g_Dynamic_Gimbal1->start_y = 786;
    ui_g_Dynamic_Gimbal1->width = 3;
    ui_g_Dynamic_Gimbal1->end_x = 238;
    ui_g_Dynamic_Gimbal1->end_y = 786;

    ui_g_Dynamic_Gimbal2->figure_type = 0;
    ui_g_Dynamic_Gimbal2->operate_type = 1;
    ui_g_Dynamic_Gimbal2->layer = 0;
    ui_g_Dynamic_Gimbal2->color = 6;
    ui_g_Dynamic_Gimbal2->start_x = 158;
    ui_g_Dynamic_Gimbal2->start_y = 816;
    ui_g_Dynamic_Gimbal2->width = 3;
    ui_g_Dynamic_Gimbal2->end_x = 261;
    ui_g_Dynamic_Gimbal2->end_y = 817;


    ui_proc_7_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}

void _ui_update_g_Dynamic_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_1.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}

void _ui_remove_g_Dynamic_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_Dynamic_1.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Dynamic_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_1, sizeof(ui_g_Dynamic_1));
}
ui_1_frame_t ui_g_Dynamic_2;

ui_interface_line_t *ui_g_Dynamic_Trigger = (ui_interface_line_t*)&(ui_g_Dynamic_2.data[0]);

void _ui_init_g_Dynamic_2() {
    for (int i = 0; i < 1; i++) {
        ui_g_Dynamic_2.data[i].figure_name[0] = 0;
        ui_g_Dynamic_2.data[i].figure_name[1] = 0;
        ui_g_Dynamic_2.data[i].figure_name[2] = i + 14;
        ui_g_Dynamic_2.data[i].operate_type = 1;
    }
    for (int i = 1; i < 1; i++) {
        ui_g_Dynamic_2.data[i].operate_type = 0;
    }

    ui_g_Dynamic_Trigger->figure_type = 0;
    ui_g_Dynamic_Trigger->operate_type = 1;
    ui_g_Dynamic_Trigger->layer = 0;
    ui_g_Dynamic_Trigger->color = 6;
    ui_g_Dynamic_Trigger->start_x = 187;
    ui_g_Dynamic_Trigger->start_y = 648;
    ui_g_Dynamic_Trigger->width = 3;
    ui_g_Dynamic_Trigger->end_x = 187;
    ui_g_Dynamic_Trigger->end_y = 648;


    ui_proc_1_frame(&ui_g_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_2, sizeof(ui_g_Dynamic_2));
}

void _ui_update_g_Dynamic_2() {
    for (int i = 0; i < 1; i++) {
        ui_g_Dynamic_2.data[i].operate_type = 2;
    }

    ui_proc_1_frame(&ui_g_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_2, sizeof(ui_g_Dynamic_2));
}

void _ui_remove_g_Dynamic_2() {
    for (int i = 0; i < 1; i++) {
        ui_g_Dynamic_2.data[i].operate_type = 3;
    }

    ui_proc_1_frame(&ui_g_Dynamic_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_2, sizeof(ui_g_Dynamic_2));
}

ui_string_frame_t ui_g_Dynamic_3;
ui_interface_string_t* ui_g_Dynamic_Yaw_Offset = &(ui_g_Dynamic_3.option);

void _ui_init_g_Dynamic_3() {
    ui_g_Dynamic_3.option.figure_name[0] = 0;
    ui_g_Dynamic_3.option.figure_name[1] = 0;
    ui_g_Dynamic_3.option.figure_name[2] = 15;
    ui_g_Dynamic_3.option.operate_type = 1;

    ui_g_Dynamic_Yaw_Offset->figure_type = 7;
    ui_g_Dynamic_Yaw_Offset->operate_type = 1;
    ui_g_Dynamic_Yaw_Offset->layer = 0;
    ui_g_Dynamic_Yaw_Offset->color = 6;
    ui_g_Dynamic_Yaw_Offset->start_x = 1142;
    ui_g_Dynamic_Yaw_Offset->start_y = 534;
    ui_g_Dynamic_Yaw_Offset->width = 2;
    ui_g_Dynamic_Yaw_Offset->font_size = 20;
    ui_g_Dynamic_Yaw_Offset->str_length = 6;
    strcpy(ui_g_Dynamic_Yaw_Offset->string, "Y:+0.1");


    ui_proc_string_frame(&ui_g_Dynamic_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_3, sizeof(ui_g_Dynamic_3));
}

void _ui_update_g_Dynamic_3() {
    ui_g_Dynamic_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Dynamic_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_3, sizeof(ui_g_Dynamic_3));
}

void _ui_remove_g_Dynamic_3() {
    ui_g_Dynamic_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Dynamic_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_3, sizeof(ui_g_Dynamic_3));
}
ui_string_frame_t ui_g_Dynamic_4;
ui_interface_string_t* ui_g_Dynamic_Pitch_Offset = &(ui_g_Dynamic_4.option);

void _ui_init_g_Dynamic_4() {
    ui_g_Dynamic_4.option.figure_name[0] = 0;
    ui_g_Dynamic_4.option.figure_name[1] = 0;
    ui_g_Dynamic_4.option.figure_name[2] = 16;
    ui_g_Dynamic_4.option.operate_type = 1;

    ui_g_Dynamic_Pitch_Offset->figure_type = 7;
    ui_g_Dynamic_Pitch_Offset->operate_type = 1;
    ui_g_Dynamic_Pitch_Offset->layer = 0;
    ui_g_Dynamic_Pitch_Offset->color = 6;
    ui_g_Dynamic_Pitch_Offset->start_x = 1142;
    ui_g_Dynamic_Pitch_Offset->start_y = 505;
    ui_g_Dynamic_Pitch_Offset->width = 2;
    ui_g_Dynamic_Pitch_Offset->font_size = 20;
    ui_g_Dynamic_Pitch_Offset->str_length = 6;
    strcpy(ui_g_Dynamic_Pitch_Offset->string, "P:+0.1");


    ui_proc_string_frame(&ui_g_Dynamic_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_4, sizeof(ui_g_Dynamic_4));
}

void _ui_update_g_Dynamic_4() {
    ui_g_Dynamic_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Dynamic_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_4, sizeof(ui_g_Dynamic_4));
}

void _ui_remove_g_Dynamic_4() {
    ui_g_Dynamic_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Dynamic_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Dynamic_4, sizeof(ui_g_Dynamic_4));
}

void ui_init_g_Dynamic() {
    _ui_init_g_Dynamic_0();
    _ui_init_g_Dynamic_1();
    _ui_init_g_Dynamic_2();
    _ui_init_g_Dynamic_3();
    _ui_init_g_Dynamic_4();
}

void ui_update_g_Dynamic() {
    _ui_update_g_Dynamic_0();
    _ui_update_g_Dynamic_1();
    _ui_update_g_Dynamic_2();
    _ui_update_g_Dynamic_3();
    _ui_update_g_Dynamic_4();
}

void ui_remove_g_Dynamic() {
    _ui_remove_g_Dynamic_0();
    _ui_remove_g_Dynamic_1();
    _ui_remove_g_Dynamic_2();
    _ui_remove_g_Dynamic_3();
    _ui_remove_g_Dynamic_4();
}

ui_7_frame_t ui_g_Static_0;

ui_interface_line_t *ui_g_Static_Star_6 = (ui_interface_line_t*)&(ui_g_Static_0.data[0]);
ui_interface_line_t *ui_g_Static_Star_7 = (ui_interface_line_t*)&(ui_g_Static_0.data[1]);
ui_interface_arc_t *ui_g_Static_SuperCap_1 = (ui_interface_arc_t*)&(ui_g_Static_0.data[2]);
ui_interface_arc_t *ui_g_Static_SuperCap_2 = (ui_interface_arc_t*)&(ui_g_Static_0.data[3]);
ui_interface_arc_t *ui_g_Static_SuperCap_3 = (ui_interface_arc_t*)&(ui_g_Static_0.data[4]);
ui_interface_arc_t *ui_g_Static_SuperCap_4 = (ui_interface_arc_t*)&(ui_g_Static_0.data[5]);
ui_interface_arc_t *ui_g_Static_SuperCap_5 = (ui_interface_arc_t*)&(ui_g_Static_0.data[6]);

void _ui_init_g_Static_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_0.data[i].figure_name[0] = 0;
        ui_g_Static_0.data[i].figure_name[1] = 1;
        ui_g_Static_0.data[i].figure_name[2] = i + 0;
        ui_g_Static_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Static_0.data[i].operate_type = 0;
    }

    ui_g_Static_Star_6->figure_type = 0;
    ui_g_Static_Star_6->operate_type = 1;
    ui_g_Static_Star_6->layer = 0;
    ui_g_Static_Star_6->color = 8;
    ui_g_Static_Star_6->start_x = 710;
    ui_g_Static_Star_6->start_y = 340;
    ui_g_Static_Star_6->width = 1;
    ui_g_Static_Star_6->end_x = 1208;
    ui_g_Static_Star_6->end_y = 340;

    ui_g_Static_Star_7->figure_type = 0;
    ui_g_Static_Star_7->operate_type = 1;
    ui_g_Static_Star_7->layer = 0;
    ui_g_Static_Star_7->color = 8;
    ui_g_Static_Star_7->start_x = 960;
    ui_g_Static_Star_7->start_y = 599;
    ui_g_Static_Star_7->width = 1;
    ui_g_Static_Star_7->end_x = 960;
    ui_g_Static_Star_7->end_y = 250;

    ui_g_Static_SuperCap_1->figure_type = 4;
    ui_g_Static_SuperCap_1->operate_type = 1;
    ui_g_Static_SuperCap_1->layer = 0;
    ui_g_Static_SuperCap_1->color = 8;
    ui_g_Static_SuperCap_1->start_x = 960;
    ui_g_Static_SuperCap_1->start_y = 540;
    ui_g_Static_SuperCap_1->width = 16;
    ui_g_Static_SuperCap_1->start_angle = 49;
    ui_g_Static_SuperCap_1->end_angle = 50;
    ui_g_Static_SuperCap_1->rx = 380;
    ui_g_Static_SuperCap_1->ry = 380;

    ui_g_Static_SuperCap_2->figure_type = 4;
    ui_g_Static_SuperCap_2->operate_type = 1;
    ui_g_Static_SuperCap_2->layer = 0;
    ui_g_Static_SuperCap_2->color = 8;
    ui_g_Static_SuperCap_2->start_x = 960;
    ui_g_Static_SuperCap_2->start_y = 540;
    ui_g_Static_SuperCap_2->width = 16;
    ui_g_Static_SuperCap_2->start_angle = 130;
    ui_g_Static_SuperCap_2->end_angle = 131;
    ui_g_Static_SuperCap_2->rx = 380;
    ui_g_Static_SuperCap_2->ry = 380;

    ui_g_Static_SuperCap_3->figure_type = 4;
    ui_g_Static_SuperCap_3->operate_type = 1;
    ui_g_Static_SuperCap_3->layer = 0;
    ui_g_Static_SuperCap_3->color = 8;
    ui_g_Static_SuperCap_3->start_x = 960;
    ui_g_Static_SuperCap_3->start_y = 540;
    ui_g_Static_SuperCap_3->width = 16;
    ui_g_Static_SuperCap_3->start_angle = 90;
    ui_g_Static_SuperCap_3->end_angle = 91;
    ui_g_Static_SuperCap_3->rx = 360;
    ui_g_Static_SuperCap_3->ry = 360;

    ui_g_Static_SuperCap_4->figure_type = 4;
    ui_g_Static_SuperCap_4->operate_type = 1;
    ui_g_Static_SuperCap_4->layer = 0;
    ui_g_Static_SuperCap_4->color = 8;
    ui_g_Static_SuperCap_4->start_x = 960;
    ui_g_Static_SuperCap_4->start_y = 540;
    ui_g_Static_SuperCap_4->width = 16;
    ui_g_Static_SuperCap_4->start_angle = 115;
    ui_g_Static_SuperCap_4->end_angle = 116;
    ui_g_Static_SuperCap_4->rx = 360;
    ui_g_Static_SuperCap_4->ry = 360;

    ui_g_Static_SuperCap_5->figure_type = 4;
    ui_g_Static_SuperCap_5->operate_type = 1;
    ui_g_Static_SuperCap_5->layer = 0;
    ui_g_Static_SuperCap_5->color = 8;
    ui_g_Static_SuperCap_5->start_x = 960;
    ui_g_Static_SuperCap_5->start_y = 540;
    ui_g_Static_SuperCap_5->width = 16;
    ui_g_Static_SuperCap_5->start_angle = 65;
    ui_g_Static_SuperCap_5->end_angle = 66;
    ui_g_Static_SuperCap_5->rx = 360;
    ui_g_Static_SuperCap_5->ry = 360;


    ui_proc_7_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

void _ui_update_g_Static_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}

void _ui_remove_g_Static_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Static_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_0, sizeof(ui_g_Static_0));
}
ui_7_frame_t ui_g_Static_1;

ui_interface_arc_t *ui_g_Static_Pitch_5 = (ui_interface_arc_t*)&(ui_g_Static_1.data[0]);
ui_interface_arc_t *ui_g_Static_Pitch_4 = (ui_interface_arc_t*)&(ui_g_Static_1.data[1]);
ui_interface_arc_t *ui_g_Static_Pitch_3 = (ui_interface_arc_t*)&(ui_g_Static_1.data[2]);
ui_interface_arc_t *ui_g_Static_Pitch_2 = (ui_interface_arc_t*)&(ui_g_Static_1.data[3]);
ui_interface_arc_t *ui_g_Static_Pitch_1 = (ui_interface_arc_t*)&(ui_g_Static_1.data[4]);
ui_interface_round_t *ui_g_Static_Star_Round = (ui_interface_round_t*)&(ui_g_Static_1.data[5]);
ui_interface_line_t *ui_g_Static_Star_1 = (ui_interface_line_t*)&(ui_g_Static_1.data[6]);

void _ui_init_g_Static_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_1.data[i].figure_name[0] = 0;
        ui_g_Static_1.data[i].figure_name[1] = 1;
        ui_g_Static_1.data[i].figure_name[2] = i + 7;
        ui_g_Static_1.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Static_1.data[i].operate_type = 0;
    }

    ui_g_Static_Pitch_5->figure_type = 4;
    ui_g_Static_Pitch_5->operate_type = 1;
    ui_g_Static_Pitch_5->layer = 0;
    ui_g_Static_Pitch_5->color = 8;
    ui_g_Static_Pitch_5->start_x = 960;
    ui_g_Static_Pitch_5->start_y = 540;
    ui_g_Static_Pitch_5->width = 16;
    ui_g_Static_Pitch_5->start_angle = 245;
    ui_g_Static_Pitch_5->end_angle = 246;
    ui_g_Static_Pitch_5->rx = 360;
    ui_g_Static_Pitch_5->ry = 360;

    ui_g_Static_Pitch_4->figure_type = 4;
    ui_g_Static_Pitch_4->operate_type = 1;
    ui_g_Static_Pitch_4->layer = 0;
    ui_g_Static_Pitch_4->color = 8;
    ui_g_Static_Pitch_4->start_x = 960;
    ui_g_Static_Pitch_4->start_y = 540;
    ui_g_Static_Pitch_4->width = 16;
    ui_g_Static_Pitch_4->start_angle = 295;
    ui_g_Static_Pitch_4->end_angle = 296;
    ui_g_Static_Pitch_4->rx = 360;
    ui_g_Static_Pitch_4->ry = 360;

    ui_g_Static_Pitch_3->figure_type = 4;
    ui_g_Static_Pitch_3->operate_type = 1;
    ui_g_Static_Pitch_3->layer = 0;
    ui_g_Static_Pitch_3->color = 8;
    ui_g_Static_Pitch_3->start_x = 960;
    ui_g_Static_Pitch_3->start_y = 540;
    ui_g_Static_Pitch_3->width = 16;
    ui_g_Static_Pitch_3->start_angle = 270;
    ui_g_Static_Pitch_3->end_angle = 271;
    ui_g_Static_Pitch_3->rx = 360;
    ui_g_Static_Pitch_3->ry = 360;

    ui_g_Static_Pitch_2->figure_type = 4;
    ui_g_Static_Pitch_2->operate_type = 1;
    ui_g_Static_Pitch_2->layer = 0;
    ui_g_Static_Pitch_2->color = 8;
    ui_g_Static_Pitch_2->start_x = 960;
    ui_g_Static_Pitch_2->start_y = 540;
    ui_g_Static_Pitch_2->width = 16;
    ui_g_Static_Pitch_2->start_angle = 310;
    ui_g_Static_Pitch_2->end_angle = 311;
    ui_g_Static_Pitch_2->rx = 380;
    ui_g_Static_Pitch_2->ry = 380;

    ui_g_Static_Pitch_1->figure_type = 4;
    ui_g_Static_Pitch_1->operate_type = 1;
    ui_g_Static_Pitch_1->layer = 0;
    ui_g_Static_Pitch_1->color = 8;
    ui_g_Static_Pitch_1->start_x = 960;
    ui_g_Static_Pitch_1->start_y = 540;
    ui_g_Static_Pitch_1->width = 16;
    ui_g_Static_Pitch_1->start_angle = 229;
    ui_g_Static_Pitch_1->end_angle = 230;
    ui_g_Static_Pitch_1->rx = 380;
    ui_g_Static_Pitch_1->ry = 380;

    ui_g_Static_Star_Round->figure_type = 2;
    ui_g_Static_Star_Round->operate_type = 1;
    ui_g_Static_Star_Round->layer = 0;
    ui_g_Static_Star_Round->color = 8;
    ui_g_Static_Star_Round->start_x = 959;
    ui_g_Static_Star_Round->start_y = 541;
    ui_g_Static_Star_Round->width = 2;
    ui_g_Static_Star_Round->r = 20;

    ui_g_Static_Star_1->figure_type = 0;
    ui_g_Static_Star_1->operate_type = 1;
    ui_g_Static_Star_1->layer = 0;
    ui_g_Static_Star_1->color = 8;
    ui_g_Static_Star_1->start_x = 660;
    ui_g_Static_Star_1->start_y = 539;
    ui_g_Static_Star_1->width = 3;
    ui_g_Static_Star_1->end_x = 1262;
    ui_g_Static_Star_1->end_y = 541;


    ui_proc_7_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_update_g_Static_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_1.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}

void _ui_remove_g_Static_1() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_1.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Static_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_1, sizeof(ui_g_Static_1));
}
ui_7_frame_t ui_g_Static_2;

ui_interface_line_t *ui_g_Static_L1 = (ui_interface_line_t*)&(ui_g_Static_2.data[0]);
ui_interface_line_t *ui_g_Static_Star_2 = (ui_interface_line_t*)&(ui_g_Static_2.data[1]);
ui_interface_line_t *ui_g_Static_L2 = (ui_interface_line_t*)&(ui_g_Static_2.data[2]);
ui_interface_line_t *ui_g_Static_L3 = (ui_interface_line_t*)&(ui_g_Static_2.data[3]);
ui_interface_line_t *ui_g_Static_L4 = (ui_interface_line_t*)&(ui_g_Static_2.data[4]);
ui_interface_line_t *ui_g_Static_L5 = (ui_interface_line_t*)&(ui_g_Static_2.data[5]);
ui_interface_line_t *ui_g_Static_L6 = (ui_interface_line_t*)&(ui_g_Static_2.data[6]);

void _ui_init_g_Static_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_2.data[i].figure_name[0] = 0;
        ui_g_Static_2.data[i].figure_name[1] = 1;
        ui_g_Static_2.data[i].figure_name[2] = i + 14;
        ui_g_Static_2.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Static_2.data[i].operate_type = 0;
    }

    ui_g_Static_L1->figure_type = 0;
    ui_g_Static_L1->operate_type = 1;
    ui_g_Static_L1->layer = 0;
    ui_g_Static_L1->color = 8;
    ui_g_Static_L1->start_x = 1586;
    ui_g_Static_L1->start_y = 678;
    ui_g_Static_L1->width = 5;
    ui_g_Static_L1->end_x = 1634;
    ui_g_Static_L1->end_y = 705;

    ui_g_Static_Star_2->figure_type = 0;
    ui_g_Static_Star_2->operate_type = 1;
    ui_g_Static_Star_2->layer = 0;
    ui_g_Static_Star_2->color = 8;
    ui_g_Static_Star_2->start_x = 910;
    ui_g_Static_Star_2->start_y = 500;
    ui_g_Static_Star_2->width = 1;
    ui_g_Static_Star_2->end_x = 1010;
    ui_g_Static_Star_2->end_y = 500;

    ui_g_Static_L2->figure_type = 0;
    ui_g_Static_L2->operate_type = 1;
    ui_g_Static_L2->layer = 0;
    ui_g_Static_L2->color = 8;
    ui_g_Static_L2->start_x = 1682;
    ui_g_Static_L2->start_y = 673;
    ui_g_Static_L2->width = 5;
    ui_g_Static_L2->end_x = 1636;
    ui_g_Static_L2->end_y = 704;

    ui_g_Static_L3->figure_type = 0;
    ui_g_Static_L3->operate_type = 1;
    ui_g_Static_L3->layer = 0;
    ui_g_Static_L3->color = 8;
    ui_g_Static_L3->start_x = 1634;
    ui_g_Static_L3->start_y = 696;
    ui_g_Static_L3->width = 5;
    ui_g_Static_L3->end_x = 1635;
    ui_g_Static_L3->end_y = 758;

    ui_g_Static_L4->figure_type = 0;
    ui_g_Static_L4->operate_type = 1;
    ui_g_Static_L4->layer = 0;
    ui_g_Static_L4->color = 8;
    ui_g_Static_L4->start_x = 1633;
    ui_g_Static_L4->start_y = 705;
    ui_g_Static_L4->width = 5;
    ui_g_Static_L4->end_x = 1595;
    ui_g_Static_L4->end_y = 725;

    ui_g_Static_L5->figure_type = 0;
    ui_g_Static_L5->operate_type = 1;
    ui_g_Static_L5->layer = 0;
    ui_g_Static_L5->color = 8;
    ui_g_Static_L5->start_x = 1635;
    ui_g_Static_L5->start_y = 705;
    ui_g_Static_L5->width = 5;
    ui_g_Static_L5->end_x = 1673;
    ui_g_Static_L5->end_y = 722;

    ui_g_Static_L6->figure_type = 0;
    ui_g_Static_L6->operate_type = 1;
    ui_g_Static_L6->layer = 0;
    ui_g_Static_L6->color = 8;
    ui_g_Static_L6->start_x = 1633;
    ui_g_Static_L6->start_y = 662;
    ui_g_Static_L6->width = 5;
    ui_g_Static_L6->end_x = 1634;
    ui_g_Static_L6->end_y = 700;


    ui_proc_7_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_update_g_Static_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_2.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}

void _ui_remove_g_Static_2() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_2.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Static_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_2, sizeof(ui_g_Static_2));
}
ui_7_frame_t ui_g_Static_3;

ui_interface_round_t *ui_g_Static_R2 = (ui_interface_round_t*)&(ui_g_Static_3.data[0]);
ui_interface_round_t *ui_g_Static_R1 = (ui_interface_round_t*)&(ui_g_Static_3.data[1]);
ui_interface_line_t *ui_g_Static_Star_3 = (ui_interface_line_t*)&(ui_g_Static_3.data[2]);
ui_interface_line_t *ui_g_Static_Star_Offset = (ui_interface_line_t*)&(ui_g_Static_3.data[3]);
ui_interface_line_t *ui_g_Static_Body1 = (ui_interface_line_t*)&(ui_g_Static_3.data[4]);
ui_interface_line_t *ui_g_Static_Body2 = (ui_interface_line_t*)&(ui_g_Static_3.data[5]);
ui_interface_line_t *ui_g_Static_Body3 = (ui_interface_line_t*)&(ui_g_Static_3.data[6]);

void _ui_init_g_Static_3() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_3.data[i].figure_name[0] = 0;
        ui_g_Static_3.data[i].figure_name[1] = 1;
        ui_g_Static_3.data[i].figure_name[2] = i + 21;
        ui_g_Static_3.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_Static_3.data[i].operate_type = 0;
    }

    ui_g_Static_R2->figure_type = 2;
    ui_g_Static_R2->operate_type = 1;
    ui_g_Static_R2->layer = 0;
    ui_g_Static_R2->color = 8;
    ui_g_Static_R2->start_x = 1633;
    ui_g_Static_R2->start_y = 703;
    ui_g_Static_R2->width = 5;
    ui_g_Static_R2->r = 84;

    ui_g_Static_R1->figure_type = 2;
    ui_g_Static_R1->operate_type = 1;
    ui_g_Static_R1->layer = 0;
    ui_g_Static_R1->color = 8;
    ui_g_Static_R1->start_x = 1633;
    ui_g_Static_R1->start_y = 703;
    ui_g_Static_R1->width = 5;
    ui_g_Static_R1->r = 140;

    ui_g_Static_Star_3->figure_type = 0;
    ui_g_Static_Star_3->operate_type = 1;
    ui_g_Static_Star_3->layer = 0;
    ui_g_Static_Star_3->color = 8;
    ui_g_Static_Star_3->start_x = 860;
    ui_g_Static_Star_3->start_y = 460;
    ui_g_Static_Star_3->width = 1;
    ui_g_Static_Star_3->end_x = 1061;
    ui_g_Static_Star_3->end_y = 460;

    ui_g_Static_Star_Offset->figure_type = 0;
    ui_g_Static_Star_Offset->operate_type = 1;
    ui_g_Static_Star_Offset->layer = 0;
    ui_g_Static_Star_Offset->color = 8;
    ui_g_Static_Star_Offset->start_x = 860;
    ui_g_Static_Star_Offset->start_y = 440;
    ui_g_Static_Star_Offset->width = 1;
    ui_g_Static_Star_Offset->end_x = 860;
    ui_g_Static_Star_Offset->end_y = 408;

    ui_g_Static_Body1->figure_type = 0;
    ui_g_Static_Body1->operate_type = 1;
    ui_g_Static_Body1->layer = 0;
    ui_g_Static_Body1->color = 8;
    ui_g_Static_Body1->start_x = 107;
    ui_g_Static_Body1->start_y = 824;
    ui_g_Static_Body1->width = 3;
    ui_g_Static_Body1->end_x = 143;
    ui_g_Static_Body1->end_y = 824;

    ui_g_Static_Body2->figure_type = 0;
    ui_g_Static_Body2->operate_type = 1;
    ui_g_Static_Body2->layer = 0;
    ui_g_Static_Body2->color = 8;
    ui_g_Static_Body2->start_x = 139;
    ui_g_Static_Body2->start_y = 825;
    ui_g_Static_Body2->width = 3;
    ui_g_Static_Body2->end_x = 198;
    ui_g_Static_Body2->end_y = 785;

    ui_g_Static_Body3->figure_type = 0;
    ui_g_Static_Body3->operate_type = 1;
    ui_g_Static_Body3->layer = 0;
    ui_g_Static_Body3->color = 8;
    ui_g_Static_Body3->start_x = 197;
    ui_g_Static_Body3->start_y = 785;
    ui_g_Static_Body3->width = 3;
    ui_g_Static_Body3->end_x = 280;
    ui_g_Static_Body3->end_y = 783;


    ui_proc_7_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void _ui_update_g_Static_3() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_3.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}

void _ui_remove_g_Static_3() {
    for (int i = 0; i < 7; i++) {
        ui_g_Static_3.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Static_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_3, sizeof(ui_g_Static_3));
}
ui_7_frame_t ui_g_Static_4;

ui_interface_line_t *ui_g_Static_Body4 = (ui_interface_line_t*)&(ui_g_Static_4.data[0]);
ui_interface_line_t *ui_g_Static_Body5 = (ui_interface_line_t*)&(ui_g_Static_4.data[1]);
ui_interface_line_t *ui_g_Static_Body6 = (ui_interface_line_t*)&(ui_g_Static_4.data[2]);
ui_interface_line_t *ui_g_Static_Star_4 = (ui_interface_line_t*)&(ui_g_Static_4.data[3]);
ui_interface_round_t *ui_g_Static_Trigger_Round1 = (ui_interface_round_t*)&(ui_g_Static_4.data[4]);
ui_interface_line_t *ui_g_Static_Star_5 = (ui_interface_line_t*)&(ui_g_Static_4.data[5]);

void _ui_init_g_Static_4() {
    for (int i = 0; i < 6; i++) {
        ui_g_Static_4.data[i].figure_name[0] = 0;
        ui_g_Static_4.data[i].figure_name[1] = 1;
        ui_g_Static_4.data[i].figure_name[2] = i + 28;
        ui_g_Static_4.data[i].operate_type = 1;
    }
    for (int i = 6; i < 7; i++) {
        ui_g_Static_4.data[i].operate_type = 0;
    }

    ui_g_Static_Body4->figure_type = 0;
    ui_g_Static_Body4->operate_type = 1;
    ui_g_Static_Body4->layer = 0;
    ui_g_Static_Body4->color = 8;
    ui_g_Static_Body4->start_x = 107;
    ui_g_Static_Body4->start_y = 823;
    ui_g_Static_Body4->width = 3;
    ui_g_Static_Body4->end_x = 107;
    ui_g_Static_Body4->end_y = 751;

    ui_g_Static_Body5->figure_type = 0;
    ui_g_Static_Body5->operate_type = 1;
    ui_g_Static_Body5->layer = 0;
    ui_g_Static_Body5->color = 8;
    ui_g_Static_Body5->start_x = 105;
    ui_g_Static_Body5->start_y = 752;
    ui_g_Static_Body5->width = 3;
    ui_g_Static_Body5->end_x = 281;
    ui_g_Static_Body5->end_y = 753;

    ui_g_Static_Body6->figure_type = 0;
    ui_g_Static_Body6->operate_type = 1;
    ui_g_Static_Body6->layer = 0;
    ui_g_Static_Body6->color = 8;
    ui_g_Static_Body6->start_x = 278;
    ui_g_Static_Body6->start_y = 783;
    ui_g_Static_Body6->width = 3;
    ui_g_Static_Body6->end_x = 278;
    ui_g_Static_Body6->end_y = 753;

    ui_g_Static_Star_4->figure_type = 0;
    ui_g_Static_Star_4->operate_type = 1;
    ui_g_Static_Star_4->layer = 0;
    ui_g_Static_Star_4->color = 8;
    ui_g_Static_Star_4->start_x = 810;
    ui_g_Static_Star_4->start_y = 420;
    ui_g_Static_Star_4->width = 1;
    ui_g_Static_Star_4->end_x = 1110;
    ui_g_Static_Star_4->end_y = 420;

    ui_g_Static_Trigger_Round1->figure_type = 2;
    ui_g_Static_Trigger_Round1->operate_type = 1;
    ui_g_Static_Trigger_Round1->layer = 0;
    ui_g_Static_Trigger_Round1->color = 8;
    ui_g_Static_Trigger_Round1->start_x = 187;
    ui_g_Static_Trigger_Round1->start_y = 648;
    ui_g_Static_Trigger_Round1->width = 3;
    ui_g_Static_Trigger_Round1->r = 50;

    ui_g_Static_Star_5->figure_type = 0;
    ui_g_Static_Star_5->operate_type = 1;
    ui_g_Static_Star_5->layer = 0;
    ui_g_Static_Star_5->color = 8;
    ui_g_Static_Star_5->start_x = 760;
    ui_g_Static_Star_5->start_y = 380;
    ui_g_Static_Star_5->width = 1;
    ui_g_Static_Star_5->end_x = 1160;
    ui_g_Static_Star_5->end_y = 380;


    ui_proc_7_frame(&ui_g_Static_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_4, sizeof(ui_g_Static_4));
}

void _ui_update_g_Static_4() {
    for (int i = 0; i < 6; i++) {
        ui_g_Static_4.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_Static_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_4, sizeof(ui_g_Static_4));
}

void _ui_remove_g_Static_4() {
    for (int i = 0; i < 6; i++) {
        ui_g_Static_4.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_Static_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_4, sizeof(ui_g_Static_4));
}

ui_string_frame_t ui_g_Static_5;
ui_interface_string_t* ui_g_Static_WUST_RM = &(ui_g_Static_5.option);

void _ui_init_g_Static_5() {
    ui_g_Static_5.option.figure_name[0] = 0;
    ui_g_Static_5.option.figure_name[1] = 1;
    ui_g_Static_5.option.figure_name[2] = 34;
    ui_g_Static_5.option.operate_type = 1;

    ui_g_Static_WUST_RM->figure_type = 7;
    ui_g_Static_WUST_RM->operate_type = 1;
    ui_g_Static_WUST_RM->layer = 0;
    ui_g_Static_WUST_RM->color = 8;
    ui_g_Static_WUST_RM->start_x = 1592;
    ui_g_Static_WUST_RM->start_y = 551;
    ui_g_Static_WUST_RM->width = 4;
    ui_g_Static_WUST_RM->font_size = 40;
    ui_g_Static_WUST_RM->str_length = 7;
    strcpy(ui_g_Static_WUST_RM->string, "WUST_RM");


    ui_proc_string_frame(&ui_g_Static_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_5, sizeof(ui_g_Static_5));
}

void _ui_update_g_Static_5() {
    ui_g_Static_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_5, sizeof(ui_g_Static_5));
}

void _ui_remove_g_Static_5() {
    ui_g_Static_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_5, sizeof(ui_g_Static_5));
}
ui_string_frame_t ui_g_Static_6;
ui_interface_string_t* ui_g_Static_Kawshiro = &(ui_g_Static_6.option);

void _ui_init_g_Static_6() {
    ui_g_Static_6.option.figure_name[0] = 0;
    ui_g_Static_6.option.figure_name[1] = 1;
    ui_g_Static_6.option.figure_name[2] = 35;
    ui_g_Static_6.option.operate_type = 1;

    ui_g_Static_Kawshiro->figure_type = 7;
    ui_g_Static_Kawshiro->operate_type = 1;
    ui_g_Static_Kawshiro->layer = 0;
    ui_g_Static_Kawshiro->color = 6;
    ui_g_Static_Kawshiro->start_x = 1692;
    ui_g_Static_Kawshiro->start_y = 467;
    ui_g_Static_Kawshiro->width = 2;
    ui_g_Static_Kawshiro->font_size = 20;
    ui_g_Static_Kawshiro->str_length = 9;
    strcpy(ui_g_Static_Kawshiro->string, "Kawashiro");


    ui_proc_string_frame(&ui_g_Static_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_6, sizeof(ui_g_Static_6));
}

void _ui_update_g_Static_6() {
    ui_g_Static_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_6, sizeof(ui_g_Static_6));
}

void _ui_remove_g_Static_6() {
    ui_g_Static_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_6, sizeof(ui_g_Static_6));
}
ui_string_frame_t ui_g_Static_7;
ui_interface_string_t* ui_g_Static_Star_Text_1 = &(ui_g_Static_7.option);

void _ui_init_g_Static_7() {
    ui_g_Static_7.option.figure_name[0] = 0;
    ui_g_Static_7.option.figure_name[1] = 1;
    ui_g_Static_7.option.figure_name[2] = 36;
    ui_g_Static_7.option.operate_type = 1;

    ui_g_Static_Star_Text_1->figure_type = 7;
    ui_g_Static_Star_Text_1->operate_type = 1;
    ui_g_Static_Star_Text_1->layer = 0;
    ui_g_Static_Star_Text_1->color = 8;
    ui_g_Static_Star_Text_1->start_x = 891;
    ui_g_Static_Star_Text_1->start_y = 534;
    ui_g_Static_Star_Text_1->width = 2;
    ui_g_Static_Star_Text_1->font_size = 15;
    ui_g_Static_Star_Text_1->str_length = 1;
    strcpy(ui_g_Static_Star_Text_1->string, "1");


    ui_proc_string_frame(&ui_g_Static_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_7, sizeof(ui_g_Static_7));
}

void _ui_update_g_Static_7() {
    ui_g_Static_7.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_7, sizeof(ui_g_Static_7));
}

void _ui_remove_g_Static_7() {
    ui_g_Static_7.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_7);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_7, sizeof(ui_g_Static_7));
}
ui_string_frame_t ui_g_Static_8;
ui_interface_string_t* ui_g_Static_Star_Text_2 = &(ui_g_Static_8.option);

void _ui_init_g_Static_8() {
    ui_g_Static_8.option.figure_name[0] = 0;
    ui_g_Static_8.option.figure_name[1] = 1;
    ui_g_Static_8.option.figure_name[2] = 37;
    ui_g_Static_8.option.operate_type = 1;

    ui_g_Static_Star_Text_2->figure_type = 7;
    ui_g_Static_Star_Text_2->operate_type = 1;
    ui_g_Static_Star_Text_2->layer = 0;
    ui_g_Static_Star_Text_2->color = 8;
    ui_g_Static_Star_Text_2->start_x = 840;
    ui_g_Static_Star_Text_2->start_y = 493;
    ui_g_Static_Star_Text_2->width = 2;
    ui_g_Static_Star_Text_2->font_size = 15;
    ui_g_Static_Star_Text_2->str_length = 1;
    strcpy(ui_g_Static_Star_Text_2->string, "2");


    ui_proc_string_frame(&ui_g_Static_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_8, sizeof(ui_g_Static_8));
}

void _ui_update_g_Static_8() {
    ui_g_Static_8.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_8, sizeof(ui_g_Static_8));
}

void _ui_remove_g_Static_8() {
    ui_g_Static_8.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_8);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_8, sizeof(ui_g_Static_8));
}
ui_string_frame_t ui_g_Static_9;
ui_interface_string_t* ui_g_Static_Star_Text_3 = &(ui_g_Static_9.option);

void _ui_init_g_Static_9() {
    ui_g_Static_9.option.figure_name[0] = 0;
    ui_g_Static_9.option.figure_name[1] = 1;
    ui_g_Static_9.option.figure_name[2] = 38;
    ui_g_Static_9.option.operate_type = 1;

    ui_g_Static_Star_Text_3->figure_type = 7;
    ui_g_Static_Star_Text_3->operate_type = 1;
    ui_g_Static_Star_Text_3->layer = 0;
    ui_g_Static_Star_Text_3->color = 8;
    ui_g_Static_Star_Text_3->start_x = 789;
    ui_g_Static_Star_Text_3->start_y = 454;
    ui_g_Static_Star_Text_3->width = 2;
    ui_g_Static_Star_Text_3->font_size = 15;
    ui_g_Static_Star_Text_3->str_length = 1;
    strcpy(ui_g_Static_Star_Text_3->string, "3");


    ui_proc_string_frame(&ui_g_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_9, sizeof(ui_g_Static_9));
}

void _ui_update_g_Static_9() {
    ui_g_Static_9.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_9, sizeof(ui_g_Static_9));
}

void _ui_remove_g_Static_9() {
    ui_g_Static_9.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_9);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_9, sizeof(ui_g_Static_9));
}
ui_string_frame_t ui_g_Static_10;
ui_interface_string_t* ui_g_Static_Star_Text_4 = &(ui_g_Static_10.option);

void _ui_init_g_Static_10() {
    ui_g_Static_10.option.figure_name[0] = 0;
    ui_g_Static_10.option.figure_name[1] = 1;
    ui_g_Static_10.option.figure_name[2] = 39;
    ui_g_Static_10.option.operate_type = 1;

    ui_g_Static_Star_Text_4->figure_type = 7;
    ui_g_Static_Star_Text_4->operate_type = 1;
    ui_g_Static_Star_Text_4->layer = 0;
    ui_g_Static_Star_Text_4->color = 8;
    ui_g_Static_Star_Text_4->start_x = 738;
    ui_g_Static_Star_Text_4->start_y = 412;
    ui_g_Static_Star_Text_4->width = 2;
    ui_g_Static_Star_Text_4->font_size = 15;
    ui_g_Static_Star_Text_4->str_length = 1;
    strcpy(ui_g_Static_Star_Text_4->string, "4");


    ui_proc_string_frame(&ui_g_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_10, sizeof(ui_g_Static_10));
}

void _ui_update_g_Static_10() {
    ui_g_Static_10.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_10, sizeof(ui_g_Static_10));
}

void _ui_remove_g_Static_10() {
    ui_g_Static_10.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_10);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_10, sizeof(ui_g_Static_10));
}
ui_string_frame_t ui_g_Static_11;
ui_interface_string_t* ui_g_Static_Star_Text_5 = &(ui_g_Static_11.option);

void _ui_init_g_Static_11() {
    ui_g_Static_11.option.figure_name[0] = 0;
    ui_g_Static_11.option.figure_name[1] = 1;
    ui_g_Static_11.option.figure_name[2] = 40;
    ui_g_Static_11.option.operate_type = 1;

    ui_g_Static_Star_Text_5->figure_type = 7;
    ui_g_Static_Star_Text_5->operate_type = 1;
    ui_g_Static_Star_Text_5->layer = 0;
    ui_g_Static_Star_Text_5->color = 8;
    ui_g_Static_Star_Text_5->start_x = 689;
    ui_g_Static_Star_Text_5->start_y = 374;
    ui_g_Static_Star_Text_5->width = 2;
    ui_g_Static_Star_Text_5->font_size = 15;
    ui_g_Static_Star_Text_5->str_length = 1;
    strcpy(ui_g_Static_Star_Text_5->string, "5");


    ui_proc_string_frame(&ui_g_Static_11);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_11, sizeof(ui_g_Static_11));
}

void _ui_update_g_Static_11() {
    ui_g_Static_11.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Static_11);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_11, sizeof(ui_g_Static_11));
}

void _ui_remove_g_Static_11() {
    ui_g_Static_11.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Static_11);
    SEND_MESSAGE((uint8_t *) &ui_g_Static_11, sizeof(ui_g_Static_11));
}

void ui_init_g_Static() {
    _ui_init_g_Static_0();
    _ui_init_g_Static_1();
    _ui_init_g_Static_2();
    _ui_init_g_Static_3();
    _ui_init_g_Static_4();
    _ui_init_g_Static_5();
    _ui_init_g_Static_6();
    _ui_init_g_Static_7();
    _ui_init_g_Static_8();
    _ui_init_g_Static_9();
    _ui_init_g_Static_10();
    _ui_init_g_Static_11();
}

void ui_update_g_Static() {
    _ui_update_g_Static_0();
    _ui_update_g_Static_1();
    _ui_update_g_Static_2();
    _ui_update_g_Static_3();
    _ui_update_g_Static_4();
    _ui_update_g_Static_5();
    _ui_update_g_Static_6();
    _ui_update_g_Static_7();
    _ui_update_g_Static_8();
    _ui_update_g_Static_9();
    _ui_update_g_Static_10();
    _ui_update_g_Static_11();
}

void ui_remove_g_Static() {
    _ui_remove_g_Static_0();
    _ui_remove_g_Static_1();
    _ui_remove_g_Static_2();
    _ui_remove_g_Static_3();
    _ui_remove_g_Static_4();
    _ui_remove_g_Static_5();
    _ui_remove_g_Static_6();
    _ui_remove_g_Static_7();
    _ui_remove_g_Static_8();
    _ui_remove_g_Static_9();
    _ui_remove_g_Static_10();
    _ui_remove_g_Static_11();
}

