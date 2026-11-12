//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_arc_t *ui_g_Dynamic_SuperCap;
extern ui_interface_arc_t *ui_g_Dynamic_Pitch;
extern ui_interface_number_t *ui_g_Dynamic_Pitch_Num;
extern ui_interface_arc_t *ui_g_Dynamic_Fric_First_L;
extern ui_interface_arc_t *ui_g_Dynamic_Fric_First_R;
extern ui_interface_arc_t *ui_g_Dynamic_Chassis_1;
extern ui_interface_arc_t *ui_g_Dynamic_Chassis_3;
extern ui_interface_arc_t *ui_g_Dynamic_Chassis_2;
extern ui_interface_arc_t *ui_g_Dynamic_Chassis_4;
extern ui_interface_line_t *ui_g_Dynamic_Armor;
extern ui_interface_number_t *ui_g_Dynamic_Speed;
extern ui_interface_rect_t *ui_g_Dynamic_Aim;
extern ui_interface_line_t *ui_g_Dynamic_Gimbal1;
extern ui_interface_line_t *ui_g_Dynamic_Gimbal2;
extern ui_interface_line_t *ui_g_Dynamic_Trigger;
extern ui_interface_string_t *ui_g_Dynamic_Yaw_Offset;
extern ui_interface_string_t *ui_g_Dynamic_Pitch_Offset;

void ui_init_g_Dynamic();
void ui_update_g_Dynamic();
void ui_remove_g_Dynamic();

extern ui_interface_line_t *ui_g_Static_Star_6;
extern ui_interface_line_t *ui_g_Static_Star_7;
extern ui_interface_arc_t *ui_g_Static_SuperCap_1;
extern ui_interface_arc_t *ui_g_Static_SuperCap_2;
extern ui_interface_arc_t *ui_g_Static_SuperCap_3;
extern ui_interface_arc_t *ui_g_Static_SuperCap_4;
extern ui_interface_arc_t *ui_g_Static_SuperCap_5;
extern ui_interface_arc_t *ui_g_Static_Pitch_5;
extern ui_interface_arc_t *ui_g_Static_Pitch_4;
extern ui_interface_arc_t *ui_g_Static_Pitch_3;
extern ui_interface_arc_t *ui_g_Static_Pitch_2;
extern ui_interface_arc_t *ui_g_Static_Pitch_1;
extern ui_interface_round_t *ui_g_Static_Star_Round;
extern ui_interface_line_t *ui_g_Static_Star_1;
extern ui_interface_line_t *ui_g_Static_L1;
extern ui_interface_line_t *ui_g_Static_Star_2;
extern ui_interface_line_t *ui_g_Static_L2;
extern ui_interface_line_t *ui_g_Static_L3;
extern ui_interface_line_t *ui_g_Static_L4;
extern ui_interface_line_t *ui_g_Static_L5;
extern ui_interface_line_t *ui_g_Static_L6;
extern ui_interface_round_t *ui_g_Static_R2;
extern ui_interface_round_t *ui_g_Static_R1;
extern ui_interface_line_t *ui_g_Static_Star_3;
extern ui_interface_line_t *ui_g_Static_Star_Offset;
extern ui_interface_line_t *ui_g_Static_Body1;
extern ui_interface_line_t *ui_g_Static_Body2;
extern ui_interface_line_t *ui_g_Static_Body3;
extern ui_interface_line_t *ui_g_Static_Body4;
extern ui_interface_line_t *ui_g_Static_Body5;
extern ui_interface_line_t *ui_g_Static_Body6;
extern ui_interface_line_t *ui_g_Static_Star_4;
extern ui_interface_round_t *ui_g_Static_Trigger_Round1;
extern ui_interface_line_t *ui_g_Static_Star_5;
extern ui_interface_string_t *ui_g_Static_WUST_RM;
extern ui_interface_string_t *ui_g_Static_Kawshiro;
extern ui_interface_string_t *ui_g_Static_Star_Text_1;
extern ui_interface_string_t *ui_g_Static_Star_Text_2;
extern ui_interface_string_t *ui_g_Static_Star_Text_3;
extern ui_interface_string_t *ui_g_Static_Star_Text_4;
extern ui_interface_string_t *ui_g_Static_Star_Text_5;

void ui_init_g_Static();
void ui_update_g_Static();
void ui_remove_g_Static();


#endif // UI_g_H
