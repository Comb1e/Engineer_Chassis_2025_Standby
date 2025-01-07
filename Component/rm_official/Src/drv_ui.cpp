//
// Created by 34147 on 2024/2/21.
//

#include "drv_ui.h"
#include "cstring"
#include "cstdio"
#include "drv_keyboard.h"
#include "drv_judgement.h"

ui_device ui;

ui_device::ui_device() : using_custom_ctrl_flag(false), clear_all_flag(false), remind_big_island_flag(false) {}

void ui_device::update_data_send() {
    if (this->judgement->judge_rece_mesg.game_robot_state_data.robot_id == RED_ENGINEER_ID) {//红二
        this->student_interactive_header.sender_ID = RED_ENGINEER_ID;
        this->student_interactive_header.receiver_ID = RED_ENGINEER_CUSTOM_ID;
        this->send();
    } else if (this->judgement->judge_rece_mesg.game_robot_state_data.robot_id == BLUE_ENGINEER_ID) {//蓝二
        this->student_interactive_header.sender_ID = BLUE_ENGINEER_ID;
        this->student_interactive_header.receiver_ID = BLUE_ENGINEER_CUSTOM_ID;
        this->send();
    }
}

void ui_device::update_data() {
    this->data.track_point.x = this->robot->Get_Arm_Track_Point(X);
    this->data.track_point.y = this->robot->Get_Arm_Track_Point(Y);
    this->data.track_point.z = this->robot->Get_Arm_Track_Point(Z);
    this->data.track_point.arm_yaw = (int16_t) this->robot->Get_Arm_Track_Point(ARM_YAW);
    this->data.track_point.sucker_roll_deg = this->robot->Get_Arm_Track_Point(ROLL);
    this->data.track_point.sucker_yaw_deg = this->robot->Get_Arm_Track_Point(YAW);
    this->data.track_point.sucker_pitch_deg = this->robot->Get_Arm_Track_Point(PITCH);

    this->data.target_point.x = this->robot->Get_Arm_Final_Point(X);
    this->data.target_point.y = this->robot->Get_Arm_Final_Point(Y);
    this->data.target_point.z = this->robot->Get_Arm_Final_Point(Z);

    this->data.mode = this->robot->Get_KB_Control_Mode();
    this->data.arm_type = this->robot->Get_Pose_Mode();

    this->data.tof_dist = this->robot->Get_Tof_Dist();

    this->data.camera_catch = this->robot->Get_Camera_Catching();

    this->data.auto_case = this->robot->Get_Auto_Situation();

    this->data.arm_sucker_state = this->robot->Get_Sucker_State(ARM_SUCKER);
    this->data.left_sucker_state = this->robot->Get_Sucker_State(LEFT_SUCKER);
    this->data.right_sucker_state = this->robot->Get_Sucker_State(RIGHT_SUCKER);

    this->data.big_island_dir = this->robot->Get_BigIsland_Dir();

    this->data.error_code = this->robot->Get_Error_Code();
    this->data.gimbal_error_code = this->robot->Get_Gimbal_Error_Code();

    /*this->data.remind_take_back_claw_flag = this->g_robot->get_remind_take_back_claw();
    this->data.remind_extend_claw_flag = this->g_robot->get_remind_extend_claw();*/

}

robot_error_type ui_device::get_error_type() {
    if (this->data.error_code.remote) {
        return remote;
    }
    if (this->data.error_code.arm_sucker && this->data.error_code.right_sucker && this->data.error_code.left_sucker) {
        return arl_sucker;
    } else if (this->data.error_code.arm_sucker && this->data.error_code.right_sucker) {
        return ar_sucker;
    } else if (this->data.error_code.arm_sucker && this->data.error_code.left_sucker) {
        return al_sucker;
    } else if (this->data.error_code.right_sucker && this->data.error_code.left_sucker) {
        return rl_sucker;
    }

    if (this->data.error_code.arm_sucker) {
        return arm_sucker;
    } else if (this->data.error_code.left_sucker) {
        return left_sucker;
    } else if (this->data.error_code.right_sucker) {
        return right_sucker;
    } else if (this->data.error_code.vision) {
        return vision;
    } else if (this->data.error_code.gimbal_yaw) {
        return gimbal_yaw;
    } else if (this->data.error_code.gimbal_arm) {
        return gimbal_arm;
    } else if (this->data.error_code.chassis_lf) {
        return chassis_lf;
    } else if (this->data.error_code.chassis_lb) {
        return chassis_lb;
    } else if (this->data.error_code.chassis_rb) {
        return chassis_rb;
    } else if (this->data.error_code.chassis_rf) {
        return chassis_rf;
    }
    return none;
}

robot_gimbal_error_type ui_device::get_gimbal_error_type() const {
    if(this->data.gimbal_error_code.temp){
        return temp;
    }
    if (this->data.gimbal_error_code.gyro && this->data.gimbal_error_code.motor) {
        return gy_mo;
    }
    if (this->data.gimbal_error_code.gyro) {
        return gyro;
    }

    if (this->data.gimbal_error_code.motor) {
        return motor;
    }

    return None;
}

void ui_device::ptr_init(Robot_Device *robot, judgement_device *judgement) {
    this->robot = robot;
    this->judgement = judgement;
}

void ui_device::set_not_remind_big_island() {
    this->remind_big_island_flag = false;
}

void ui_device::set_to_remind_big_island() {
    this->remind_big_island_flag = true;
}

void ui_device::add() {
    static uint8_t _ui_add_case = 0;

    _ui_add_case = (_ui_add_case + 1) % 35;
    if (_ui_add_case < 8) {
        this->lines_send(ui_add);
    } else if (_ui_add_case < 35) {
        this->character_init(&Character_Graph);
    } else
        _ui_add_case = 0;//没什么用 用作保护
}

void ui_device::update() {
    static uint8_t _ui_update_case = 0;
    _ui_update_case = (_ui_update_case + 1) % 15;
    if (_ui_update_case < 10) {
        this->character_update(&Character_Graph);//只有字符需要改变
    } else if (_ui_update_case < 14) {

        this->rec_send();
    } else {
        _ui_update_case = 0;
    }
}

void ui_device::send() {
    static uint8_t _ui_send_case = 200;//先添加好多次,防止没添加上

    this->update_data();//给ui自身数据进行update
    if (this->clear_all_flag) {
        this->clear_all_UI();
        return;
    }

    if (_ui_send_case > 50) {
        this->add();
        _ui_send_case--;
        return;
    }

    _ui_send_case = (_ui_send_case + 1) % 50;
    if (_ui_send_case % 10 == 0)  //5:1进行 add
        this->add();
    else
        this->update();
}




/// ----------------------------------------------- drv综合 ----------------------------------------------- ///

/// ------------------------------------------ UI显示 ------------------------------------------ ///
/// 1. 当前自动化运行过程状态  √
/// 3. 空接对齐线  √
/// 5. xyzryp  √
/// 6. 三个吸盘的状态：开关与是否吸住  √
/// 7. 血量过低需要自动回位,因此需要读出血量,进行自动化操作
/// 8. 提醒自己记得是否开启自定义控制器或者一些基本键位  暂时不需要

/// ----------------------------------------------- 分部发送 ----------------------------------------------- ///
void ui_device::character_init(ext_client_custom_character_t *_character) {
    static uint8_t _character_init_case = 0;
    for (int i = 0; i < 30; i++)
        _character->data[i] = '\0';

    _character_init_case = (_character_init_case + 1) % 27;

    if (_character_init_case < 3) {//30
        character_config(&_character->grapic_data_struct, "R1", ui_add, 2, ui_red_blue, 15, 9, 2, 20, 890);
        strcat(_character->data, "X\n\n");
        strcat(_character->data, "Y\n\n");
        strcat(_character->data, "Z\n\n");
        strcat(_character->data, "ROLL\n\n");
        strcat(_character->data, "YAW\n\n");
        strcat(_character->data, "PITCH\n");
    } else if (_character_init_case < 6) {
        character_config(&_character->grapic_data_struct, "R21", ui_add, 2, ui_red_blue, 15, 9, 2, 160, 890);
        strcat(_character->data, "0.0\n\n");
        strcat(_character->data, "0.0\n\n");
        strcat(_character->data, "0.0\n");

    } else if (_character_init_case < 9) {
        character_config(&_character->grapic_data_struct, "R22", ui_add, 2, ui_red_blue, 15, 9, 2, 160, 755);
        strcat(_character->data, "0.0\n\n");
        strcat(_character->data, "0.0\n\n");
        strcat(_character->data, "0.0\n");
    } else if (_character_init_case < 12) {
        character_config(&_character->grapic_data_struct, "R3", ui_add, 2, ui_red_blue, 15, 9, 2, 1500, 850);
        strcat(_character->data, "ARM_YAW\n\n");
        strcat(_character->data, "TOF\n\n");
        strcat(_character->data, "MODE\n");
    } else if (_character_init_case < 15) {
        character_config(&_character->grapic_data_struct, "R4", ui_add, 2, ui_red_blue, 15, 9, 2, 1750, 850);
        strcat(_character->data, "0.0\n\n");
        strcat(_character->data, "0.0\n\n");
        strcat(_character->data, "Steer1\n\n");

    } else if (_character_init_case < 18) {
        character_config(&_character->grapic_data_struct, "R5", ui_add, 2, ui_red_blue, 15, 9, 2, 1500, 675);
        strcat(_character->data, "AUTO\n\n");
        strcat(_character->data, "VISION\n\n");
    } else if (_character_init_case < 21) {
        character_config(&_character->grapic_data_struct, "R6", ui_add, 2, ui_red_blue, 15, 9, 2, 1750, 675);
        strcat(_character->data, "None\n\n");
        strcat(_character->data, "None\n\n");
    } else if (_character_init_case < 24) {
        character_config(&_character->grapic_data_struct, "R7", ui_add, 2, ui_red_blue, 15, 9, 2, 1500, 550);
        strcat(_character->data, "SUCKER\n\n");
        strcat(_character->data, "ERROR\n\n");
        strcat(_character->data, "G_ERROR");
    } else if (_character_init_case < 27) {
        character_config(&_character->grapic_data_struct, "R8", ui_add, 2, ui_red_blue, 15, 9, 2, 1750, 550);
        strcat(_character->data, "C C C\n\n");
        strcat(_character->data, "None\n\n");
    }

    student_interactive_header.data_cmd_id = STU_CUSTOM_CHARACTER_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Character_Graph, sizeof(Character_Graph),
                                student_interactive_header);
}

void ui_device::character_update(ext_client_custom_character_t *_character) {
    static uint8_t _character_update_case = 0;
    for (int i = 0; i < 30; i++)
        _character->data[i] = '\0';

    _character_update_case = (_character_update_case + 1) % 50;
    if (_character_update_case % 5 == 0) {

        character_config(&_character->grapic_data_struct, "R21", ui_modify, 2, ui_red_blue, 15, 9, 2, 160, 890);

        char _str[8];
        sprintf(_str, "%.1f\n\n", this->data.track_point.x);//0+3+2+2 = 7
        strcat(_character->data, _str);
        sprintf(_str, "%.1f\n\n", this->data.track_point.y);//7
        strcat(_character->data, _str);
        sprintf(_str, "%.1f\n\n", this->data.track_point.z);//1+3+2+2 = 8
        strcat(_character->data, _str);
    } else if (_character_update_case % 5 == 1) {
        character_config(&_character->grapic_data_struct, "R22", ui_modify, 2, ui_red_blue, 15, 9, 2, 160, 755);

        char _str[8];
        sprintf(_str, "%.1f\n\n", this->data.track_point.sucker_roll_deg);//2=3
        strcat(_character->data, _str);
        sprintf(_str, "%.1f\n\n", this->data.track_point.sucker_yaw_deg);//1
        strcat(_character->data, _str);
        sprintf(_str, "%.1f\n", this->data.track_point.sucker_pitch_deg);//1
        strcat(_character->data, _str);
    } else if (_character_update_case % 5 == 2) {
        character_config(&_character->grapic_data_struct, "R4", ui_modify, 2, ui_red_blue, 15, 9, 2, 1750, 850);

        char _str[8];//8+7+8 = 23
        sprintf(_str, "%.1f\n\n", this->data.track_point.arm_yaw);
        strcat(_character->data, _str);

        sprintf(_str, "%.1f\n\n", this->data.tof_dist);
        strcat(_character->data, _str);

        if (this->data.mode == MINE_MODE) {
            switch (this->data.arm_type) {
                case single:strcat(_character->data, "Exchange1\n\n");
                    break;
                case concentric_double:strcat(_character->data, "Exchange2\n\n");
                    break;
                default:break;
            }
        } else if (this->data.mode == STEER_MODE) {
            switch (this->data.arm_type) {
                case single:strcat(_character->data, "Steer1\n\n");
                    break;
                case concentric_double:strcat(_character->data, "Steer2\n\n");
                    break;
                default:break;
            }
        }

    } else if (_character_update_case % 5 == 3) {
        character_config(&_character->grapic_data_struct, "R6", ui_modify, 2, ui_red_blue, 15, 9, 2, 1750, 675);

        char _str[8];//8+7+8 = 23

        switch (this->data.auto_case) {//3
            case Big_Island:static uint32_t big_island_case = 0;
                if (this->remind_big_island_flag) {
                    big_island_case = (big_island_case + 1) % 200;
                    if (big_island_case < 100) {
                        strcat(_character->data, "Bi\n\n");
                    } else {
                        strcat(_character->data, "   \n\n");
                    }
                } else {
                    switch (this->data.big_island_dir) {
                        case CENTER:strcat(_character->data, "Bi-Center\n\n");
                            break;
                        case LEFT:strcat(_character->data, "Bi-Left\n\n");
                            break;
                        case RIGHT:strcat(_character->data, "Bi-Right\n\n");
                            break;
                    }
                }

                break;             // B    S   A   E   N
            case Small_Island:strcat(_character->data, "Small\n\n");
                break;             // B    S   A   E   N
            case Exchange_Mine:strcat(_character->data, "Exchange\n\n");
                break;             // B    S   A   E   N
            case Ground_Mine:strcat(_character->data, "Ground\n\n");
                break;
            case Auto_None:strcat(_character->data, "None\n\n");
                break;             // B    S   A   E   N
            default:strcat(_character->data, "None\n\n");
                break;             // B    S   A   E   N
        }

        if (this->data.camera_catch) {
            strcat(_character->data, "CATCH\n\n");
        } else {//未识别
            strcat(_character->data, "None\n\n");

        }//todo 识别侧面和未识别的区别
    } else if (_character_update_case % 5 == 4) {
        character_config(&_character->grapic_data_struct, "R8", ui_modify, 2, ui_red_blue, 15, 9, 2, 1750, 550);

        switch (this->data.arm_sucker_state) {
            case CLOSE:strcat(_character->data, "C ");
                break;
            case OPEN_NOT_HOLDING:strcat(_character->data, "O ");
                break;
            case OPEN_HOLDING:strcat(_character->data, "H ");
                break;
            default:break;
        }
        switch (this->data.left_sucker_state) {
            case CLOSE:strcat(_character->data, "C ");
                break;
            case OPEN_NOT_HOLDING:strcat(_character->data, "O ");
                break;
            case OPEN_HOLDING:strcat(_character->data, "H ");
                break;
            default:break;
        }
        switch (this->data.right_sucker_state) {
            case CLOSE:strcat(_character->data, "C\n\n");
                break;
            case OPEN_NOT_HOLDING:strcat(_character->data, "O\n\n");
                break;
            case OPEN_HOLDING:strcat(_character->data, "H\n\n");
                break;
            default:break;
        }

        strcat(_character->data, error_types[this->get_error_type()].c_str());
        strcat(_character->data, "\n\n");
        strcat(_character->data, gimbal_error_types[this->get_gimbal_error_type()].c_str());
    }

    student_interactive_header.data_cmd_id = STU_CUSTOM_CHARACTER_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Character_Graph, sizeof(Character_Graph),
                                student_interactive_header);
}

void ui_device::lines_send(enum ui_operation _operation) {
    show_line(&Double_Graph.grapic_data_struct[0], "L0", _operation, ui_red_blue, 2, 5, 970 + 120, 0, 970 + 120, 1000);
    show_line(&Double_Graph.grapic_data_struct[1], "L1", _operation, ui_red_blue, 2, 5, 970 - 120, 0, 970 - 120, 1000);

    student_interactive_header.data_cmd_id = STU_CUSTOM_TWO_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Double_Graph, sizeof(Double_Graph),
                                student_interactive_header);
}

void ui_device::rec_send() {
    if (this->data.remind_take_back_claw_flag) {
        show_rectangle(&Double_Graph.grapic_data_struct[0],
                       "A0",
                       ui_add,
                       ui_pink,
                       2,
                       10,
                       970 - 50,
                       540 + 50,
                       970 + 50,
                       970 - 50);
    } else {
        show_rectangle(&Double_Graph.grapic_data_struct[0],
                       "A0",
                       ui_clear,
                       ui_pink,
                       2,
                       10,
                       970 - 50,
                       540 + 50,
                       970 + 50,
                       970 - 50);
    }

    if (this->data.remind_extend_claw_flag) {

        show_circle(&Double_Graph.grapic_data_struct[1], "C0", ui_add, ui_green, 2, 10, 970, 650, 50);

    } else {

        show_circle(&Double_Graph.grapic_data_struct[1],
                       "C0",
                       ui_clear,
                       ui_green,
                       2,
                       10,
                       970,
                       650,50);

    }

    student_interactive_header.data_cmd_id = STU_CUSTOM_TWO_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Double_Graph, sizeof(Double_Graph),
                                student_interactive_header);
}

void ui_device::clear_all_UI(void) {
    Delete_Graph.operate_type = ui_clear;
    student_interactive_header.data_cmd_id = STU_CUSTOM_DELETE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Delete_Graph, sizeof(Delete_Graph),
                                student_interactive_header);
}

/// ----------------------------------------------- 基本操作 ----------------------------------------------- ///
void ui_device::ClearAll(void) {
    Delete_Graph.operate_type = ui_delete_all;
    student_interactive_header.data_cmd_id = STU_CUSTOM_DELETE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Delete_Graph, sizeof(Delete_Graph),
                                student_interactive_header);
}

void ui_device::ClearLayer(uint32_t _layer) {
    VAL_LIMIT(_layer, 0, 9);
    Delete_Graph.operate_type = ui_delete_layer;
    Delete_Graph.layer = _layer;
    student_interactive_header.data_cmd_id = STU_CUSTOM_DELETE_PICTURE_ID;
    judgement->data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Delete_Graph, sizeof(Delete_Graph),
                                student_interactive_header);
}

//现在是单个发送
// width 5
void
ui_device::show_line(graphic_data_struct_t *_graphic,
                     const char *name,
                     enum ui_operation _operation,
                     enum ui_color _color,
                     uint32_t _layer,
                     uint32_t _width,
                     uint32_t start_x,
                     uint32_t start_y,
                     uint32_t end_x,
                     uint32_t end_y) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(end_x, 0, 2048);
    VAL_LIMIT(end_y, 0, 2048);

    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_line;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = 0;
    _graphic->end_x = end_x;
    _graphic->end_y = end_y;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void ui_device::show_rectangle(graphic_data_struct_t *_graphic, const char *name, enum ui_operation _operation,
                               enum ui_color _color, uint32_t _layer, uint32_t _width,
                               uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(end_x, 0, 2048);
    VAL_LIMIT(end_y, 0, 2048);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_rectangle;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = 0;
    _graphic->end_x = end_x;
    _graphic->end_y = end_y;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_circle(graphic_data_struct_t *_graphic,
                       const char *name,
                       enum ui_operation _operation,
                       enum ui_color _color,
                       uint32_t _layer,
                       uint32_t _width,
                       uint32_t center_x,
                       uint32_t center_y,
                       uint32_t _radius) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(center_x, 0, 2048);
    VAL_LIMIT(center_y, 0, 2048);
    VAL_LIMIT(_radius, 0, 1024);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_circle;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = center_x;
    _graphic->start_y = center_y;
    _graphic->radius = _radius;
    _graphic->end_x = 0;
    _graphic->end_y = 0;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_oval(graphic_data_struct_t *_graphic,
                     const char *name,
                     enum ui_operation _operation,
                     enum ui_color _color,
                     uint32_t _layer,
                     uint32_t _width,
                     uint32_t center_x,
                     uint32_t center_y,
                     uint32_t x_axis_len,
                     uint32_t y_axis_len) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(center_x, 0, 2048);
    VAL_LIMIT(center_y, 0, 2048);
    VAL_LIMIT(x_axis_len, 0, 2048);
    VAL_LIMIT(y_axis_len, 0, 2048);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_oval;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = 0;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = center_x;
    _graphic->start_y = center_y;
    _graphic->radius = 0;
    _graphic->end_x = x_axis_len;
    _graphic->end_y = y_axis_len;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_arc(graphic_data_struct_t *_graphic,
                    const char *name,
                    enum ui_operation _operation,
                    enum ui_color _color,
                    uint32_t _layer,
                    uint32_t _width,
                    uint32_t center_x,
                    uint32_t center_y,
                    uint32_t start_ang,
                    uint32_t end_ang,
                    uint32_t x_axis_len,
                    uint32_t y_axis_len) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(center_x, 0, 2048);
    VAL_LIMIT(center_y, 0, 2048);
    VAL_LIMIT(start_ang, 0, 360);
    VAL_LIMIT(end_ang, 0, 360);
    VAL_LIMIT(x_axis_len, 0, 2048);
    VAL_LIMIT(y_axis_len, 0, 2048);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_arc;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = start_ang;
    _graphic->end_angle = end_ang;
    _graphic->width = _width;
    _graphic->start_x = center_x;
    _graphic->start_y = center_y;
    _graphic->radius = 0;
    _graphic->end_x = x_axis_len;
    _graphic->end_y = y_axis_len;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void ui_device::show_floating(graphic_data_struct_t *_graphic, const char *name, enum ui_operation _operation,
                              enum ui_color _color, uint32_t _layer, uint32_t _width,
                              uint32_t start_x, uint32_t start_y, uint32_t font_size, uint32_t signi_digits,
                              float num) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(font_size, 0, 360);
    VAL_LIMIT(signi_digits, 0, 360);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_floating;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = font_size;
    _graphic->end_angle = signi_digits;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = (int32_t) (1000 * num);

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

// width 5
void
ui_device::show_integer(graphic_data_struct_t *_graphic,
                        const char *name,
                        enum ui_operation _operation,
                        enum ui_color _color,
                        uint32_t _layer,
                        uint32_t _width,
                        uint32_t start_x,
                        uint32_t start_y,
                        uint32_t font_size,
                        int32_t num) {
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);
    VAL_LIMIT(font_size, 0, 360);
    memcpy(_graphic->graphic_name, name, 3);
    _graphic->operate_type = _operation;//1增加2修改3删除
    _graphic->graphic_type = ui_integer;//1是矩形,0是直线,2是整圆
    _graphic->layer = _layer;
    _graphic->color = _color;
    _graphic->start_angle = font_size;
    _graphic->end_angle = 0;
    _graphic->width = _width;
    _graphic->start_x = start_x;
    _graphic->start_y = start_y;
    _graphic->radius = (int32_t) num;

//    student_interactive_header.data_cmd_id = STU_CUSTOM_ONE_PICTURE_ID;//图形
//    data_packet_pack(STU_INTERACTIVE_ID, (uint8_t *) &Single_Graph, sizeof(Single_Graph),
//                     student_interactive_header);
}

//font_size 20 _len 9 _width 2 layer 7
void ui_device::character_config(graphic_data_struct_t *_character, const char *name, enum ui_operation _operation,
                                 uint32_t _layer,
                                 enum ui_color _color, uint32_t font_size, uint32_t _len, uint32_t _width,
                                 uint32_t start_x, uint32_t start_y) {
    VAL_LIMIT(font_size, 0, 360);
    VAL_LIMIT(_len, 0, 360);
    VAL_LIMIT(_width, 0, 10);
    VAL_LIMIT(_layer, 0, 9);
    VAL_LIMIT(start_x, 0, 2048);
    VAL_LIMIT(start_y, 0, 2048);

    student_interactive_header.data_cmd_id = STU_CUSTOM_CHARACTER_ID;//绘制字符

//    strcpy(Character_Graph.grapic_data_struct.graphic_name, "R1");
    memcpy(_character->graphic_name, name, 3);
    _character->graphic_type = ui_character;
    _character->operate_type = _operation;
    _character->layer = _layer;
    _character->color = _color;
    _character->start_angle = font_size;//字体大小
    _character->end_angle = _len;//字符长度
    _character->width = _width;//线条宽度
    _character->start_x = start_x;
    _character->start_y = start_y;
    _character->radius = 0;
    _character->end_x = 0;
    _character->end_y = 0;
}