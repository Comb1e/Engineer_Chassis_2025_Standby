//
// Created by 34147 on 2024/2/21.
//

#ifndef ENGINEER_CHASSIS_2024_DRV_UI_H
#define ENGINEER_CHASSIS_2024_DRV_UI_H

#include "User_Lib.h"
#include "Drv_USB.h"
#include "Drv_Robot.h"
#include "task_communicate.h"
#include "Eigen/Dense"
#ifdef __cplusplus
extern "C" {
#endif
//C

//相机底座不动点
#define IMAGE_BASE_X                   (-107.0f)
#define IMAGE_BASE_Y                   (70.5f)
#define IMAGE_BASE_Z                   (137.5f)
#define IMAGE_YAW_PITCH_DISTANCE               (10.f)
#define IMAGE_FOCUS_Z_DISTANCE                (2.5f)
#define IMAGE_FOCUS_Y_DISTANCE                (7.71f)
#define IMAGE_FOCUS_X_DISTANCE                (26.5f)

typedef struct {

  arm_data_t track_point;
  arm_data_t target_point;
  uint8_t arm_sucker_state;
  uint8_t left_sucker_state;
  uint8_t right_sucker_state;

  bool camera_catch;

  kb_control_mode_e mode;
  pose_mode_e arm_type;

  autoSituation_e auto_case;

  direction_need_e big_island_dir;//大资源岛的那个矿道

  robot_error_u error_code;
  robot_gimbal_error_u gimbal_error_code;

  bool remind_take_back_claw_flag;
  bool remind_extend_claw_flag;

  float tof_dist;
} ui_data_t;

typedef struct {
  float x;
  float y;
  float z;
  float z0;
  float y0;
  Eigen::Matrix4f t;
} image_transfer_data_t;
#ifdef __cplusplus
}
#endif
//C++





class ui_device {
 private:
  Robot_Device *robot;
  judgement_device *judgement;
  bool remind_big_island_flag;
  const std::string error_types[none + 1] = {
      "remote","chassis_lf",
                                             "chassis_lb",
                                             "chassis_rf",
                                             "chassis_rb",
                                             "gimbal_yaw",
                                             "gimbal_arm",
                                             "vision",
                                             "left_sucker",
                                             "right_sucker",
                                             "arm_sucker",
                                             "ar_sucker",
                                             "al_sucker",
                                             "rl_sucker",
                                             "arl_sucker",
                                             "none"};

  const std::string gimbal_error_types[None + 1] = {
      "gyro","motor","gy_mo","temp","none"
  };

 public:
/*----------------------------------绘制UI所需要的数据---------------------------*/

  ui_data_t data;
  image_transfer_data_t tran_data;
  bool using_custom_ctrl_flag;

/*------------------------------------绘制UI需要的变量--------------------------*/
  ext_client_custom_graphic_delete_t Delete_Graph;
  ext_client_custom_graphic_single_t Single_Graph;
  ext_client_custom_graphic_double_t Double_Graph;
  ext_client_custom_graphic_five_t Five_Graph;
  ext_client_custom_graphic_seven_t Seven_Graph;
  ext_client_custom_graphic_t Graph_Struct;

  ext_client_custom_character_t Character_Graph;
  uint32_t BodyStartX = 960;
  uint32_t BodyStartY = 540;
  bool clear_all_flag;
  ext_student_interactive_header_data_t student_interactive_header;
 public:
  ui_device();
  void update_data_send();
  void update_data();
  robot_error_type get_error_type();
  robot_gimbal_error_type get_gimbal_error_type() const;
  void ptr_init(Robot_Device *robot, judgement_device *judgement);
  void set_to_remind_big_island();
  void set_not_remind_big_island();
  void add();
  void update();
  void send();
  void clear_all_UI();
  void lines_send(enum ui_operation _operation);
  void rec_send();
  void character_update(ext_client_custom_character_t *_character);
  void character_init(ext_client_custom_character_t *_character);
  void ClearAll();
  void ClearLayer(uint32_t _layer);
  void show_line(graphic_data_struct_t *_graphic,
                 const char *name,
                 enum ui_operation _operation,
                 enum ui_color _color,
                 uint32_t _layer,
                 uint32_t _width,
                 uint32_t start_x,
                 uint32_t start_y,
                 uint32_t end_x,
                 uint32_t end_y);
  void show_rectangle(graphic_data_struct_t *_graphic,
                      const char *name,
                      enum ui_operation _operation,
                      enum ui_color _color,
                      uint32_t _layer,
                      uint32_t _width,
                      uint32_t start_x,
                      uint32_t start_y,
                      uint32_t end_x,
                      uint32_t end_y);
  void show_circle(graphic_data_struct_t *_graphic,
                   const char *name,
                   enum ui_operation _operation,
                   enum ui_color _color,
                   uint32_t _layer,
                   uint32_t _width,
                   uint32_t center_x,
                   uint32_t center_y,
                   uint32_t _radius);
  void show_oval(graphic_data_struct_t *_graphic,
                 const char *name,
                 enum ui_operation _operation,
                 enum ui_color _color,
                 uint32_t _layer,
                 uint32_t _width,
                 uint32_t center_x,
                 uint32_t center_y,
                 uint32_t x_axis_len,
                 uint32_t y_axis_len);
  void show_arc(graphic_data_struct_t *_graphic,
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
                uint32_t y_axis_len);
  void show_floating(graphic_data_struct_t *_graphic,
                     const char *name,
                     enum ui_operation _operation,
                     enum ui_color _color,
                     uint32_t _layer,
                     uint32_t _width,
                     uint32_t start_x,
                     uint32_t start_y,
                     uint32_t font_size,
                     uint32_t signi_digits,
                     float num);
  void show_integer(graphic_data_struct_t *_graphic,
                    const char *name,
                    enum ui_operation _operation,
                    enum ui_color _color,
                    uint32_t _layer,
                    uint32_t _width,
                    uint32_t start_x,
                    uint32_t start_y,
                    uint32_t font_size,
                    int32_t num);
  void character_config(graphic_data_struct_t *_character,
                        const char *name,
                        enum ui_operation _operation,
                        uint32_t _layer,
                        enum ui_color _color,
                        uint32_t font_size,
                        uint32_t _len,
                        uint32_t _width,
                        uint32_t start_x,
                        uint32_t start_y);
};

extern ui_device ui;

#endif //ENGINEER_CHASSIS_2024_DRV_UI_H
