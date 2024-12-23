//
// Created by 34147 on 2024/1/19.
//

#ifndef ENGINEER_CHASSIS_ABOARD_ROTATION_MATRIX_H
#define ENGINEER_CHASSIS_ABOARD_ROTATION_MATRIX_H

#ifdef __cplusplus
extern "C" {
#endif

#include "User_Lib.h"
#include "math.h"
#include "arm_math.h"

void matrix_multiply_matrix(float left[9], float right[9], float out[9]);
void set_euler_zyx_from_rotMatrix(float mat_arr[9], float *z, float *y, float *x);
void set_neg_euler_xyx_from_rotMatrix(float mat_arr[9], float *x1, float *y, float *x2);
void set_pos_euler_xyx_from_rotMatrix(float mat_arr[9], float *x1, float *y, float *x2);

void set_rotMatrix_from_euler_zyx(float z, float y, float x, float rotMat_arr[9]);
void set_rotMatrix_from_fixed_xyz(float x, float y, float z, float rotMat_arr[9]);
void set_rotMatrix_from_euler_zyz(float z1, float y, float z2, float rotMat_arr[9]);
void set_rotMatrix_from_euler_xyx(float x1, float y, float x2, float rotMat_arr[9]);

#ifdef __cplusplus
}
#endif

#include "Eigen/Geometry"

Eigen::Vector3f RotMatrix_To_Euler_ZYX(Eigen::Matrix3f rot);
#endif //ENGINEER_CHASSIS_ABOARD_ROTATION_MATRIX_H
