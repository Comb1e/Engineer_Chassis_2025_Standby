//
// Created by 34147 on 2024/1/19.
//

#include "rotation_matrix.h"

#define s arm_sin_f32
#define c arm_cos_f32
// ------------------------------- euler angle to rotation matrix ------------------------------- //

//ZYX欧拉角坐标系变换
/* 弧度
 * 逻辑是 使用3*3矩阵
 * 假设视觉返回一个row pitch yaw坐标
 * 先利用ZYX欧拉角坐标变换转化为旋转矩阵
 * 得到矩阵后
 * 利用XYX欧拉角逆变换逆推出三个电机（舵机）角度
 * 返回三个XYX欧拉角并控制相应电机（舵机）
 */
void set_rotMatrix_from_euler_zyx(float z, float y, float x, float rotMat_arr[9]) {
    //first row
    rotMat_arr[0] = c(z) * c(y);
    rotMat_arr[1] = c(z) * s(y) * s(x) - s(z) * c(x);
    rotMat_arr[2] = c(z) * s(y) * c(x) + s(z) * s(x);
    //second row
    rotMat_arr[3] = s(z) * c(y);
    rotMat_arr[4] = s(z) * s(y) * s(x) + c(z) * c(x);
    rotMat_arr[5] = s(z) * s(y) * c(x) - c(z) * s(x);
    //third row
    rotMat_arr[6] = -s(y);
    rotMat_arr[7] = c(y) * s(x);
    rotMat_arr[8] = c(y) * c(x);
}

//弧度
void set_rotMatrix_from_fixed_xyz(float x, float y, float z, float rotMat_arr[9]) {
    //first row
    rotMat_arr[0] = c(x) * c(y);
    rotMat_arr[1] = c(x) * s(y) * s(z) - s(x) * c(z);
    rotMat_arr[2] = c(x) * s(y) * c(z) + s(x) * s(z);
    //second row
    rotMat_arr[3] = s(x) * c(y);
    rotMat_arr[4] = s(x) * s(y) * s(z) + c(x) * c(z);
    rotMat_arr[5] = s(x) * s(y) * c(z) - c(x) * s(z);
    //third row
    rotMat_arr[6] = -s(y);
    rotMat_arr[7] = c(y) * s(z);
    rotMat_arr[8] = c(y) * c(z);
}

//弧度
void set_rotMatrix_from_euler_zyz(float z1, float y, float z2, float rotMat_arr[9]) {
    //first row
    rotMat_arr[0] = c(z1) * c(y) * c(z2) - s(z1) * s(z2);
    rotMat_arr[1] = -c(z1) * c(y) * s(z2) - s(z1) * c(z2);
    rotMat_arr[2] = c(z1) * s(y);
    //second row
    rotMat_arr[3] = s(z1) * c(y) * c(z2) + c(z1) * s(z2);
    rotMat_arr[4] = -s(z1) * c(y) * s(z2) + c(z1) * c(z2);
    rotMat_arr[5] = s(z1) * s(y);
    //third row
    rotMat_arr[6] = -s(y) * c(z2);
    rotMat_arr[7] = s(y) * s(z2);
    rotMat_arr[8] = c(y);
}

//弧度
void set_rotMatrix_from_euler_xyx(float x1, float y, float x2, float rotMat_arr[9]) {
    //first row
    rotMat_arr[0] = c(y);
    rotMat_arr[1] = s(y) * s(x2);
    rotMat_arr[2] = s(y) * c(x2);
    //second row
    rotMat_arr[3] = s(x1) * s(y);
    rotMat_arr[4] = -s(x1) * c(y) * s(x2) + c(x1) * c(x2);
    rotMat_arr[5] = -s(x1) * c(y) * c(x2) - c(x1) * s(x2);
    //third row
    rotMat_arr[6] = -c(x1) * s(y);
    rotMat_arr[7] = c(x1) * c(y) * s(x2) + s(x1) * c(x2);
    rotMat_arr[8] = c(x1) * c(y) * c(x2) - s(x1) * s(x2);
}


// ------------------------------- rotation matrix to euler angle ------------------------------- //

//默认y取[-180,180]
//弧度
//void set_euler_zyx_from_rotMatrix(float mat_arr[9], float* z, float* y, float* x){
//    *y = asinf(-mat_arr[6]);
//    if(ABS(*y - PI/2) < 1e-6){
//        *y = PI/2;
//        *z = 0;
//        arm_atan2_f32(mat_arr[1],mat_arr[2],x);
//    }
//    else{
//        *y = ABS(*y);
//        arm_atan2_f32(mat_arr[3],mat_arr[0],z);
//        arm_atan2_f32(mat_arr[7],mat_arr[8],x);
//    }
//}

//待测,应该是arm_atan2已经给处理好了,防止发生错误
void set_euler_zyx_from_rotMatrix(float mat_arr[9], float *z, float *y, float *x) {
    float temp = 0.0f;
//    if((mat_arr[6]-1) < 1e-6){//r31==1
//        *y = -PI/2;
//        *z = 0;
//        arm_atan2_f32(mat_arr[1],mat_arr[4],x);
//        *x = -(*x);
//    }
//    else if((mat_arr[6]-(-1)) < 1e-6){//r31==-1
//        *y = PI/2;
//        *z = 0;
//        arm_atan2_f32(mat_arr[1],mat_arr[4],x);
//    }
//    else{
    arm_sqrt_f32(mat_arr[0] * mat_arr[0] + mat_arr[3] * mat_arr[3], &temp);
    arm_atan2_f32(-mat_arr[6], temp, y);
    arm_atan2_f32(mat_arr[3], mat_arr[0], z);
    arm_atan2_f32(mat_arr[7], mat_arr[8], x);
//    }
}





/*
* 将机械臂想象为大pitch为0
* pitch-yaw转化轴为第一个x轴变换
* 舵机轴向右或向左充当Y轴变换
* row轴充当最后一个x轴变换
*/
//XYX坐标系变换
//将矩阵理解为已经转为旋转矩阵,开始准备求相应的电机转角,怎么转其实是自己定的

//弧度
//positive result
void set_pos_euler_xyx_from_rotMatrix(float mat_arr[9], float *x1, float *y, float *x2) {
    //x1即yaw-picth转化轴  y舵机轴  x2 row轴
    float sin_y;
    float temp_y;
    arm_sqrt_f32(mat_arr[1] * mat_arr[1] + mat_arr[2] * mat_arr[2], &sin_y);
    arm_atan2_f32(sin_y, mat_arr[0], &temp_y);//取正号 y有范围为[0,180]
    if (ABS(temp_y) < 1e-6) {
        *x1 = 0;
        *y = 0;
        arm_atan2_f32(mat_arr[7], mat_arr[8], x2);
    } else if (ABS(temp_y - PI) < 1e-6) {
        *x1 = 0;
        *y = PI;
        arm_atan2_f32(-mat_arr[7], -mat_arr[8], x2);
    } else {  //x1取值-PI ~ PI,所以有限幅的时候得+—
        arm_atan2_f32(mat_arr[3] / sin_y, -mat_arr[6] / sin_y, x1);
        arm_atan2_f32(mat_arr[1] / sin_y, mat_arr[2] / sin_y, x2);
        *y = temp_y;
    }
}

//弧度
//negative result
void set_neg_euler_xyx_from_rotMatrix(float mat_arr[9], float *x1, float *y, float *x2) {
    float sin_y;
    float temp_y;
    arm_sqrt_f32(mat_arr[1] * mat_arr[1] + mat_arr[2] * mat_arr[2], &sin_y);
    sin_y = -sin_y;
    arm_atan2_f32(sin_y, mat_arr[0], &temp_y); //[-180,0]
    if (ABS(temp_y) < 1e-6) {
        *x1 = 0;
        *y = 0;
        arm_atan2_f32(mat_arr[7], mat_arr[8], x2);
    } else if (ABS(temp_y + PI) < 1e-6) {
        *x1 = 0;
        *y = -PI;
        arm_atan2_f32(-mat_arr[7], -mat_arr[8], x2);
    } else {
        arm_atan2_f32(mat_arr[3] / sin_y, -mat_arr[6] / sin_y, x1);
        arm_atan2_f32(mat_arr[1] / sin_y, mat_arr[2] / sin_y, x2);
        *y = temp_y;
    }
}

//A * B = C
//1,2,3,4,5,6,7,8,9
//1,2,3
//4,5,6
//7,8,9
void matrix_multiply_matrix(float left[9], float right[9], float out[9]) {
    arm_matrix_instance_f32 matrix_left;
    arm_matrix_instance_f32 matrix_right;
    arm_matrix_instance_f32 matrix_out;
    arm_mat_init_f32(&matrix_left, 3, 3, left);
    arm_mat_init_f32(&matrix_right, 3, 3, right);
    arm_mat_init_f32(&matrix_out, 3, 3, out);
    arm_mat_mult_f32(&matrix_left, &matrix_right, &matrix_out);
}

//弧度
void mat_rot_x(float alpha, float mat[9]) {
    mat[0] = 1;
    mat[1] = 0;
    mat[2] = 0;
    mat[3] = 0;
    mat[4] = c(alpha);
    mat[5] = -s(alpha);
    mat[6] = 0;
    mat[7] = s(alpha);
    mat[8] = c(alpha);
}

//弧度
void mat_rot_y(float alpha, float mat[9]) {
    mat[0] = c(alpha);
    mat[1] = 0;
    mat[2] = s(alpha);
    mat[3] = 0;
    mat[4] = 1;
    mat[5] = 0;
    mat[6] = -s(alpha);
    mat[7] = 0;
    mat[8] = c(alpha);
}

//弧度
void mat_rot_z(float alpha, float mat[9]) {
    mat[0] = c(alpha);
    mat[1] = -s(alpha);
    mat[2] = 0;
    mat[3] = s(alpha);
    mat[4] = c(alpha);
    mat[5] = 0;
    mat[6] = 0;
    mat[7] = 0;
    mat[8] = 1;
}

//弧度
void mat_rot_y_T(float a, float mat[9]) {
    mat[0] = c(a);
    mat[1] = 0;
    mat[2] = -s(a);
    mat[3] = 0;
    mat[4] = 1;
    mat[5] = 0;
    mat[6] = s(a);
    mat[7] = 0;
    mat[8] = c(a);
}//绕y轴转a角度，即pitch

//ZYX坐标系变换 //默认y取[-180,180]
Eigen::Vector3f RotMatrix_To_Euler_ZYX(Eigen::Matrix3f rot)
{
    Eigen::Vector3f euler_zyx;
    euler_zyx << 0,0,0;

    float r1;
    float r2;
    float r3;
    float r4;

    arm_atan2_f32(rot(2, 1),rot(2, 2),&r1);
    arm_sqrt_f32( rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2),&r2);
    arm_atan2_f32(-rot(2, 0),r2,&r3);
    arm_atan2_f32(rot(1, 0), rot(0, 0),&r4);
    euler_zyx << r4, r3, r1;
    return euler_zyx;
}

#undef s
#undef c