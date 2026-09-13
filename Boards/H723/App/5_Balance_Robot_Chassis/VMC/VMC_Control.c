//
// Created by qza on 2026/7/17.
//

#include "Horizon_MATH.h"
#include "math.h"
#include "VMC_Control.h"
#include <string.h>
#include "Robot_Config.h"
#include "All_define.h"
#include "IMU_Task.h"
// #include "F_Hold_Calc.h"
#include "Robot_Cmd.h"
float Lowpass_Filter(float *last_output, float input, float alpha)
{
    *last_output = alpha * input + (1.0f - alpha) * (*last_output);
    return *last_output;
}

float Diff(float now, float last, float dt)
{
    return (now - last) / dt;
}

void VMC_Simplify_Init(VMC_Simplify_t *vmc, float l1, float l2, float l3, float l4) {
    vmc->l1 = l1;
    vmc->l2 = l2;
    vmc->l3 = l3;
    vmc->l4 = l4;
    vmc->leg.left.length[0] = 0.0f;
    vmc->leg.left.phi= 0.0f;
    vmc->leg.right.length[0] = 0.0f;
    vmc->leg.right.phi = 0.0f;
    memset(vmc->JRM_l, 0, sizeof(vmc->JRM_l));
    memset(vmc->JRM_r, 0, sizeof(vmc->JRM_r));
}

void VMC_Update_Kinematics_Simplify(VMC_Simplify_t *vmc, float JRM[2][2],Leg_site_t *leg, float theta_front, float theta_back, float imu_pitch) {
    float x_B = 0.0f, y_B = 0.0f, x_C = 0.0f, y_C = 0.0f, x_D = 0.0f, y_D = 0.0f;
    float A0 = 0.0f, B0 = 0.0f, C0 = 0.0f;
    float l_BD = 0.0f;
    float phi2 = 0.0f, phi3 = 0.0f;

    // 四连杆正运动学几何解算
    x_B = cosf(theta_back) * vmc->l1;
    y_B = sinf(theta_back) * vmc->l1;
    x_D = cosf(theta_front)  * vmc->l4;
    y_D = sinf(theta_front)  * vmc->l4;

    A0 = 2.0f * vmc->l2 * (x_D - x_B);
    B0 = 2.0f * vmc->l2 * (y_D - y_B);
    l_BD = sqrtf((x_D - x_B) * (x_D - x_B) + (y_D - y_B) * (y_D - y_B));
    C0 = vmc->l2 * vmc->l2 + l_BD * l_BD - vmc->l3 * vmc->l3;

    phi3 = 2.0f * atan2f(B0 + sqrtf(A0 * A0 + B0 * B0 - C0 * C0), A0 + C0);

    x_C = x_B + vmc->l2 * cosf(phi3);
    y_C = y_B + vmc->l2 * sinf(phi3);

    phi2 = atan2f(y_C - y_D, x_C - x_D);

    // 更新虚拟状态
    leg->length_last[0] = leg->length[0];
    leg->theta_last[0] = leg->theta[0];

    leg->length[0] = sqrtf(x_C * x_C + y_C * y_C);
    // leg->phi = atan2f(y_C, x_C);
    leg->theta[0] = -atan2f(x_C, y_C)+imu_pitch * DEG2RAD;
    // 雅可比矩阵解算
    float inv_denom1 = 1.0f/sinf(phi2 - phi3);
    float inv_denom2 = inv_denom1/leg->length[0] ;
    float inv_denom3 = sinf(phi2 - theta_front);
    float inv_denom4 = sinf(theta_back - phi3);

    JRM[0][0] = vmc->l4 * cosf(leg->theta[0] - phi3) * inv_denom3 * inv_denom1;
    JRM[0][1] = -vmc->l4 * sinf(leg->theta[0] - phi3) * inv_denom3 * inv_denom2;
    JRM[1][0] = vmc->l1 * cosf(leg->theta[0] - phi2) * inv_denom4 * inv_denom1;
    JRM[1][1] = -vmc->l1 * sinf(leg->theta[0] - phi2) * inv_denom4 * inv_denom2;

    // float inv_denom5 = 1.0f / (vmc->l1 * inv_denom3);
    // float inv_denom6 = 1.0f / (vmc->l4 * inv_denom4);
    // 计算 J.t RM 的逆矩阵
    // vmc->JRM_inv[0][0] = -cosf(leg->phi - phi2) * inv_denom5;
    // vmc->JRM_inv[0][1] =  cosf(leg->phi - phi3) * inv_denom5;
    // vmc->JRM_inv[1][0] =  leg->length[0] * sinf(leg->phi - phi2) * inv_denom6;
    // vmc->JRM_inv[1][1] = -leg->length[0] * sinf(leg->phi - phi3) * inv_denom6;
}

void VMC_Inverse_Dynamics_Simplify(float JRM[2][2], float F0_target, float Tp_target, Leg_site_t *leg) {
    leg->Tp_front = JRM[0][0] * F0_target + JRM[0][1] * Tp_target;
    leg->Tp_back  = JRM[1][0] * F0_target + JRM[1][1] * Tp_target;
}

void VMC_leg_speed(Leg_site_t *leg, float dt) {

    leg->length[1]=Diff(leg->length[0],leg->length_last[0],dt);
    leg->length[2]=Diff(leg->length[1],leg->length_last[1],dt);
    leg->length_last[1]=leg->length[1];

    leg->theta[1]=Diff(leg->theta[0],leg->theta_last[0],dt);
    leg->theta[2]=Diff(leg->theta[1],leg->theta_last[1],dt);
    leg->theta_last[1]=leg->theta[1];
}

void VMC_task(VMC_Simplify_t *vmc,float dt) {

    vmc->theta_front_left = leg_motors.BM_P1010B_Leg[0].pos_rad;
    vmc->theta_back_left = leg_motors.BM_P1010B_Leg[2].pos_rad;
    vmc->theta_front_right = -leg_motors.BM_P1010B_Leg[1].pos_rad;
    vmc->theta_back_right = -leg_motors.BM_P1010B_Leg[3].pos_rad;

    VMC_Update_Kinematics_Simplify(vmc, vmc->JRM_l, &vmc->leg.left,  vmc->theta_front_left,  vmc->theta_back_left, IMU_Data.pitch);
    VMC_Update_Kinematics_Simplify(vmc, vmc->JRM_r, &vmc->leg.right,  vmc->theta_front_right,  vmc->theta_back_right, IMU_Data.pitch);

    VMC_leg_speed(&vmc->leg.left,dt);
    VMC_leg_speed(&vmc->leg.right,dt);

}
