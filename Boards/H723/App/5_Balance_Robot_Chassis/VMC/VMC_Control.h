//
// Created by qza on 2026/7/17.
//

#ifndef H7_FRAMEWORK_VMC_CONTROL_H
#define H7_FRAMEWORK_VMC_CONTROL_H

#include "MPC_Task.h"
#include "Robot_Config.h"



#define L1_LENGTH     0.215f
#define L2_LENGTH     0.258f
#define L3_LENGTH     0.258f
#define L4_LENGTH     0.215f

typedef struct {
    float length[3];       // 虚拟腿长 (m)
    float phi;
    float theta[3];        // 虚拟腿角 (rad)
    float length_last[2];
    float theta_last[2];
    float Tp_front;
    float Tp_back;
}Leg_site_t;

typedef struct {
    Leg_site_t left;
    Leg_site_t right;
}Leg_t;

typedef struct {
    // 物理结构参数 (连杆长度)
    float l1, l2, l3, l4;

    float theta_front_left ;
    float theta_back_left ;
    float theta_front_right ;
    float theta_back_right ;
    // 雅可比矩阵及其逆矩阵
    float JRM_l[2][2];     // 雅可比矩阵
    float JRM_r[2][2];
    float JRM_inv[2][2]; // 雅可比逆矩阵
    Leg_t leg;
} VMC_Simplify_t;

void VMC_Simplify_Init(VMC_Simplify_t *vmc, float l1, float l2, float l3, float l4);
void VMC_Inverse_Dynamics_Simplify(float JRM[2][2], float F0_target, float Tp_target, Leg_site_t *leg) ;
void VMC_task(VMC_Simplify_t *vmc,float dt);


#endif //H7_FRAMEWORK_VMC_CONTROL_H
