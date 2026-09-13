//
// Created by CaoKangqi on 2026/6/20.
//

#ifndef H7_FRAMEWORK_CHASSIS_CTRL_H
#define H7_FRAMEWORK_CHASSIS_CTRL_H

#include <stdint.h>
#include "Robot_Config.h"
#include "MPC_Task.h"
#include "VMC_Control.h"
#include "Kalman_Observer.h"
#include "Chassis_Calc.h"
#include "IMU_Task.h"

typedef enum {
    LEAP = 0,    //
    LIE,      //
    STAND,        //
} State_Mode_e;

typedef struct {
    float dot_s;
}Chassis_Observer_t;

typedef struct {
    float d;
    float r;
    uint8_t mode;
    Chassis_Observer_t observer;
    MPC_coeffs_t mpc_coeffs;
    MPC_t mpc;
    VMC_Simplify_t vmc;
    Kalman_Observer_t kalman;
}Chassis_t;

typedef struct {
    float left_F;
    float right_F;
}F_Hold_t;

typedef struct {
    float m_body;
    float m_big_leg;
    float m_small_leg;
    float air_spring;
    float air_spring_f;
    float l_air_spring;
    float l_air_spring_init;
    float l_big_leg;
    float l_small_leg;
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    float theta6;
    float s1;
    float s2;
    float s3;
    float k;
    F_Hold_t leg_hold;
}F_Hold_Calc_t;

extern Chassis_t chassis_ctrl;
extern F_Hold_Calc_t f_hold;

void Chassis_Forward_Calc(Chassis_t *chassis_data, Chassis_Motor_Group_t chassis_m);
uint8_t Chassis_Control_Init(void);
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor,const Leg_Motor_Group_t *l_motor, float dt);
#endif //H7_FRAMEWORK_CHASSIS_CTRL_H
