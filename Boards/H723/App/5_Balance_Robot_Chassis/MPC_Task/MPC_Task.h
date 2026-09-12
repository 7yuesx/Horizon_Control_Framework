//
// Created by qza on 2026/8/29.
//

#ifndef HORIZON_CONTROL_FRAMEWORK_MPC_H
#define HORIZON_CONTROL_FRAMEWORK_MPC_H

#include "stdint.h"
#include "arm_math.h"
#include "All_define.h"

#define state_num 10
#define input_num 4
#define output_num 10
#define coeffs_num 10
#define prediction_num 8
#define iteration_count 15

typedef struct {
    float P[100];
    float K[40];
    float A[100];
    float B[40];
    float S[16];
    float AmBK_T[100];
    float s[prediction_num+1][10];
    float measure[10];
    float ref[10];
    float e[prediction_num+1][10];
    float u[prediction_num][4];
    float z[prediction_num][4];
    float lambda[prediction_num][4];
    float rho;
    float temp1[prediction_num][4];
    float temp2[4];
    float temp3[100];
    float temp4[40];
    float temp5[10];
    float temp6[40];
    float temp7[4];
    float temp8[4];
    float theta_leg_l;
    float theta_leg_r;
    arm_matrix_instance_f32 P_matrix;
    arm_matrix_instance_f32 K_matrix;
    arm_matrix_instance_f32 A_matrix;
    arm_matrix_instance_f32 B_matrix;
    arm_matrix_instance_f32 S_matrix;
    arm_matrix_instance_f32 AmBK_T_matrix;
    arm_matrix_instance_f32 temp3_matrix;
    arm_matrix_instance_f32 temp4_matrix;
    arm_matrix_instance_f32 temp6_matrix;

}MPC_t;
typedef struct {
    float P_coeffs[1000];
    float K_coeffs[400];
    float A_coeffs[1000];
    float B_coeffs[400];
    float S_coeffs[1000];
    float AmBK_T_coeffs[1000];
    float theta_leg_l_coeffs[10];
    float theta_leg_r_coeffs[10];
    float input[10];
    arm_matrix_instance_f32 P_coeffs_matrix;
    arm_matrix_instance_f32 K_coeffs_matrix;
    arm_matrix_instance_f32 A_coeffs_matrix;
    arm_matrix_instance_f32 B_coeffs_matrix;
    arm_matrix_instance_f32 S_coeffs_matrix;
    arm_matrix_instance_f32 AmBK_T_coeffs_matrix;
}MPC_coeffs_t;

void Coeffs_Init(MPC_coeffs_t* mpc_coeffs);
void Mpc_Init(MPC_coeffs_t* mpc_coeffs,MPC_t* mpc);
void Update_Matrix_Simplify(MPC_coeffs_t* mpc_coeffs, MPC_t* mpc,float x,float y);
void Error_Calculation(MPC_t *mpc, float dt);
void MPC_Admm(MPC_t* mpc);

#endif //HORIZON_CONTROL_FRAMEWORK_MPC_H
