//
// Created by qza on 2026/8/29.
//

#include "MPC_Task.h"
#include "arm_math.h"
#include "VMC_Control.h"
#include "Robot_Cmd.h"
#include "IMU_Task.h"
#include "Chassis_Ctrl.h"

void Coeffs_Init(MPC_coeffs_t* mpc_coeffs) {
    static float a_coeffs[1000]=A_Coeffs_Def;
    static float b_coeffs[400]=B_Coeffs_Def;
    static float p_coeffs[1000]=P_Coeffs_Def;
    static float k_coeffs[400]=K_Coeffs_Def;
    static float s_coeffs[160]=S_Coeffs_Def;
    static float ambk_t_coeffs[1000]=AmBK_T_Coeffs_Def;
    static float theta_leg_l_coeffs[10]=theta_l_Coeffs_Def;
    static float theta_leg_r_coeffs[10]=theta_r_Coeffs_Def;
    memcpy(mpc_coeffs->A_coeffs,a_coeffs,sizeof(a_coeffs));
    memcpy(mpc_coeffs->B_coeffs,b_coeffs,sizeof(b_coeffs));
    memcpy(mpc_coeffs->P_coeffs,p_coeffs,sizeof(p_coeffs));
    memcpy(mpc_coeffs->K_coeffs,k_coeffs,sizeof(k_coeffs));
    memcpy(mpc_coeffs->S_coeffs,s_coeffs,sizeof(s_coeffs));
    memcpy(mpc_coeffs->AmBK_T_coeffs,ambk_t_coeffs,sizeof(ambk_t_coeffs));
    memcpy(mpc_coeffs->theta_leg_l_coeffs,theta_leg_l_coeffs,sizeof(theta_leg_l_coeffs));
    memcpy(mpc_coeffs->theta_leg_r_coeffs,theta_leg_r_coeffs,sizeof(theta_leg_r_coeffs));
}
void Mpc_Init(MPC_coeffs_t* mpc_coeffs,MPC_t* mpc) {
    mpc->Q[0]=10;
    mpc->Q[1]=1;
    mpc->Q[2]=5000;
    mpc->Q[3]=4000;
    mpc->Q[4]=4000;
    mpc->Q[5]=1;
    mpc->Q[6]=1;
    mpc->Q[7]=1;
    mpc->Q[8]=1;
    mpc->Q[9]=1;

    arm_mat_init_f32(&mpc_coeffs->K_coeffs_matrix,40,10,mpc_coeffs->K_coeffs);
    arm_mat_init_f32(&mpc_coeffs->P_coeffs_matrix,100,10,mpc_coeffs->P_coeffs);
    arm_mat_init_f32(&mpc_coeffs->B_coeffs_matrix,40,10,mpc_coeffs->B_coeffs);
    arm_mat_init_f32(&mpc_coeffs->A_coeffs_matrix,100,10,mpc_coeffs->A_coeffs);
    arm_mat_init_f32(&mpc_coeffs->S_coeffs_matrix,16,10,mpc_coeffs->S_coeffs);
    arm_mat_init_f32(&mpc_coeffs->AmBK_T_coeffs_matrix,100,10,mpc_coeffs->AmBK_T_coeffs);

    arm_mat_init_f32(&mpc->K_matrix,4,10,mpc->K);
    arm_mat_init_f32(&mpc->P_matrix,10,10,mpc->P);
    arm_mat_init_f32(&mpc->A_matrix,10,10,mpc->A);
    arm_mat_init_f32(&mpc->B_matrix,10,4,mpc->B);
    arm_mat_init_f32(&mpc->S_matrix,4,4,mpc->S);
    arm_mat_init_f32(&mpc->AmBK_T_matrix,10,10,mpc->AmBK_T);

    arm_mat_init_f32(&mpc->temp3_matrix,10,10,mpc->temp3);
    arm_mat_init_f32(&mpc->K_T_matrix,10,4,mpc->K_T);
    arm_mat_init_f32(&mpc->B_T_matrix,4,10,mpc->B_T);

    memset(mpc->e, 0, sizeof(mpc->e));
    memset(mpc->u, 0, sizeof(mpc->u));
    memset(mpc->z, 0, sizeof(mpc->z));
    memset(mpc->lambda, 0, sizeof(mpc->lambda));
    memset(mpc_coeffs->input, 0, sizeof(mpc_coeffs->input));
    mpc->rho=0.5f;
}

void Update_Matrix_Simplify(MPC_coeffs_t* mpc_coeffs, MPC_t* mpc,const float x,const float y) {
    const float x2 = x*x;
    const float xy = x*y;
    const float y2 = y*y;
    const float x3 = x2*x;
    const float x2y = x2*y;
    const float xy2 = x*y2;
    const float y3 = y*y2;

    mpc_coeffs->input[0]=1;
    mpc_coeffs->input[1]=x;
    mpc_coeffs->input[2]=y;
    mpc_coeffs->input[3]=x2;
    mpc_coeffs->input[4]=xy;
    mpc_coeffs->input[5]=y2;
    mpc_coeffs->input[6]=x3;
    mpc_coeffs->input[7]=x2y;
    mpc_coeffs->input[8]=xy2;
    mpc_coeffs->input[9]=y3;

    arm_mat_vec_mult_f32(&mpc_coeffs->P_coeffs_matrix,mpc_coeffs->input,mpc->P_matrix.pData);
    arm_mat_vec_mult_f32(&mpc_coeffs->K_coeffs_matrix,mpc_coeffs->input,mpc->K_matrix.pData);
    arm_mat_vec_mult_f32(&mpc_coeffs->A_coeffs_matrix,mpc_coeffs->input,mpc->A_matrix.pData);
    arm_mat_vec_mult_f32(&mpc_coeffs->B_coeffs_matrix,mpc_coeffs->input,mpc->B_matrix.pData);
    arm_mat_vec_mult_f32(&mpc_coeffs->S_coeffs_matrix,mpc_coeffs->input,mpc->S_matrix.pData);
    arm_mat_vec_mult_f32(&mpc_coeffs->AmBK_T_coeffs_matrix,mpc_coeffs->input,mpc->AmBK_T_matrix.pData);

    arm_mat_trans_f32(&mpc->K_matrix,&mpc->K_T_matrix);
    arm_mat_trans_f32(&mpc->B_matrix,&mpc->B_T_matrix);

    float theta_leg_l = 0;
    float theta_leg_r = 0;
    for (int i=0;i<10;i++) {
        theta_leg_l += mpc_coeffs->theta_leg_l_coeffs[i]*mpc_coeffs->input[i];
        theta_leg_r += mpc_coeffs->theta_leg_r_coeffs[i]*mpc_coeffs->input[i];
    }
    mpc->theta_leg_l = theta_leg_l;
    mpc->theta_leg_r = theta_leg_r;
}

void Get_State_And_Target(float *measure, float *ref, const float dt) {
    measure[0] = chassis_ctrl.kalman.outstate.s;
    measure[1] = IMU_Data.yaw*DEG2RAD;
    measure[2] = IMU_Data.pitch*DEG2RAD;
    measure[3] = chassis_ctrl.vmc.leg.left.theta[0];
    measure[4] = chassis_ctrl.vmc.leg.right.theta[0];
    measure[5] = chassis_ctrl.kalman.outstate.dot_s;
    measure[6] = IMU_Data.gyro[2];
    measure[7] = IMU_Data.gyro[1];
    measure[8] = chassis_ctrl.vmc.leg.left.theta[1];
    measure[9] = chassis_ctrl.vmc.leg.right.theta[1];

    ref[0] += chassis_cmd.target_vx* dt;;
    ref[1] += chassis_cmd.target_vw* dt;
    ref[1] = normalize_to_pi(ref[1]);
    ref[1] = measure[1]+normalize_to_pi(ref[1]-measure[1]);
    ref[2] = 0;
    ref[3] = chassis_ctrl.mpc.theta_leg_l;
    ref[4] = chassis_ctrl.mpc.theta_leg_r;
    ref[5] = chassis_cmd.target_vx;
    ref[6] = chassis_cmd.target_vw;
    ref[7] = 0;
    ref[8] = 0;
    ref[9] = 0;
}

void Error_Calculation(MPC_t* mpc, const float dt) {
    Get_State_And_Target(mpc->measure,mpc->ref, dt);
    arm_sub_f32(mpc->measure,mpc->ref,mpc->e[0],10);
}

void MPC_Admm(MPC_t* mpc) {
    for(int i=0;i<prediction_num;i++) {
        arm_mat_vec_mult_f32(&mpc->K_matrix,mpc->e[i],mpc->temp2);

        arm_clip_f32(&mpc->temp2[0],&mpc->z[i][0],-40,40,2);
        arm_clip_f32(&mpc->temp2[2],&mpc->z[i][2],-6,6,2);

        arm_mat_vec_mult_f32(&mpc->B_matrix,mpc->z[i],mpc->temp5);
        arm_mat_vec_mult_f32(&mpc->A_matrix,mpc->e[i],mpc->e[i+1]);
        arm_add_f32(mpc->temp5,mpc->e[i+1],mpc->e[i+1],10);

    }
    for(int i=0;i<iteration_count;i++) {
        arm_mat_vec_mult_f32(&mpc->P_matrix,mpc->e[prediction_num],mpc->s[prediction_num]);
        for(int j=0;j<prediction_num;j++) {
            arm_mat_vec_mult_f32(&mpc->AmBK_T_matrix,mpc->s[prediction_num-j],mpc->temp4);
            arm_mult_f32(mpc->Q, mpc->e[prediction_num-1-j], mpc->temp5, 10);
            arm_add_f32(mpc->temp4,mpc->temp5,mpc->temp6,10);

            arm_scale_f32(mpc->z[prediction_num-1-j],mpc->rho,mpc->temp2,4);
            arm_sub_f32(mpc->lambda[prediction_num-1-j],mpc->temp2,mpc->temp1[prediction_num-1-j],4);

            arm_mat_vec_mult_f32(&mpc->K_T_matrix,mpc->temp1[prediction_num-1-j],mpc->temp5);
            arm_add_f32(mpc->temp5,mpc->temp6,mpc->s[prediction_num-1-j],10);
        }
        for(int j=0;j<prediction_num;j++) {
            arm_mat_vec_mult_f32(&mpc->K_matrix,mpc->e[j],mpc->temp8);

            arm_mat_vec_mult_f32(&mpc->B_T_matrix,mpc->s[j+1],mpc->temp2);
            arm_add_f32(mpc->temp2,mpc->temp1[j],mpc->temp7,4);
            arm_mat_vec_mult_f32(&mpc->S_matrix,mpc->temp7,mpc->temp2);
            arm_sub_f32(mpc->temp8,mpc->temp2,mpc->u[j],4);

            arm_mat_vec_mult_f32(&mpc->B_matrix,mpc->u[j],mpc->temp5);
            arm_mat_vec_mult_f32(&mpc->A_matrix,mpc->e[j],mpc->e[j+1]);
            arm_add_f32(mpc->e[j+1],mpc->temp5,mpc->e[j+1],10);


            arm_scale_f32(mpc->lambda[j],(1.0f/mpc->rho),mpc->temp2,4);
            arm_add_f32(mpc->u[j],mpc->temp2,mpc->temp7,4);
            arm_clip_f32(&mpc->temp7[0],&mpc->z[j][0],-40,40,2);
            arm_clip_f32(&mpc->temp7[2],&mpc->z[j][2],-6,6,2);

            arm_sub_f32(mpc->u[j],mpc->z[j],mpc->temp2,4);
            arm_scale_f32(mpc->temp2,mpc->rho,mpc->temp7,4);
            arm_add_f32(mpc->temp7,mpc->lambda[j],mpc->lambda[j],4);
        }

    }

}