//
// Created by CaoKangqi on 2026/6/20.
//
#include "Chassis_Ctrl.h"
#include "All_define.h"
#include "Comm_DualBoard.h"
#include "Robot_Config.h"
#include "Power_CAP.h"
#include "Power_Ctrl.h"
#include "Referee.h"
#include "System_State.h"
#include "VMC_Control.h"
#include "Robot_Cmd.h"
#include "All_define.h"

#define REDUCE_RATIO (-1.0f/15.76f)
#define NM_ENCODER (1.0f/(0.315*20)*16384)


Chassis_t chassis_ctrl;
F_Hold_Calc_t f_hold;

PID_t roll={0};
PID_t left_leg_pos={0};
PID_t right_leg_pos={0};
PID_t left_leg_spd={0};
PID_t right_leg_spd={0};

//功率控制
static Power_Ctrl_t chassis_model;
static Motor_Power_State_t m_states[2];//底盘共8个电机

static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge,
                                      float *out_cap_limit);

void F_Hold_Init(F_Hold_Calc_t *f) {
    f->m_body=15.0f;
    f->m_big_leg=0;
    f->m_small_leg=0;
    f->l_air_spring_init = 0.190f;
    f->l_big_leg = 0.215f;
    f->l_small_leg = 0.258f;
    f->s1=0.05217f;
    f->s2=0.05750f;
    f->s3=0.18623f;
    f->air_spring = 120.0f;
    f->k=200.0f/0.0608f;
    f->theta1=12.59*DEG2RAD;
}

void Air_Spring_Calc(F_Hold_Calc_t *f, float length) {
    f->theta2 = acosf((f->l_big_leg*f->l_big_leg + f->l_small_leg*f->l_small_leg - length*length)/
        (2*f->l_big_leg*f->l_small_leg));
    f->theta3 = f->theta2-f->theta1;
    f->theta4 = acosf((f->l_small_leg*f->l_small_leg+length*length-f->l_big_leg*f->l_big_leg)/
        (2.0f*f->l_small_leg*length));

    f->l_air_spring = sqrtf(f->s3*f->s3+f->s2*f->s2-2.0f*cosf(f->theta3)*f->s3*f->s2);
    f->theta5 = acosf((f->s2*f->s2+f->l_air_spring*f->l_air_spring-f->s3*f->s3)/
        (2.0f*f->s2*f->l_air_spring));
    f->theta6 = f->theta5-f->theta4;
    f->air_spring_f = cosf(f->theta6)*(f->air_spring+f->k*(f->l_air_spring_init-f->l_air_spring));
}

void Chassis_Forward_Calc(Chassis_t *chassis_data, Chassis_Motor_Group_t chassis_m) {
    //chassis_data->observer.s=(float)(chassis_m.DJI_3508_Chassis[0].Angle_Infinite-chassis_m.DJI_3508_Chassis[1].Angle_Infinite)*ENCODER_TO_RAD*chassis_data->r*REDUCE_RATIO*0.5f+(sinf(vmc_data.left.theta[0])*vmc_data.left.length[0]+sinf(vmc_data.right.theta[0])*vmc_data.right.length[0])*0.5f;
    chassis_data->observer.dot_s=(float)(chassis_m.DJI_3508_Chassis[0].Speed_now-chassis_m.DJI_3508_Chassis[1].Speed_now)*RPM_TO_RADS*chassis_data->r*REDUCE_RATIO*0.5f+(cosf(chassis_ctrl.vmc.leg.left.theta[0])*chassis_ctrl.vmc.leg.left.theta[1]*chassis_ctrl.vmc.leg.right.length[0]+cosf(chassis_ctrl.vmc.leg.right.theta[0])*chassis_ctrl.vmc.leg.right.theta[1]*chassis_ctrl.vmc.leg.right.length[0])*0.5f;
}

/**
 * @brief 底盘控制初始化
 * @param MOTOR 底盘电机总结构体指针
 * @return uint8_t 初始化状态
 */
uint8_t Chassis_Control_Init(void)
{
    float PID_roll_Pos[3] = {1500.0f,   0.0f,  0.0f};

    float PID_left_leg_Pos[3] = {1500.0f, 0.0f, 0.0f};
    float PID_left_leg_Spd[3] = {25.0f,  0.0f, 0.0f};

    float PID_right_leg_Pos[3] = {1500.0f, 0.0f, 0.0f};
    float PID_right_leg_Spd[3] = {25.0f,  0.0f, 0.0f};

    PID_Init(&roll,100.0f,2.0f,PID_roll_Pos,0,0,0,0,0,Integral_Limit | ErrorHandle);
    PID_Init(&left_leg_pos,100.0f,0.0f,PID_left_leg_Pos,0,0,0,0,0,Integral_Limit | ErrorHandle);
    PID_Init(&left_leg_spd,50.0f,0.0f,PID_left_leg_Spd,0,0,0,0,0,Integral_Limit | ErrorHandle);
    PID_Init(&right_leg_pos,100.0f,0.0f,PID_right_leg_Pos,0,0,0,0,0,Integral_Limit | ErrorHandle);
    PID_Init(&right_leg_spd,50.0f,0.0f,PID_right_leg_Spd,0,0,0,0,0,Integral_Limit | ErrorHandle);


    chassis_ctrl.d=0.2f;
    chassis_ctrl.r=0.06f;
    Coeffs_Init(&chassis_ctrl.mpc_coeffs);
    F_Hold_Init(&f_hold);
    Mpc_Init(&chassis_ctrl.mpc_coeffs,&chassis_ctrl.mpc);
    VMC_Simplify_Init(&chassis_ctrl.vmc,L1_LENGTH,L2_LENGTH,L3_LENGTH,L4_LENGTH);
    Estimator_Leg_Init(&chassis_ctrl.kalman);
    return 1;
}

/**
 * @brief 底盘控制任务
 */
void Chassis_Control_Task(const Chassis_Motor_Group_t *c_motor,const Leg_Motor_Group_t *l_motor, float dt)
{
     if (c_motor == NULL && l_motor == NULL) {
         System_State_Report(ID_CHASSIS, STATUS_ERROR);
         return;
     }
     if (!Is_Group_Online(CHASSIS)) {
         System_State_Report(ID_CHASSIS, STATUS_LOST);
     }
     else{System_State_Report(ID_CHASSIS, STATUS_RUN);}
     // 判断系统状态
     bool is_system_locked = (sys_state.global_mode == GLOBAL_SAFE_LOCK ||
                              sys_state.global_mode == GLOBAL_STANDBY ||
                              sys_state.global_mode == GLOBAL_MODULE_ERROR ||
                              sys_state.global_mode == GLOBAL_INIT_STAGE);
     if (chassis_cmd.mode == CHASSIS_CMD_SAFE || is_system_locked)
     {
         BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
         DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
     }

     else if (chassis_cmd.mode == CHASSIS_CMD_FREE)
     {
         // 底盘跟随云台

         PID_Calculate(&left_leg_pos,chassis_ctrl.vmc.leg.left.length[0],chassis_cmd.target_length);
         PID_Calculate(&right_leg_pos,chassis_ctrl.vmc.leg.right.length[0],chassis_cmd.target_length);
         PID_Calculate(&left_leg_spd,chassis_ctrl.vmc.leg.left.length[1],0);
         PID_Calculate(&right_leg_spd,chassis_ctrl.vmc.leg.right.length[1],0);
         PID_Calculate(&roll,IMU_Data.roll*DEG2RAD,chassis_cmd.target_roll);

         // Air_Spring_Calc(&f_hold,chassis_ctrl.vmc.leg.left.length[0]);
         // Air_Spring_Calc(&f_hold,chassis_ctrl.vmc.leg.right.length[0]);
         f_hold.leg_hold.left_F = -f_hold.air_spring_f+left_leg_pos.Output+left_leg_spd.Output+roll.Output*0.02f;
         f_hold.leg_hold.right_F= -f_hold.air_spring_f+right_leg_pos.Output+right_leg_spd.Output-roll.Output*0.02f;

         VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_l, f_hold.leg_hold.left_F, 0, &chassis_ctrl.vmc.leg.left);
         VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_r, f_hold.leg_hold.right_F, 0, &chassis_ctrl.vmc.leg.right);

         //
         // BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
         // DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
         BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.vmc.leg.left.Tp_front,-chassis_ctrl.vmc.leg.right.Tp_front,chassis_ctrl.vmc.leg.left.Tp_back,-chassis_ctrl.vmc.leg.right.Tp_back);
         DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
     }
     else if (chassis_cmd.mode == CHASSIS_CMD_FOLLOW)
     {
         float F_gravity_stand_l=0;
         float F_gravity_stand_r=0;


         chassis_ctrl.mode=STAND;
         static uint8_t key = 1;
         switch (chassis_ctrl.mode) {
             case LEAP:



                 break;
             case LIE:

                 PID_Calculate(&left_leg_pos,chassis_ctrl.vmc.leg.left.length[0],chassis_cmd.target_length);
                 PID_Calculate(&right_leg_pos,chassis_ctrl.vmc.leg.right.length[0],chassis_cmd.target_length);
                 PID_Calculate(&left_leg_spd,chassis_ctrl.vmc.leg.left.length[1],0);
                 PID_Calculate(&right_leg_spd,chassis_ctrl.vmc.leg.right.length[1],0);
                 PID_Calculate(&roll,IMU_Data.roll*DEG2RAD,chassis_cmd.target_roll);

                 // Air_Spring_Calc(&f_hold,chassis_ctrl.vmc.leg.left.length[0]);
                 // Air_Spring_Calc(&f_hold,chassis_ctrl.vmc.leg.right.length[0]);
                 f_hold.leg_hold.left_F = -f_hold.air_spring_f+left_leg_pos.Output+left_leg_spd.Output+roll.Output*0.02f;
                 f_hold.leg_hold.right_F= -f_hold.air_spring_f+right_leg_pos.Output+right_leg_spd.Output-roll.Output*0.02f;

                 VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_l, f_hold.leg_hold.left_F, 0, &chassis_ctrl.vmc.leg.left);
                 VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_r, f_hold.leg_hold.right_F, 0, &chassis_ctrl.vmc.leg.right);


                 BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.vmc.leg.left.Tp_front,-chassis_ctrl.vmc.leg.right.Tp_front,chassis_ctrl.vmc.leg.left.Tp_back,-chassis_ctrl.vmc.leg.right.Tp_back);
                 DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);
                 // BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
                 // DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);

                 break;
             case STAND:
                 if (chassis_ctrl.vmc.leg.left.theta[0]<1.57 && chassis_ctrl.vmc.leg.left.theta[0]>-1.57)
                     F_gravity_stand_l = cosf(chassis_ctrl.vmc.leg.left.theta[0])*(f_hold.m_body*0.5f)*9.81f;
                 else F_gravity_stand_l = 0.0f;
                 if (chassis_ctrl.vmc.leg.right.theta[0]<1.57 && chassis_ctrl.vmc.leg.right.theta[0]>-1.57)
                     F_gravity_stand_r = cosf(chassis_ctrl.vmc.leg.right.theta[0])*(f_hold.m_body*0.5f)*9.81f;
                 else F_gravity_stand_r = 0.0f;

                 PID_Calculate(&left_leg_pos,chassis_ctrl.vmc.leg.left.length[0],chassis_cmd.target_length);
                 PID_Calculate(&right_leg_pos,chassis_ctrl.vmc.leg.right.length[0],chassis_cmd.target_length);
                 PID_Calculate(&left_leg_spd,chassis_ctrl.vmc.leg.left.length[1],0);
                 PID_Calculate(&right_leg_spd,chassis_ctrl.vmc.leg.right.length[1],0);
                 PID_Calculate(&roll,IMU_Data.roll*DEG2RAD,chassis_cmd.target_roll);

                 // Air_Spring_Calc(&f_hold,chassis_ctrl.vmc.leg.left.length[0]);
                 // Air_Spring_Calc(&f_hold,chassis_ctrl.vmc.leg.right.length[0]);
                 f_hold.leg_hold.left_F = -f_hold.air_spring_f+left_leg_pos.Output+left_leg_spd.Output+roll.Output*0.2f;
                 f_hold.leg_hold.right_F= -f_hold.air_spring_f+right_leg_pos.Output+right_leg_spd.Output-roll.Output*0.2f;

                 if (key){
                     VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_l, f_hold.leg_hold.left_F, chassis_ctrl.mpc.z[0][0], &chassis_ctrl.vmc.leg.left);
                     VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_r, f_hold.leg_hold.right_F, chassis_ctrl.mpc.z[0][1], &chassis_ctrl.vmc.leg.right);

                     DJI_Motor_Send(&hfdcan1,0x200,(int16_t)(chassis_ctrl.mpc.z[0][3] *NM_ENCODER),0,(int16_t)(-(chassis_ctrl.mpc.z[0][2]*NM_ENCODER)),0);
                 }

                 else if (!key){
                     VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_l, f_hold.leg_hold.left_F, chassis_ctrl.mpc.z[1][0], &chassis_ctrl.vmc.leg.left);
                     VMC_Inverse_Dynamics_Simplify(chassis_ctrl.vmc.JRM_r, f_hold.leg_hold.right_F, chassis_ctrl.mpc.z[1][1], &chassis_ctrl.vmc.leg.right);

                     DJI_Motor_Send(&hfdcan1,0x200,(int16_t)(chassis_ctrl.mpc.z[1][3] *NM_ENCODER),0,(int16_t)(-(chassis_ctrl.mpc.z[1][2]*NM_ENCODER)),0);
                 }
                 BM_Send_torque(&hfdcan2, 0x032, chassis_ctrl.vmc.leg.left.Tp_front,-chassis_ctrl.vmc.leg.right.Tp_front,chassis_ctrl.vmc.leg.left.Tp_back,-chassis_ctrl.vmc.leg.right.Tp_back);
                 // DJI_Motor_Send(&hfdcan1,0x200,(int16_t)(chassis_ctrl.mpc.z[0][2] *NM_ENCODER),0,(int16_t)(-(chassis_ctrl.mpc.z[0][3]*NM_ENCODER)),0);

                 // BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
                 // DJI_Motor_Send(&hfdcan1,0x200,0,0,0,0);

                 break;

         }
         // for (int i = 0; i < 4; i++) {
         //     chassis_ctrl.swerve_fb.steer_angle_rad[i] = (float)c_motor->DJI_6020_Steer[i].Angle_Infinite * ENCODER_TO_RAD;
         //     chassis_ctrl.swerve_fb.steer_rpm[i]       = (float)c_motor->DJI_6020_Steer[i].Speed_now;
         //     chassis_ctrl.swerve_fb.wheel_rpm[i]       = (float)c_motor->DJI_3508_Chassis[i].Speed_now;
         }
//
//         Swerve_Forward_Calc(&S_Now, &chassis_ctrl.swerve_fb);
//
//         float vx_tar = chassis_cmd.target_vx;
//         float vy_tar = chassis_cmd.target_vy;
//         float vw_tar = chassis_cmd.target_vw;
//
//         PID_Calculate(&chassis_ctrl.PID_Vx, S_Now.vx, vx_tar);
//         PID_Calculate(&chassis_ctrl.PID_Vy, S_Now.vy, vy_tar);
//         PID_Calculate(&chassis_ctrl.PID_Vw, S_Now.vw, vw_tar);
//
//         Swerve_Inverse_Calc(&chassis_ctrl.swerve_cmd, &S_Now,
//                             chassis_ctrl.PID_Vx.Output, chassis_ctrl.PID_Vy.Output, chassis_ctrl.PID_Vw.Output,
//                             vx_tar, vy_tar, vw_tar,
//                             &chassis_ctrl.swerve_fb);
//
//         for (int i = 0; i < 4; i++)
//         {
//             PID_Calculate(&chassis_ctrl.Steer_P[i],
//                           chassis_ctrl.swerve_fb.steer_angle_rad[i],
//                           chassis_ctrl.swerve_cmd.target_steer_angle_rad[i]);
//
//             PID_Calculate(&chassis_ctrl.Steer_S[i],
//                           chassis_ctrl.swerve_fb.steer_rpm[i],
//                           chassis_ctrl.Steer_P[i].Output);
//
//             PID_Calculate(&chassis_ctrl.Drive_S[i],
//                           chassis_ctrl.swerve_fb.wheel_rpm[i],
//                           chassis_ctrl.swerve_cmd.target_wheel_rpm[i]);
//
//             chassis_ctrl.Drive_S[i].Output += chassis_ctrl.swerve_cmd.ff_torque_raw[i];
//
//             chassis_ctrl.Steer_S[i].Output = MATH_Limit_float(chassis_ctrl.Steer_S[i].Output, -16384, 16384);
//             chassis_ctrl.Drive_S[i].Output = MATH_Limit_float(chassis_ctrl.Drive_S[i].Output, -16384, 16384);
//         }
//
//         for(int i=0; i<4; i++) {
//             m_states[i].speed_rpm = chassis_ctrl.swerve_fb.wheel_rpm[i];
//             m_states[i].original_cmd = chassis_ctrl.Drive_S[i].Output;
//
//             m_states[i+4].speed_rpm = chassis_ctrl.swerve_fb.steer_rpm[i];
//             m_states[i+4].original_cmd = chassis_ctrl.Steer_S[i].Output;
//         }
//
//         bool trigger_discharge = true;
//         float cap_board_limit = 0.0f;
//         float final_limit = 0.0f;
//         if (Referee.offline.is_online) {
//             final_limit = Chassis_Power_Arbitrator(
//                                     Referee.robot_status.chassis_power_limit,
//                                     Referee.power_heat_data.buffer_energy,
//                                     1, &cap, &trigger_discharge, &cap_board_limit);
//         }
//         else {
//             trigger_discharge = FALSE;
//             cap_board_limit = 75.0f;//
//             final_limit = 75.0f;
//         }
//         Power_Ctrl_Calculate(&chassis_model, final_limit, pwr_groups, 2);
//
//         for(int i=0; i<4; i++) {
//             chassis_ctrl.Drive_S[i].Output = m_states[i].limited_cmd;
//             chassis_ctrl.Steer_S[i].Output = m_states[i+4].limited_cmd;
//         }
//
//         CapSetData_t cap_cmd = {0};
//         cap_cmd.Control.power_key     = 1;
//         cap_cmd.Control.capPowerLimit = (uint8_t)cap_board_limit;
//         cap_cmd.Control.buffer_now    = (uint8_t)Referee.power_heat_data.buffer_energy;
//         cap_cmd.Control.robot_state   = (Referee.robot_status.current_HP > 0) ? 1 : 0;
//         Power_Cap_Tx(&hfdcan2, 0x252, &cap_cmd);
//     }
//     //电流发送
//     if (!is_system_locked)
//     {
//         DJI_Motor_Send(&hfdcan1, 0x200,
//                        (int16_t)chassis_ctrl.Drive_S[0].Output,
//                        (int16_t)chassis_ctrl.Drive_S[1].Output,
//                        (int16_t)chassis_ctrl.Drive_S[2].Output,
//                        (int16_t)chassis_ctrl.Drive_S[3].Output);
//
//         DJI_Motor_Send(&hfdcan2, 0x1FE,
//                        (int16_t)chassis_ctrl.Steer_S[0].Output,
//                        (int16_t)chassis_ctrl.Steer_S[1].Output,
//                        (int16_t)chassis_ctrl.Steer_S[2].Output,
//                        (int16_t)chassis_ctrl.Steer_S[3].Output);
//     }
}

// 超级电容与缓冲能量调参宏定义
#define BUFFER_COMP_KP      2.0f    // 缓冲能量补偿的比例系数 (Kp)
#define TARGET_BUFFER       40.0f   // 目标期望缓冲能量 (J)
#define MIN_CAP_VOLTAGE     23.0f   // 超级电容最低放电阈值 (百分比)
#define RAMP_CAP_VOLTAGE    27.0f   // 斜坡衰减开始阈值 (百分比)
#define MAX_BOOST_POWER     150.0f  // 超级电容输出的最大冲刺功率 (W)

/**
 * @brief 功率策略仲裁器
 * * @param base_power_limit  裁判系统当前的基础功率上限
 * @param cur_buffer        裁判系统当前剩余的缓冲能量 (0~60J)
 * @param boost_intent      输入指令是否开启超电
 * @param cap_data          超级电容状态反馈 (包含在线状态、电量、故障码等)
 * @param out_discharge     [输出参数] 发送给超电是否开启
 * @param out_cap_limit     [输出参数] 发送给超电的功率限制
 * * @return float            返回最终决定的目标功率上限 (W)
 */
static float Chassis_Power_Arbitrator(float base_power_limit,
                                      float cur_buffer,
                                      bool boost_intent,
                                      const Cap_t *cap_data,
                                      bool *out_discharge,
                                      float *out_cap_limit)
{
    // 公式: power_comp = -Kp * (目标缓冲 - 当前缓冲)
    // float power_comp = -BUFFER_COMP_KP * (TARGET_BUFFER - cur_buffer);
    // float base_allowable_power = base_power_limit + power_comp;
    // // 发给电容的功率限制
    // *out_cap_limit = base_allowable_power;
    // // 电机的目标功率上限初始化为基础功率
    // float final_target_power = base_allowable_power;
    // // 超级电容离线/硬件故障保护
    // if (cap_data->get.offline.is_online == 0 || cap_data->get.cap_state != 0)
    // {
    //     *out_discharge = false;
    //     return final_target_power - 5.0f;
    // }
    // // 在线且正常状态下的 放电/充电 逻辑
    // if (boost_intent && cap_data->get.Cap_Capacity > MIN_CAP_VOLTAGE)
    // {
    //     float boost_allowance = MAX_BOOST_POWER;
    //     // 斜坡衰减保护机制
    //     if (cap_data->get.Cap_Capacity < RAMP_CAP_VOLTAGE) {
    //         float ratio = (float)(cap_data->get.Cap_Capacity - MIN_CAP_VOLTAGE) /
    //                       (float)(RAMP_CAP_VOLTAGE - MIN_CAP_VOLTAGE);
    //         boost_allowance *= ratio;
    //     }
    //     // 最终允许的底盘功率上限 = 基础可用功率 + 超电补偿功率
    //     final_target_power += boost_allowance;
    //     *out_discharge = true;
    // }
    // else
    // {
    //     // 留 5W 功率给超级电容充电
    //     final_target_power -= 5.0f;
    //     *out_discharge = false;
    // }
    // return final_target_power; // 返回给电机的最终功率限制
}


