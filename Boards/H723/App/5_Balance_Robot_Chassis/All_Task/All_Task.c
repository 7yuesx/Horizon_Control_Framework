//
// Created by CaoKangqi on 2026/6/14.
//
#include "All_Task.h"
#include "BSP_SPI.h"
#include "Robot_Config.h"
// #include "Catapult_Ctrl.h"
#include "Chassis_Ctrl.h"
#include "DBUS.h"
#include "Power_CAP.h"
#include "Referee.h"
#include "Robot_Cmd.h"
#include "System_State.h"
#include "WS2812.h"
#include "System_Indicator.h"
#include "Vofa.h"
#include "VT13.h"
#include "MPC_Task.h"
#include "VMC_Control.h"

//指令中心任务 200Hz
static uint32_t CMD_DWT_Count = 0;
static float cmd_period_s = 0.0f;
void Command_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(5);//绝对延时5ms

    CMD_DWT_Count = DWT->CYCCNT;
    Robot_Cmd_Init();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        cmd_period_s = DWT_GetDeltaT(&CMD_DWT_Count);
        Robot_Cmd_Update();
    }
}

// IMU任务 中断触发
static TaskHandle_t xIMUTaskHandle = NULL;
static void IMU_Interrupt_Handler(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xIMUTaskHandle != NULL) {
        xTaskNotifyFromISR(xIMUTaskHandle, 0, eIncrement, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
static uint32_t INS_DWT_Count = 0;
static float imu_period_s = 0.0f;
void IMU_Task(void *argument) {
    (void)argument;
    xIMUTaskHandle = xTaskGetCurrentTaskHandle();
    // 向 BSP 层注册中断回调
    BSP_SPI_RegisterIRQCallback(IMU_Interrupt_Handler);
    INS_DWT_Count = DWT->CYCCNT;
    for(;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        imu_period_s = DWT_GetDeltaT(&INS_DWT_Count);
        IMU_Update_Task(&IMU_Data, imu_period_s);
    }
}

//运动控制任务 1000Hz
static IMU_Data_t imu ={0};
static Chassis_Motor_Group_t chassis_m = {0};
static Gimbal_Motor_Group_t gimbal_m = {0};
static Shoot_Motor_Group_t shoot_m = {0};
static uint32_t motor_DWT_Count = 0;
static float motor_period_s = 0.0f;
void Motor_Task(void *argument)
{
    (void)argument;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(2);//绝对延时1ms

    motor_DWT_Count = DWT->CYCCNT;
    Chassis_Control_Init();
    //Shoot_Control_Init();
    BM_EnableDisable(&hfdcan2, 0x02);
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);

        motor_period_s = DWT_GetDeltaT(&motor_DWT_Count);

        Chassis_Control_Task(&chassis_motors,&leg_motors,motor_period_s);

        // BM_Send_torque(&hfdcan2, 0x032, 0,0,0,0);
        // Shoot_Control_Task(&shoot_motors, &gimbal_motors,motor_period_s);
    }
}


// 自定义任务1 1000Hz
static uint32_t TASK1_DWT_Count = 0;
static float TASK1_Period_S = 0.0f;
DWT_Profiler_t DWT_Profiler = {0};
void StartTask01(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(2);//绝对延时4ms
    TASK1_DWT_Count = DWT->CYCCNT;
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);
        TASK1_Period_S = DWT_GetDeltaT(&TASK1_DWT_Count);
        //在这里加代码
        Chassis_Forward_Calc(&chassis_ctrl,chassis_motors);
        Estimator_Task(&chassis_ctrl.kalman,IMU_Data,TASK1_Period_S);
        VMC_task(&chassis_ctrl.vmc,TASK1_Period_S);
        static uint8_t key=1;
        if(key) {
            VOFA_JustFloat(&huart1,15,
                DWT_Profiler.cost_us,
                chassis_ctrl.mpc.z[0][0],
                chassis_ctrl.mpc.z[0][1],
                chassis_ctrl.mpc.z[0][2],
                chassis_ctrl.mpc.z[0][3],
                chassis_ctrl.mpc.s[0][0],
                chassis_ctrl.mpc.s[0][1],
                chassis_ctrl.mpc.s[0][2],
                chassis_ctrl.mpc.s[0][3],
                chassis_ctrl.mpc.s[0][4],
                chassis_ctrl.mpc.s[0][5],
                chassis_ctrl.mpc.s[0][6],
                chassis_ctrl.mpc.s[0][7],
                chassis_ctrl.mpc.s[0][8]);
                key=0;
        }
        else if (!key) {
            VOFA_JustFloat(&huart1,15,DWT_Profiler.cost_us,chassis_ctrl.mpc.z[1][0],chassis_ctrl.mpc.z[1][1],chassis_ctrl.mpc.z[1][2],chassis_ctrl.mpc.z[1][3],
                chassis_ctrl.mpc.s[0][0],
                chassis_ctrl.mpc.s[0][1],
                chassis_ctrl.mpc.s[0][2],
                chassis_ctrl.mpc.s[0][3],
                chassis_ctrl.mpc.s[0][4],
                chassis_ctrl.mpc.s[0][5],
                chassis_ctrl.mpc.s[0][6],
                chassis_ctrl.mpc.s[0][7],
                chassis_ctrl.mpc.s[0][8]);
            key=1;
        }
    }
}

// 自定义任务2 1000Hz
static uint32_t TASK2_DWT_Count = 0;
static float TASK2_Period_S = 0.0f;

void StartTask02(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xTimeIncrement = pdMS_TO_TICKS(4);//绝对延时1ms
    TASK2_DWT_Count = DWT->CYCCNT;

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xTimeIncrement);
        TASK2_Period_S = DWT_GetDeltaT(&TASK2_DWT_Count);

        DWT_Profile_Start(&DWT_Profiler);

        //在这里加代码
        Update_Matrix_Simplify(&chassis_ctrl.mpc_coeffs,&chassis_ctrl.mpc,
            chassis_ctrl.vmc.leg.left.length[0],chassis_ctrl.vmc.leg.right.length[0]);
        Error_Calculation(&chassis_ctrl.mpc,TASK2_Period_S);
        MPC_Admm(&chassis_ctrl.mpc);

        DWT_Profile_Stop(&DWT_Profiler);
    }
}

//定时器中断
void MY_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    //定时器4 1000Hz
    if (htim->Instance == TIM4) {
        DWT_SysTimeUpdate();
        Offline_Monitor();
        System_State_Update(&Referee);
        System_Indicator_Ticks();
    }
    //定时器6 500Hz
    if (htim->Instance == TIM6) {

    }
    //定时器7 200Hz
    if (htim->Instance == TIM7) {

    }
}