//
// Created by JIANG on 2021/8/10.
//

#include "tim_irq.h"
#include "tim.h"
#include "main.h"
#include "oled.h"
#include "hc05.h"
#include "math.h"
#include "mpu6050.h"
#include "stdbool.h"
#include "pid.h"
#include "motor.h"

extern float kP, kI, kD;
extern bool start_flag;
int motor, dir1, dir2 = 0;
float a_pitch, a_gyro;
int out1 = 0, out2 = 0, out = 0;          //初始化参数
int Encoder_Left, Encoder_Right, Encoder;
//extern TxPack txpack;
int t = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        t += 1;
        Encoder_Left = __HAL_TIM_GET_COUNTER(&htim4) - 5000;
        __HAL_TIM_SET_COUNTER(&htim4, 5000);  // 编码器初始化
        Encoder_Right = __HAL_TIM_GET_COUNTER(&htim3) - 5000;
        __HAL_TIM_SET_COUNTER(&htim3, 5000);  // 编码器初始化
        Encoder = Encoder_Right + Encoder_Left - 0;
        if (Read_DMP() == 0 && t >= 200 &&
            start_flag == true)                                       //每5ms读一次6050数据 给6050初始化时间
        {
            a_pitch = get_error(0, Pitch);
            a_gyro = gyro[1];
            out1 = PID_calc_A(&Rp_A_PID, a_pitch, a_gyro);
            out2 = PID_calc_P(&Rp_P_PID, Encoder);
            out = out1 + out2;//out1-out2;
            motor1(-out);
            motor2(out);
        }
        if (Pitch <= -30 || Pitch >= 30) {
            out = 0;
            out1 = 0;
            out2 = 0;
            PID_Clear(&Rp_P_PID);
            t = 0;
            motor1(0);
            motor2(0);
        }   //停止
    }
    HAL_TIM_Base_Start_IT(htim);
}

//            Encoder_Left = __HAL_TIM_GET_COUNTER(&htim4) - 32768;
//              Encoder_Right = 32768 - __HAL_TIM_GET_COUNTER(&htim3);
//           __HAL_TIM_SET_COUNTER(&htim3, 32768);  // 编码器初始化
//        out2 = PID_calc_P(&Rp_P_PID, Encoder_Right + Encoder_Left - 0);
//
//        sendValuePack(&txpack);