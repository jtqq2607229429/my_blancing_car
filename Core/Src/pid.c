//
// Created by JIANG on 2021/7/9.
//
#include "PID.h"

PID_struct BC_A_PID;
PID_struct BC_P_PID;
PID_struct BC_T_PID;

float get_error(float target, float now) {
    float error;
    error = target - now;
    return error;
}

void PID_Init(PID_struct *PID, float kp, float ki, float kd)//  pid参数设置
{
    PID->err = 0;    //PID误差
    PID->err_last = 0;   //上一次误差
    PID->integral = 0;  //累计误差
    PID->Kp = kp;
    PID->Ki = ki;
    PID->Kd = kd;
    PID->out = 0;
}

int PID_calc_A(PID_struct *PID, float e, float g) {
    int out;
    out = (int) (e * PID->Kp + g * PID->Kd);         //
    return out;
}

int PID_calc_P(PID_struct *PID, int encode) {
    static float bias;

    bias *= 0.8;
    bias += encode * 0.2;

    PID->integral += bias;
    if (PID->integral >= 10000)
        PID->integral = 10000;
    if (PID->integral <= -10000)
        PID->integral = -10000;

    PID->out = PID->Kp * bias + (PID->Kp / 200.0) * PID->integral + PID->Kd * (bias - PID->err_last);

    PID->err_last = bias;
    return PID->out;
}

int PID_calc_T(PID_struct *PID, float z , float g) {
    int out;
    out = (int)(PID->Kp*z+PID->Kd*g);
    return out;
}

void PID_Clear(PID_struct *PID) {
    PID->err = 0;    //PID误差
    PID->err_last = 0;   //上一次误差
    PID->integral = 0;  //累计误差
    PID->out = 0;
}

//
//float Last_Position;
//float PID_calc_P(PID_struct *PID, float e)  //PID计算
//{
//    float Position_Bias = 0, Position_Differential;
//
//    Position_Bias *= 0.7;
//    Position_Bias += e * 0.3;                 // 一阶低通 Bias = 0.8*Bias + 位置差值*0.2
//    Position_Differential = Position_Bias - Last_Position;  // 偏差变化率
//
//    PID->integral += PID->err;
//
//    if (PID->integral >= 10000)
//        PID->integral = 10000;
//    if (PID->integral <= -10000)
//        PID->integral = -10000;
////    if (PID->err >= -1 && PID->err <= 1)
////        PID->integral = 0;
//
//    PID->out = PID->Kp * Position_Bias +
//               PID->Ki * PID->integral +
//               PID->Kd * Position_Differential;
//
//    Last_Position = Position_Bias;
//
//    return PID->out;
//}
//


//float PID_calc_z_y(PID_struct *PID,float e)   //PID计算
//{
//    PID->err =e;
//    PID->integral += PID->err;
//
//    PID->out = PID->Kp * PID->err +
//               PID->Ki * PID->integral +
//               PID->Kd *( PID->err - PID->err_last );
//    PID->err_last = PID->err;
//    return PID->out;
//}