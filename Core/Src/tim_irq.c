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

int motor, dir1, dir2 = 0;
int out1, out2;          //初始化参数

bool start_flag = false;  //启动

TxPack txpack;
RxPack rxpack;  //蓝牙使用结构体

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        Read_DMP(); //每5ms读一次6050数据
        if (Pitch >= -33 && Pitch <= 37 && start_flag == true) {
            out1 = PID_calc_A(&Rp_A_PID, get_error(2, Pitch), gyro[1]);
            motor1(-out1);
            motor2(out1);
        } else {
            out1 = 0;
            motor1(-out1);
            motor2(out1);
            PID_Clear(&Rp_A_PID);
        }
        if (readValuePack(&rxpack)) {          //读蓝牙传输数据
            motor = rxpack.integers[0];
            dir1 = rxpack.integers[1];
            dir2 = rxpack.integers[2];
            kP = (float) (rxpack.floats[0]);
            kI = (float) (rxpack.floats[1]);
            kD = (float) (rxpack.floats[2]);                  //获得一些设定值

            if (rxpack.bools[0]) {                            //将参数存入flash
                save();
            }

            if (rxpack.bytes[0]) {                            //启动
                start_flag = true;
                PID_Init(&Rp_A_PID, kP, kI, kD);
                PID_Init(&Rp_P_PID, kP, kI, kD);
            } else {
                start_flag = false;
            }
        }   //读蓝牙数据
        txpack.floats[0] = kP;  //目标
        txpack.floats[1] = out1;  //实际
        txpack.floats[2] = get_error(2, Pitch);   //输出
        sendValuePack(&txpack);
    }
    HAL_TIM_Base_Start_IT(htim);
}

