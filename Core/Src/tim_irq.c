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

int motor, dir1,dir2 = 0;
int out1,out2;          //初始化参数

bool start_flag = false;  //启动

TxPack txpack;
RxPack rxpack;  //蓝牙使用结构体

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
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

        }

        if (start_flag == true) {
            Read_DMP(); //每10ms读一次6050数据
            out1 = PID_calc_A(&Rp_A_PID, get_error(90, Pitch));
            motor1(out1);
            motor2(out1);
        }

        txpack.floats[0] = out1;  //out
        txpack.floats[1] = Pitch;  //实际
        txpack.floats[2] = 90;   //目标
        sendValuePack(&txpack);
    }
    HAL_TIM_Base_Start_IT(htim);
}


