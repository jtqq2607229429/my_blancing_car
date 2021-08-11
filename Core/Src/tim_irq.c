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

extern float kP, kI, kD;

int i = 0;
int motor, dir1 = 0;
float dir2 = 0;
TxPack txpack;
RxPack rxpack;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == htim1.Instance) {
        if (readValuePack(&rxpack)) {
            if (rxpack.bools[0]) {
                save();
            } else {}                                         //保存参数进入flash
            if (rxpack.bytes[0]) {
            } else {
                dir2 = 180.0;
            }                                                 //启动
            motor = rxpack.integers[0];
            dir1 = rxpack.integers[1];
            dir2 = rxpack.integers[2];
            kP = (float) (rxpack.floats[0]);
            kI = (float) (rxpack.floats[1]);
            kD = (float) (rxpack.floats[2]);                              //获得一些设定值
//            PID_Init(&Rp_A_PID, 1, 1, 1);
//            PID_Init(&Rp_P_PID, 1, 1, 1);
        }

        Read_DMP();

        txpack.floats[0] = 20;  //out
        txpack.floats[1] = Pitch;  //实际
        txpack.floats[2] = 0;   //目标
        sendValuePack(&txpack);
    }
    HAL_TIM_Base_Start_IT(htim);
}


