// encoding UTF-8
/******************************************************************************
;  *       @模块                 : motor（电机，对外接口）
;  *       @职责                 : 档位状态、开机启动、切档、停机、非阻塞换向
;  *       @说明                 : 同一时刻只驱动 P16/P17 中的一路；
;  *                               换向过程是"关断 → 死区 1s → 反向"，由主循环推进
******************************************************************************/
#ifndef MOTOR_H
#define MOTOR_H

#include "user.h"

/* 档位：GEAR_1~GEAR_3 为运行档，GEAR_4 为停转档 */
enum gear {
    GEAR_1 = 0,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_COUNT, /* 档位总数，用于循环 */
};

extern volatile uint8_t gear_index; /* 当前档位 */
extern uint8_t motor_reverse_req;   /* 请求换向（切档时置 1） */

void motor_init(void);       /* 初始化档位/通道（上电调用一次） */
void motor_start(void);      /* 开机：档位 1 + P17 通道，开始 2min 计时 */
void motor_gear_next(void);  /* 短按模式键：切下一档并请求换向 */
void motor_stop(void);       /* 立即停机（关机/休眠） */
void motor_reverse_task(void); /* 换向状态机，主循环每轮调用 */

#endif
