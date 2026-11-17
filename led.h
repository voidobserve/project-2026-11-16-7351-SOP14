// encoding UTF-8
/******************************************************************************
;  *       @模块                 : led（指示灯，对外接口）
;  *       @职责                 : 指示灯周期刷新（白灯=运行，红灯=加热/低电量闪烁）
;  *       @说明                 : 内部是非阻塞状态机，主循环每轮调用一次即可
******************************************************************************/
#ifndef LED_H
#define LED_H

#include "user.h"

void led_task(void);        /* 刷新指示灯状态（主循环，非阻塞） */
void led_breath_start(void); /* 启动充电呼吸灯（配置 Timer2 为 10kHz） */
void led_breath_stop(void);  /* 停止充电呼吸灯（关 Timer2、关红灯） */
void led_breath_tick(void);  /* 呼吸灯软件 PWM（Timer2 中断 100us 调用一次） */

#endif
