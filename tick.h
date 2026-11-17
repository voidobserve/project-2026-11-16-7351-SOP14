// encoding UTF-8
/******************************************************************************
;  *       @模块                 : tick（时基，对外接口）
;  *       @职责                 : 提供 1ms / 100ms 两个软时钟和超时判断宏
;  *       @说明                 : tick_ms 由 Timer3 中断自增；主循环必须用
;  *                               tick_now_*() 原子读取，不能直接读全局变量
******************************************************************************/
#ifndef TICK_H
#define TICK_H

#include "user.h"

extern volatile uint16_t tick_ms;    /* 1ms 计数（65536ms 回绕） */
extern volatile uint16_t tick_100ms; /* 100ms 计数（6553.6s 回绕） */

void tick_init(void);  /* 配置 1ms 中断并使能 T3IE（GIE 由 main 打开） */
void tick_start(void); /* 休眠唤醒后重启节拍 */
void tick_stop(void);  /* 休眠前停止节拍 */

uint16_t tick_now_ms(void);    /* 关中断读取 tick_ms */
uint16_t tick_now_100ms(void); /* 关中断读取 tick_100ms */

/* 距起始点已经过的时间（16 位减法自动处理回绕） */
#define elapsed_ms(start) ((uint16_t)(tick_now_ms() - (uint16_t)(start)))
#define elapsed_100ms(start) ((uint16_t)(tick_now_100ms() - (uint16_t)(start)))

#endif
