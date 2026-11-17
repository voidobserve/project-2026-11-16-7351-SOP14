// encoding UTF-8
/******************************************************************************
;  *       @模块                 : key（按键，对外接口）
;  *       @职责                 : 键值枚举、按键事件枚举、扫描接口
;  *       @说明                 : key_scan() 由 Timer3 中断每 10ms 调用一次；
;  *                               key_event 是"单槽"事件变量，主循环取走后清零
******************************************************************************/
#ifndef KEY_H
#define KEY_H

#include "user.h"

/* 物理键值：由引脚电平解析得到 */
enum key_id {
    KEY_NONE = 0, /* 无按键 */
    KEY_MODE,     /* P01：开关机/模式键 */
    KEY_HEAT,     /* P11：加热键 */
};

/* 按键事件：交给主循环处理的动作 */
enum key_event {
    KEY_EVENT_NONE = 0,   /* 无事件 */
    KEY_EVENT_MODE_SHORT, /* 模式键短按（<750ms）：切档 */
    KEY_EVENT_MODE_LONG,  /* 模式键长按（2s）：开机/关机 */
    KEY_EVENT_HEAT_SHORT, /* 加热键短按：开/关加热 */
};

extern volatile uint8_t key_event; /* 中断写入、主循环读取后清零 */

void key_scan(void); /* 10ms 调用一次（必须在定时器中断内调用） */

#endif
