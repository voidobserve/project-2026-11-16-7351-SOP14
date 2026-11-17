// encoding UTF-8
/******************************************************************************
;  *       @模块                 : power（电源管理，对外接口）
;  *       @职责                 : 电池采样判定、充电器检测、统一关机、空闲休眠
;  *       @说明                 : 四个 volatile 状态变量是整机的核心状态，
;  *                               主循环/其它模块都读它们决定行为
******************************************************************************/
#ifndef POWER_H
#define POWER_H

#include "user.h"

extern volatile uint8_t device_on;  /* 1=运行中 */
extern volatile uint8_t heater_on;  /* 1=加热开启 */
extern volatile uint8_t charger_in; /* 1=充电器已插入（插入即强制关机、且不允许开机） */
extern volatile uint8_t low_battery; /* 1=低电量报警中（红灯闪，直到关机） */
extern volatile uint8_t charging;   /* 1=充电器插入且 5052A 正在充电（P03 为低） */
#if (BAT_ABSENT_DETECT_ENABLE)
extern volatile uint8_t battery_absent; /* 1=没装电池（关机态电压≈0 或充电中电压异常高） */

void battery_presence_scan(void); /* 关机态定期检测电池是否安装（主循环调用，内部 200ms 节流） */
#endif

void battery_scan(void);   /* 采一次电池电压并做平均/判定（关机态直接返回） */
void charger_scan(void);   /* 充电器插拔消抖（插入 → power_off） */
void power_task(void);     /* 15min 自动关机 + 执行低压关机请求 */
void power_off(void);      /* 统一关机入口（按键/低压/充电器/自动关机共用） */
void sleep_if_idle(void);  /* 空闲则进入 HOLD2，直到按键或充电器事件 */

#endif
