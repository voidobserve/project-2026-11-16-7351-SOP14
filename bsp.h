// encoding UTF-8
/******************************************************************************
;  *       @模块                 : bsp（板级支持包，对外接口）
;  *       @职责                 : 引脚映射、指示灯/加热开关宏、ADC/PWM/延时/看门狗接口
;  *       @说明                 : 本头文件把"引脚号/有效电平"集中定义，换板时只改这里；
;  *                               7351 与 7361 两个工程的差异也集中在本文件对应的 .c 中
;  *                               （7361 的模拟/数字选择寄存器叫 P0ADCR/P1ADCR）
******************************************************************************/
#ifndef BSP_H
#define BSP_H

#include "user.h"

/* ---------------- 引脚映射（按 YK-08 SOP14 原理图） ---------------- */
#define PIN_CHARGER_IN P00D /* 充电器插入检测：高=已插入（3K 下拉 + 3.3K 上拉至 5V） */
#define PIN_KEY_MODE P01D   /* 开关机/模式按键：低=按下 */
#define PIN_KEY_HEAT P11D   /* 加热按键：低=按下 */
#define PIN_LED_WHITE P14D  /* 白灯（电源/工作指示）：低电平点亮 */
#define PIN_LED_RED P04D    /* 红灯（加热/低电量）：低电平点亮 */
#define PIN_HEATER P12D     /* 加热控制：高电平加热 */
#define PIN_CHARGE_STATUS P03D /* 5052A 充电芯片状态脚：低=正在充电，高=充满/未充（MCU 只读） */

/* ---------------- 有效电平 ---------------- */
#define LED_LEVEL_ON (0)       /* 指示灯低电平点亮 */
#define LED_LEVEL_OFF (1)
#define HEATER_LEVEL_ON (1)    /* 加热高电平有效 */
#define CHARGE_STATUS_CHARGING (0) /* P03 读到该电平表示正在充电 */

/* ---------------- 指示灯/加热的快捷操作 ---------------- */
#define led_white_on() (PIN_LED_WHITE = LED_LEVEL_ON)
#define led_white_off() (PIN_LED_WHITE = LED_LEVEL_OFF)
#define led_red_on() (PIN_LED_RED = LED_LEVEL_ON)
#define led_red_off() (PIN_LED_RED = LED_LEVEL_OFF)
#define heater_switch(on) (PIN_HEATER = (on) ? HEATER_LEVEL_ON : 0)

/* ---------------- 电机 PWM 通道编号 ---------------- */
#define PWM_CHANNEL_P16 (0) /* P16：正转（PWM0） */
#define PWM_CHANNEL_P17 (1) /* P17：反转（PWM1） */

/* ---------------- 初始化与运行期接口 ---------------- */
void bsp_init(void);      /* 清 RAM + IO/PWM/ADC 初始化（GIE 仍为 0） */
void adc_init(void);      /* ADC 配置 + 1ms 稳定延时（休眠唤醒后需重调） */
uint16_t adc_read_battery(void); /* 读一次电池电压；超时返回 0xFFFF */
void pwm_init(void);      /* Timer0/1 配成 23.3kHz PWM，先不输出 */
void pwm_set_duty(uint8_t duty);     /* 同时设置两路占空比 */
void pwm_channel_enable(uint8_t channel); /* 开指定通道（另一路自动关闭） */
void pwm_channel_disable_all(void);  /* 两路全关（停机/换向死区/休眠） */
void wdt_feed(void);      /* 喂狗 */

#endif
