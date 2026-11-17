// encoding UTF-8
/******************************************************************************
;  *       @模块                 : motor（电机）
;  *       @职责                 : 档位（占空比）管理、正/反转通道切换、2min 自动换向
;  *       @上下文               : 全部在主循环里调用；换向死区用 tick 计时，不阻塞
;  *       @依赖                 : bsp.h（PWM 接口）、tick.h（计时）、power.h（开机状态）
;  *
;  *       硬件说明：P16（PWM0）与 P17（PWM1）接电机驱动两端，
;  *                 同一时刻只允许开一路（另一路关）；换向前必须先全关并等待
;  *                 1s 让电机绕组放电，否则会出现"刹车"拉着电机转不动。
;  *
;  *       档位表：GEAR_1=150/172、GEAR_2=160/172、GEAR_3=255(100%)、GEAR_4=0(停)
;  *       换向时机：短按模式键切档时立即换向；否则每 2min 自动换向一次
******************************************************************************/
#include "motor.h"
#include "bsp.h"
#include "tick.h"
#include "power.h"

volatile uint8_t gear_index; /* 当前档位（GEAR_1~GEAR_4） */
uint8_t motor_reverse_req;   /* 请求换向标志（切档时置 1，换向任务消费） */

static uint8_t motor_channel;       /* 当前驱动通道：P16 正转 / P17 反转 */
static uint8_t dead_wait;           /* 1=正处于换向死区（两路已关，等放电） */
static uint16_t dead_start_ms;      /* 死区起始时刻（1ms 计数） */
static uint16_t reverse_mark_100ms; /* 上次换向的时刻（100ms 计数） */

/************************************************
;  *    @函数名    : gear_duty
;  *    @参数        : gear 档位
;  *    @返回        : 该档位对应的 PWM 占空比计数值
;  *    @说明      : 用 switch 而不是数组，避免常量表占用 RAM 或代码空间
;  ***********************************************/
static uint8_t gear_duty(uint8_t gear)
{
    switch (gear) {
    case GEAR_1:
        return PWM_GEAR_1;
    case GEAR_2:
        return PWM_GEAR_2;
    case GEAR_3:
        return PWM_GEAR_3;
    default:
        return PWM_GEAR_4; /* GEAR_4 及异常值都按"停"处理 */
    }
}

/************************************************
;  *    @函数名    : motor_apply
;  *    @说明      : 把当前档位/通道真正写到 PWM 寄存器
;  *    @规则      : 占空比 0 → 两路全关；否则只在开机状态下打开当前通道
;  ***********************************************/
static void motor_apply(void)
{
    uint8_t duty = gear_duty(gear_index);

    pwm_set_duty(duty);

    if (0 == duty) {
        pwm_channel_disable_all(); /* 停转档 */
    } else if (0 != device_on) {
        pwm_channel_enable(motor_channel); /* 运行中才输出，关机时保持关闭 */
    }
}

/************************************************
;  *    @函数名    : motor_init
;  *    @说明      : 上电初始化档位/通道/计时（RAM 已被 bsp_init 清零，这里只设非 0 值）
;  *    @注意      : 初始通道选 P17（反转），与原工程的上电行为一致
;  ***********************************************/
void motor_init(void)
{
    gear_index = GEAR_1;
    motor_reverse_req = 0;
    motor_channel = PWM_CHANNEL_P17; /* 与原工程一致：开机先走 P17 */
    dead_wait = 0;
    dead_start_ms = 0;
    reverse_mark_100ms = 0;
}

/************************************************
;  *    @函数名    : motor_start
;  *    @说明      : 开机时调用：回到 P17 通道、档位 1，并开始 2min 换向计时
;  ***********************************************/
void motor_start(void)
{
    motor_channel = PWM_CHANNEL_P17;
    reverse_mark_100ms = tick_now_100ms();
    motor_apply();
}

/************************************************
;  *    @函数名    : motor_gear_next
;  *    @说明      : 短按模式键：档位循环 +1，并把 2min 计时归零、请求立即换向
;  *    @注意      : 换向本身由 motor_reverse_task() 完成（先关断、等 1s、再反向）
;  ***********************************************/
void motor_gear_next(void)
{
    uint8_t next = gear_index + 1;

    if (next >= GEAR_COUNT) {
        next = GEAR_1; /* 4 档循环 */
    }

    gear_index = next;
    motor_apply();

    reverse_mark_100ms = tick_now_100ms();
    motor_reverse_req = 1;
}

/************************************************
;  *    @函数名    : motor_stop
;  *    @说明      : 立即停机（关机/休眠前调用）：两路 PWM 全关并清换向状态
;  ***********************************************/
void motor_stop(void)
{
    pwm_channel_disable_all();
    dead_wait = 0;
    motor_reverse_req = 0;
}

/************************************************
;  *    @函数名    : motor_reverse_task
;  *    @说明      : 非阻塞换向状态机（主循环每轮调用一次）
;  *    @流程      : 1) 关机状态 → 复位死区标志后返回
;  *                 2) 死区中 → 等够 1s 后切换到另一个通道并输出
;  *                 3) 非死区 → 收到换向请求或满 2min → 关断两路、进入死区
;  *    @注意      : 死区期间电机断电，但主循环照常跑，按键/ADC 不受影响
;  ***********************************************/
void motor_reverse_task(void)
{
    if (0 == device_on) {
        dead_wait = 0;
        return;
    }

    /* 死区阶段：两路已关闭，等电机放电完成 */
    if (dead_wait) {
        if (elapsed_ms(dead_start_ms) >= MOTOR_DEAD_MS) {
            dead_wait = 0;
            /* 通道取反：P16 ↔ P17 */
            motor_channel = (PWM_CHANNEL_P16 == motor_channel)
                                ? PWM_CHANNEL_P17
                                : PWM_CHANNEL_P16;
            motor_apply();
        }
        return;
    }

    /* 触发换向：切档请求优先，其次看是否满 2min */
    if ((0 != motor_reverse_req) ||
        (elapsed_100ms(reverse_mark_100ms) >= MOTOR_REVERSE_100MS)) {
        motor_reverse_req = 0;
        reverse_mark_100ms = tick_now_100ms();
        pwm_channel_disable_all(); /* 先断电 */
        dead_wait = 1;             /* 进入 1s 死区 */
        dead_start_ms = tick_now_ms();
    }
}
