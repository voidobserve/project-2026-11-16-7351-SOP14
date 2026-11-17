// encoding UTF-8
/******************************************************************************
;  *       @型号                 : MC32P7351
;  *       @说明                 : YK-08 SOP14 快充板主程序（按键事件处理 + 主循环）
;  *       @硬件                 : 充电由 5052A 充电芯片完成，MCU 只检测充电器插拔；
;  *                               电机 P16/P17、加热 P12、白灯 P14、红灯 P04
;  *
;  *       主循环节奏：每轮约 1ms（1 次 ADC 转换 + 各任务状态机），
;  *                   所有任务都是非阻塞的，禁止在运行期使用 delay
;  *
;  *       任务顺序说明：
;  *         1) handle_key_event  先消化按键事件（开机/关机/切档/加热）
;  *         2) battery_scan      采一次电池电压（关机态直接返回）
;  *         3) charger_scan      充电器插拔消抖（插入即关机）
;  *         4) motor_reverse_task 换向状态机（切档或 2min 触发）
;  *         5) power_task        15min 自动关机 / 低压关机执行
;  *         6) led_task          指示灯显示（含低电量闪烁）
;  *         7) sleep_if_idle     满足条件就进 HOLD2（唤醒后返回本循环）
;  *         8) wdt_feed          喂狗
******************************************************************************/
#include "user.h"
#include "bsp.h"
#include "tick.h"
#include "key.h"
#include "motor.h"
#include "power.h"
#include "led.h"

/************************************************
;  *    @函数名    : handle_key_event
;  *    @说明      : 消费 key_event，处理开机/关机/切档/加热四类动作
;  *    @上下文    : 主循环（每轮调用），非中断上下文
;  *    @规则      :
;  *      关机态：只有"长按模式键"能开机，且插着充电器时不允许开机
;  *      运行态：长按模式键 → 关机；短按模式键 → 切档；短按加热键 → 开关加热
;  *    @注意      : 事件读出后立即清零，避免同一次按键被处理两次
;  ***********************************************/
static void handle_key_event(void)
{
    uint8_t event = key_event;

    if (KEY_EVENT_NONE == event) {
        return;
    }

    key_event = KEY_EVENT_NONE;

    if (0 == device_on) {
        /* 关机态：只有长按开机；插着充电器或没装电池时不允许开机 */
#if (BAT_ABSENT_DETECT_ENABLE)
        if ((KEY_EVENT_MODE_LONG == event) && (0 == charger_in) &&
            (0 == battery_absent)) {
#else
        if ((KEY_EVENT_MODE_LONG == event) && (0 == charger_in)) {
#endif
            device_on = 1;
            heater_on = 0;
            low_battery = 0;
            gear_index = GEAR_1;
            motor_start(); /* 档位 1 + P17 通道，开始 2min 换向计时 */
        }
        return;
    }

    switch (event) {
    case KEY_EVENT_MODE_LONG:
        power_off(); /* 运行中长按 2s → 关机（关机流程统一走 power_off） */
        break;

    case KEY_EVENT_MODE_SHORT:
        motor_gear_next(); /* 切档 + 请求换向 */
        break;

    case KEY_EVENT_HEAT_SHORT:
        heater_on = (0 == heater_on) ? 1 : 0; /* 取反 */
        heater_switch(heater_on);             /* 立即驱动 P12 */
        break;

    default:
        break;
    }
}

/************************************************
;  *    @函数名    : main
;  *    @说明      : 初始化各模块后进入永不退出的主循环
;  *    @初始化    : bsp_init（清 RAM + IO/PWM/ADC）→ tick_init（1ms 节拍）
;  *                 → motor_init（档位/通道）→ 开总中断
;  *    @注意      : bsp_init 会清掉全部 RAM，所以其它模块的初值都必须在它之后设置
;  ***********************************************/
void main(void)
{
    bsp_init(); /* 含 CLR_RAM：所有全局/静态变量初始为 0 */
    tick_init();
    motor_init();

    GIE = 1; /* 初始化完成，允许中断（1ms 节拍开始走动） */

    while (1) {
        handle_key_event();
        battery_scan();
#if (BAT_ABSENT_DETECT_ENABLE)
        battery_presence_scan(); /* 关机态每 200ms 检测一次电池是否在位 */
#endif
        charger_scan();
        motor_reverse_task();
        power_task();
        led_task();
        sleep_if_idle();
        wdt_feed();
    }
}
