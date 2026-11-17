// encoding UTF-8
/******************************************************************************
;  *       @模块                 : power（电源管理）
;  *       @职责                 : 电池电压采样与判定（低电量报警 / 低压关机）、
;  *                               充电器插拔检测、统一关机、空闲休眠
;  *       @上下文               : 全部在主循环里调用；sleep_if_idle() 会进入 HOLD2
;  *       @依赖                 : bsp.h（ADC/引脚/灯）、tick.h（计时）、motor.h（停机）
;  *
;  *       电池采样：每轮主循环采 1 次，累计 8 次求平均（约 8~16ms 更新一次），
;  *                 再与阈值比较；阈值由 user.h 的 BAT_MV_TO_RAW() 按分压比算出：
;  *                 6.5V → 580、6.2V → 553
;  *
;  *       判定策略（与原工程一致）：
;  *         低于 6.2V 连续 1s   → 关机
;  *         低于 6.5V 连续 5s   → 低电量报警（红灯闪，一直保持到关机）
;  *         电压恢复            → 取消"待确认"计时（但报警一旦触发不自动取消）
;  *
;  *       休眠策略（HOLD2 + Timer2）：
;  *         条件：已关机 + 无充电器 + 按键未按下
;  *         唤醒：Timer2 用 FLOSC（32kHz/128）每 32ms 溢出一次，仅唤醒不进中断；
;  *               醒来只读两个引脚，没事件立刻继续睡；WDT（1.024s）作为兜底唤醒源
******************************************************************************/
#include "power.h"
#include "bsp.h"
#include "tick.h"
#include "motor.h"

volatile uint8_t device_on;  /* 1=设备运行中（电机可输出） */
volatile uint8_t heater_on;  /* 1=加热开启 */
volatile uint8_t charger_in; /* 1=充电器已插入 */
volatile uint8_t low_battery; /* 1=低电量报警中（红灯闪） */
volatile uint8_t charging;   /* 1=5052A 正在充电（P03 为低），红灯呼吸用 */

#if (BAT_ABSENT_DETECT_ENABLE)
volatile uint8_t battery_absent; /* 1=没装电池（总开关关闭时该变量不存在） */

/******************************************************************************
 * 电池是否安装的检测（替代原工程"开机瞬间打开升压、看电池侧电压是否过高"的做法）
 *
 * 原工程：MCU 自己控制升压（Timer2 PWM），上电先把升压打开，若电池侧电压远高于
 *         满电电压，说明电池没装（没有电池吸收电流，电压被顶到很高）。
 * 现方案：升压由 5052A 硬件芯片完成，MCU 不再控制；但 P02 仍接着电池分压网络，
 *         于是用两个判据：
 *   1) 没插充电器时：分压节点只能由电池驱动（1M 上拉 / 70K 下拉）。
 *        没有电池 → 节点被 70K 拉到 0V，ADC 读数≈0 → 判定"没装电池"；
 *        装了电池 → 读到电池电压（6.0~8.4V 对应 536~750）→ 判定"已装电池"。
 *      这是主判据，稳定可靠，且不需要 MCU 控制任何功率电路。
 *   2) 充电中：电池被拔掉时，充电芯片输出侧没有负载，电压会被顶到 CV 之上，
 *        若持续读到 > 8.7V → 判定"电池被拔掉"。该阈值需按 5052A 实测微调。
 ******************************************************************************/

/************************************************
;  *    @函数名    : battery_presence_check
;  *    @说明      : 采一次电池电压并更新 battery_absent
;  *    @上下文    : 关机态（device_on=0）调用；内部会临时打开 ADC，用完再关掉
;  *    @注意      : 采样区间（40~200）内保持原判断，形成回差，避免临界抖动
;  ***********************************************/
static void battery_presence_check(void)
{
    uint16_t raw;

    adc_init(); /* ADC 平时是关的，这里临时打开并等稳定（约 1ms） */
    raw = adc_read_battery();
    ADEN = 0; /* 用完立刻关掉，降低关机态功耗 */

    if (raw > 0x0FFE) {
        return; /* 转换超时，本次不更新 */
    }

    if (0 == charger_in) {
        if (raw < BAT_ABSENT_RAW_LOW) {
            battery_absent = 1; /* 节点≈0V：电池没装 */
        } else if (raw > BAT_PRESENT_RAW_MIN) {
            battery_absent = 0; /* 读到电池电压：装了电池 */
        }
        /* 介于两者之间：保持原状态 */
    } else {
        if (raw > BAT_ABSENT_RAW_HIGH) {
            battery_absent = 1; /* 充电中电压异常高：电池被拔掉 */
        } else if (raw < BAT_CHARGING_RAW_MIN) {
            battery_absent = 0; /* 电压低于 8.2V，说明确实在给电池充电 */
        }
    }
}

/************************************************
;  *    @函数名    : battery_presence_scan
;  *    @说明      : 关机态电池在位检测（主循环调用，内部按 200ms 节流）
;  *    @上下文    : 只在 device_on=0 时调用；开机后由 battery_scan 负责采样
;  ***********************************************/
void battery_presence_scan(void)
{
    static uint16_t mark_ms;

    if (0 != device_on) {
        return; /* 运行态不检测（battery_scan 正在用 ADC） */
    }

    if (elapsed_ms(mark_ms) < BAT_PRESENCE_SCAN_MS) {
        return;
    }
    mark_ms = tick_now_ms();

    battery_presence_check();
}
#endif /* BAT_ABSENT_DETECT_ENABLE */

static uint16_t battery_sum;   /* 8 次采样的累加和 */
static uint8_t battery_count;  /* 已累计的采样次数 */
static uint16_t battery_raw;   /* 8 次平均后的电池 ADC 值 */

static uint8_t low_active;   /* 1=正在累计"低电量持续时间" */
static uint16_t low_start_ms; /* 低电量计时起点 */
static uint8_t shut_active;  /* 1=正在累计"关机电压持续时间" */
static uint16_t shut_start_ms; /* 关机计时起点 */
static uint8_t shut_request; /* 1=已确认需要关机，等 power_task 执行 */

static uint8_t charger_level; /* 消抖过程中记录的电平 */
static uint8_t charger_count; /* 连续相同次数 */
static uint8_t charger_state; /* 消抖后的稳定状态 */

static uint8_t charge_level; /* 充电状态（P03）消抖中的电平 */
static uint8_t charge_count; /* 充电状态连续相同次数 */

static uint8_t was_on;         /* 上一轮的运行状态（用于捕捉开机瞬间） */
static uint16_t on_mark_100ms; /* 本次开机的时刻（100ms 计数） */

/************************************************
;  *    @函数名    : evaluate_battery
;  *    @说明      : 用平均后的电池值做阈值判定（只在 device_on 时有效）
;  *    @流程      : 1) 关机状态：清所有判定状态，直接返回
;  *                 2) < 关机阈值：累计 1s → 置关机请求（此区间不再报低电量）
;  *                 3) < 低电阈值：累计 5s → 置低电量报警
;  *                 4) 电压回升：清"待确认"计时
;  *    @注意      : 报警标志 low_battery 一旦置 1 不自动清除，
;  *                 只能由 power_off() 清（与原工程行为一致）
;  ***********************************************/
static void evaluate_battery(void)
{
    if (0 == device_on) {
        low_active = 0;
        shut_active = 0;
        low_battery = 0;
        return;
    }

    if (battery_raw < SHUTDOWN_RAW) {
        /* 低于关机电压：优先判关机，不再点亮低电量报警 */
        low_active = 0;

        if (0 == shut_active) {
            shut_active = 1; /* 开始计时 */
            shut_start_ms = tick_now_ms();
        } else if (elapsed_ms(shut_start_ms) >= SHUTDOWN_CONFIRM_MS) {
            shut_request = 1; /* 持续 1s，确认关机 */
        }
        return;
    }

    shut_active = 0; /* 电压回到关机阈值以上，取消关机计时 */

    if (battery_raw < LOW_BATTERY_RAW) {
        if (0 == low_active) {
            low_active = 1;
            low_start_ms = tick_now_ms();
        } else if (elapsed_ms(low_start_ms) >= LOW_BATTERY_CONFIRM_MS) {
            low_battery = 1; /* 持续 5s，确认低电量 */
        }
    } else {
        low_active = 0; /* 电压恢复，取消低电量计时 */
    }
}

/************************************************
;  *    @函数名    : battery_scan
;  *    @说明      : 电池采样 + 8 次滑动平均 + 触发判定
;  *    @上下文    : 主循环每轮调用一次（关机态直接返回，省电）
;  *    @注意      : 采样值 0 或 ≥4095 视为异常（引脚浮空/短路），直接丢弃；
;  *                 累加用 uint16：8 × 4095 = 32760，不会溢出
;  ***********************************************/
void battery_scan(void)
{
    uint16_t raw;

    if (0 == device_on) {
        return; /* 关机态不采样，降低功耗 */
    }

    if (0 == ADEN) {
        adc_init(); /* 关机态的电池在位检测会把 ADC 关掉，这里按需恢复 */
    }

    raw = adc_read_battery();

    if ((0 == raw) || (raw >= 0x0FFF)) {
        return; /* 采样异常丢弃 */
    }

    battery_sum += raw;

    if (++battery_count < BATTERY_SAMPLES) {
        return; /* 还没采够 8 次 */
    }

    battery_count = 0;
    battery_raw = battery_sum >> 3; /* 除以 8（BATTERY_SAMPLES 固定为 8） */
    battery_sum = 0;

    evaluate_battery();
}

/************************************************
;  *    @函数名    : charger_scan
;  *    @说明      : 充电器插拔检测（P00，插入为高电平）+ 充电状态检测（P03）
;  *    @流程      : 连续 3 次读到相同电平才认；插入 → 立即关机；拔出 → 清标志；
;  *                 插入期间用 P03 判断"正在充电 / 已充满"（供红灯呼吸灯使用）
;  *    @注意      : 插着充电器时不允许开机（见 handle_key_event），
;  *                 且 charger_in=1 时不会进入休眠（见 sleep_if_idle）
;  ***********************************************/
void charger_scan(void)
{
    uint8_t level = (0 != PIN_CHARGER_IN) ? 1 : 0;
    uint8_t charge_raw;

    /* ---- 充电状态（P03）：只在插着充电器时有效，做相同的消抖 ---- */
    if (0 == charger_in) {
        charging = 0; /* 没插充电器，充电芯片不工作 */
        charge_level = 0;
        charge_count = 0;
    } else {
        charge_raw = (CHARGE_STATUS_CHARGING == PIN_CHARGE_STATUS) ? 1 : 0;

        if (charge_raw != charge_level) {
            charge_level = charge_raw;
            charge_count = 0;
        } else if (charge_count < CHARGER_DEBOUNCE_COUNT) {
            charge_count++;
        } else {
            charging = charge_raw; /* 稳定后更新：1=正在充电，0=已充满 */
        }
    }

    /* ---- 充电器插拔（P00）---- */
    if (level != charger_level) {
        charger_level = level;
        charger_count = 0;
        return;
    }

    if (charger_count < CHARGER_DEBOUNCE_COUNT) {
        charger_count++;
        return;
    }

    if (level == charger_state) {
        return; /* 状态没变，什么都不做 */
    }

    charger_state = level;
    charger_in = level;

    if (0 != level) {
        /* 刚插入充电器：先按"正在充电"处理（充电芯片通常会立刻拉低 P03），
         * 若确实已经充满，几个消抖周期后会自动切到"充满"显示，避免插入瞬间误显白灯 */
        charging = 1;
        charge_level = 0;
        charge_count = 0;

        power_off(); /* 插入充电器：无论处于什么状态都关机 */
    }
}

/************************************************
;  *    @函数名    : power_task
;  *    @说明      : 运行态的电源相关任务：15min 自动关机 + 执行电池关机请求
;  *    @流程      : 1) 捕捉 device_on 的上升沿，记下开机时刻
;  *                 2) 关机状态：清关机请求，返回
;  *                 3) 有电池关机请求 → 关机
;  *                 4) 开机满 15min → 关机
;  ***********************************************/
void power_task(void)
{
    if (device_on != was_on) {
        was_on = device_on;
        on_mark_100ms = tick_now_100ms(); /* 以运行状态的变化时刻重新计时 */
    }

    if (0 == device_on) {
        shut_request = 0;
        return;
    }

    if (0 != shut_request) {
        shut_request = 0;
        power_off(); /* 低压关机 */
        return;
    }

    if (elapsed_100ms(on_mark_100ms) >= AUTO_OFF_100MS) {
        power_off(); /* 15min 自动关机 */
    }
}

/************************************************
;  *    @函数名    : power_off
;  *    @说明      : 统一关机入口（按键长按/低压/充电器插入/自动关机都走这里）
;  *    @动作      : 清所有运行状态 → 关加热 → 停电机 → 灭灯
;  *    @注意      : 不改变 charger_in（充电器状态由 charger_scan 维护）；
;  *                 关机后 gear 复位到 1 档，下次开机从 150 占空比起步
;  ***********************************************/
void power_off(void)
{
    device_on = 0;
    heater_on = 0;
    low_battery = 0;
    low_active = 0;
    shut_active = 0;
    shut_request = 0;
    was_on = 0;

    heater_switch(0);
    motor_stop();
    led_white_off();
    led_red_off();
}

/************************************************
;  *    @函数名    : sleep_if_idle
;  *    @说明      : 空闲休眠：关机 + 无充电器 + 按键未按下 → HOLD2，直到有事件
;  *    @休眠配置  : HFEN=0（停高频）、LFEN=1（保低频）→ HOLD2 模式；
;  *                 Timer2 选 FLOSC(32kHz)/128 = 250Hz，重载 7 → 32ms 溢出一次，
;  *                 只置中断标志唤醒 CPU（GIE=0，不进中断服务程序）
;  *    @唤醒判断  : 醒来只读 P01（按键）与 P00（充电器），都没有就继续睡
;  *    @恢复      : 重新初始化 ADC、重启 1ms 节拍、开总中断后返回主循环
;  *    @注意      : GIE=0 期间 tick_ms 不走，醒来后计时从原值继续（不影响按键计数）
;  ***********************************************/
void sleep_if_idle(void)
{
#if (BAT_ABSENT_DETECT_ENABLE)
    static uint8_t presence_div; /* 休眠中电池在位检测的分频计数 */
#endif

    if (0 != device_on) {
        return;
    }
    if (0 != charger_in) {
        return;
    }
    if (0 == PIN_KEY_MODE) {
        return; /* 按键仍按着，交给主循环处理 */
    }

    GIE = 0; /* 休眠期间不响应中断 */
    tick_stop();
    pwm_channel_disable_all();
    heater_switch(0);
    led_white_off();
    led_red_off();
    ADEN = 0; /* 关闭 ADC */

    /* 低功耗：P03（充电状态）此时一定没有充电器，关掉内部上拉并改下拉，
     * 避免引脚悬空漏电；唤醒后 adc_init() 会重新打开上拉 */
    P03PU = 0;
    P03PD = 1;

#if (WAKE_SOURCE == WAKE_SOURCE_T2)
    /* 唤醒定时器：Timer2 用低频时钟，休眠中仍可工作 */
    T2CKS0 = 0; /* T2CKS[1:0] = 10：时钟源 FLOSC */
    T2CKS1 = 1;
    T2PRS0 = 1; /* T2PRS[2:0] = 111：1:128 预分频 */
    T2PRS1 = 1;
    T2PRS2 = 1;
    T2LOAD = WAKE_T2_LOAD; /* (7+1) / 250Hz = 32ms */
    T2IF = 0;
    T2IE = 1; /* 只使能"唤醒"，GIE=0 所以不会进中断 */
    T2EN = 1;
#endif

    HFEN = 0; /* HOLD2：高频停止、低频继续工作 */
    LFEN = 1;

    for (;;) {
#if (BAT_ABSENT_DETECT_ENABLE)
        /* 每 8 次唤醒（约 256ms）测一次电池是否在位：
         * 休眠中也要能发现"电池被拔掉/没装"，否则插着充电器时会误判 */
        if (++presence_div >= 8) {
            presence_div = 0;
            battery_presence_check();
        }
#endif

#if (WAKE_SOURCE == WAKE_SOURCE_T2)
        T2IF = 0; /* 清唤醒源标志，否则 STOP 会立刻返回 */
#endif
        Nop();
        Nop();
        Stop(); /* 执行 STOP 会同时清零看门狗 */
        Nop();
        Nop();

        if (0 == PIN_KEY_MODE) {
            break; /* 按键按下 → 回主循环处理开机 */
        }
        if (0 != PIN_CHARGER_IN) {
            break; /* 插入了充电器 → 回主循环消抖后关机 */
        }

#if (WAKE_SOURCE != WAKE_SOURCE_T2)
        break; /* WDT 唤醒：交回主循环重新判断 */
#endif
    }

#if (WAKE_SOURCE == WAKE_SOURCE_T2)
    /* 退出休眠前必须关掉 T2 中断，否则开 GIE 后会反复进中断 */
    T2EN = 0;
    T2IE = 0;
    T2IF = 0;
#endif

    HFEN = 1;    /* 恢复高频时钟 */
    adc_init();  /* ADC 已关闭过，重新配置并等稳定 */
    tick_start(); /* 重启 1ms 节拍 */
    GIE = 1;
}
