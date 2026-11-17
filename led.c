// encoding UTF-8
/******************************************************************************
;  *       @模块                 : led（指示灯）
;  *       @职责                 : 白灯/红灯状态显示；低电量闪烁；充电时红灯呼吸
;  *       @上下文               : led_task()        —— 主循环每轮调用（状态机）
;  *                               led_breath_tick() —— Timer2 中断里调用（10kHz 软件 PWM）
;  *       @依赖                 : bsp.h（引脚宏）、tick.h（闪烁计时）、power.h（状态）
;  *
;  *       显示约定：
;  *         白灯（P14）  运行中且未加热 → 亮
;  *         红灯（P04）  加热中 → 常亮
;  *         低电量报警   红灯 200ms 亮 / 200ms 灭，白灯灭（优先于其它显示）
;  *         充电中       红灯呼吸（渐亮渐暗），白灯灭
;  *         充电完成     白灯常亮、红灯灭（P03 高阻被内部上拉读成高）
;  *
;  *       呼吸灯实现（与参考工程一致）：
;  *         P04 没有硬件 PWM，用 Timer2 做软件 PWM：
;  *           Timer2 = FCPU/8 = 1MHz，重载 99 → 每 100us 中断一次（10kHz）
;  *           100 级亮度 → PWM 帧周期 100 × 100us = 10ms（100Hz，看不到闪烁）
;  *           每 200 次中断（20ms）亮度加/减 1 级 → 100 级 = 2s 渐亮 + 2s 渐暗
;  *         只有在"正在充电"时才启动 Timer2，其它时间不占中断资源。
******************************************************************************/
#include "led.h"
#include "bsp.h"
#include "tick.h"
#include "power.h"

static uint16_t blink_mark_ms; /* 低电量闪烁：上次翻转的时刻 */
static uint8_t blink_on;       /* 低电量闪烁：当前相位 1 亮 / 0 灭 */

static uint8_t breath_level;   /* 呼吸灯当前亮度（0~LED_BREATH_LEVELS） */
static uint8_t breath_dir;     /* 呼吸方向：0 渐亮 / 1 渐暗 */
static uint8_t breath_pwm_pos; /* PWM 帧内位置（0~LED_BREATH_LEVELS-1） */
static uint8_t breath_step_div; /* 渐变分频计数（0~LED_BREATH_STEP_TICKS-1） */
static uint8_t breath_running;  /* 1=Timer2 呼吸中断正在运行 */

/************************************************
;  *    @函数名    : led_breath_start
;  *    @说明      : 启动呼吸灯：配置 Timer2 为 10kHz 中断，亮度从 0 开始
;  *    @上下文    : 主循环（led_task 检测到"正在充电"时调用）
;  *    @注意      : Timer2 与休眠唤醒共用，两者互斥（充电时不会休眠）
;  ***********************************************/
void led_breath_start(void)
{
    breath_level = 0;    /* 从最暗开始渐亮 */
    breath_dir = 0;
    breath_pwm_pos = 0;
    breath_step_div = 0;

    /* Timer2：时钟源 FCPU(8MHz)，1:8 预分频 → 1MHz，重载 99 → 100us 溢出 */
    T2CKS0 = 0;
    T2CKS1 = 0;
    T2PRS0 = 1; /* T2PRS[2:0] = 011：1:8 */
    T2PRS1 = 1;
    T2PRS2 = 0;
    T2LOAD = LED_BREATH_T2_LOAD;

    T2IF = 0;
    T2IE = 1;
    T2EN = 1;

    breath_running = 1;
}

/************************************************
;  *    @函数名    : led_breath_stop
;  *    @说明      : 停止呼吸灯：关 Timer2、关红灯
;  *    @上下文    : 主循环（充电结束/拔出充电器时调用）
;  ***********************************************/
void led_breath_stop(void)
{
    T2EN = 0;
    T2IE = 0;
    T2IF = 0;

    breath_running = 0;
    led_red_off();
}

/************************************************
;  *    @函数名    : led_breath_tick
;  *    @说明      : 呼吸灯的软件 PWM + 渐变（每 100us 由 Timer2 中断调用）
;  *    @原理      : 一帧 100 个时隙，亮度 N 表示前 N 个时隙点亮 → 平均亮度 N%
;  *                 每 200 帧（20ms）把 N 加/减 1 → 100 级 → 2s 渐亮 / 2s 渐暗
;  *    @注意      : 只在 breath_running=1 时被调用；ISR 内不含延时/浮点
;  ***********************************************/
void led_breath_tick(void)
{
    /* 一帧 100 个时隙（10ms → 100Hz），按当前亮度决定亮/灭 */
    if (breath_pwm_pos >= LED_BREATH_LEVELS) {
        breath_pwm_pos = 0;
    }
    if (breath_pwm_pos < breath_level) {
        led_red_on();
    } else {
        led_red_off();
    }
    breath_pwm_pos++;

    /* 每 20ms 调整一级亮度：0 → 100 → 0 循环 */
    if (++breath_step_div >= LED_BREATH_STEP_TICKS) {
        breath_step_div = 0;

        if (0 == breath_dir) {
            if (breath_level >= LED_BREATH_LEVELS) {
                breath_dir = 1; /* 到最亮，转为渐暗 */
            } else {
                breath_level++;
            }
        } else {
            if (breath_level > 0) {
                breath_level--;
            } else {
                breath_dir = 0; /* 到最暗，转为渐亮 */
            }
        }
    }
}

/************************************************
;  *    @函数名    : blink_red
;  *    @说明      : 红灯按 LED_BLINK_MS 节奏闪烁（低电量、无电池告警共用）
;  *    @上下文    : 主循环；非阻塞，靠 tick 判断时间
;  ***********************************************/
static void blink_red(void)
{
    if (elapsed_ms(blink_mark_ms) < LED_BLINK_MS) {
        return;
    }

    blink_mark_ms = tick_now_ms();
    blink_on = (0 == blink_on) ? 1 : 0; /* 相位翻转 */

    if (0 != blink_on) {
        led_red_on();
    } else {
        led_red_off();
    }
}

/************************************************
;  *    @函数名    : led_task
;  *    @说明      : 指示灯状态机（主循环每轮调用）
;  *    @优先级    : 无电池告警 > 低电量报警 > 充电显示（呼吸/充满） > 正常运行显示
;  *    @注意      : 不阻塞、不延时；tick_now_ms() 内部关中断保证取值原子
;  ***********************************************/
void led_task(void)
{
#if (BAT_ABSENT_DETECT_ENABLE)
    /* 没装电池 + 插着充电器：红灯 200ms 闪烁告警（与低电量同节奏），白灯灭 */
    if ((0 != charger_in) && (0 != battery_absent)) {
        if (0 != breath_running) {
            led_breath_stop();
        }
        led_white_off();
        blink_red();
        return;
    }
#endif

    if (0 != low_battery) {
        /* 低电量报警：红灯 200ms 亮 / 200ms 灭，白灯灭 */
        if (0 != breath_running) {
            led_breath_stop();
        }
        led_white_off();
        blink_red();
        return;
    }

    /* 充电器已插入：设备处于关机态，指示灯只反映充电过程 */
    if (0 != charger_in) {
        if (0 != charging) {
            if (0 == breath_running) {
                led_breath_start(); /* 进入充电：启动 10kHz 呼吸中断 */
            }
            led_white_off(); /* 充电中：白灯灭，红灯由 led_breath_tick 做呼吸 */
        } else {
            /* 充满（P03 高阻 → 内部上拉读成高）：白灯常亮，红灯灭 */
            if (0 != breath_running) {
                led_breath_stop();
            }
            led_white_on();
        }
        return;
    }

    /* 非充电状态：确保呼吸中断已停，复位闪烁计时，进入常规显示 */
    if (0 != breath_running) {
        led_breath_stop();
    }
    blink_mark_ms = tick_now_ms();
    blink_on = 0;

    /* 白灯：运行中且未加热时亮；红灯：加热时亮（与原工程一致） */
    if ((0 != device_on) && (0 == heater_on)) {
        led_white_on();
    } else {
        led_white_off();
    }

    if (0 != heater_on) {
        led_red_on();
    } else {
        led_red_off();
    }
}
