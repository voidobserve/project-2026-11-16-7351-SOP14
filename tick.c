// encoding UTF-8
/******************************************************************************
;  *       @模块                 : tick（时基）
;  *       @职责                 : Timer3 每 1ms 中断一次，产生 tick_ms / tick_100ms，
;  *                               并按 10ms 节拍调用按键扫描
;  *       @上下文               : int_isr 运行在中断里；其余函数在主循环里调用
;  *       @依赖                 : user.h（T3 分频/重载值、扫描节拍）、key.h（key_scan）
;  *
;  *       时钟链：FHOSC 32MHz → 配置字 FCPUS=/4 → FCPU=8MHz
;  *               → T3 预分频 1:32 → 250kHz → 重载 249（250 次）→ 1ms 中断
;  *
;  *       为什么需要 tick_now_*()：
;  *               16 位变量在 8 位核上是"分两次读"的，若读到一半被中断改掉，
;  *               主循环会读到错误值（可能偏差 256 个计数）。因此主循环读取时
;  *               统一调用 tick_now_*()，用"关中断-取值-恢复"保证原子性。
******************************************************************************/
#include "tick.h"
#include "key.h"
#include "led.h"

volatile uint16_t tick_ms;    /* 1ms 计数，65536ms 后回绕 */
volatile uint16_t tick_100ms; /* 100ms 计数，6553.6s 后回绕 */

/* 中断现场保存变量：由 int_isr 的内联汇编按符号 _abuf / _statusbuf 引用，
 * 必须是非 static 全局变量（汇编里直接写符号名） */
uint8_t abuf;
uint8_t statusbuf;

static uint8_t scan_div;  /* 1ms → 10ms 的分频计数（按键扫描节拍） */
static uint8_t div_100ms; /* 1ms → 100ms 的分频计数 */

/************************************************
;  *    @函数名    : tick_init
;  *    @说明      : 配置 Timer3 为 1ms 中断，并使能中断（总中断由 main 打开）
;  *    @寄存器    : T3LOAD 重载值；T3CR = 预分频位 | 0x80(T3EN)
;  *    @注意      : 此时 GIE 仍为 0，只是把 T3IE 置 1，不会立刻进中断
;  ***********************************************/
void tick_init(void)
{
    scan_div = 0;
    div_100ms = 0;
    tick_ms = 0;
    tick_100ms = 0;

    T3LOAD = (uint8_t)T3_COUNT_1MS; /* 250 次计数 = 1ms */
    T3CR = T3_PRESCALER_32 | 0x80;  /* 1:32 预分频 + 使能定时器 */
    T3IF = 0;                       /* 清中断标志 */
    T3IE = 1;                       /* 允许 T3 中断 */
}

/************************************************
;  *    @函数名    : tick_start
;  *    @说明      : 休眠唤醒后重新启动 1ms 节拍
;  *    @注意      : 写 T3CNT 会同时清掉预分频器，保证第一次中断仍是完整 1ms
;  ***********************************************/
void tick_start(void)
{
    T3CNT = (uint8_t)T3_COUNT_1MS; /* 写入计数会同时清零预分频器 */
    T3IF = 0;
    T3IE = 1;
    T3EN = 1;
}

/************************************************
;  *    @函数名    : tick_stop
;  *    @说明      : 休眠前停止节拍（省电，且避免休眠中被 T3 唤醒）
;  ***********************************************/
void tick_stop(void)
{
    T3EN = 0;
    T3IE = 0;
    T3IF = 0;
}

/************************************************
;  *    @函数名    : tick_now_ms
;  *    @返回        : 当前 1ms 计数值（原子读取）
;  *    @说明      : 关中断读取 16 位变量后恢复原中断状态
;  *    @上下文    : 只能在主循环上下文调用（中断里调用会破坏 GIE 语义）
;  ***********************************************/
uint16_t tick_now_ms(void)
{
    uint16_t value;
    uint8_t gie = GIE; /* 保存原中断状态，读完后恢复 */

    GIE = 0;
    value = tick_ms;
    GIE = gie;

    return value;
}

/************************************************
;  *    @函数名    : tick_now_100ms
;  *    @返回        : 当前 100ms 计数值（原子读取）
;  *    @说明      : 同上，用于 2min 换向、15min 自动关机等长计时
;  ***********************************************/
uint16_t tick_now_100ms(void)
{
    uint16_t value;
    uint8_t gie = GIE;

    GIE = 0;
    value = tick_100ms;
    GIE = gie;

    return value;
}

/************************************************
;  *    @函数名    : int_isr
;  *    @说明      : 中断服务程序
;  *                 1) Timer3 每 1ms 一次：系统节拍、10ms 按键扫描、100ms 计数
;  *                 2) Timer2 每 100us 一次：仅充电时运行，做红灯呼吸灯的软件 PWM
;  ***********************************************/
void int_isr(void) __interrupt
{
    __asm;
    movra _abuf;      /* 保存累加器 A */
    swapar _PFLAG;    /* 保存状态标志 PFLAG */
    movra _statusbuf;
    __endasm;

    if (T3IF & T3IE) {
        T3IF = 0; /* 清中断标志（必须软件清零） */
        tick_ms++;

        /* 10ms 节拍：按键扫描 */
        if (++scan_div >= KEY_SCAN_MS) {
            scan_div = 0;
            key_scan();
        }

        /* 100ms 节拍：供 2min / 15min 计时使用 */
        if (++div_100ms >= TICK_100MS_DIV) {
            div_100ms = 0;
            tick_100ms++;
        }
    }

    /* Timer2：只在充电呼吸灯启动时工作（10kHz，100 级软件 PWM） */
    if (T2IF & T2IE) {
        T2IF = 0;
        led_breath_tick();
    }

    __asm;
    swapar _statusbuf; /* 恢复 PFLAG */
    movra _PFLAG;
    swapr _abuf;       /* 恢复累加器 A */
    swapar _abuf;
    __endasm;
}
