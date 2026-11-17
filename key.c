// encoding UTF-8
/******************************************************************************
;  *       @模块                 : key（按键）
;  *       @职责                 : 10ms 定时扫描两次按键，做消抖、短按/长按识别，
;  *                               把结果写入 key_event 供主循环消费
;  *       @上下文               : key_scan() 在 Timer3 中断里运行（每 10ms 一次）；
;  *                               key_event 由主循环读取并清零
;  *       @依赖                 : bsp.h（按键引脚宏）、user.h（消抖/时长常量）
;  *
;  *       计时方式：用"扫描次数"而不是 tick_ms：
;  *         每个扫描周期固定 10ms，所以 1 次计数 = 10ms，既省 RAM 又避免
;  *         中断里读取 16 位毫秒计数器的非原子问题。
;  *
;  *       按键行为（与原工程一致）：
;  *         短按（<750ms）      模式键 → 切档；加热键 → 开/关加热
;  *         长按 2s             模式键 → 关机态开机 / 运行态关机（同一阈值）
;  *         长按事件在"按住期间"只上报一次（hold_fired 标记），松手不再重复上报
******************************************************************************/
#include "key.h"
#include "bsp.h"

volatile uint8_t key_event; /* 按键事件：中断写入，主循环取走后清零 */

static uint8_t last_id;    /* 上一轮消抖后的键值（KEY_NONE/KEY_MODE/KEY_HEAT） */
static uint8_t filter_id;  /* 消抖过程中正在确认的键值 */
static uint8_t filter_cnt; /* 连续读到相同键值的次数 */
static uint8_t hold_fired; /* 本次按住是否已经上报过长按 */
static uint8_t hold_count; /* 按住时长，单位：10ms 扫描次数 */

/************************************************
;  *    @函数名    : read_key_id
;  *    @返回        : 当前按下的键（KEY_NONE 表示都没按）
;  *    @说明      : 两个键都是低电平有效；若同时按下，模式键优先
;  ***********************************************/
static uint8_t read_key_id(void)
{
    if (0 == PIN_KEY_MODE) {
        return KEY_MODE;
    }
    if (0 == PIN_KEY_HEAT) {
        return KEY_HEAT;
    }
    return KEY_NONE;
}

/************************************************
;  *    @函数名    : key_scan
;  *    @说明      : 10ms 调用一次的按键状态机（在 Timer3 中断内执行）
;  *    @流程      : 1) 读当前键值，与消抖值不同 → 重新计数
;  *                 2) 连续 3 次相同（30ms）才算稳定
;  *                 3) 稳定后与上一次的稳定值比较：
;  *                    按下 → 清按住计数；松开 → 若未上报长按且 <750ms 则算短按
;  *                 4) 一直按住 → 累计 10ms 计数，够 2s 上报长按（只一次）
;  *    @注意      : 本函数在中断中执行，只做几十条指令的逻辑，不含延时/浮点
;  ***********************************************/
void key_scan(void)
{
    uint8_t now = read_key_id();

    /* 第 1 步：键值变化则重新开始消抖 */
    if (now != filter_id) {
        filter_id = now;
        filter_cnt = 0;
        return;
    }

    /* 第 2 步：连续相同次数不够，继续等待 */
    if (filter_cnt < KEY_DEBOUNCE_COUNT) {
        filter_cnt++;
        return;
    }

    /* 第 3 步：消抖完成，处理"按下/松开"的边沿事件 */
    if (now != last_id) {
        if (KEY_NONE == now) {
            /* 松开：长按已在按住期间上报过，这里只处理短按 */
            if ((0 == hold_fired) && (hold_count < KEY_SHORT_COUNT)) {
                key_event = (KEY_MODE == last_id) ? KEY_EVENT_MODE_SHORT
                                                  : KEY_EVENT_HEAT_SHORT;
            }
        } else {
            /* 按下：清按住计数和长按标记 */
            hold_count = 0;
            hold_fired = 0;
        }

        last_id = now;
        return;
    }

    /* 第 4 步：一直按住，累计时长并判断长按（开机、关机都是 2s） */
    if (0 == hold_fired) {
        if (hold_count < 0xFF) {
            hold_count++; /* 封顶 255（2.55s），防止溢出后误判 */
        }

        /* 只有模式键有长按事件；加热键长按无意义 */
        if ((KEY_MODE == now) && (hold_count >= KEY_HOLD_COUNT)) {
            hold_fired = 1; /* 标记已上报，松手时不再算短按 */
            key_event = KEY_EVENT_MODE_LONG;
        }
    }
}
