// encoding UTF-8
/******************************************************************************
;  *       @型号                 : MC32P7351
;  *       @模块                 : user（工程公共配置）
;  *       @职责                 : 类型定义、时间基准、按键/电机/电池等全部常量
;  *       @约束                 : 程序存储器 2K×16 位 / SRAM 256 字节
;  *       @注意                 : 本文件只放宏和类型，禁止定义变量或函数
;  *                               （变量定义在各模块 .c 中，头文件只做 extern 声明）
******************************************************************************/
#ifndef USER_H
#define USER_H

#include "mc32-common.h"
#include "MC32P7351.h"

/* 该工具链不带 stdint.h，这里给出等价的定长类型 */
typedef unsigned char uint8_t;
typedef unsigned int uint16_t;  /* 16 位 */
typedef unsigned long uint32_t; /* 32 位 */

/* ===================================================
 * 时间基准
 * T3：FCPU(8MHz) / 32 = 250kHz，250 个计数 = 1ms
 * 若实测节拍不对，说明配置字 FCPUS 不是 /4，按比例改 T3_COUNT_1MS 即可
 * =================================================== */
#define T3_PRESCALER_32 (0x05)    /* T3PRS[2:0]=101 → 1:32 预分频 */
#define T3_COUNT_1MS    (250 - 1) /* 重载值：250 次计数溢出 → 1ms 中断 */

#define KEY_SCAN_MS    (10)  /* 10 × 1ms = 10ms 按键扫描周期 */
#define TICK_100MS_DIV (100) /* 100 × 1ms = 100ms（长计时用节拍） */

/* ===================================================
 * 按键（扫描周期 10ms，因此用"扫描次数"计时，避免中断与主循环
 *       同时读写 1ms 计数器带来的非原子问题）
 * =================================================== */
#define KEY_DEBOUNCE_COUNT (3)   /* 3 × 10ms = 30ms 消抖 */
#define KEY_SHORT_COUNT    (75)  /* 75 × 10ms = 750ms 短按上限 */
#define KEY_HOLD_COUNT     (200) /* 200 × 10ms = 2000ms 长按（开机/关机一致） */

/* ===================================================
 * 电机
 * PWM 频率 = FCPU / (预分频 × PWM_PERIOD) = 8MHz / (2 × 172) ≈ 23.3kHz
 * 档位占空比 = duty / PWM_PERIOD
 * =================================================== */
#define PWM_PERIOD          (172)  /* T0LOAD/T1LOAD = 171，即 172 个计数一周期 */
#define PWM_GEAR_1          (150)  /* 档位 1：150/172 ≈ 87% */
#define PWM_GEAR_2          (160)  /* 档位 2：160/172 ≈ 93% */
#define PWM_GEAR_3          (255)  /* 档位 3：大于周期即 100% */
#define PWM_GEAR_4          (0)    /* 档位 4：停转 */
#define MOTOR_DEAD_MS       (1000) /* 换向死区 1s：等电机绕组放电，防止刹车 */
#define MOTOR_REVERSE_100MS (1200) /* 1200 × 100ms = 2min 自动换向周期 */

/* ===================================================
 * 电池电压检测（P02/AN1）
 * 分压：BAT+ --1M-- P02 --470K-- GND
 *   （按实物实测标定：空电池时插充电器 P02 读 2.61V，芯片空载输出约 8.16V，
 *     2.61 × (1000+470)/470 ≈ 8.16V，说明下拉是 470K（readme 原写的 70K 有误，已更正）
 * ADC：12 位、内部 3V 参考
 *   raw = Vbat(mV) × Rbottom/(Rtop+Rbottom) / Vref(mV) × 4096
 * 例：6.5V → 6500×470/(1470×3000)×4096 = 2837
 * 若换分压电阻，只改下面两个电阻宏，阈值会自动重算
 * =================================================== */
#define BAT_DIVIDER_TOP_KOHM    (1000UL) /* 上拉电阻 1M */
#define BAT_DIVIDER_BOTTOM_KOHM (470UL)  /* 下拉电阻 470K（实测标定值） */
#define ADC_VREF_MV             (3000UL) /* ADC 内部参考电压 3V */
#define ADC_FULL_SCALE          (4096UL) /* 12 位满量程 */

/* 电池电压(mV) → ADC 期望码值（编译期常量表达式，不占运行时间）
 * 先除以分压比、再乘满量程，避免 mV×Ω×4096 在 32 位里溢出 */
#define BAT_MV_TO_RAW(vbat_mv)                                                  \
    ((uint16_t)((((uint32_t)(vbat_mv) * BAT_DIVIDER_BOTTOM_KOHM)               \
                  / (BAT_DIVIDER_TOP_KOHM + BAT_DIVIDER_BOTTOM_KOHM))          \
                 * ADC_FULL_SCALE / ADC_VREF_MV))

#define LOW_BATTERY_MV         (6500)                        /* 低电量电压 6.5V */
#define SHUTDOWN_MV            (6200)                        /* 关机电压 6.2V */
#define LOW_BATTERY_RAW        BAT_MV_TO_RAW(LOW_BATTERY_MV) /* 6.5V → 2837 */
#define SHUTDOWN_RAW           BAT_MV_TO_RAW(SHUTDOWN_MV)    /* 6.2V → 2706 */
#define LOW_BATTERY_CONFIRM_MS (5000) /* 低于 6.5V 持续 5s 才报警（防瞬时压降误报） */
#define SHUTDOWN_CONFIRM_MS    (1000) /* 低于 6.2V 持续 1s 才关机 */
#define AUTO_OFF_100MS         (9000) /* 9000 × 100ms = 15min 自动关机 */

/* ---------------------------------------------------------------
 * 空电池（未装电池）检测总开关
 *   0 = 屏蔽（当前默认值）
 *   1 = 启用 power.c 里的在位判据
 * 屏蔽原因：实测「只插充电器、不接电池」时 P02 = 2.61V，与「电池已充满」完全一样，
 *           现有电压判据无法可靠区分；等确定最终方案（纹波 / 负载脉冲 / 硬件）再打开。
 * --------------------------------------------------------------- */
#define BAT_ABSENT_DETECT_ENABLE (0)

/* 电池是否安装的判定参数（仅在总开关为 1 时生效，详见 power.c 的说明） */
#define BAT_ABSENT_RAW_LOW      (40)  /* 无充电器时低于该值（≈0.45V）→ 判定为没装电池 */
#define BAT_PRESENT_RAW_MIN     (2100) /* 无充电器时高于该值（≈4.8V 电池）→ 判定为装了电池 */
#define BAT_ABSENT_RAW_HIGH     BAT_MV_TO_RAW(8700) /* 充电中电压超过 8.7V → 电池被拔掉 */
#define BAT_CHARGING_RAW_MIN    BAT_MV_TO_RAW(8200) /* 充电中低于 8.2V → 确实在给电池充电 */
#define BAT_PRESENCE_SCAN_MS    (200) /* 主循环里检测电池是否安装的间隔 */

/* ===================================================
 * 充电器插拔检测（P00 数字输入，插入为高电平）
 * =================================================== */
#define CHARGER_DEBOUNCE_COUNT (3) /* 连续 3 次相同电平才认（主循环约 1ms 一次） */

/* ===================================================
 * LED
 * =================================================== */
#define LED_BLINK_MS        (200) /* 低电量报警：红灯 200ms 亮 / 200ms 灭 */
#define LED_BREATH_LEVELS     (100) /* 呼吸灯亮度级数（100 级 = 1% 占空比分辨率） */
#define LED_BREATH_T2_LOAD    (99)  /* T2 重载：FCPU/8=1MHz，100 计数=100us（10kHz） */
#define LED_BREATH_STEP_TICKS (200) /* 每 200 次中断（20ms）变一级 → 2s 渐亮 + 2s 渐暗 */

/* ===================================================
 * ADC
 * =================================================== */
#define BATTERY_SAMPLES (8)   /* 采样次数，必须是 2 的幂（代码里用 >>3 求平均） */
#define ADC_WAIT_LOOP   (200) /* 等待 ADEOC 的轮询上限，防 ADC 异常时死等 */

#define ADCR0_CONFIG       (0x0A)      /* ADRSEL=1（12 位）、ADEOC=1（未启动） */
#define ADCR0_CHANNEL_MASK (0x0F << 4) /* ADCHS[3:0] 通道选择位掩码 */
#define ADCR0_CHANNEL_AIN1 (0x01 << 4) /* 通道 AN1--P02（电池分压） */
#define ADCR1_CONFIG       (0xE1)      /* ADCKS=111(FHIRC/256≈125kHz)、ADVRS=001(内部3V) */
#define ADCR2_CONFIG       (0xFF)      /* ADSPS=1111（采样 15 个 ADCLK，最稳） */

/* ===================================================
 * 休眠唤醒源（编译期二选一）
 * WAKE_SOURCE_T2 ：Timer2 + FLOSC 32kHz，约 32ms 唤醒一次（响应快，静态电流略高）
 * WAKE_SOURCE_WDT：只靠配置字 WDTT（1024ms）溢出唤醒（响应慢，静态电流最低）
 * =================================================== */
#define WAKE_SOURCE_T2  (1)
#define WAKE_SOURCE_WDT (0)
#define WAKE_SOURCE     (WAKE_SOURCE_T2) /* 需要最低功耗时改成 WAKE_SOURCE_WDT */
#define WAKE_T2_LOAD    (7)              /* (7+1) / (32kHz/128) = 32ms */

#endif
