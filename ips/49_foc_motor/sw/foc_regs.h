/* SPDX-License-Identifier: Apache-2.0 */
/* foc_regs.h - Register & Bitfield Definitions for Hardware FOC Motor Control IP */

#ifndef FOC_REGS_H
#define FOC_REGS_H

#include <stdint.h>

/* Expected ID_VERSION value ("FOC\x01") */
#define FOC_ID_VERSION_VAL         0x464F4301u

/* Register Byte Offsets */
#define FOC_REG_CTRL               0x00u
#define FOC_REG_STATUS             0x04u
#define FOC_REG_IRQ_EN             0x08u
#define FOC_REG_IRQ_STAT           0x0Cu
#define FOC_REG_ID_IQ_REF          0x10u
#define FOC_REG_KP_KI_D            0x14u
#define FOC_REG_KP_KI_Q            0x18u
#define FOC_REG_V_LIMIT            0x1Cu
#define FOC_REG_VD_VQ_OVR          0x20u
#define FOC_REG_PWM_CFG            0x24u
#define FOC_REG_CSR_I_AB           0x28u
#define FOC_REG_CSR_IC_THETA       0x2Cu
#define FOC_REG_OBS_SINCOS         0x30u
#define FOC_REG_OBS_IALPHA_BETA    0x34u
#define FOC_REG_OBS_ID_IQ          0x38u
#define FOC_REG_OBS_VD_VQ          0x3Cu
#define FOC_REG_OBS_VALPHA_BETA    0x40u
#define FOC_REG_OBS_DUTY_AB        0x44u
#define FOC_REG_OBS_DUTY_C         0x48u
#define FOC_REG_OBS_ACC_D          0x4Cu
#define FOC_REG_OBS_ACC_Q          0x50u
#define FOC_REG_ID_VERSION         0x54u

/* CTRL (0x00) Bitfields */
#define FOC_CTRL_FOC_EN_BIT        (1u << 0)
#define FOC_CTRL_PWM_EN_BIT        (1u << 1)
#define FOC_CTRL_HW_TRIG_EN_BIT    (1u << 2)
#define FOC_CTRL_THREE_SHUNT_BIT   (1u << 3)
#define FOC_CTRL_OPEN_LOOP_BIT     (1u << 4)
#define FOC_CTRL_CIRCLE_LIM_BIT    (1u << 5)
#define FOC_CTRL_ADC_TRIG_SHIFT    6u
#define FOC_CTRL_ADC_TRIG_MASK     (3u << FOC_CTRL_ADC_TRIG_SHIFT)
#define FOC_CTRL_ADC_TRIG_PEAK     (0u << FOC_CTRL_ADC_TRIG_SHIFT)
#define FOC_CTRL_ADC_TRIG_VALLEY   (1u << FOC_CTRL_ADC_TRIG_SHIFT)
#define FOC_CTRL_ADC_TRIG_BOTH     (2u << FOC_CTRL_ADC_TRIG_SHIFT)
#define FOC_CTRL_ADC_TRIG_OFF      (3u << FOC_CTRL_ADC_TRIG_SHIFT)
#define FOC_CTRL_CLR_INT_BIT       (1u << 8)
#define FOC_CTRL_SW_START_BIT      (1u << 9)
#define FOC_CTRL_SW_EXT_ADC_BIT    (1u << 10)

/* STATUS (0x04) Bitfields */
#define FOC_STATUS_BUSY_BIT        (1u << 0)
#define FOC_STATUS_FAULT_ACT_BIT   (1u << 1)
#define FOC_STATUS_FAULT_PIN_BIT   (1u << 2)
#define FOC_STATUS_SATURATED_BIT   (1u << 3)
#define FOC_STATUS_SECTOR_SHIFT    4u
#define FOC_STATUS_SECTOR_MASK     (7u << FOC_STATUS_SECTOR_SHIFT)
#define FOC_STATUS_PWM_ACT_BIT     (1u << 7)
#define FOC_STATUS_CYC_CNT_SHIFT   16u

/* IRQ_EN (0x08) & IRQ_STAT (0x0C, W1C) Bitfields */
#define FOC_IRQ_CYCLE_DONE_BIT     (1u << 0)
#define FOC_IRQ_FAULT_BIT          (1u << 1)
#define FOC_IRQ_SATURATED_BIT      (1u << 2)
#define FOC_IRQ_ALL_MASK           0x07u

/* Fixed-Point Conversion Helpers (Q1.15 and Q4.12) */
#define FOC_Q15_MAX                32767
#define FOC_Q15_MIN                (-32768)

static inline int16_t foc_float_to_q15(double x) {
    double scaled = x * 32768.0;
    if (scaled >= 32767.0) {
        return (int16_t)FOC_Q15_MAX;
    }
    if (scaled <= -32768.0) {
        return (int16_t)FOC_Q15_MIN;
    }
    return (int16_t)(scaled >= 0.0 ? (scaled + 0.5) : (scaled - 0.5));
}

static inline double foc_q15_to_float(int16_t q) {
    return (double)q / 32768.0;
}

static inline uint16_t foc_float_to_q4_12(double x) {
    double scaled = x * 4096.0;
    if (scaled <= 0.0) {
        return 0u;
    }
    if (scaled >= 65535.0) {
        return 65535u;
    }
    return (uint16_t)(scaled + 0.5);
}

static inline uint32_t foc_pack_i16(int16_t lo, int16_t hi) {
    return ((uint32_t)(uint16_t)hi << 16) | (uint32_t)(uint16_t)lo;
}

static inline uint32_t foc_pack_u16(uint16_t lo, uint16_t hi) {
    return ((uint32_t)hi << 16) | (uint32_t)lo;
}

/* MMIO Accessors */
static inline void foc_wr32(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + off) = val;
}

static inline uint32_t foc_rd32(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + off);
}

#endif /* FOC_REGS_H */
