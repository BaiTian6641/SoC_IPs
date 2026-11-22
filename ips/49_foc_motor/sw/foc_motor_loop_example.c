/* SPDX-License-Identifier: Apache-2.0 */
/* foc_motor_loop_example.c - Bare-Metal PMSM FOC Bring-Up, Speed Loop & Fault ISR */

#include "foc_regs.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uintptr_t base;
    uint16_t  pwm_period;
    uint16_t  dead_time_cycles;
    bool      three_shunt;
    double    kp_speed;
    double    ki_speed;
    double    speed_integrator;
    int16_t   iq_max_q15;
} foc_drv_t;

/* Initialize PWM carrier, dead-time, current PI loops, and safety IRQs */
int foc_init_pmsm(foc_drv_t *drv, double kp_d, double ki_d, double kp_q, double ki_q,
                  double v_max_pu) {
    if (foc_rd32(drv->base, FOC_REG_ID_VERSION) != FOC_ID_VERSION_VAL) {
        return -1;
    }

    /* Disable outputs while configuring */
    foc_wr32(drv->base, FOC_REG_CTRL, FOC_CTRL_CLR_INT_BIT);
    foc_wr32(drv->base, FOC_REG_IRQ_STAT, FOC_IRQ_ALL_MASK);

    /* Configure center-aligned PWM period and dead-time */
    uint32_t pwm_cfg = ((uint32_t)(drv->dead_time_cycles & 0x0FFFu) << 16) |
                       (uint32_t)drv->pwm_period;
    foc_wr32(drv->base, FOC_REG_PWM_CFG, pwm_cfg);

    /* Configure d/q PI gains (Q4.12) and voltage modulation limit (Q1.15) */
    foc_wr32(drv->base, FOC_REG_KP_KI_D,
             foc_pack_u16(foc_float_to_q4_12(kp_d), foc_float_to_q4_12(ki_d)));
    foc_wr32(drv->base, FOC_REG_KP_KI_Q,
             foc_pack_u16(foc_float_to_q4_12(kp_q), foc_float_to_q4_12(ki_q)));
    foc_wr32(drv->base, FOC_REG_V_LIMIT, (uint32_t)foc_float_to_q15(v_max_pu));

    /* Enable Fault & Cycle-Done interrupts */
    foc_wr32(drv->base, FOC_REG_IRQ_EN, FOC_IRQ_CYCLE_DONE_BIT | FOC_IRQ_FAULT_BIT);
    drv->speed_integrator = 0.0;
    return 0;
}

/* Open-loop rotor alignment: hold fixed d-axis voltage vector at theta_e = 0 */
void foc_align_rotor_open_loop(const foc_drv_t *drv, double vd_align_pu) {
    uint32_t shunt_bit = drv->three_shunt ? FOC_CTRL_THREE_SHUNT_BIT : 0u;

    foc_wr32(drv->base, FOC_REG_VD_VQ_OVR,
             foc_pack_i16(foc_float_to_q15(vd_align_pu), 0));
    foc_wr32(drv->base, FOC_REG_CSR_IC_THETA, foc_pack_u16(0u, 0u));

    /* Enable PWM + Open-Loop + SW_START to latch SVPWM duties for theta=0 */
    foc_wr32(drv->base, FOC_REG_CTRL,
             FOC_CTRL_FOC_EN_BIT | FOC_CTRL_PWM_EN_BIT | FOC_CTRL_OPEN_LOOP_BIT |
             FOC_CTRL_CIRCLE_LIM_BIT | shunt_bit | FOC_CTRL_SW_START_BIT);

    while ((foc_rd32(drv->base, FOC_REG_STATUS) & FOC_STATUS_BUSY_BIT) != 0u) {
    }
}

/* Close autonomous hardware-triggered current loop (Id_ref = 0, Iq_ref = torque_cmd) */
void foc_close_current_loop(const foc_drv_t *drv, double id_ref_pu, double iq_ref_pu) {
    uint32_t shunt_bit = drv->three_shunt ? FOC_CTRL_THREE_SHUNT_BIT : 0u;

    foc_wr32(drv->base, FOC_REG_ID_IQ_REF,
             foc_pack_i16(foc_float_to_q15(id_ref_pu), foc_float_to_q15(iq_ref_pu)));

    /* Clear integrators once, then hand over to adc_sample_valid_i hardware trigger */
    foc_wr32(drv->base, FOC_REG_CTRL,
             FOC_CTRL_FOC_EN_BIT | FOC_CTRL_PWM_EN_BIT | FOC_CTRL_HW_TRIG_EN_BIT |
             FOC_CTRL_CIRCLE_LIM_BIT | FOC_CTRL_ADC_TRIG_PEAK | shunt_bit |
             FOC_CTRL_CLR_INT_BIT);
}

/* Outer software speed PI loop (called at e.g. 1 kHz) updating Iq_ref */
void foc_speed_loop_tick(foc_drv_t *drv, double speed_ref_pu, double speed_meas_pu,
                         double id_ref_pu) {
    double err = speed_ref_pu - speed_meas_pu;
    double iq_max = foc_q15_to_float(drv->iq_max_q15);
    double iq_cmd = drv->kp_speed * err + drv->speed_integrator;

    /* Conditional-clamping anti-windup on outer speed PI */
    if (!((iq_cmd >= iq_max && err > 0.0) || (iq_cmd <= -iq_max && err < 0.0))) {
        drv->speed_integrator += drv->ki_speed * err;
        if (drv->speed_integrator > iq_max) {
            drv->speed_integrator = iq_max;
        } else if (drv->speed_integrator < -iq_max) {
            drv->speed_integrator = -iq_max;
        }
    }

    if (iq_cmd > iq_max) {
        iq_cmd = iq_max;
    } else if (iq_cmd < -iq_max) {
        iq_cmd = -iq_max;
    }

    foc_wr32(drv->base, FOC_REG_ID_IQ_REF,
             foc_pack_i16(foc_float_to_q15(id_ref_pu), foc_float_to_q15(iq_cmd)));
}

/* Interrupt Service Routine: handles hardware fault recovery & cycle telemetry */
bool foc_isr_handler(foc_drv_t *drv) {
    uint32_t irq = foc_rd32(drv->base, FOC_REG_IRQ_STAT);

    if ((irq & FOC_IRQ_FAULT_BIT) != 0u) {
        /* Wait until external fault_i pin clears before re-arming */
        uint32_t st = foc_rd32(drv->base, FOC_REG_STATUS);
        if ((st & FOC_STATUS_FAULT_PIN_BIT) == 0u) {
            /* Reset integrators & clear sticky fault W1C */
            foc_wr32(drv->base, FOC_REG_CTRL,
                     foc_rd32(drv->base, FOC_REG_CTRL) | FOC_CTRL_CLR_INT_BIT);
            foc_wr32(drv->base, FOC_REG_IRQ_STAT, FOC_IRQ_ALL_MASK);
            drv->speed_integrator = 0.0;
            return true; /* Recovered */
        }
        return false; /* Fault pin still active; PWM stays tri-stated/off */
    }

    if ((irq & FOC_IRQ_CYCLE_DONE_BIT) != 0u) {
        foc_wr32(drv->base, FOC_REG_IRQ_STAT, FOC_IRQ_CYCLE_DONE_BIT | FOC_IRQ_SATURATED_BIT);
    }
    return true;
}
