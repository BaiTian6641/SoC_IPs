/* SPDX-License-Identifier: Apache-2.0 */
/* smbus_pmbus_example.c - Bare-Metal SMBus 3.0 / PMBus 1.3 Driver for 50_smbus_pmbus */

#include <stdint.h>
#include "smbus_pmbus_regs.h"

#define SMBUS0_BASE         ((uintptr_t)0x40070000u)
#define SMBUS_POLL_TIMEOUT  1000000u

typedef struct {
    uint32_t vout_mv;
    int32_t  iout_ma;
    int32_t  temp_mc;
    uint16_t status_word;
} pmbus_telemetry_t;

static int smbus_wait_done(uintptr_t base, uint32_t *status_out) {
    uint32_t st = 0u;
    for (uint32_t i = 0u; i < SMBUS_POLL_TIMEOUT; i++) {
        st = smbus_read32(base, SMBUS_REG_STATUS);
        if ((st & SMBUS_STAT_HOST_DONE) != 0u) {
            if (status_out != 0) {
                *status_out = st;
            }
            if ((st & SMBUS_STAT_ERR_MASK) != 0u) {
                return -1;
            }
            return 0;
        }
    }
    return -2;
}

/* Initialize SMBus controller for given SCL speed (100k / 400k / 1M) with PEC and SMBus timeouts */
int smbus_init(uintptr_t base, uint32_t sys_clk_hz, uint32_t scl_hz, uint8_t pec_en) {
    if (smbus_read32(base, SMBUS_REG_CAP_ID) != SMBUS_CAP_ID_VAL) {
        return -1;
    }

    uint32_t qdiv = sys_clk_hz / (4u * scl_hz);
    if (qdiv < 6u) {
        qdiv = 6u;
    }

    smbus_write32(base, SMBUS_REG_CTRL, SMBUS_CTRL_SW_RST | SMBUS_CTRL_FIFO_CLR);
    smbus_write32(base, SMBUS_REG_CLK_DIV, ((qdiv * 2u) << 16) | (qdiv & 0xFFFFu));

    /* SMBus 3.0 standard timeouts: tTIMEOUT=30ms, tLOW:SEXT=25ms, tLOW:MEXT=10ms */
    smbus_write32(base, SMBUS_REG_T_TIMEOUT_CFG, (sys_clk_hz / 1000u) * 30u);
    smbus_write32(base, SMBUS_REG_T_SEXT_CFG,    (sys_clk_hz / 1000u) * 25u);
    smbus_write32(base, SMBUS_REG_T_MEXT_CFG,    (sys_clk_hz / 1000u) * 10u);

    uint32_t ctrl = SMBUS_CTRL_HOST_EN | (pec_en ? SMBUS_CTRL_PEC_EN : 0u);
    smbus_write32(base, SMBUS_REG_CTRL, ctrl);
    smbus_write32(base, SMBUS_REG_IRQ_STAT, 0x1FFu);
    return 0;
}

/* PMBus Read Word (little-endian 16-bit) with hardware PEC checking */
int pmbus_read_word_pec(uintptr_t base, uint8_t addr7, uint8_t cmd, uint16_t *word_out) {
    smbus_write32(base, SMBUS_REG_CMD,
                  SMBUS_CMD_PACK(SMBUS_OP_READ_WORD, addr7, cmd, 0u, 0u));
    if (smbus_wait_done(base, 0) != 0) {
        return -1;
    }
    uint8_t lo = (uint8_t)(smbus_read32(base, SMBUS_REG_RX_FIFO) & 0xFFu);
    uint8_t hi = (uint8_t)(smbus_read32(base, SMBUS_REG_RX_FIFO) & 0xFFu);
    *word_out = (uint16_t)(((uint16_t)hi << 8) | lo);
    return 0;
}

/* Program PMBus VOUT_COMMAND in millivolts using VOUT_MODE Linear16 exponent */
int pmbus_write_vout(uintptr_t base, uint8_t addr7, uint32_t target_mv) {
    smbus_write32(base, SMBUS_REG_CMD,
                  SMBUS_CMD_PACK(SMBUS_OP_READ_BYTE, addr7, PMBUS_CMD_VOUT_MODE, 0u, 0u));
    if (smbus_wait_done(base, 0) != 0) {
        return -1;
    }
    uint8_t vout_mode = (uint8_t)(smbus_read32(base, SMBUS_REG_RX_FIFO) & 0xFFu);
    uint16_t vout_cmd = pmbus_mv_to_linear16(target_mv, vout_mode);

    smbus_write32(base, SMBUS_REG_TX_FIFO, (uint32_t)(vout_cmd & 0xFFu));
    smbus_write32(base, SMBUS_REG_TX_FIFO, (uint32_t)((vout_cmd >> 8) & 0xFFu));
    smbus_write32(base, SMBUS_REG_CMD,
                  SMBUS_CMD_PACK(SMBUS_OP_WRITE_WORD, addr7, PMBUS_CMD_VOUT_COMMAND, 0u, 0u));
    return smbus_wait_done(base, 0);
}

/* Read Vout (mV), Iout (mA), Temperature (milli-degC), and STATUS_WORD */
int pmbus_read_telemetry(uintptr_t base, uint8_t addr7, pmbus_telemetry_t *tel) {
    uint16_t raw_vout = 0u, raw_iout = 0u, raw_temp = 0u, raw_stat = 0u;

    smbus_write32(base, SMBUS_REG_CMD,
                  SMBUS_CMD_PACK(SMBUS_OP_READ_BYTE, addr7, PMBUS_CMD_VOUT_MODE, 0u, 0u));
    if (smbus_wait_done(base, 0) != 0) {
        return -1;
    }
    uint8_t vout_mode = (uint8_t)(smbus_read32(base, SMBUS_REG_RX_FIFO) & 0xFFu);

    if (pmbus_read_word_pec(base, addr7, PMBUS_CMD_READ_VOUT, &raw_vout) != 0) {
        return -2;
    }
    if (pmbus_read_word_pec(base, addr7, PMBUS_CMD_READ_IOUT, &raw_iout) != 0) {
        return -3;
    }
    if (pmbus_read_word_pec(base, addr7, PMBUS_CMD_READ_TEMPERATURE_1, &raw_temp) != 0) {
        return -4;
    }
    if (pmbus_read_word_pec(base, addr7, PMBUS_CMD_STATUS_WORD, &raw_stat) != 0) {
        return -5;
    }

    tel->vout_mv     = pmbus_linear16_to_mv(raw_vout, vout_mode);
    tel->iout_ma     = pmbus_linear11_to_milli(raw_iout);
    tel->temp_mc     = pmbus_linear11_to_milli(raw_temp);
    tel->status_word = raw_stat;
    return 0;
}

/* Service SMBALERT# by issuing ARA (7'h0C) reads until all alerting devices are identified */
int smbus_handle_alert_ara(uintptr_t base, uint8_t *addrs_out, uint32_t max_addrs) {
    uint32_t count = 0u;
    while ((smbus_read32(base, SMBUS_REG_STATUS) & SMBUS_STAT_ALERT_ACTIVE) != 0u &&
           count < max_addrs) {
        smbus_write32(base, SMBUS_REG_CMD,
                      SMBUS_CMD_PACK(SMBUS_OP_ARA, SMBUS_ADDR_ARA, 0u, 0u, 0u));
        if (smbus_wait_done(base, 0) != 0) {
            break;
        }
        uint32_t ara_reg = smbus_read32(base, SMBUS_REG_ARA_ADDR);
        (void)smbus_read32(base, SMBUS_REG_RX_FIFO);
        if ((ara_reg & SMBUS_ARA_VALID) != 0u && (ara_reg & SMBUS_ARA_PEC_OK) != 0u) {
            addrs_out[count++] = SMBUS_ARA_ADDR7(ara_reg);
            smbus_write32(base, SMBUS_REG_ARA_ADDR, SMBUS_ARA_VALID);
        } else {
            break;
        }
    }
    smbus_write32(base, SMBUS_REG_IRQ_STAT, SMBUS_IRQ_ALERT | SMBUS_IRQ_ARA_DONE);
    return (int)count;
}

/* SMBus ARP (7'h61) Discovery: Prepare to ARP -> Get UDID (16B) -> Assign Address (17B) */
int smbus_arp_discover(uintptr_t base, uint8_t base_assign_addr7, uint8_t max_devs) {
    uint8_t assigned = 0u;
    uint8_t udid[16];

    /* 1. Prepare to ARP (Send Byte 0x01 to 7'h61) */
    smbus_write32(base, SMBUS_REG_TX_FIFO, SMBUS_ARP_CMD_PREPARE);
    smbus_write32(base, SMBUS_REG_CMD,
                  SMBUS_CMD_PACK(SMBUS_OP_SEND_BYTE, SMBUS_ADDR_ARP, 0u, 0u, 0u));
    if (smbus_wait_done(base, 0) != 0) {
        return 0;
    }

    /* 2. Loop Get UDID (General, 0x03) + Assign Address (0x04) until no slave ACKs */
    while (assigned < max_devs) {
        uint32_t st = 0u;
        smbus_write32(base, SMBUS_REG_CMD,
                      SMBUS_CMD_PACK(SMBUS_OP_BLOCK_READ, SMBUS_ADDR_ARP,
                                     SMBUS_ARP_CMD_GET_UDID_GEN, 0u, 0u));
        if (smbus_wait_done(base, &st) != 0) {
            break;
        }
        if (SMBUS_STAT_RX_CNT(st) != 16u) {
            break;
        }
        for (uint32_t i = 0u; i < 16u; i++) {
            udid[i] = (uint8_t)(smbus_read32(base, SMBUS_REG_RX_FIFO) & 0xFFu);
        }

        uint8_t new_addr7 = (uint8_t)((base_assign_addr7 + assigned) & 0x7Fu);
        for (uint32_t i = 0u; i < 16u; i++) {
            smbus_write32(base, SMBUS_REG_TX_FIFO, udid[i]);
        }
        smbus_write32(base, SMBUS_REG_TX_FIFO, (uint32_t)(new_addr7 << 1));
        smbus_write32(base, SMBUS_REG_CMD,
                      SMBUS_CMD_PACK(SMBUS_OP_BLOCK_WRITE, SMBUS_ADDR_ARP,
                                     SMBUS_ARP_CMD_ASSIGN_ADDR, 17u, 0u));
        if (smbus_wait_done(base, 0) != 0) {
            break;
        }
        assigned++;
    }
    return (int)assigned;
}

/* Example bare-metal main sequence */
int smbus_pmbus_demo_main(void) {
    pmbus_telemetry_t tel;
    uint8_t alerted[8];

    if (smbus_init(SMBUS0_BASE, 100000000u, 400000u, 1u) != 0) {
        return -1;
    }
    (void)smbus_arp_discover(SMBUS0_BASE, 0x40u, 4u);
    if (pmbus_write_vout(SMBUS0_BASE, 0x20u, 850u) != 0) {
        return -2;
    }
    if (pmbus_read_telemetry(SMBUS0_BASE, 0x20u, &tel) != 0) {
        return -3;
    }
    (void)smbus_handle_alert_ara(SMBUS0_BASE, alerted, 8u);
    return (tel.vout_mv > 0u) ? 0 : -4;
}
