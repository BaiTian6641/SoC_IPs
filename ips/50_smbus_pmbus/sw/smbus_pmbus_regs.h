/* SPDX-License-Identifier: Apache-2.0 */
/* smbus_pmbus_regs.h - CSR Register Map, PMBus 1.3 Commands, & Linear11/16 Helpers for 50_smbus_pmbus */

#ifndef SMBUS_PMBUS_REGS_H
#define SMBUS_PMBUS_REGS_H

#include <stdint.h>

/* IP Capability Identifier ("SMB3") */
#define SMBUS_CAP_ID_VAL              0x534D4233u

/* Reserved SMBus 7-bit Addresses */
#define SMBUS_ADDR_HOST_NOTIFY        0x08u
#define SMBUS_ADDR_ARA                0x0Cu
#define SMBUS_ADDR_ARP                0x61u

/* SMBus ARP Commands (addressed to 7'h61) */
#define SMBUS_ARP_CMD_PREPARE         0x01u
#define SMBUS_ARP_CMD_RESET_DEV       0x02u
#define SMBUS_ARP_CMD_GET_UDID_GEN    0x03u
#define SMBUS_ARP_CMD_ASSIGN_ADDR     0x04u

/* CSR Byte Offsets */
#define SMBUS_REG_CTRL                0x00u
#define SMBUS_REG_CLK_DIV             0x04u
#define SMBUS_REG_CMD                 0x08u
#define SMBUS_REG_STATUS              0x0Cu
#define SMBUS_REG_TX_FIFO             0x10u
#define SMBUS_REG_RX_FIFO             0x14u
#define SMBUS_REG_FIFO_STAT           0x18u
#define SMBUS_REG_IRQ_EN              0x1Cu
#define SMBUS_REG_IRQ_STAT            0x20u
#define SMBUS_REG_ARA_ADDR            0x24u
#define SMBUS_REG_T_TIMEOUT_CFG       0x28u
#define SMBUS_REG_T_SEXT_CFG          0x2Cu
#define SMBUS_REG_T_MEXT_CFG          0x30u
#define SMBUS_REG_TGT_CFG             0x34u
#define SMBUS_REG_TGT_TX_FIFO         0x38u
#define SMBUS_REG_TGT_RX_FIFO         0x3Cu
#define SMBUS_REG_CAP_ID              0x40u

/* CTRL Bitfields */
#define SMBUS_CTRL_HOST_EN            (1u << 0)
#define SMBUS_CTRL_TARGET_EN          (1u << 1)
#define SMBUS_CTRL_PEC_EN             (1u << 2)
#define SMBUS_CTRL_TGT_PEC_EN         (1u << 3)
#define SMBUS_CTRL_AUTO_ARA_EN        (1u << 4)
#define SMBUS_CTRL_HOST_NOTIFY_EN     (1u << 5)
#define SMBUS_CTRL_TARGET_ALERT_EN    (1u << 6)
#define SMBUS_CTRL_TGT_STRETCH_EN     (1u << 7)
#define SMBUS_CTRL_FIFO_CLR           (1u << 8)
#define SMBUS_CTRL_TGT_FIFO_CLR       (1u << 9)
#define SMBUS_CTRL_SW_RST             (1u << 10)
#define SMBUS_CTRL_FILT_DIS           (1u << 11)

/* Host Protocol Opcodes (CMD.OP[3:0]) */
#define SMBUS_OP_NOP                  0u
#define SMBUS_OP_QUICK_CMD            1u
#define SMBUS_OP_SEND_BYTE            2u
#define SMBUS_OP_RECV_BYTE            3u
#define SMBUS_OP_WRITE_BYTE           4u
#define SMBUS_OP_READ_BYTE            5u
#define SMBUS_OP_WRITE_WORD           6u
#define SMBUS_OP_READ_WORD            7u
#define SMBUS_OP_PROCESS_CALL         8u
#define SMBUS_OP_BLOCK_WRITE          9u
#define SMBUS_OP_BLOCK_READ           10u
#define SMBUS_OP_BLOCK_PROC_CALL      11u
#define SMBUS_OP_ARA                  12u
#define SMBUS_OP_BUS_RECOVER          13u

#define SMBUS_CMD_RW_BIT              (1u << 4)
#define SMBUS_CMD_NO_PEC              (1u << 5)

#define SMBUS_CMD_PACK(op, addr7, cmd_code, tx_len, flags) \
    ((((uint32_t)(tx_len)   & 0xFFu) << 24) | \
     (((uint32_t)(cmd_code) & 0xFFu) << 16) | \
     (((uint32_t)(addr7)    & 0x7Fu) <<  8) | \
     ((uint32_t)(flags)     & 0x30u)        | \
     ((uint32_t)(op)        & 0x0Fu))

/* STATUS Bitfields */
#define SMBUS_STAT_HOST_BUSY          (1u << 0)
#define SMBUS_STAT_HOST_DONE          (1u << 1)
#define SMBUS_STAT_BUS_BUSY           (1u << 2)
#define SMBUS_STAT_ALERT_ACTIVE       (1u << 3)
#define SMBUS_STAT_ERR_NACK           (1u << 4)
#define SMBUS_STAT_ERR_PEC            (1u << 5)
#define SMBUS_STAT_ERR_ARB_LOST       (1u << 6)
#define SMBUS_STAT_ERR_TIMEOUT        (1u << 7)
#define SMBUS_STAT_ERR_SEXT           (1u << 8)
#define SMBUS_STAT_ERR_MEXT           (1u << 9)
#define SMBUS_STAT_ERR_PROTO          (1u << 10)
#define SMBUS_STAT_ERR_MASK           (0x7Fu << 4)
#define SMBUS_STAT_TGT_BUSY           (1u << 11)
#define SMBUS_STAT_TGT_ERR_PEC        (1u << 12)
#define SMBUS_STAT_TGT_RD_STRETCH     (1u << 13)
#define SMBUS_STAT_SCL_LEVEL          (1u << 14)
#define SMBUS_STAT_SDA_LEVEL          (1u << 15)
#define SMBUS_STAT_RX_CNT(s)          (((s) >> 16) & 0xFFu)
#define SMBUS_STAT_LAST_PEC(s)        (((s) >> 24) & 0xFFu)

/* IRQ_EN / IRQ_STAT Bitfields */
#define SMBUS_IRQ_HOST_DONE           (1u << 0)
#define SMBUS_IRQ_HOST_ERR            (1u << 1)
#define SMBUS_IRQ_ALERT               (1u << 2)
#define SMBUS_IRQ_ARA_DONE            (1u << 3)
#define SMBUS_IRQ_TGT_WR_DONE         (1u << 4)
#define SMBUS_IRQ_TGT_RD_REQ          (1u << 5)
#define SMBUS_IRQ_TGT_ERR             (1u << 6)
#define SMBUS_IRQ_TGT_ARA_WON         (1u << 7)
#define SMBUS_IRQ_HOST_NOTIFY         (1u << 8)

/* ARA_ADDR Bitfields */
#define SMBUS_ARA_VALID               (1u << 31)
#define SMBUS_ARA_PEC_OK              (1u << 7)
#define SMBUS_ARA_ADDR7(v)            ((uint8_t)((v) & 0x7Fu))

/* Standard PMBus 1.3 Command Codes */
#define PMBUS_CMD_PAGE                0x00u
#define PMBUS_CMD_OPERATION           0x01u
#define PMBUS_CMD_CLEAR_FAULTS        0x03u
#define PMBUS_CMD_PAGE_PLUS_WRITE     0x05u
#define PMBUS_CMD_PAGE_PLUS_READ      0x06u
#define PMBUS_CMD_VOUT_MODE           0x20u
#define PMBUS_CMD_VOUT_COMMAND        0x21u
#define PMBUS_CMD_STATUS_BYTE         0x78u
#define PMBUS_CMD_STATUS_WORD         0x79u
#define PMBUS_CMD_READ_VIN            0x88u
#define PMBUS_CMD_READ_VOUT           0x8Bu
#define PMBUS_CMD_READ_IOUT           0x8Cu
#define PMBUS_CMD_READ_TEMPERATURE_1  0x8Du
#define PMBUS_CMD_READ_POUT           0x96u
#define PMBUS_CMD_MFR_ID              0x99u
#define PMBUS_CMD_MFR_MODEL           0x9Au

/* MMIO Accessors */
static inline void smbus_write32(uintptr_t base, uint32_t off, uint32_t val) {
    *(volatile uint32_t *)(base + (uintptr_t)off) = val;
}

static inline uint32_t smbus_read32(uintptr_t base, uint32_t off) {
    return *(volatile uint32_t *)(base + (uintptr_t)off);
}

/* Sign-extend a 5-bit two's-complement exponent [4:0] */
static inline int32_t pmbus_sext5(uint32_t raw5) {
    uint32_t v = raw5 & 0x1Fu;
    return (v & 0x10u) ? (int32_t)(v | 0xFFFFFFE0u) : (int32_t)v;
}

/* Sign-extend an 11-bit two's-complement mantissa [10:0] */
static inline int32_t pmbus_sext11(uint32_t raw11) {
    uint32_t v = raw11 & 0x7FFu;
    return (v & 0x400u) ? (int32_t)(v | 0xFFFFF800u) : (int32_t)v;
}

/* Convert PMBus Linear11 (5-bit signed exp, 11-bit signed mantissa) to milli-units (x1000) */
static inline int32_t pmbus_linear11_to_milli(uint16_t raw) {
    int32_t exp  = pmbus_sext5((uint32_t)(raw >> 11));
    int32_t mant = pmbus_sext11((uint32_t)raw);
    int64_t scaled = (int64_t)mant * 1000;
    if (exp >= 0) {
        return (int32_t)(scaled << exp);
    } else {
        int32_t shift = -exp;
        if (scaled >= 0) {
            return (int32_t)((scaled + ((int64_t)1 << (shift - 1))) >> shift);
        } else {
            return (int32_t)(-(((-scaled) + ((int64_t)1 << (shift - 1))) >> shift));
        }
    }
}

/* Convert PMBus Linear16 unsigned mantissa + VOUT_MODE[4:0] exponent to millivolts */
static inline uint32_t pmbus_linear16_to_mv(uint16_t raw_vout, uint8_t vout_mode) {
    int32_t exp = pmbus_sext5((uint32_t)(vout_mode & 0x1Fu));
    uint64_t scaled = (uint64_t)raw_vout * 1000u;
    if (exp >= 0) {
        return (uint32_t)(scaled << exp);
    } else {
        int32_t shift = -exp;
        return (uint32_t)((scaled + (1ull << (shift - 1))) >> shift);
    }
}

/* Encode millivolts to PMBus Linear16 using VOUT_MODE[4:0] exponent */
static inline uint16_t pmbus_mv_to_linear16(uint32_t mv, uint8_t vout_mode) {
    int32_t exp = pmbus_sext5((uint32_t)(vout_mode & 0x1Fu));
    uint64_t val;
    if (exp < 0) {
        int32_t shift = -exp;
        val = (((uint64_t)mv << shift) + 500u) / 1000u;
    } else {
        val = ((uint64_t)mv / 1000u) >> exp;
    }
    if (val > 0xFFFFu) {
        val = 0xFFFFu;
    }
    return (uint16_t)val;
}

#endif /* SMBUS_PMBUS_REGS_H */
