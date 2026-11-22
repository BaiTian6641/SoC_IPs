/* SPDX-License-Identifier: Apache-2.0 */
/* ddr_dimm_spd_example.c - Bare-Metal Bring-Up Driver for Multi-Rank DDR3/4 DIMM Controller
 *
 * Demonstrates:
 *   1. Hardware I2C SPD EEPROM read (0x50..0x57) into internal 256B SRAM cache
 *   2. JEDEC SPD parsing: DDR3 vs DDR4, UDIMM/SO-DIMM/RDIMM/LRDIMM, Ranks, ECC, Mirroring
 *   3. Controller CFG, 4x4 Dynamic ODT matrix, and JEDEC Multi-Rank + RCD/DB Init
 *   4. Hardware MR1[7] Fly-By Write-Leveling & MR3[2] MPR Read Gate Training (9 lanes)
 *   5. Enabling 72-bit SECDED(72,64) ECC & JC42.4 TSOD thermal 2x-refresh monitoring
 */

#include "ddr_dimm_regs.h"

static uint8_t spd_cache_read_byte(uintptr_t base, uint8_t offset) {
    ddr_dimm_wr(base, DDR_DIMM_REG_SPD_ADDR, (uint32_t)offset);
    return (uint8_t)(ddr_dimm_rd(base, DDR_DIMM_REG_SPD_DATA) & 0xFFu);
}

static uint8_t i2c_raw_read_byte(uintptr_t base, uint8_t dev7, uint8_t reg_addr, uint8_t clk_div) {
    uint32_t addr_reg = ((uint32_t)reg_addr << 16) | (((uint32_t)dev7 & 0x7Fu) << 8);
    ddr_dimm_wr(base, DDR_DIMM_REG_SPD_ADDR, addr_reg);
    ddr_dimm_wr(base, DDR_DIMM_REG_SPD_CTRL,
                DDR_DIMM_SPD_DIV(clk_div) | DDR_DIMM_SPD_RAW_RNW | DDR_DIMM_SPD_RAW_GO);
    while ((ddr_dimm_rd(base, DDR_DIMM_REG_SPD_CTRL) & DDR_DIMM_SPD_BUSY) != 0u) {
        /* Poll I2C completion */
    }
    return (uint8_t)((ddr_dimm_rd(base, DDR_DIMM_REG_SPD_DATA) >> 8) & 0xFFu);
}

static int rcd_db_write_cw(uintptr_t base, uint8_t addr, uint8_t data, int is_db, int is_8bit) {
    uint32_t cmd = DDR_DIMM_CW_ADDR(addr) | DDR_DIMM_CW_DATA(data) | DDR_DIMM_CW_GO;
    if (is_db)   cmd |= DDR_DIMM_CW_TARGET_DB;
    if (is_8bit) cmd |= DDR_DIMM_CW_IS_8BIT;
    ddr_dimm_wr(base, DDR_DIMM_REG_RCD_DB_CMD, cmd);
    while ((ddr_dimm_rd(base, DDR_DIMM_REG_RCD_DB_CMD) & DDR_DIMM_CW_BUSY) != 0u) {
        /* Poll RCD/DB CW completion */
    }
    return 0;
}

int ddr_dimm_bringup(uintptr_t ctrl_base, uintptr_t dram_base, uint8_t dimm_slot) {
    uint8_t  dram_type, mod_type, mod_org, bus_w, mirror_byte;
    uint32_t std, dtype, ranks_m1, ecc_cap, mirror_en;
    uint32_t cfg, ctrl, st;
    uint8_t  i2c_div = 4u;

    /* 1. Trigger hardware I2C SPD EEPROM auto-read (256 bytes, SPD_LEN=0) */
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_SPD_CTRL,
                DDR_DIMM_SPD_SLOT(dimm_slot) |
                DDR_DIMM_TSOD_SLOT(dimm_slot) |
                DDR_DIMM_SPD_DIV(i2c_div) |
                DDR_DIMM_SPD_LEN(0u) |
                DDR_DIMM_SPD_READ_GO);

    while ((ddr_dimm_rd(ctrl_base, DDR_DIMM_REG_SPD_CTRL) & DDR_DIMM_SPD_BUSY) != 0u) {
        /* Wait for 256B SPD read */
    }
    if ((ddr_dimm_rd(ctrl_base, DDR_DIMM_REG_SPD_CTRL) & DDR_DIMM_SPD_NACK) != 0u) {
        return -1; /* No DIMM present in slot */
    }

    /* 2. Parse JEDEC SPD bytes from internal SRAM cache */
    dram_type = spd_cache_read_byte(ctrl_base, 2u);
    mod_type  = (uint8_t)(spd_cache_read_byte(ctrl_base, 3u) & 0x0Fu);
    mod_org   = spd_cache_read_byte(ctrl_base, 12u);
    bus_w     = spd_cache_read_byte(ctrl_base, 13u);

    std = (dram_type == 0x0Cu) ? DDR_DIMM_STD_DDR4 : DDR_DIMM_STD_DDR3;
    switch (mod_type) {
        case 0x01u: dtype = DDR_DIMM_TYPE_RDIMM;  break;
        case 0x03u: dtype = DDR_DIMM_TYPE_SODIMM; break;
        case 0x04u: dtype = DDR_DIMM_TYPE_LRDIMM; break;
        default:    dtype = DDR_DIMM_TYPE_UDIMM;  break;
    }

    ranks_m1 = (uint32_t)((mod_org >> 3) & 0x7u);
    if (ranks_m1 > 3u) ranks_m1 = 3u;
    ecc_cap  = ((bus_w & 0x18u) == 0x08u) ? 1u : 0u;

    /* Check odd-rank address mirroring (DDR4 byte 136 bit0, DDR3 byte 63 bit0) */
    mirror_byte = (std == DDR_DIMM_STD_DDR4)
                  ? spd_cache_read_byte(ctrl_base, 136u)
                  : i2c_raw_read_byte(ctrl_base, (uint8_t)(0x50u + (dimm_slot & 7u)), 63u, i2c_div);
    mirror_en = ((mirror_byte & 0x01u) != 0u) ? 1u : 0u;

    /* 3. Program controller topology, ODT 4x4 matrix, and trigger JEDEC + RCD/DB Init */
    cfg = DDR_DIMM_CFG_STD(std) |
          DDR_DIMM_CFG_TYPE(dtype) |
          DDR_DIMM_CFG_RANKS_M1(ranks_m1) |
          DDR_DIMM_CFG_BL8_MODE |
          DDR_DIMM_CFG_RCD_B_INV |
          DDR_DIMM_CFG_ANTI_STARVE(8u) |
          DDR_DIMM_CFG_REF_URG_WM(4u);
    if (mirror_en) cfg |= DDR_DIMM_CFG_RANK_MIRROR;
    if (dtype == DDR_DIMM_TYPE_LRDIMM && ranks_m1 == 3u) {
        cfg |= DDR_DIMM_CFG_LRDIMM_RM(1u); /* Example: 2x Rank Multiplication */
    }
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_CFG, cfg);

    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_ODT_RD_MAP, 0x00008412u);
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_ODT_WR_MAP, 0x00008421u);

    ctrl = DDR_DIMM_CTRL_INIT_START |
           DDR_DIMM_CTRL_AUTO_REF_EN |
           DDR_DIMM_CTRL_ZQCS_EN |
           DDR_DIMM_CTRL_IRQ_EN |
           DDR_DIMM_CTRL_THERMAL_2X_EN;
    if (ecc_cap) ctrl |= DDR_DIMM_CTRL_ECC_EN;
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_CTRL, ctrl);

    while ((ddr_dimm_rd(ctrl_base, DDR_DIMM_REG_STATUS) & DDR_DIMM_STATUS_INIT_DONE) == 0u) {
        /* Wait for multi-rank + RCD/DB init */
    }

    /* Optional custom RCD/DB Control Word override for RDIMM/LRDIMM */
    if (dtype == DDR_DIMM_TYPE_RDIMM || dtype == DDR_DIMM_TYPE_LRDIMM) {
        rcd_db_write_cw(ctrl_base, 0x0Au, 0x03u, 0, 0); /* RC0A operating speed */
    }

    /* 4. Run hardware Write-Leveling (MR1[7]) + Read Gate MPR (MR3[2]) across 9 lanes */
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_TRAIN_CTRL,
                DDR_DIMM_TRAIN_MODE_BOTH |
                DDR_DIMM_TRAIN_RANK(0u) |
                DDR_DIMM_TRAIN_WL_MAX(7u) |
                DDR_DIMM_TRAIN_GO);
    do {
        st = ddr_dimm_rd(ctrl_base, DDR_DIMM_REG_TRAIN_CTRL);
    } while ((st & DDR_DIMM_TRAIN_DONE) == 0u);
    if ((st & (DDR_DIMM_TRAIN_WL_PASS | DDR_DIMM_TRAIN_RG_PASS)) !=
        (DDR_DIMM_TRAIN_WL_PASS | DDR_DIMM_TRAIN_RG_PASS)) {
        return -2;
    }

    /* 5. Enable periodic JC42.4 TSOD polling (+85 C = 0x0550 -> automatic 2x refresh) */
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_TSOD_CFG, (1024u << 16) | 0x0550u);
    ddr_dimm_wr(ctrl_base, DDR_DIMM_REG_SPD_CTRL,
                DDR_DIMM_SPD_SLOT(dimm_slot) |
                DDR_DIMM_TSOD_SLOT(dimm_slot) |
                DDR_DIMM_SPD_DIV(i2c_div) |
                DDR_DIMM_SPD_TSOD_POLL_EN);

    /* 6. Smoke test 64-bit DIMM memory + ECC72 RMW */
    *(volatile uint32_t *)(dram_base + 0x00u) = 0xA55A1234u;
    *(volatile uint32_t *)(dram_base + 0x04u) = 0x5AA5FEDCu;
    if (*(volatile uint32_t *)(dram_base + 0x00u) != 0xA55A1234u) return -3;
    if (*(volatile uint32_t *)(dram_base + 0x04u) != 0x5AA5FEDCu) return -4;

    return 0;
}
