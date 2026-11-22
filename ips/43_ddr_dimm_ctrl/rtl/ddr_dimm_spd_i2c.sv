// SPDX-License-Identifier: Apache-2.0
// ddr_dimm_spd_i2c.sv - I2C/SMBus Master for JEDEC SPD EEPROM (0x50..0x57)
//                       & JC42.4 Thermal Sensor (TSOD, 0x18..0x1F)
//
// Features:
//   - Internal 256-byte SPD cache implemented via `tech_sram_sp` (clock-only writes)
//   - Hardware auto-capture of JEDEC SPD summary bytes (2, 3, 4, 12, 13, 131, 136)
//   - Periodic or on-demand JC42.4 TSOD temperature polling -> thermal_2x_ref_o
//   - Generic manual I2C register read/write (e.g. DDR4 EE1004 page select)

module ddr_dimm_spd_i2c (
  input  logic        clk_i,
  input  logic        rst_ni,

  // Control & Configuration
  input  logic [2:0]  spd_slot_i,
  input  logic [2:0]  tsod_slot_i,
  input  logic        tsod_poll_en_i,
  input  logic [15:0] tsod_poll_int_i,
  input  logic [12:0] tsod_thresh_i,    // Signed 13-bit JC42.4 (0.0625 C/LSB)
  input  logic [7:0]  spd_read_len_i,   // 0 = 256 bytes, 1..255 = N bytes
  input  logic [7:0]  i2c_clk_div_i,

  input  logic        spd_read_go_i,
  input  logic        tsod_read_go_i,
  input  logic        raw_go_i,
  input  logic        raw_rnw_i,
  input  logic [6:0]  raw_dev_addr_i,
  input  logic [7:0]  raw_reg_addr_i,
  input  logic [7:0]  raw_wdata_i,

  // 256-byte SPD Cache CSR Read Port
  input  logic [7:0]  cache_addr_i,
  output logic [7:0]  cache_rdata_o,

  // Status & Parsed Outputs
  output logic        busy_o,
  output logic        done_o,
  output logic        nack_o,
  output logic [7:0]  raw_rdata_o,
  output logic [7:0]  spd_byte2_o,      // DRAM Type (0x0B=DDR3, 0x0C=DDR4)
  output logic [7:0]  spd_byte3_o,      // Module Type (1=RDIMM,2=UDIMM,3=SO-DIMM,4=LRDIMM)
  output logic [7:0]  spd_byte4_o,      // Density & Banks
  output logic [7:0]  spd_byte12_o,     // Module Organization (Ranks, Width)
  output logic [7:0]  spd_byte13_o,     // Bus Width & ECC extension
  output logic [7:0]  spd_byte131_o,    // DDR4 RCD / Module Height
  output logic [7:0]  spd_byte136_o,    // DDR4 Address Mapping (bit0 = Odd-Rank Mirror)
  output logic [15:0] tsod_raw_o,
  output logic [12:0] tsod_temp_o,
  output logic        tsod_valid_o,
  output logic        thermal_2x_ref_o,
  output logic        thermal_irq_p_o,

  // Open-Drain Split I2C Pads (o=0, oe=1 pulls bus low)
  input  logic        spd_scl_i,
  output logic        spd_scl_o,
  output logic        spd_scl_oe_o,
  input  logic        spd_sda_i,
  output logic        spd_sda_o,
  output logic        spd_sda_oe_o
);

  typedef enum logic [1:0] {
    OP_NONE = 2'd0,
    OP_SPD  = 2'd1,
    OP_TSOD = 2'd2,
    OP_RAW  = 2'd3
  } op_mode_e;

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_START,
    ST_RSTART,
    ST_TX_BYTE,
    ST_RX_BYTE,
    ST_STOP,
    ST_NEXT_STEP
  } i2c_state_e;

  i2c_state_e  state_q;
  op_mode_e    op_q;
  logic [2:0]  step_q;
  logic [1:0]  phase_q;
  logic [3:0]  bit_cnt_q;
  logic [7:0]  div_cnt_q;
  logic [7:0]  tx_shift_q, rx_shift_q;
  logic        rx_nack_bit_q;

  logic        scl_oe_q, sda_oe_q;
  logic        busy_q, done_q, nack_q;
  logic        pend_spd_q, pend_tsod_q, pend_raw_q;
  logic [15:0] poll_cnt_q;

  // Latched raw request
  logic        raw_rnw_q;
  logic [6:0]  raw_dev_q;
  logic [7:0]  raw_reg_q, raw_wdata_q, raw_rdata_q;

  // SPD capture tracking
  logic [7:0]  spd_idx_q, spd_waddr_q, spd_len_m1_q;
  logic [7:0]  spd_b2_q, spd_b3_q, spd_b4_q, spd_b12_q, spd_b13_q, spd_b131_q, spd_b136_q;

  // TSOD temperature state
  logic [15:0] tsod_raw_q;
  logic        tsod_valid_q;
  logic        thermal_2x_q;

  // SRAM write interface for 256-byte SPD cache
  logic        sram_we;
  logic [7:0]  sram_addr;
  logic [7:0]  sram_wdata;
  logic [7:0]  sram_rdata;

  assign sram_addr  = sram_we ? spd_waddr_q : cache_addr_i;
  assign sram_wdata = rx_shift_q;

  tech_sram_sp #(
    .DATA_W (8),
    .DEPTH  (256),
    .BYTE_W (8)
  ) u_spd_cache (
    .clk_i   (clk_i),
    .en_i    (1'b1),
    .we_i    (sram_we),
    .wstrb_i (1'b1),
    .addr_i  (sram_addr),
    .wdata_i (sram_wdata),
    .rdata_o (sram_rdata)
  );

  assign cache_rdata_o    = sram_rdata;
  assign busy_o           = busy_q;
  assign done_o           = done_q;
  assign nack_o           = nack_q;
  assign raw_rdata_o      = raw_rdata_q;
  assign spd_byte2_o      = spd_b2_q;
  assign spd_byte3_o      = spd_b3_q;
  assign spd_byte4_o      = spd_b4_q;
  assign spd_byte12_o     = spd_b12_q;
  assign spd_byte13_o     = spd_b13_q;
  assign spd_byte131_o    = spd_b131_q;
  assign spd_byte136_o    = spd_b136_q;
  assign tsod_raw_o       = tsod_raw_q;
  assign tsod_temp_o      = tsod_raw_q[12:0];
  assign tsod_valid_o     = tsod_valid_q;
  assign thermal_2x_ref_o = thermal_2x_q;

  assign spd_scl_o    = 1'b0;
  assign spd_scl_oe_o = scl_oe_q;
  assign spd_sda_o    = 1'b0;
  assign spd_sda_oe_o = sda_oe_q;

  // Signed 13-bit JC42.4 temperature comparison (2's complement: bit 12 is sign)
  wire signed [12:0] cur_temp_s   = $signed(tsod_raw_q[12:0]);
  wire signed [12:0] thresh_s     = $signed(tsod_thresh_i);
  wire               temp_hot_now = tsod_valid_q && (cur_temp_s >= thresh_s);

  wire unused_scl_i = &{1'b0, spd_scl_i};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q         <= ST_IDLE;
      op_q            <= OP_NONE;
      step_q          <= 3'd0;
      phase_q         <= 2'd0;
      bit_cnt_q       <= 4'd0;
      div_cnt_q       <= 8'd0;
      tx_shift_q      <= 8'd0;
      rx_shift_q      <= 8'd0;
      rx_nack_bit_q   <= 1'b0;
      scl_oe_q        <= 1'b0;
      sda_oe_q        <= 1'b0;
      busy_q          <= 1'b0;
      done_q          <= 1'b0;
      nack_q          <= 1'b0;
      pend_spd_q      <= 1'b0;
      pend_tsod_q     <= 1'b0;
      pend_raw_q      <= 1'b0;
      poll_cnt_q      <= 16'd0;
      raw_rnw_q       <= 1'b0;
      raw_dev_q       <= 7'd0;
      raw_reg_q       <= 8'd0;
      raw_wdata_q     <= 8'd0;
      raw_rdata_q     <= 8'd0;
      spd_idx_q       <= 8'd0;
      spd_waddr_q     <= 8'd0;
      spd_len_m1_q    <= 8'hFF;
      spd_b2_q        <= 8'd0;
      spd_b3_q        <= 8'd0;
      spd_b4_q        <= 8'd0;
      spd_b12_q       <= 8'd0;
      spd_b13_q       <= 8'd0;
      spd_b131_q      <= 8'd0;
      spd_b136_q      <= 8'd0;
      tsod_raw_q      <= 16'd0;
      tsod_valid_q    <= 1'b0;
      thermal_2x_q    <= 1'b0;
      thermal_irq_p_o <= 1'b0;
      sram_we         <= 1'b0;
    end else begin
      sram_we         <= 1'b0;
      thermal_irq_p_o <= 1'b0;

      if (temp_hot_now && !thermal_2x_q) begin
        thermal_irq_p_o <= 1'b1;
      end
      thermal_2x_q <= temp_hot_now;

      // Latch triggers
      if (spd_read_go_i)  pend_spd_q  <= 1'b1;
      if (tsod_read_go_i) pend_tsod_q <= 1'b1;
      if (raw_go_i) begin
        pend_raw_q  <= 1'b1;
        raw_rnw_q   <= raw_rnw_i;
        raw_dev_q   <= raw_dev_addr_i;
        raw_reg_q   <= raw_reg_addr_i;
        raw_wdata_q <= raw_wdata_i;
      end

      // Periodic TSOD polling timer
      if (tsod_poll_en_i) begin
        if (poll_cnt_q == 16'd0) begin
          poll_cnt_q  <= (tsod_poll_int_i == 16'd0) ? 16'd512 : tsod_poll_int_i;
          pend_tsod_q <= 1'b1;
        end else begin
          poll_cnt_q <= poll_cnt_q - 16'd1;
        end
      end else begin
        poll_cnt_q <= (tsod_poll_int_i == 16'd0) ? 16'd512 : tsod_poll_int_i;
      end

      // I2C State Machine
      if (state_q == ST_IDLE) begin
        scl_oe_q <= 1'b0;
        sda_oe_q <= 1'b0;
        if (pend_spd_q || pend_tsod_q || pend_raw_q) begin
          busy_q    <= 1'b1;
          done_q    <= 1'b0;
          nack_q    <= 1'b0;
          step_q    <= 3'd0;
          phase_q   <= 2'd0;
          bit_cnt_q <= 4'd0;
          div_cnt_q <= i2c_clk_div_i;
          state_q   <= ST_START;
          if (pend_spd_q) begin
            op_q         <= OP_SPD;
            pend_spd_q   <= 1'b0;
            spd_idx_q    <= 8'd0;
            spd_len_m1_q <= (spd_read_len_i == 8'd0) ? 8'hFF : (spd_read_len_i - 8'd1);
          end else if (pend_tsod_q) begin
            op_q        <= OP_TSOD;
            pend_tsod_q <= 1'b0;
          end else begin
            op_q       <= OP_RAW;
            pend_raw_q <= 1'b0;
          end
        end
      end else if (state_q == ST_NEXT_STEP) begin
        // Dispatch next I2C byte/action in the active operation sequence
        phase_q   <= 2'd0;
        bit_cnt_q <= 4'd0;
        div_cnt_q <= i2c_clk_div_i;
        if (nack_q) begin
          state_q <= ST_STOP;
        end else begin
          unique case (op_q)
            OP_SPD: begin
              unique case (step_q)
                3'd0: begin // Write SPD addr + W
                  tx_shift_q <= {4'b1010, spd_slot_i, 1'b0};
                  step_q     <= 3'd1;
                  state_q    <= ST_TX_BYTE;
                end
                3'd1: begin // Write EEPROM byte offset 0x00
                  tx_shift_q <= 8'h00;
                  step_q     <= 3'd2;
                  state_q    <= ST_TX_BYTE;
                end
                3'd2: begin // Repeated START
                  step_q  <= 3'd3;
                  state_q <= ST_RSTART;
                end
                3'd3: begin // Write SPD addr + R
                  tx_shift_q <= {4'b1010, spd_slot_i, 1'b1};
                  step_q     <= 3'd4;
                  state_q    <= ST_TX_BYTE;
                end
                3'd4: begin // Read SPD bytes 0 .. spd_len_m1_q
                  rx_nack_bit_q <= (spd_idx_q == spd_len_m1_q);
                  state_q       <= ST_RX_BYTE;
                end
                default: state_q <= ST_STOP;
              endcase
            end

            OP_TSOD: begin
              unique case (step_q)
                3'd0: begin // Write TSOD addr + W
                  tx_shift_q <= {4'b0011, tsod_slot_i, 1'b0};
                  step_q     <= 3'd1;
                  state_q    <= ST_TX_BYTE;
                end
                3'd1: begin // Register pointer 0x05 (Ambient Temperature)
                  tx_shift_q <= 8'h05;
                  step_q     <= 3'd2;
                  state_q    <= ST_TX_BYTE;
                end
                3'd2: begin // Repeated START
                  step_q  <= 3'd3;
                  state_q <= ST_RSTART;
                end
                3'd3: begin // Write TSOD addr + R
                  tx_shift_q <= {4'b0011, tsod_slot_i, 1'b1};
                  step_q     <= 3'd4;
                  state_q    <= ST_TX_BYTE;
                end
                3'd4: begin // Read Temp MSB (ACK)
                  rx_nack_bit_q <= 1'b0;
                  step_q        <= 3'd5;
                  state_q       <= ST_RX_BYTE;
                end
                3'd5: begin // Read Temp LSB (NACK)
                  rx_nack_bit_q <= 1'b1;
                  step_q        <= 3'd6;
                  state_q       <= ST_RX_BYTE;
                end
                default: state_q <= ST_STOP;
              endcase
            end

            OP_RAW: begin
              unique case (step_q)
                3'd0: begin
                  tx_shift_q <= {raw_dev_q, 1'b0};
                  step_q     <= 3'd1;
                  state_q    <= ST_TX_BYTE;
                end
                3'd1: begin
                  tx_shift_q <= raw_reg_q;
                  step_q     <= 3'd2;
                  state_q    <= ST_TX_BYTE;
                end
                3'd2: begin
                  if (raw_rnw_q) begin
                    step_q  <= 3'd3;
                    state_q <= ST_RSTART;
                  end else begin
                    tx_shift_q <= raw_wdata_q;
                    step_q     <= 3'd5;
                    state_q    <= ST_TX_BYTE;
                  end
                end
                3'd3: begin
                  tx_shift_q <= {raw_dev_q, 1'b1};
                  step_q     <= 3'd4;
                  state_q    <= ST_TX_BYTE;
                end
                3'd4: begin
                  rx_nack_bit_q <= 1'b1;
                  step_q        <= 3'd5;
                  state_q       <= ST_RX_BYTE;
                end
                default: state_q <= ST_STOP;
              endcase
            end

            default: state_q <= ST_STOP;
          endcase
        end
      end else begin
        // Quarter-bit clock divider tick
        if (div_cnt_q != 8'd0) begin
          div_cnt_q <= div_cnt_q - 8'd1;
        end else begin
          div_cnt_q <= i2c_clk_div_i;

          unique case (state_q)
            ST_START: begin
              unique case (phase_q)
                2'd0: begin scl_oe_q <= 1'b0; sda_oe_q <= 1'b0; phase_q <= 2'd1; end
                2'd1: begin scl_oe_q <= 1'b0; sda_oe_q <= 1'b1; phase_q <= 2'd2; end
                default: begin
                  scl_oe_q <= 1'b1;
                  sda_oe_q <= 1'b1;
                  phase_q  <= 2'd0;
                  state_q  <= ST_NEXT_STEP;
                end
              endcase
            end

            ST_RSTART: begin
              unique case (phase_q)
                2'd0: begin scl_oe_q <= 1'b1; sda_oe_q <= 1'b0; phase_q <= 2'd1; end
                2'd1: begin scl_oe_q <= 1'b0; sda_oe_q <= 1'b0; phase_q <= 2'd2; end
                2'd2: begin scl_oe_q <= 1'b0; sda_oe_q <= 1'b1; phase_q <= 2'd3; end
                default: begin
                  scl_oe_q <= 1'b1;
                  sda_oe_q <= 1'b1;
                  phase_q  <= 2'd0;
                  state_q  <= ST_NEXT_STEP;
                end
              endcase
            end

            ST_TX_BYTE: begin
              unique case (phase_q)
                2'd0: begin
                  scl_oe_q <= 1'b1;
                  sda_oe_q <= (bit_cnt_q == 4'd8) ? 1'b0 : ~tx_shift_q[7];
                  phase_q  <= 2'd1;
                end
                2'd1: begin
                  scl_oe_q <= 1'b0;
                  phase_q  <= 2'd2;
                end
                2'd2: begin
                  scl_oe_q <= 1'b0;
                  if (bit_cnt_q == 4'd8) begin
                    if (spd_sda_i) nack_q <= 1'b1;
                  end
                  phase_q <= 2'd3;
                end
                default: begin
                  scl_oe_q <= 1'b1;
                  phase_q  <= 2'd0;
                  if (bit_cnt_q == 4'd8) begin
                    bit_cnt_q <= 4'd0;
                    state_q   <= ST_NEXT_STEP;
                  end else begin
                    tx_shift_q <= {tx_shift_q[6:0], 1'b0};
                    bit_cnt_q  <= bit_cnt_q + 4'd1;
                  end
                end
              endcase
            end

            ST_RX_BYTE: begin
              unique case (phase_q)
                2'd0: begin
                  scl_oe_q <= 1'b1;
                  sda_oe_q <= (bit_cnt_q == 4'd8) ? ~rx_nack_bit_q : 1'b0;
                  phase_q  <= 2'd1;
                end
                2'd1: begin
                  scl_oe_q <= 1'b0;
                  phase_q  <= 2'd2;
                end
                2'd2: begin
                  scl_oe_q <= 1'b0;
                  if (bit_cnt_q < 4'd8) begin
                    rx_shift_q <= {rx_shift_q[6:0], spd_sda_i};
                  end
                  phase_q <= 2'd3;
                end
                default: begin
                  scl_oe_q <= 1'b1;
                  phase_q  <= 2'd0;
                  if (bit_cnt_q == 4'd8) begin
                    bit_cnt_q <= 4'd0;
                    // Commit received byte
                    if (op_q == OP_SPD) begin
                      sram_we     <= 1'b1;
                      spd_waddr_q <= spd_idx_q;
                      unique case (spd_idx_q)
                        8'd2:   spd_b2_q   <= rx_shift_q;
                        8'd3:   spd_b3_q   <= rx_shift_q;
                        8'd4:   spd_b4_q   <= rx_shift_q;
                        8'd12:  spd_b12_q  <= rx_shift_q;
                        8'd13:  spd_b13_q  <= rx_shift_q;
                        8'd131: spd_b131_q <= rx_shift_q;
                        8'd136: spd_b136_q <= rx_shift_q;
                        default: ;
                      endcase
                      if (spd_idx_q == spd_len_m1_q) begin
                        step_q  <= 3'd5;
                        state_q <= ST_STOP;
                      end else begin
                        spd_idx_q <= spd_idx_q + 8'd1;
                        state_q   <= ST_NEXT_STEP;
                      end
                    end else if (op_q == OP_TSOD) begin
                      if (step_q == 3'd5) begin
                        tsod_raw_q[15:8] <= rx_shift_q;
                        state_q          <= ST_NEXT_STEP;
                      end else begin
                        tsod_raw_q[7:0] <= rx_shift_q;
                        tsod_valid_q    <= 1'b1;
                        state_q         <= ST_STOP;
                      end
                    end else begin
                      raw_rdata_q <= rx_shift_q;
                      state_q     <= ST_STOP;
                    end
                  end else begin
                    bit_cnt_q <= bit_cnt_q + 4'd1;
                  end
                end
              endcase
            end

            ST_STOP: begin
              unique case (phase_q)
                2'd0: begin scl_oe_q <= 1'b1; sda_oe_q <= 1'b1; phase_q <= 2'd1; end
                2'd1: begin scl_oe_q <= 1'b0; sda_oe_q <= 1'b1; phase_q <= 2'd2; end
                2'd2: begin scl_oe_q <= 1'b0; sda_oe_q <= 1'b0; phase_q <= 2'd3; end
                default: begin
                  scl_oe_q <= 1'b0;
                  sda_oe_q <= 1'b0;
                  phase_q  <= 2'd0;
                  busy_q   <= 1'b0;
                  done_q   <= 1'b1;
                  op_q     <= OP_NONE;
                  state_q  <= ST_IDLE;
                end
              endcase
            end

            default: state_q <= ST_IDLE;
          endcase
        end
      end
    end
  end

endmodule : ddr_dimm_spd_i2c
