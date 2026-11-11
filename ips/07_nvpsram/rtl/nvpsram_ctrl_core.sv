// SPDX-License-Identifier: Apache-2.0
// nvpsram_ctrl_core.sv - Non-Volatile PSRAM Controller with Selectable Parallel SRAM & QSPI Interfaces
//
// Features:
//   - Interface 0 (`CTRL[1]=0`): Parallel 16-bit SRAM/PSRAM Interface (`ce_n`, `oe_n`, `we_n`, `adv_n`,
//     `be_n[1:0]`, `wait_i` variable refresh collision handshake)
//   - Interface 1 (`CTRL[1]=1`): Quad SPI (QSPI) PSRAM Interface (`sck`, `cs_n`, `sio[3:0]`,
//     Quad Fast Read `0xEB`, Quad Write `0x38`, Enter Quad Mode `0x35`, NV Store `0x71`, NV Recall `0x72`)
//   - Hardware `tCEM` Maximum Chip-Enable Low Watchdog Guard (enforces mandatory CE# high window so
//     internal PSRAM DRAM self-refresh never starves!)
//
// CSR Register Map:
//   0x00 CTRL    : [0]=enable, [1]=if_sel(0:Parallel_SRAM, 1:QSPI), [2]=qspi_quad_mode,
//                  [7:4]=qspi_read_wait_nibbles (default 6), [15:8]=sck_or_access_div
//   0x04 TCEM    : [15:0]=max_ce_low_cycles (tCEM guard), [23:16]=min_ce_high_cycles (tCPH)
//   0x08 NV_CMD  : [0]=nv_store(WO), [1]=nv_recall(WO), [2]=enter_qpi_35h(WO)
//   0x0C STATUS  : [0]=busy, [1]=nv_cmd_done(W1C), [2]=tcem_split_occurred(RO)

module nvpsram_ctrl_core
  import bus_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  input  csr_req_t    mem_win_req_i,
  output csr_rsp_t    mem_win_rsp_o,

  // Interface A: Parallel SRAM / PSRAM Bus
  output logic        psram_ce_no,
  output logic        psram_oe_no,
  output logic        psram_we_no,
  output logic        psram_adv_no,
  output logic [1:0]  psram_be_no,
  output logic [21:0] psram_addr_o,
  input  logic [15:0] psram_dq_i,
  output logic [15:0] psram_dq_o,
  output logic        psram_dq_oe_o,
  input  logic        psram_wait_i,

  // Interface B: QSPI PSRAM Bus
  output logic        qspi_sck_o,
  output logic        qspi_cs_no,
  input  logic [3:0]  qspi_sio_i,
  output logic [3:0]  qspi_sio_o,
  output logic [3:0]  qspi_sio_oe_o,

  output logic        irq_o
);

  logic [31:0] ctrl_q;
  logic [23:0] tcem_cfg_q;
  logic        nv_done_q, tcem_flag_q;

  wire enable    = ctrl_q[0];
  wire qspi_sel  = ctrl_q[1];
  wire quad_cmd  = ctrl_q[2];
  wire [3:0] qspi_dummy = ctrl_q[7:4];
  wire [7:0] acc_div    = (ctrl_q[15:8] == 8'd0) ? 8'd1 : ctrl_q[15:8];

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_PAR_H0,
    ST_PAR_TCEM_GAP,
    ST_PAR_H1,
    ST_QSPI_SHIFT,
    ST_CE_RECOVERY
  } state_e;

  state_e      state_q;
  logic [15:0] celow_cnt_q;
  logic [7:0]  div_cnt_q;
  logic        is_wr_q, is_nv_op_q;
  logic [31:0] addr_q, wdata_q, rdata_q;
  logic [3:0]  wstrb_q;
  logic        mem_done_q;

  // Parallel signals
  logic        p_ce_nq, p_oe_nq, p_we_nq, p_adv_nq, p_dq_oe_q;
  logic [1:0]  p_be_nq;
  logic [21:0] p_addr_q;
  logic [15:0] p_dq_out_q;

  // QSPI signals (transfers up to 8 nibbles cmd/addr + dummy + 8 nibbles 32b data)
  logic        q_sck_q, q_cs_nq;
  logic [3:0]  q_sio_out_q, q_sio_oe_q;
  logic [63:0] q_shreg_q;
  logic [6:0]  q_steps_rem_q, q_rd_start_step_q;
  logic        q_1bit_cmd_q;

  assign psram_ce_no   = p_ce_nq;
  assign psram_oe_no   = p_oe_nq;
  assign psram_we_no   = p_we_nq;
  assign psram_adv_no  = p_adv_nq;
  assign psram_be_no   = p_be_nq;
  assign psram_addr_o  = p_addr_q;
  assign psram_dq_o    = p_dq_out_q;
  assign psram_dq_oe_o = p_dq_oe_q;

  assign qspi_sck_o    = q_sck_q;
  assign qspi_cs_no    = q_cs_nq;
  assign qspi_sio_o    = q_sio_out_q;
  assign qspi_sio_oe_o = q_sio_oe_q;

  wire [7:0] reg_off = csr_req_i.addr[7:0];
  wire nv_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h08) && (|csr_req_i.wdata[2:0]) && (state_q == ST_IDLE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q            <= 32'h0000_0165; // Enabled, Parallel=0/QSPI selectable, Quad=1, 6 dummy, div=1
      tcem_cfg_q        <= {8'd4, 16'd400};
      nv_done_q         <= 1'b0;
      tcem_flag_q       <= 1'b0;
      state_q           <= ST_IDLE;
      celow_cnt_q       <= '0;
      div_cnt_q         <= '0;
      is_wr_q           <= 1'b0;
      is_nv_op_q        <= 1'b0;
      addr_q            <= '0;
      wdata_q           <= '0;
      rdata_q           <= '0;
      wstrb_q           <= '0;
      mem_done_q        <= 1'b0;
      p_ce_nq           <= 1'b1;
      p_oe_nq           <= 1'b1;
      p_we_nq           <= 1'b1;
      p_adv_nq          <= 1'b1;
      p_be_nq           <= 2'b11;
      p_addr_q          <= '0;
      p_dq_out_q        <= '0;
      p_dq_oe_q         <= 1'b0;
      q_sck_q           <= 1'b0;
      q_cs_nq           <= 1'b1;
      q_sio_out_q       <= '0;
      q_sio_oe_q        <= '0;
      q_shreg_q         <= '0;
      q_steps_rem_q     <= '0;
      q_rd_start_step_q <= '0;
      q_1bit_cmd_q      <= 1'b0;
    end else begin
      mem_done_q <= 1'b0;

      if (!p_ce_nq || !q_cs_nq) celow_cnt_q <= celow_cnt_q + 16'd1;
      else                      celow_cnt_q <= '0;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q     <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: tcem_cfg_q <= csr_req_i.wdata[23:0];
          8'h0C: if (csr_req_i.wdata[1]) nv_done_q <= 1'b0;
          default: ;
        endcase
      end

      unique case (state_q)
        ST_IDLE: begin
          p_ce_nq    <= 1'b1;
          p_oe_nq    <= 1'b1;
          p_we_nq    <= 1'b1;
          p_adv_nq   <= 1'b1;
          p_dq_oe_q  <= 1'b0;
          q_sck_q    <= 1'b0;
          q_cs_nq    <= 1'b1;
          q_sio_oe_q <= 4'h0;

          if (enable && nv_trig) begin
            // Issue 8-bit QSPI / NV opcode (0x71 STORE, 0x72 RECALL, 0x35 Enter QPI)
            logic [7:0] op;
            op           = csr_req_i.wdata[0] ? 8'h71 : (csr_req_i.wdata[1] ? 8'h72 : 8'h35);
            is_nv_op_q   <= 1'b1;
            is_wr_q      <= 1'b1;
            q_cs_nq      <= 1'b0;
            div_cnt_q    <= acc_div - 8'd1;
            if (quad_cmd && !csr_req_i.wdata[2]) begin
              q_1bit_cmd_q  <= 1'b0;
              q_shreg_q     <= {op[3:0], 60'd0};
              q_sio_out_q   <= op[7:4];
              q_sio_oe_q    <= 4'hF;
              q_steps_rem_q <= 7'd2;
            end else begin
              q_1bit_cmd_q  <= 1'b1;
              q_shreg_q     <= {op[6:0], 57'd0};
              q_sio_out_q   <= {3'd0, op[7]};
              q_sio_oe_q    <= 4'b0001;
              q_steps_rem_q <= 7'd8;
            end
            state_q <= ST_QSPI_SHIFT;
          end else if (enable && mem_win_req_i.valid && !mem_done_q) begin
            is_nv_op_q <= 1'b0;
            is_wr_q    <= mem_win_req_i.write;
            addr_q     <= mem_win_req_i.addr;
            wdata_q    <= mem_win_req_i.wdata;
            wstrb_q    <= mem_win_req_i.wstrb;
            div_cnt_q  <= acc_div - 8'd1;

            if (!qspi_sel) begin
              // Parallel 16-bit PSRAM/SRAM mode
              p_ce_nq    <= 1'b0;
              p_adv_nq   <= 1'b0;
              p_oe_nq    <= mem_win_req_i.write;
              p_we_nq    <= !mem_win_req_i.write;
              p_be_nq    <= mem_win_req_i.write ? ~mem_win_req_i.wstrb[1:0] : 2'b00;
              p_addr_q   <= {mem_win_req_i.addr[22:2], 1'b0};
              p_dq_out_q <= mem_win_req_i.wdata[15:0];
              p_dq_oe_q  <= mem_win_req_i.write;
              state_q    <= ST_PAR_H0;
            end else begin
              // QSPI PSRAM Quad Read (0xEB) or Quad Write (0x38) in QPI 4-4-4 mode
              logic [7:0] op;
              logic [63:0] pkt;
              op  = mem_win_req_i.write ? 8'h38 : 8'hEB;
              pkt = {op, mem_win_req_i.addr[23:2], 2'b00, mem_win_req_i.wdata};
              q_1bit_cmd_q <= 1'b0;
              q_cs_nq      <= 1'b0;
              q_sio_out_q  <= pkt[63:60];
              q_shreg_q    <= {pkt[59:0], 4'd0};
              q_sio_oe_q   <= 4'hF;
              if (mem_win_req_i.write) begin
                q_steps_rem_q     <= 7'd16; // 2 cmd + 6 addr + 8 data nibbles
                q_rd_start_step_q <= 7'd0;
              end else begin
                q_steps_rem_q     <= 7'd8 + {3'd0, qspi_dummy} + 7'd8;
                q_rd_start_step_q <= 7'd8; // Final 8 nibbles are read data
              end
              state_q <= ST_QSPI_SHIFT;
            end
          end
        end

        ST_PAR_H0: begin
          p_adv_nq <= 1'b1;
          if (!psram_wait_i) begin
            if (div_cnt_q == 8'd0) begin
              if (!is_wr_q) rdata_q[15:0] <= psram_dq_i;
              // Check tCEM max CS low time guard before issuing second halfword
              if (celow_cnt_q >= (tcem_cfg_q[15:0] >> 1)) begin
                p_ce_nq     <= 1'b1;
                p_oe_nq     <= 1'b1;
                p_we_nq     <= 1'b1;
                p_dq_oe_q   <= 1'b0;
                tcem_flag_q <= 1'b1;
                div_cnt_q   <= tcem_cfg_q[23:16];
                state_q     <= ST_PAR_TCEM_GAP;
              end else begin
                p_adv_nq   <= 1'b0;
                p_addr_q   <= {addr_q[22:2], 1'b1};
                p_be_nq    <= is_wr_q ? ~wstrb_q[3:2] : 2'b00;
                p_dq_out_q <= wdata_q[31:16];
                div_cnt_q  <= acc_div - 8'd1;
                state_q    <= ST_PAR_H1;
              end
            end else div_cnt_q <= div_cnt_q - 8'd1;
          end
        end

        ST_PAR_TCEM_GAP: begin
          if (div_cnt_q == 8'd0) begin
            p_ce_nq    <= 1'b0;
            p_adv_nq   <= 1'b0;
            p_oe_nq    <= is_wr_q;
            p_we_nq    <= !is_wr_q;
            p_addr_q   <= {addr_q[22:2], 1'b1};
            p_be_nq    <= is_wr_q ? ~wstrb_q[3:2] : 2'b00;
            p_dq_out_q <= wdata_q[31:16];
            p_dq_oe_q  <= is_wr_q;
            div_cnt_q  <= acc_div - 8'd1;
            state_q    <= ST_PAR_H1;
          end else div_cnt_q <= div_cnt_q - 8'd1;
        end

        ST_PAR_H1: begin
          p_adv_nq <= 1'b1;
          if (!psram_wait_i) begin
            if (div_cnt_q == 8'd0) begin
              if (!is_wr_q) rdata_q[31:16] <= psram_dq_i;
              p_ce_nq   <= 1'b1;
              p_oe_nq   <= 1'b1;
              p_we_nq   <= 1'b1;
              p_dq_oe_q <= 1'b0;
              div_cnt_q <= tcem_cfg_q[23:16];
              state_q   <= ST_CE_RECOVERY;
            end else div_cnt_q <= div_cnt_q - 8'd1;
          end
        end

        ST_QSPI_SHIFT: begin
          if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= acc_div - 8'd1;
            if (!q_sck_q) begin
              q_sck_q <= 1'b1;
              if (!is_wr_q && !is_nv_op_q && (q_steps_rem_q <= q_rd_start_step_q)) begin
                rdata_q <= {rdata_q[27:0], qspi_sio_i};
              end
            end else begin
              q_sck_q <= 1'b0;
              if (q_steps_rem_q <= 7'd1) begin
                q_cs_nq    <= 1'b1;
                q_sio_oe_q <= 4'h0;
                div_cnt_q  <= tcem_cfg_q[23:16];
                state_q    <= ST_CE_RECOVERY;
              end else begin
                logic [6:0] nxt_rem;
                nxt_rem       = q_steps_rem_q - 7'd1;
                q_steps_rem_q <= nxt_rem;
                if (q_1bit_cmd_q) begin
                  q_sio_out_q <= {3'd0, q_shreg_q[63]};
                  q_shreg_q   <= {q_shreg_q[62:0], 1'b0};
                end else if (!is_wr_q && !is_nv_op_q && (nxt_rem <= (q_rd_start_step_q + {3'd0, qspi_dummy}))) begin
                  q_sio_oe_q <= 4'h0; // Tristate IO lines during QSPI dummy & read nibbles
                end else begin
                  q_sio_out_q <= q_shreg_q[63:60];
                  q_shreg_q   <= {q_shreg_q[59:0], 4'd0};
                end
              end
            end
          end
        end

        ST_CE_RECOVERY: begin
          if (div_cnt_q == 8'd0) begin
            if (is_nv_op_q) nv_done_q  <= 1'b1;
            else            mem_done_q <= 1'b1;
            state_q <= ST_IDLE;
          end else div_cnt_q <= div_cnt_q - 8'd1;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

  always_comb begin
    mem_win_rsp_o = '0;
    if (mem_win_req_i.valid) begin
      if (!enable) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.err   = 1'b1;
      end else if (mem_done_q) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.rdata = rdata_q;
      end
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {8'd0, tcem_cfg_q};
      8'h08: csr_rsp_o.rdata = '0;
      8'h0C: csr_rsp_o.rdata = {29'd0, tcem_flag_q, nv_done_q, (state_q != ST_IDLE)};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  assign irq_o = nv_done_q;

endmodule : nvpsram_ctrl_core
