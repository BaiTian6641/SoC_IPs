// SPDX-License-Identifier: Apache-2.0
// ext_mem_ctrl_core.sv - Unified SDR SDRAM, Async SRAM, and Non-Volatile nvSRAM Controller
//
// Modes (`CTRL[1:0]`):
//   2'd0 : Synchronous 16-bit SDR SDRAM (Auto-Init: Precharge-All -> 2x Refresh -> Load Mode Reg;
//          Periodic tREFI Auto-Refresh; Row Activate -> 2-beat 16-bit Column Read/Write -> Precharge)
//   2'd1 : Asynchronous 16-bit SRAM (2x 16-bit halfword beats per 32-bit bus word with programmable tAA/tWP)
//   2'd2 : Non-Volatile SRAM (nvSRAM, e.g. Infineon/Cypress STK14C88 / CY14B101):
//          Monitors `nvsram_hsb_ni` (Hardware Store Busy), supports hardware pin trigger `nvsram_hsb_oe_o`
//          AND 6-address Software STORE (`0x4E38,0xB1C7,0x83E0,0x7C1F,0x703F,0x8FC0`) /
//          Software RECALL (`0x4E38,0xB1C7,0x83E0,0x7C1F,0x703F,0x4C63`) sequence generator!
//
// CSR Register Map:
//   0x00 CTRL       : [1:0]=mem_type(0:SDRAM, 1:SRAM, 2:nvSRAM), [2]=enable, [5:4]=cas_lat(2 or 3)
//   0x04 SDRAM_TIM  : [3:0]=tRP, [7:4]=tRCD, [11:8]=tRFC, [27:12]=tREFI_cycles
//   0x08 SRAM_TIM   : [7:0]=setup_cycles, [15:8]=access_wait_cycles, [23:16]=hold_cycles
//   0x0C NVSRAM_CMD : [0]=trig_sw_store(WO), [1]=trig_sw_recall(WO), [2]=trig_hw_hsb_pulse(WO)
//   0x10 STATUS     : [0]=init_done, [1]=busy, [2]=nvsram_hsb_busy, [3]=nv_op_done(W1C)

module ext_mem_ctrl_core
  import bus_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,

  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  input  csr_req_t    mem_win_req_i,
  output csr_rsp_t    mem_win_rsp_o,

  // SDR SDRAM Pins
  output logic        sdram_clk_o,
  output logic        sdram_cke_o,
  output logic        sdram_cs_no,
  output logic        sdram_ras_no,
  output logic        sdram_cas_no,
  output logic        sdram_we_no,
  output logic [1:0]  sdram_ba_o,
  output logic [12:0] sdram_addr_o,
  output logic [1:0]  sdram_dqm_o,

  // Async SRAM / nvSRAM Pins (Shared 16-bit DQ bus)
  output logic        sram_ce_no,
  output logic        sram_oe_no,
  output logic        sram_we_no,
  output logic [1:0]  sram_be_no,
  output logic [21:0] sram_addr_o,
  input  logic        nvsram_hsb_ni,
  output logic        nvsram_hsb_oe_o, // Pulls HSB# low externally to initiate hardware STORE

  // Shared 16-bit Bidirectional Data Bus
  input  logic [15:0] mem_dq_i,
  output logic [15:0] mem_dq_o,
  output logic        mem_dq_oe_o,

  output logic        irq_o
);

  // SDRAM Command Encodings {CS#, RAS#, CAS#, WE#}
  localparam logic [3:0] CMD_NOP       = 4'b0111;
  localparam logic [3:0] CMD_ACTIVE    = 4'b0011;
  localparam logic [3:0] CMD_READ      = 4'b0101;
  localparam logic [3:0] CMD_WRITE     = 4'b0100;
  localparam logic [3:0] CMD_PRECHARGE = 4'b0010;
  localparam logic [3:0] CMD_REFRESH   = 4'b0001;
  localparam logic [3:0] CMD_LOAD_MR   = 4'b0000;

  logic [31:0] ctrl_q, sdram_tim_q, sram_tim_q;
  logic        sdram_init_done_q, nv_op_done_q;

  wire [1:0] mem_type = ctrl_q[1:0];
  wire       enable   = ctrl_q[2];
  wire [1:0] cas_lat  = (ctrl_q[5:4] == 2'd0) ? 2'd2 : ctrl_q[5:4];

  logic hsb_n_sync;
  edge_detect #(.WIDTH(1), .INIT(1'b1)) u_hsb_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(nvsram_hsb_ni), .sync_o(hsb_n_sync), .rise_o(), .fall_o()
  );

  assign sdram_clk_o = ~clk_i;
  assign sdram_cke_o = enable && (mem_type == 2'd0);

  typedef enum logic [3:0] {
    ST_RESET_INIT,
    ST_INIT_PRE,
    ST_INIT_REF1,
    ST_INIT_REF2,
    ST_INIT_LMR,
    ST_IDLE,
    ST_SDRAM_ACT,
    ST_SDRAM_RW0,
    ST_SDRAM_RW1,
    ST_SDRAM_CAS_WAIT,
    ST_SDRAM_RD_CAP1,
    ST_SDRAM_PRE,
    ST_SDRAM_AREF,
    ST_SRAM_HALF0,
    ST_SRAM_HALF1,
    ST_NVSRAM_SEQ
  } state_e;

  state_e      state_q;
  logic [15:0] wait_cnt_q, refi_cnt_q;
  logic        ref_pend_q;
  logic [3:0]  sd_cmd_q;
  logic [1:0]  sd_ba_q, sd_dqm_q;
  logic [12:0] sd_a_q;

  logic        sr_ce_nq, sr_oe_nq, sr_we_nq, hsb_drv_q;
  logic [1:0]  sr_be_nq;
  logic [21:0] sr_addr_q;

  logic [15:0] dq_out_q;
  logic        dq_oe_q;
  logic        is_wr_q;
  logic [31:0] req_addr_q, req_wdata_q, rdata_acc_q;
  logic [3:0]  req_wstrb_q;
  logic        mem_done_q;

  // nvSRAM 6-step magic read address table
  logic        nv_is_recall_q;
  logic [2:0]  nv_step_q;
  function automatic logic [15:0] nv_magic_addr(input logic [2:0] step, input logic is_recall);
    unique case (step)
      3'd0: return 16'h4E38;
      3'd1: return 16'hB1C7;
      3'd2: return 16'h83E0;
      3'd3: return 16'h7C1F;
      3'd4: return 16'h703F;
      default: return is_recall ? 16'h4C63 : 16'h8FC0;
    endcase
  endfunction

  assign {sdram_cs_no, sdram_ras_no, sdram_cas_no, sdram_we_no} = sd_cmd_q;
  assign sdram_ba_o      = sd_ba_q;
  assign sdram_addr_o    = sd_a_q;
  assign sdram_dqm_o     = sd_dqm_q;

  assign sram_ce_no      = sr_ce_nq;
  assign sram_oe_no      = sr_oe_nq;
  assign sram_we_no      = sr_we_nq;
  assign sram_be_no      = sr_be_nq;
  assign sram_addr_o     = sr_addr_q;
  assign nvsram_hsb_oe_o = hsb_drv_q;

  assign mem_dq_o        = dq_out_q;
  assign mem_dq_oe_o     = dq_oe_q;

  wire [7:0] reg_off = csr_req_i.addr[7:0];
  wire nv_cmd_wr = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h0C) && (state_q == ST_IDLE);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q            <= 32'h0000_0024; // Enabled, SDRAM mode, CAS=2
      sdram_tim_q       <= {4'd0, 16'd780, 4'd7, 4'd2, 4'd2}; // tREFI=780, tRFC=7, tRCD=2, tRP=2
      sram_tim_q        <= 32'h0001_0201;
      sdram_init_done_q <= 1'b0;
      nv_op_done_q      <= 1'b0;
      state_q           <= ST_RESET_INIT;
      wait_cnt_q        <= 16'd16; // Short power-up pause (scalable in CSR/sim)
      refi_cnt_q        <= '0;
      ref_pend_q        <= 1'b0;
      sd_cmd_q          <= CMD_NOP;
      sd_ba_q           <= '0;
      sd_a_q            <= '0;
      sd_dqm_q          <= 2'b11;
      sr_ce_nq          <= 1'b1;
      sr_oe_nq          <= 1'b1;
      sr_we_nq          <= 1'b1;
      sr_be_nq          <= 2'b11;
      sr_addr_q         <= '0;
      hsb_drv_q         <= 1'b0;
      dq_out_q          <= '0;
      dq_oe_q           <= 1'b0;
      is_wr_q           <= 1'b0;
      req_addr_q        <= '0;
      req_wdata_q       <= '0;
      rdata_acc_q       <= '0;
      req_wstrb_q       <= '0;
      mem_done_q        <= 1'b0;
      nv_is_recall_q    <= 1'b0;
      nv_step_q         <= '0;
    end else begin
      sd_cmd_q   <= CMD_NOP;
      mem_done_q <= 1'b0;

      // Periodic Refresh Timer for SDRAM
      if (enable && mem_type == 2'd0 && sdram_init_done_q) begin
        if (refi_cnt_q >= sdram_tim_q[27:12]) begin
          refi_cnt_q <= '0;
          ref_pend_q <= 1'b1;
        end else begin
          refi_cnt_q <= refi_cnt_q + 16'd1;
        end
      end

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: begin
            ctrl_q <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wdata[1:0] != 2'd0) begin
              sdram_init_done_q <= 1'b1;
              state_q           <= ST_IDLE;
            end
          end
          8'h04: sdram_tim_q <= csr_req_i.wdata;
          8'h08: sram_tim_q  <= csr_req_i.wdata;
          8'h10: if (csr_req_i.wdata[3]) nv_op_done_q <= 1'b0;
          default: ;
        endcase
      end

      unique case (state_q)
        ST_RESET_INIT: begin
          if (mem_type != 2'd0) begin
            sdram_init_done_q <= 1'b1;
            state_q           <= ST_IDLE;
          end else if (wait_cnt_q == 16'd0) begin
            sd_cmd_q   <= CMD_PRECHARGE;
            sd_a_q     <= 13'h0400; // A10=1 Precharge All
            wait_cnt_q <= {12'd0, sdram_tim_q[3:0]};
            state_q    <= ST_INIT_PRE;
          end else begin
            wait_cnt_q <= wait_cnt_q - 16'd1;
          end
        end

        ST_INIT_PRE: begin
          if (wait_cnt_q == 16'd0) begin
            sd_cmd_q   <= CMD_REFRESH;
            wait_cnt_q <= {12'd0, sdram_tim_q[11:8]};
            state_q    <= ST_INIT_REF1;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_INIT_REF1: begin
          if (wait_cnt_q == 16'd0) begin
            sd_cmd_q   <= CMD_REFRESH;
            wait_cnt_q <= {12'd0, sdram_tim_q[11:8]};
            state_q    <= ST_INIT_REF2;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_INIT_REF2: begin
          if (wait_cnt_q == 16'd0) begin
            sd_cmd_q   <= CMD_LOAD_MR;
            sd_ba_q    <= 2'b00;
            // Burst Length = 2 (32-bit word across 16-bit SDRAM), Sequential, CAS Latency
            sd_a_q     <= {3'b000, 1'b0, 2'b00, {1'b0, cas_lat}, 1'b0, 3'b001};
            wait_cnt_q <= 16'd2;
            state_q    <= ST_INIT_LMR;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_INIT_LMR: begin
          if (wait_cnt_q == 16'd0) begin
            sdram_init_done_q <= 1'b1;
            state_q           <= ST_IDLE;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_IDLE: begin
          dq_oe_q   <= 1'b0;
          sd_dqm_q  <= 2'b11;
          sr_ce_nq  <= 1'b1;
          sr_oe_nq  <= 1'b1;
          sr_we_nq  <= 1'b1;
          hsb_drv_q <= 1'b0;

          if (enable && mem_type == 2'd0 && ref_pend_q) begin
            ref_pend_q <= 1'b0;
            sd_cmd_q   <= CMD_REFRESH;
            wait_cnt_q <= {12'd0, sdram_tim_q[11:8]};
            state_q    <= ST_SDRAM_AREF;
          end else if (enable && mem_type == 2'd2 && nv_cmd_wr) begin
            if (csr_req_i.wdata[2]) begin
              hsb_drv_q    <= 1'b1;
              wait_cnt_q   <= 16'd8;
              nv_step_q    <= 3'd7;
              state_q      <= ST_NVSRAM_SEQ;
            end else if (csr_req_i.wdata[0] || csr_req_i.wdata[1]) begin
              nv_is_recall_q <= csr_req_i.wdata[1];
              nv_step_q      <= 3'd0;
              sr_ce_nq       <= 1'b0;
              sr_oe_nq       <= 1'b0;
              sr_we_nq       <= 1'b1;
              sr_be_nq       <= 2'b00;
              sr_addr_q      <= {6'd0, nv_magic_addr(3'd0, csr_req_i.wdata[1])};
              wait_cnt_q     <= {8'd0, sram_tim_q[15:8]};
              state_q        <= ST_NVSRAM_SEQ;
            end
          end else if (enable && mem_win_req_i.valid && !mem_done_q && (mem_type != 2'd2 || hsb_n_sync)) begin
            is_wr_q     <= mem_win_req_i.write;
            req_addr_q  <= mem_win_req_i.addr;
            req_wdata_q <= mem_win_req_i.wdata;
            req_wstrb_q <= mem_win_req_i.wstrb;
            if (mem_type == 2'd0) begin
              // SDRAM Row Activate
              sd_cmd_q   <= CMD_ACTIVE;
              sd_ba_q    <= mem_win_req_i.addr[24:23];
              sd_a_q     <= mem_win_req_i.addr[22:10];
              wait_cnt_q <= {12'd0, sdram_tim_q[7:4]};
              state_q    <= ST_SDRAM_ACT;
            end else begin
              // Async SRAM / nvSRAM Halfword 0
              sr_ce_nq   <= 1'b0;
              sr_oe_nq   <= mem_win_req_i.write;
              sr_we_nq   <= !mem_win_req_i.write;
              sr_be_nq   <= mem_win_req_i.write ? ~mem_win_req_i.wstrb[1:0] : 2'b00;
              sr_addr_q  <= {mem_win_req_i.addr[22:2], 1'b0};
              dq_out_q   <= mem_win_req_i.wdata[15:0];
              dq_oe_q    <= mem_win_req_i.write;
              wait_cnt_q <= {8'd0, sram_tim_q[15:8]};
              state_q    <= ST_SRAM_HALF0;
            end
          end
        end

        ST_SDRAM_AREF: begin
          if (wait_cnt_q == 16'd0) state_q <= ST_IDLE;
          else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_SDRAM_ACT: begin
          if (wait_cnt_q == 16'd0) begin
            sd_cmd_q <= is_wr_q ? CMD_WRITE : CMD_READ;
            sd_ba_q  <= req_addr_q[24:23];
            sd_a_q   <= {3'b000, 1'b0, req_addr_q[9:2], 1'b0}; // Column aligned to burst of 2
            sd_dqm_q <= is_wr_q ? ~req_wstrb_q[1:0] : 2'b00;
            dq_out_q <= req_wdata_q[15:0];
            dq_oe_q  <= is_wr_q;
            state_q  <= ST_SDRAM_RW1;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_SDRAM_RW1: begin
          if (is_wr_q) begin
            sd_dqm_q   <= ~req_wstrb_q[3:2];
            dq_out_q   <= req_wdata_q[31:16];
            dq_oe_q    <= 1'b1;
            wait_cnt_q <= 16'd2;
            state_q    <= ST_SDRAM_PRE;
          end else begin
            sd_dqm_q   <= 2'b00;
            wait_cnt_q <= {14'd0, cas_lat} - 16'd1;
            state_q    <= ST_SDRAM_CAS_WAIT;
          end
        end

        ST_SDRAM_CAS_WAIT: begin
          if (wait_cnt_q == 16'd0) begin
            rdata_acc_q[15:0] <= mem_dq_i;
            state_q           <= ST_SDRAM_RD_CAP1;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_SDRAM_RD_CAP1: begin
          rdata_acc_q[31:16] <= mem_dq_i;
          wait_cnt_q         <= 16'd0;
          state_q            <= ST_SDRAM_PRE;
        end

        ST_SDRAM_PRE: begin
          dq_oe_q <= 1'b0;
          if (wait_cnt_q == 16'd0) begin
            sd_cmd_q   <= CMD_PRECHARGE;
            sd_a_q     <= 13'h0400;
            mem_done_q <= 1'b1;
            state_q    <= ST_IDLE;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_SRAM_HALF0: begin
          if (wait_cnt_q == 16'd0) begin
            if (!is_wr_q) rdata_acc_q[15:0] <= mem_dq_i;
            sr_addr_q  <= {req_addr_q[22:2], 1'b1};
            sr_be_nq   <= is_wr_q ? ~req_wstrb_q[3:2] : 2'b00;
            dq_out_q   <= req_wdata_q[31:16];
            wait_cnt_q <= {8'd0, sram_tim_q[15:8]};
            state_q    <= ST_SRAM_HALF1;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_SRAM_HALF1: begin
          if (wait_cnt_q == 16'd0) begin
            if (!is_wr_q) rdata_acc_q[31:16] <= mem_dq_i;
            sr_ce_nq   <= 1'b1;
            sr_oe_nq   <= 1'b1;
            sr_we_nq   <= 1'b1;
            dq_oe_q    <= 1'b0;
            mem_done_q <= 1'b1;
            state_q    <= ST_IDLE;
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
        end

        ST_NVSRAM_SEQ: begin
          if (wait_cnt_q == 16'd0) begin
            if (nv_step_q == 3'd7 || nv_step_q == 3'd5) begin
              hsb_drv_q    <= 1'b0;
              sr_ce_nq     <= 1'b1;
              sr_oe_nq     <= 1'b1;
              nv_op_done_q <= 1'b1;
              state_q      <= ST_IDLE;
            end else begin
              logic [2:0] nxt;
              nxt        = nv_step_q + 3'd1;
              nv_step_q  <= nxt;
              sr_addr_q  <= {6'd0, nv_magic_addr(nxt, nv_is_recall_q)};
              wait_cnt_q <= {8'd0, sram_tim_q[15:8]};
            end
          end else wait_cnt_q <= wait_cnt_q - 16'd1;
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
        mem_win_rsp_o.rdata = rdata_acc_q;
      end
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = sdram_tim_q;
      8'h08: csr_rsp_o.rdata = sram_tim_q;
      8'h0C: csr_rsp_o.rdata = '0;
      8'h10: csr_rsp_o.rdata = {28'd0, nv_op_done_q, !hsb_n_sync, (state_q != ST_IDLE), sdram_init_done_q};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  assign irq_o = nv_op_done_q;

endmodule : ext_mem_ctrl_core
