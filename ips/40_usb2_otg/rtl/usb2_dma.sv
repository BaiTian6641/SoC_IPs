// SPDX-License-Identifier: Apache-2.0
// usb2_dma.sv - 4KB-Safe Burst DMA Master for USB 2.0 Packet SRAM <-> System RAM
//
// Interfaces to Wishbone B4, AXI4, and TileLink TL-UH via `bus_pkg::mem_req_t` /
// `mem_rsp_t`. Supports direct linear transfers and 2-word scatter-gather
// descriptor lists (`VALID`, `END`, `INT`, `LINK`), partial trailing-word byte
// strobes, and strict 4 KB burst boundary clamping.

module usb2_dma
  import bus_pkg::*;
#(
  parameter int unsigned NUM_EPS       = 8,
  parameter int unsigned MAX_PKT_BYTES = 512,
  localparam int unsigned EP_W         = (NUM_EPS <= 1) ? 1 : $clog2(NUM_EPS),
  localparam int unsigned WORDS_PER_BK = MAX_PKT_BYTES / 4,
  localparam int unsigned BK_OFF_W     = (WORDS_PER_BK <= 1) ? 1 : $clog2(WORDS_PER_BK),
  localparam int unsigned SRAM_ADDR_W  = EP_W + 1 + BK_OFF_W
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,

  // Control from CSR
  input  logic                    start_i,
  input  logic                    abort_i,
  input  logic                    dir_tx_i,      // 1: RAM -> USB TX SRAM, 0: USB RX SRAM -> RAM
  input  logic                    auto_commit_i,
  input  logic                    desc_en_i,
  input  logic [EP_W-1:0]         ep_i,
  input  logic                    bank_i,
  input  logic [3:0]              burst_i,       // max beats - 1 (0..15)
  input  logic [31:0]             addr_i,
  input  logic [11:0]             byte_len_i,

  // TX Packet SRAM Write Port (clk_i domain)
  output logic                    tx_sram_wen_o,
  output logic [3:0]              tx_sram_wstrb_o,
  output logic [SRAM_ADDR_W-1:0]  tx_sram_waddr_o,
  output logic [31:0]             tx_sram_wdata_o,

  // RX Packet SRAM Read Port (clk_i domain)
  output logic                    rx_sram_ren_o,
  output logic [SRAM_ADDR_W-1:0]  rx_sram_raddr_o,
  input  logic [31:0]             rx_sram_rdata_i,

  // Generic Burst Memory Master Port
  output mem_req_t                mem_req_o,
  input  mem_rsp_t                mem_rsp_i,

  // Status & Auto-Commit Outputs
  output logic                    busy_o,
  output logic                    done_o,
  output logic                    seg_irq_o,
  output logic                    err_o,
  output logic [31:0]             cur_addr_o,
  output logic                    auto_commit_pulse_o,
  output logic                    auto_commit_dir_tx_o,
  output logic [EP_W-1:0]         auto_commit_ep_o,
  output logic                    auto_commit_bank_o,
  output logic [10:0]             auto_commit_len_o
);

  typedef enum logic [3:0] {
    D_IDLE,
    D_DESC_REQ,
    D_DESC_WAIT,
    D_PLAN,
    D_FILL_REQ,
    D_FILL_WAIT,
    D_WR_BURST,
    D_WR_RESP,
    D_RD_REQ,
    D_RD_DATA
  } dma_st_e;

  dma_st_e             st_q;
  logic                dir_tx_q, auto_commit_q, desc_en_q, abort_q;
  logic                int_q, end_q, err_acc_q;
  logic [EP_W-1:0]     ep_q;
  logic                bank_q;
  logic [3:0]          burst_q;
  logic [31:0]         ptr_q, seg_addr_q, d0_q;
  logic [15:0]         seg_bytes_q;
  logic [13:0]         seg_words_rem_q;
  logic [3:0]          last_strb_q;
  logic [BK_OFF_W-1:0] sram_widx_q;
  logic [4:0]          bw_q;
  logic [4:0]          beat_q;
  logic                dbeat_q;

  // 16-word local burst staging buffer for RX SRAM -> System RAM writes
  logic [31:0] stage_q [0:15];

  // 4 KB boundary safe burst sizing
  logic [10:0] to_4k;
  logic [4:0]  bw_c;
  always_comb begin
    automatic logic [4:0] max_bw = 5'(burst_q) + 5'd1;
    to_4k = 11'(32'd1024 - {20'd0, seg_addr_q[11:2]});
    bw_c  = max_bw;
    if ({18'd0, seg_words_rem_q} < {27'd0, bw_c}) bw_c = seg_words_rem_q[4:0];
    if (to_4k < {6'd0, bw_c})                     bw_c = to_4k[4:0];
  end

  function automatic logic [3:0] calc_last_strb(input logic [1:0] rem);
    unique case (rem)
      2'd1:    return 4'b0001;
      2'd2:    return 4'b0011;
      2'd3:    return 4'b0111;
      default: return 4'b1111;
    endcase
  endfunction

  assign busy_o               = (st_q != D_IDLE);
  assign cur_addr_o           = seg_addr_q;
  assign auto_commit_dir_tx_o = dir_tx_q;
  assign auto_commit_ep_o     = ep_q;
  assign auto_commit_bank_o   = bank_q;
  assign auto_commit_len_o    = seg_bytes_q[10:0];

  wire is_final_beat = ({9'd0, bw_q} == seg_words_rem_q) && (beat_q == (bw_q - 5'd1));

  always_comb begin
    mem_req_o            = '0;
    mem_req_o.size       = 3'd2;
    mem_req_o.resp_ready = 1'b1;

    tx_sram_wen_o   = 1'b0;
    tx_sram_wstrb_o = 4'hF;
    tx_sram_waddr_o = {ep_q, bank_q, sram_widx_q};
    tx_sram_wdata_o = mem_rsp_i.rdata;

    rx_sram_ren_o   = 1'b0;
    rx_sram_raddr_o = {ep_q, bank_q, sram_widx_q};

    unique case (st_q)
      D_DESC_REQ: begin
        mem_req_o.valid = !abort_q;
        mem_req_o.write = 1'b0;
        mem_req_o.addr  = ptr_q;
        mem_req_o.len   = 8'd1;
      end

      D_FILL_REQ: begin
        rx_sram_ren_o   = 1'b1;
        rx_sram_raddr_o = {ep_q, bank_q, sram_widx_q};
      end

      D_WR_BURST: begin
        mem_req_o.valid = 1'b1;
        mem_req_o.write = 1'b1;
        mem_req_o.addr  = seg_addr_q;
        mem_req_o.len   = 8'(bw_q) - 8'd1;
        mem_req_o.wdata = stage_q[beat_q[3:0]];
        mem_req_o.wstrb = is_final_beat ? last_strb_q : 4'hF;
        mem_req_o.last  = (beat_q == (bw_q - 5'd1));
      end

      D_RD_REQ: begin
        mem_req_o.valid = 1'b1;
        mem_req_o.write = 1'b0;
        mem_req_o.addr  = seg_addr_q;
        mem_req_o.len   = 8'(bw_q) - 8'd1;
      end

      D_RD_DATA: begin
        tx_sram_wen_o = mem_rsp_i.resp_valid;
      end

      default: ;
    endcase
  end

  task automatic finish_burst(input logic had_err);
    logic [13:0] rem_n;
    rem_n = seg_words_rem_q - {9'd0, bw_q};
    seg_addr_q      <= seg_addr_q + {25'd0, bw_q, 2'b00};
    seg_words_rem_q <= rem_n;
    if (had_err) begin
      err_o <= 1'b1;
      st_q  <= D_IDLE;
    end else if (rem_n == 14'd0) begin
      if (int_q) seg_irq_o <= 1'b1;
      if (desc_en_q && !end_q) begin
        st_q <= D_DESC_REQ;
      end else begin
        done_o              <= 1'b1;
        auto_commit_pulse_o <= auto_commit_q;
        st_q                <= D_IDLE;
      end
    end else begin
      st_q <= D_PLAN;
    end
  endtask

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_q                <= D_IDLE;
      dir_tx_q            <= 1'b0;
      auto_commit_q       <= 1'b0;
      desc_en_q           <= 1'b0;
      abort_q             <= 1'b0;
      int_q               <= 1'b0;
      end_q               <= 1'b0;
      err_acc_q           <= 1'b0;
      ep_q                <= '0;
      bank_q              <= 1'b0;
      burst_q             <= '0;
      ptr_q               <= '0;
      seg_addr_q          <= '0;
      d0_q                <= '0;
      seg_bytes_q         <= '0;
      seg_words_rem_q     <= '0;
      last_strb_q         <= 4'hF;
      sram_widx_q         <= '0;
      bw_q                <= '0;
      beat_q              <= '0;
      dbeat_q             <= 1'b0;
      done_o              <= 1'b0;
      seg_irq_o           <= 1'b0;
      err_o               <= 1'b0;
      auto_commit_pulse_o <= 1'b0;
    end else begin
      done_o              <= 1'b0;
      seg_irq_o           <= 1'b0;
      err_o               <= 1'b0;
      auto_commit_pulse_o <= 1'b0;
      if (abort_i && st_q != D_IDLE) abort_q <= 1'b1;

      unique case (st_q)
        D_IDLE: begin
          abort_q <= 1'b0;
          if (start_i) begin
            dir_tx_q      <= dir_tx_i;
            auto_commit_q <= auto_commit_i;
            desc_en_q     <= desc_en_i;
            ep_q          <= ep_i;
            bank_q        <= bank_i;
            burst_q       <= burst_i;
            int_q         <= 1'b0;
            end_q         <= !desc_en_i;
            sram_widx_q   <= '0;
            if (desc_en_i) begin
              ptr_q <= {addr_i[31:2], 2'b00};
              st_q  <= D_DESC_REQ;
            end else begin
              automatic logic [15:0] bl     = {4'd0, byte_len_i};
              automatic logic [15:0] bl_rnd = bl + 16'd3;
              automatic logic [13:0] wc     = bl_rnd[15:2];
              seg_addr_q      <= {addr_i[31:2], 2'b00};
              seg_bytes_q     <= bl;
              seg_words_rem_q <= wc;
              last_strb_q     <= calc_last_strb(byte_len_i[1:0]);
              if (wc == 14'd0) begin
                done_o              <= 1'b1;
                auto_commit_pulse_o <= auto_commit_i;
                st_q                <= D_IDLE;
              end else begin
                st_q <= D_PLAN;
              end
            end
          end
        end

        D_DESC_REQ: begin
          if (abort_q) begin
            st_q <= D_IDLE;
          end else if (mem_rsp_i.req_ready) begin
            dbeat_q   <= 1'b0;
            err_acc_q <= 1'b0;
            st_q      <= D_DESC_WAIT;
          end
        end

        D_DESC_WAIT: begin
          if (mem_rsp_i.resp_valid) begin
            if (mem_rsp_i.err) err_acc_q <= 1'b1;
            if (!dbeat_q) begin
              d0_q    <= mem_rsp_i.rdata;
              dbeat_q <= 1'b1;
            end
            if (mem_rsp_i.last || dbeat_q) begin
              if (err_acc_q || mem_rsp_i.err || !mem_rsp_i.rdata[16] || (d0_q[1:0] != 2'b00)) begin
                err_o <= 1'b1;
                st_q  <= D_IDLE;
              end else if (mem_rsp_i.rdata[19]) begin
                ptr_q <= d0_q;
                st_q  <= D_DESC_REQ;
              end else begin
                automatic logic [15:0] bl     = mem_rsp_i.rdata[15:0];
                automatic logic [15:0] bl_rnd = bl + 16'd3;
                automatic logic [13:0] wc     = bl_rnd[15:2];
                ptr_q           <= ptr_q + 32'd8;
                seg_addr_q      <= d0_q;
                seg_bytes_q     <= bl;
                seg_words_rem_q <= wc;
                last_strb_q     <= calc_last_strb(bl[1:0]);
                end_q           <= mem_rsp_i.rdata[17];
                int_q           <= mem_rsp_i.rdata[18];
                sram_widx_q     <= '0;
                if (wc == 14'd0) begin
                  if (mem_rsp_i.rdata[18]) seg_irq_o <= 1'b1;
                  if (mem_rsp_i.rdata[17]) begin
                    done_o              <= 1'b1;
                    auto_commit_pulse_o <= auto_commit_q;
                    st_q                <= D_IDLE;
                  end else begin
                    st_q <= D_DESC_REQ;
                  end
                end else begin
                  st_q <= D_PLAN;
                end
              end
            end
          end
        end

        D_PLAN: begin
          if (abort_q) begin
            st_q <= D_IDLE;
          end else begin
            bw_q      <= bw_c;
            beat_q    <= 5'd0;
            err_acc_q <= 1'b0;
            st_q      <= dir_tx_q ? D_RD_REQ : D_FILL_REQ;
          end
        end

        D_FILL_REQ: begin
          st_q <= D_FILL_WAIT;
        end

        D_FILL_WAIT: begin
          stage_q[beat_q[3:0]] <= rx_sram_rdata_i;
          sram_widx_q          <= sram_widx_q + BK_OFF_W'(1);
          if (beat_q == (bw_q - 5'd1)) begin
            beat_q <= 5'd0;
            st_q   <= D_WR_BURST;
          end else begin
            beat_q <= beat_q + 5'd1;
            st_q   <= D_FILL_REQ;
          end
        end

        D_WR_BURST: begin
          if (mem_rsp_i.req_ready) begin
            if (beat_q == (bw_q - 5'd1)) begin
              st_q <= D_WR_RESP;
            end else begin
              beat_q <= beat_q + 5'd1;
            end
          end
        end

        D_WR_RESP: begin
          if (mem_rsp_i.resp_valid) begin
            finish_burst(mem_rsp_i.err);
          end
        end

        D_RD_REQ: begin
          if (mem_rsp_i.req_ready) begin
            st_q <= D_RD_DATA;
          end
        end

        D_RD_DATA: begin
          if (mem_rsp_i.resp_valid) begin
            sram_widx_q <= sram_widx_q + BK_OFF_W'(1);
            if (mem_rsp_i.err) err_acc_q <= 1'b1;
            if (mem_rsp_i.last) begin
              finish_burst(err_acc_q || mem_rsp_i.err);
            end
          end
        end

        default: st_q <= D_IDLE;
      endcase
    end
  end

endmodule : usb2_dma
