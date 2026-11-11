// SPDX-License-Identifier: Apache-2.0
// gdma_core.sv - Multi-Channel General-Purpose Scatter-Gather / Linked-List DMA (GDMA)
//
// Features:
//   - Parameterized `NUM_CH` independent channels (default 4) with QoS Round-Robin Arbitration
//   - Transfer Modes: Memory-to-Memory (M2M), Memory-to-Peripheral (M2P), Peripheral-to-Memory (P2M)
//   - Hardware Peripheral Flow Control (`periph_req_i[NUM_PERIPH-1:0]` / `periph_ack_o[NUM_PERIPH-1:0]`)
//   - Linked-List / Scatter-Gather Descriptor Chaining in System RAM (4-word 16-byte descriptors:
//     Word0=SRC_ADDR, Word1=DST_ADDR, Word2=BYTE_LEN, Word3=NEXT_DESC_PTR [bit0=chain_en, 31:4=ptr])
//   - Independent Source & Destination Increment vs Fixed-Address modes + 1B/2B/4B transfer sizes
//
// CSR Register Map:
//   0x00 GLB_CTRL   : [0]=global_enable
//   0x04 INT_EN     : [NUM_CH-1:0]=ch_done_ie, [8 +: NUM_CH]=ch_err_ie
//   0x08 INT_STAT   : W1C [NUM_CH-1:0]=ch_done, [8 +: NUM_CH]=ch_err
//   Per-Channel Registers (Base = 0x20 + ch * 0x20):
//     +0x00 CH_CTRL : [0]=ch_en(RW, auto-clears on completion), [1]=src_inc, [2]=dst_inc,
//                     [4:3]=xfer_size(0:1B,1:2B,2:4B), [5]=periph_flow_en, [9:6]=periph_sel,
//                     [10]=sg_desc_en, [15:12]=qos
//     +0x04 CH_SRC  : [31:0] Source address
//     +0x08 CH_DST  : [31:0] Destination address
//     +0x0C CH_LEN  : [23:0] Remaining transfer byte length
//     +0x10 CH_DESC : [31:0] Linked-list next descriptor address ([0]=valid_link)

module gdma_core
  import bus_pkg::*;
#(
  parameter int unsigned NUM_CH     = 4,
  parameter int unsigned NUM_PERIPH = 16
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  input  csr_req_t              csr_req_i,
  output csr_rsp_t              csr_rsp_o,

  output mem_req_t              dma_req_o,
  input  mem_rsp_t              dma_rsp_i,

  input  logic [NUM_PERIPH-1:0] periph_req_i,
  output logic [NUM_PERIPH-1:0] periph_ack_o,

  output logic                  irq_o
);

  localparam int unsigned CH_W = (NUM_CH <= 1) ? 1 : $clog2(NUM_CH);

  logic                  glb_en_q;
  logic [NUM_CH-1:0]     done_ie_q, err_ie_q, done_st_q, err_st_q;

  logic [NUM_CH-1:0][31:0] ch_ctrl_q, ch_src_q, ch_dst_q, ch_desc_q;
  logic [NUM_CH-1:0][23:0] ch_len_q;

  // Channel eligibility (enabled and either M2M or selected peripheral asserts request)
  logic [NUM_CH-1:0]       ch_req, ch_gnt;
  logic [NUM_CH-1:0][3:0]  ch_qos;
  logic [CH_W-1:0]         arb_ch;
  logic                    arb_valid;

  for (genvar c = 0; c < NUM_CH; c++) begin : gen_ch_elig
    wire p_en  = ch_ctrl_q[c][5];
    wire [3:0] psel = ch_ctrl_q[c][9:6];
    wire p_rdy = (int'(psel) < NUM_PERIPH) ? periph_req_i[psel] : 1'b1;
    assign ch_req[c] = glb_en_q && ch_ctrl_q[c][0] && (!p_en || p_rdy);
    assign ch_qos[c] = ch_ctrl_q[c][15:12];
  end

  typedef enum logic [2:0] {
    DMA_IDLE,
    DMA_SG_RD_REQ,
    DMA_SG_RD_WAIT,
    DMA_SRC_RD_REQ,
    DMA_SRC_RD_WAIT,
    DMA_DST_WR_REQ,
    DMA_DST_WR_WAIT
  } dma_st_e;

  dma_st_e     state_q;
  logic [CH_W-1:0] act_ch_q;
  logic [1:0]  sg_word_idx_q;
  logic [31:0] xfer_data_q;

  rr_arbiter #(.N(NUM_CH)) u_ch_arb (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .req_i   (ch_req),
    .qos_i   (ch_qos),
    .lock_i  (state_q != DMA_IDLE),
    .ack_i   (state_q == DMA_DST_WR_WAIT && dma_rsp_i.resp_valid),
    .gnt_o   (ch_gnt),
    .idx_o   (arb_ch),
    .valid_o (arb_valid)
  );

  wire [1:0]  act_sz_cfg = ch_ctrl_q[act_ch_q][4:3];
  wire [31:0] step_bytes = (act_sz_cfg == 2'd0) ? 32'd1 : ((act_sz_cfg == 2'd1) ? 32'd2 : 32'd4);

  // Byte-lane steering for 1B / 2B / 4B unaligned transfers
  function automatic logic [3:0] calc_wstrb(input logic [1:0] sz, input logic [1:0] low_adr);
    unique case (sz)
      2'd0:    return (4'b0001 << low_adr);
      2'd1:    return low_adr[1] ? 4'b1100 : 4'b0011;
      default: return 4'b1111;
    endcase
  endfunction

  function automatic logic [31:0] steer_data(
    input logic [31:0] raw,
    input logic [1:0]  sz,
    input logic [1:0]  s_off,
    input logic [1:0]  d_off
  );
    if (sz == 2'd0) begin
      logic [7:0] b;
      b = raw[s_off*8 +: 8];
      return {4{b}};
    end else if (sz == 2'd1) begin
      logic [15:0] h;
      h = s_off[1] ? raw[31:16] : raw[15:0];
      return {2{h}};
    end else begin
      return raw;
    end
  endfunction

  always_comb begin
    dma_req_o            = '0;
    dma_req_o.size       = 3'd2;
    dma_req_o.last       = 1'b1;
    dma_req_o.id         = 4'(act_ch_q);
    dma_req_o.qos        = ch_ctrl_q[act_ch_q][15:12];
    dma_req_o.resp_ready = 1'b1;
    periph_ack_o         = '0;

    unique case (state_q)
      DMA_SG_RD_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = {ch_desc_q[act_ch_q][31:4], 4'b0000} + {28'd0, sg_word_idx_q, 2'b00};
      end

      DMA_SRC_RD_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b0;
        dma_req_o.addr  = {ch_src_q[act_ch_q][31:2], 2'b00};
      end

      DMA_DST_WR_REQ: begin
        dma_req_o.valid = 1'b1;
        dma_req_o.write = 1'b1;
        dma_req_o.addr  = {ch_dst_q[act_ch_q][31:2], 2'b00};
        dma_req_o.wdata = steer_data(xfer_data_q, act_sz_cfg, ch_src_q[act_ch_q][1:0], ch_dst_q[act_ch_q][1:0]);
        dma_req_o.wstrb = calc_wstrb(act_sz_cfg, ch_dst_q[act_ch_q][1:0]);
      end

      DMA_DST_WR_WAIT: begin
        if (dma_rsp_i.resp_valid && ch_ctrl_q[act_ch_q][5]) begin
          logic [3:0] ps;
          ps = ch_ctrl_q[act_ch_q][9:6];
          if (int'(ps) < NUM_PERIPH) periph_ack_o[ps] = 1'b1;
        end
      end

      default: ;
    endcase
  end

  wire [7:0] reg_off = csr_req_i.addr[7:0];

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      glb_en_q      <= 1'b0;
      done_ie_q     <= '0;
      err_ie_q      <= '0;
      done_st_q     <= '0;
      err_st_q      <= '0;
      ch_ctrl_q     <= '0;
      ch_src_q      <= '0;
      ch_dst_q      <= '0;
      ch_len_q      <= '0;
      ch_desc_q     <= '0;
      state_q       <= DMA_IDLE;
      act_ch_q      <= '0;
      sg_word_idx_q <= '0;
      xfer_data_q   <= '0;
    end else begin
      // CSR Writes
      if (csr_req_i.valid && csr_req_i.write) begin
        if (reg_off == 8'h00) begin
          glb_en_q <= csr_req_i.wdata[0];
        end else if (reg_off == 8'h04) begin
          done_ie_q <= csr_req_i.wdata[NUM_CH-1:0];
          err_ie_q  <= csr_req_i.wdata[8 +: NUM_CH];
        end else if (reg_off == 8'h08) begin
          done_st_q <= done_st_q & ~csr_req_i.wdata[NUM_CH-1:0];
          err_st_q  <= err_st_q  & ~csr_req_i.wdata[8 +: NUM_CH];
        end else if (reg_off >= 8'h20) begin
          int c_idx, c_sub;
          c_idx = int'((reg_off - 8'h20) >> 5);
          c_sub = int'((reg_off - 8'h20) & 8'h1F);
          if (c_idx < NUM_CH) begin
            unique case (c_sub)
              8'h00: ch_ctrl_q[c_idx] <= csr_req_i.wdata;
              8'h04: ch_src_q[c_idx]  <= csr_req_i.wdata;
              8'h08: ch_dst_q[c_idx]  <= csr_req_i.wdata;
              8'h0C: ch_len_q[c_idx]  <= csr_req_i.wdata[23:0];
              8'h10: ch_desc_q[c_idx] <= csr_req_i.wdata;
              default: ;
            endcase
          end
        end
      end

      // GDMA Transfer & Scatter-Gather FSM
      unique case (state_q)
        DMA_IDLE: begin
          if (arb_valid) begin
            act_ch_q <= arb_ch;
            if (ch_len_q[arb_ch] == 24'd0) begin
              if (ch_ctrl_q[arb_ch][10] && ch_desc_q[arb_ch][0]) begin
                sg_word_idx_q <= 2'd0;
                state_q       <= DMA_SG_RD_REQ;
              end else begin
                ch_ctrl_q[arb_ch][0] <= 1'b0;
                done_st_q[arb_ch]    <= 1'b1;
              end
            end else begin
              state_q <= DMA_SRC_RD_REQ;
            end
          end
        end

        DMA_SG_RD_REQ: begin
          if (dma_rsp_i.req_ready) state_q <= DMA_SG_RD_WAIT;
        end

        DMA_SG_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin
              err_st_q[act_ch_q]     <= 1'b1;
              ch_ctrl_q[act_ch_q][0] <= 1'b0;
              state_q                <= DMA_IDLE;
            end else begin
              unique case (sg_word_idx_q)
                2'd0: begin ch_src_q[act_ch_q]  <= dma_rsp_i.rdata; sg_word_idx_q <= 2'd1; state_q <= DMA_SG_RD_REQ; end
                2'd1: begin ch_dst_q[act_ch_q]  <= dma_rsp_i.rdata; sg_word_idx_q <= 2'd2; state_q <= DMA_SG_RD_REQ; end
                2'd2: begin ch_len_q[act_ch_q]  <= dma_rsp_i.rdata[23:0]; sg_word_idx_q <= 2'd3; state_q <= DMA_SG_RD_REQ; end
                2'd3: begin
                  ch_desc_q[act_ch_q] <= dma_rsp_i.rdata;
                  state_q             <= DMA_SRC_RD_REQ;
                end
              endcase
            end
          end
        end

        DMA_SRC_RD_REQ: begin
          if (dma_rsp_i.req_ready) state_q <= DMA_SRC_RD_WAIT;
        end

        DMA_SRC_RD_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin
              err_st_q[act_ch_q]     <= 1'b1;
              ch_ctrl_q[act_ch_q][0] <= 1'b0;
              state_q                <= DMA_IDLE;
            end else begin
              xfer_data_q <= dma_rsp_i.rdata;
              state_q     <= DMA_DST_WR_REQ;
            end
          end
        end

        DMA_DST_WR_REQ: begin
          if (dma_rsp_i.req_ready) state_q <= DMA_DST_WR_WAIT;
        end

        DMA_DST_WR_WAIT: begin
          if (dma_rsp_i.resp_valid) begin
            if (dma_rsp_i.err) begin
              err_st_q[act_ch_q]     <= 1'b1;
              ch_ctrl_q[act_ch_q][0] <= 1'b0;
              state_q                <= DMA_IDLE;
            end else begin
              if (ch_ctrl_q[act_ch_q][1]) ch_src_q[act_ch_q] <= ch_src_q[act_ch_q] + step_bytes;
              if (ch_ctrl_q[act_ch_q][2]) ch_dst_q[act_ch_q] <= ch_dst_q[act_ch_q] + step_bytes;
              if (ch_len_q[act_ch_q] <= step_bytes[23:0]) begin
                ch_len_q[act_ch_q] <= 24'd0;
                if (ch_ctrl_q[act_ch_q][10] && ch_desc_q[act_ch_q][0]) begin
                  // Automatically chain to next scatter-gather descriptor!
                  sg_word_idx_q <= 2'd0;
                  state_q       <= DMA_SG_RD_REQ;
                end else begin
                  ch_ctrl_q[act_ch_q][0] <= 1'b0;
                  done_st_q[act_ch_q]    <= 1'b1;
                  state_q                <= DMA_IDLE;
                end
              end else begin
                ch_len_q[act_ch_q] <= ch_len_q[act_ch_q] - step_bytes[23:0];
                state_q            <= DMA_IDLE;
              end
            end
          end
        end

        default: state_q <= DMA_IDLE;
      endcase
    end
  end

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    if (reg_off == 8'h00) begin
      csr_rsp_o.rdata = {31'd0, glb_en_q};
    end else if (reg_off == 8'h04) begin
      csr_rsp_o.rdata = {16'd0, 8'(err_ie_q), 8'(done_ie_q)};
    end else if (reg_off == 8'h08) begin
      csr_rsp_o.rdata = {16'd0, 8'(err_st_q), 8'(done_st_q)};
    end else if (reg_off >= 8'h20) begin
      int c_idx, c_sub;
      c_idx = int'((reg_off - 8'h20) >> 5);
      c_sub = int'((reg_off - 8'h20) & 8'h1F);
      if (c_idx < NUM_CH) begin
        unique case (c_sub)
          8'h00: csr_rsp_o.rdata = ch_ctrl_q[c_idx];
          8'h04: csr_rsp_o.rdata = ch_src_q[c_idx];
          8'h08: csr_rsp_o.rdata = ch_dst_q[c_idx];
          8'h0C: csr_rsp_o.rdata = {8'd0, ch_len_q[c_idx]};
          8'h10: csr_rsp_o.rdata = ch_desc_q[c_idx];
          default: csr_rsp_o.err = csr_req_i.valid;
        endcase
      end else begin
        csr_rsp_o.err = csr_req_i.valid;
      end
    end else begin
      csr_rsp_o.err = csr_req_i.valid;
    end
  end

  assign irq_o = |((done_ie_q & done_st_q) | (err_ie_q & err_st_q));

endmodule : gdma_core
