// SPDX-License-Identifier: Apache-2.0
// dvi_tmds_core.sv - DVI 1.0 / TMDS Display Timing, Scanout Framebuffer DMA & Serializer Core
//
// Features:
//   - Programmable Horizontal & Vertical Timing (640x480p60 default, supports 720p/1080p/custom)
//   - Built-in Color-Bar Test Pattern Generator (`CTRL[1]=1`) OR Live Framebuffer Scanout DMA (`CTRL[1]=0`)
//   - Supports XRGB8888 (32bpp) and RGB565 (16bpp) framebuffer pixel formats
//   - Three DVI 1.0 8b/10b DC-balanced TMDS channels (Ch0=Blue+Sync, Ch1=Green, Ch2=Red)
//     Outputting both 10-bit parallel symbols (`tmds_ch*_10b_o` for FPGA OSERDES/LVDS) AND
//     10x bit-clock serialized differential pairs (`tmds_tx_p_o[3:0]` / `tmds_tx_n_o[3:0]`)!
//
// CSR Register Map:
//   0x00 CTRL      : [0]=enable, [1]=test_pattern_en, [2]=rgb565_mode, [3]=hsync_pol, [4]=vsync_pol
//   0x04 FB_BASE   : [31:0] Framebuffer base address in SoC memory
//   0x08 H_TIMING0 : [15:0]=h_active (default 640), [31:16]=h_front_porch (default 16)
//   0x0C H_TIMING1 : [15:0]=h_sync   (default 96),  [31:16]=h_back_porch  (default 48)
//   0x10 V_TIMING0 : [15:0]=v_active (default 480), [31:16]=v_front_porch (default 10)
//   0x14 V_TIMING1 : [15:0]=v_sync   (default 2),   [31:16]=v_back_porch  (default 33)
//   0x18 INT_EN    : [0]=vsync_irq_en, [1]=fifo_underrun_irq_en
//   0x1C INT_STAT  : [1:0] W1C

module dvi_tmds_core
  import bus_pkg::*;
#(
  parameter int unsigned LINE_FIFO_DEPTH = 64
) (
  input  logic       clk_i,      // System Bus Clock
  input  logic       rst_ni,

  input  logic       clk_pix_i,  // Pixel Clock (e.g. 25.175MHz / 74.25MHz; may equal clk_i)
  input  logic       clk_10x_i,  // 10x Serializer Bit Clock (optional; tie to clk_pix_i if using 10b parallel)

  input  csr_req_t   csr_req_i,
  output csr_rsp_t   csr_rsp_o,

  output mem_req_t   dma_req_o,
  input  mem_rsp_t   dma_rsp_i,

  // Parallel 10-bit TMDS Outputs (for FPGA OSERDESE2 / DDIO) + Raw Syncs
  output logic [9:0] tmds_ch0_10b_o,
  output logic [9:0] tmds_ch1_10b_o,
  output logic [9:0] tmds_ch2_10b_o,
  output logic [9:0] tmds_clk_10b_o,
  output logic       vid_de_o,
  output logic       vid_hsync_o,
  output logic       vid_vsync_o,

  // Differential TMDS Serialized Pairs {Clk, Ch2(R), Ch1(G), Ch0(B)}
  output logic [3:0] tmds_tx_p_o,
  output logic [3:0] tmds_tx_n_o,

  output logic       irq_o
);

  logic [31:0] ctrl_q, fb_base_q, h_tim0_q, h_tim1_q, v_tim0_q, v_tim1_q;
  logic [1:0]  int_en_q, int_stat_q;

  wire enable   = ctrl_q[0];
  wire tpg_en   = ctrl_q[1];
  wire rgb565   = ctrl_q[2];
  wire hpol     = ctrl_q[3];
  wire vpol     = ctrl_q[4];

  wire [15:0] h_act = h_tim0_q[15:0];
  wire [15:0] h_fp  = h_tim0_q[31:16];
  wire [15:0] h_syn = h_tim1_q[15:0];
  wire [15:0] h_bp  = h_tim1_q[31:16];
  wire [15:0] h_tot = h_act + h_fp + h_syn + h_bp;

  wire [15:0] v_act = v_tim0_q[15:0];
  wire [15:0] v_fp  = v_tim0_q[31:16];
  wire [15:0] v_syn = v_tim1_q[15:0];
  wire [15:0] v_bp  = v_tim1_q[31:16];
  wire [15:0] v_tot = v_act + v_fp + v_syn + v_bp;

  // Dual-Clock Async Pixel FIFO (System Bus Domain -> Pixel Clock Domain)
  logic        fifo_wpush, fifo_wfull, fifo_rpop, fifo_rempty;
  logic [31:0] fifo_wdata, fifo_rdata;

  async_fifo #(.DATA_W(32), .ADDR_W($clog2(LINE_FIFO_DEPTH))) u_pix_cdc_fifo (
    .wclk_i(clk_i),      .wrst_ni(rst_ni && enable),
    .wpush_i(fifo_wpush), .wdata_i(fifo_wdata), .wfull_o(fifo_wfull),
    .rclk_i(clk_pix_i),  .rrst_ni(rst_ni && enable),
    .rpop_i(fifo_rpop),  .rdata_o(fifo_rdata), .rempty_o(fifo_rempty)
  );

  // Framebuffer Scanout DMA Master (clk_i domain)
  logic [31:0] scan_ptr_q, words_per_frame_q, words_rem_q;
  logic        dma_inflight_q;
  logic        vsync_sys_rise;

  wire [31:0] total_pixels = 32'(h_act) * 32'(v_act);
  wire [31:0] frame_words  = rgb565 ? ((total_pixels + 32'd1) >> 1) : total_pixels;

  assign dma_req_o.valid      = enable && !tpg_en && !fifo_wfull && !dma_inflight_q && (words_rem_q != 32'd0);
  assign dma_req_o.write      = 1'b0;
  assign dma_req_o.addr       = scan_ptr_q;
  assign dma_req_o.wdata      = '0;
  assign dma_req_o.wstrb      = 4'h0;
  assign dma_req_o.len        = 8'd0;
  assign dma_req_o.size       = 3'd2;
  assign dma_req_o.burst_wrap = 1'b0;
  assign dma_req_o.last       = 1'b1;
  assign dma_req_o.id         = 4'hD;
  assign dma_req_o.qos        = 4'h8; // High QoS real-time display scanout!
  assign dma_req_o.resp_ready = !fifo_wfull;

  assign fifo_wpush = dma_inflight_q && dma_rsp_i.resp_valid && !fifo_wfull;
  assign fifo_wdata = dma_rsp_i.rdata;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      scan_ptr_q        <= '0;
      words_per_frame_q <= '0;
      words_rem_q       <= '0;
      dma_inflight_q    <= 1'b0;
    end else if (!enable || tpg_en || vsync_sys_rise) begin
      scan_ptr_q        <= fb_base_q;
      words_per_frame_q <= frame_words;
      words_rem_q       <= frame_words;
      dma_inflight_q    <= 1'b0;
    end else begin
      if (dma_req_o.valid && dma_rsp_i.req_ready) begin
        dma_inflight_q <= 1'b1;
      end else if (fifo_wpush) begin
        dma_inflight_q <= 1'b0;
        scan_ptr_q     <= scan_ptr_q + 32'd4;
        words_rem_q    <= words_rem_q - 32'd1;
      end
    end
  end

  // Video Timing Generator & Pixel Unpacker (clk_pix_i domain)
  logic [15:0] h_cnt_q, v_cnt_q;
  logic        rgb565_hi_q;
  logic        underrun_pix_q;

  wire active_video = enable && (h_cnt_q < h_act) && (v_cnt_q < v_act);
  wire hsync_raw    = (h_cnt_q >= (h_act + h_fp)) && (h_cnt_q < (h_act + h_fp + h_syn));
  wire vsync_raw    = (v_cnt_q >= (v_act + v_fp)) && (v_cnt_q < (v_act + v_fp + v_syn));
  wire hsync_out    = hsync_raw ^ ~hpol;
  wire vsync_out    = vsync_raw ^ ~vpol;

  assign vid_de_o    = active_video;
  assign vid_hsync_o = hsync_out;
  assign vid_vsync_o = vsync_out;

  assign fifo_rpop = active_video && !tpg_en && (!rgb565 || rgb565_hi_q) && !fifo_rempty;

  always_ff @(posedge clk_pix_i or negedge rst_ni) begin
    if (!rst_ni) begin
      h_cnt_q        <= '0;
      v_cnt_q        <= '0;
      rgb565_hi_q    <= 1'b0;
      underrun_pix_q <= 1'b0;
    end else if (!enable) begin
      h_cnt_q        <= '0;
      v_cnt_q        <= '0;
      rgb565_hi_q    <= 1'b0;
      underrun_pix_q <= 1'b0;
    end else begin
      underrun_pix_q <= active_video && !tpg_en && fifo_rempty;
      if (active_video && !tpg_en && rgb565) rgb565_hi_q <= ~rgb565_hi_q;
      else if (!active_video && vsync_raw)   rgb565_hi_q <= 1'b0;

      if (h_cnt_q + 16'd1 >= h_tot) begin
        h_cnt_q <= '0;
        if (v_cnt_q + 16'd1 >= v_tot) v_cnt_q <= '0;
        else                          v_cnt_q <= v_cnt_q + 16'd1;
      end else begin
        h_cnt_q <= h_cnt_q + 16'd1;
      end
    end
  end

  logic [7:0] pix_r, pix_g, pix_b;
  always_comb begin
    if (!active_video) begin
      pix_r = 8'd0; pix_g = 8'd0; pix_b = 8'd0;
    end else if (tpg_en) begin
      // 8-Bar SMPTE Color Bar Test Pattern
      unique case (h_cnt_q[8:6])
        3'd0: begin pix_r = 8'hFF; pix_g = 8'hFF; pix_b = 8'hFF; end
        3'd1: begin pix_r = 8'hFF; pix_g = 8'hFF; pix_b = 8'h00; end
        3'd2: begin pix_r = 8'h00; pix_g = 8'hFF; pix_b = 8'hFF; end
        3'd3: begin pix_r = 8'h00; pix_g = 8'hFF; pix_b = 8'h00; end
        3'd4: begin pix_r = 8'hFF; pix_g = 8'h00; pix_b = 8'hFF; end
        3'd5: begin pix_r = 8'hFF; pix_g = 8'h00; pix_b = 8'h00; end
        3'd6: begin pix_r = 8'h00; pix_g = 8'h00; pix_b = 8'hFF; end
        3'd7: begin pix_r = 8'h18; pix_g = 8'h18; pix_b = 8'h18; end
      endcase
    end else if (rgb565) begin
      logic [15:0] p16;
      p16   = rgb565_hi_q ? fifo_rdata[31:16] : fifo_rdata[15:0];
      pix_r = {p16[15:11], p16[15:13]};
      pix_g = {p16[10:5],  p16[10:9]};
      pix_b = {p16[4:0],   p16[4:2]};
    end else begin
      pix_r = fifo_rdata[23:16];
      pix_g = fifo_rdata[15:8];
      pix_b = fifo_rdata[7:0];
    end
  end

  // Three True DVI 1.0 8b/10b TMDS Encoders
  tmds_encoder u_enc_b (
    .clk_pix_i(clk_pix_i), .rst_ni(rst_ni),
    .din_i(pix_b), .ctrl_i({vsync_out, hsync_out}), .de_i(active_video),
    .tmds_o(tmds_ch0_10b_o)
  );

  tmds_encoder u_enc_g (
    .clk_pix_i(clk_pix_i), .rst_ni(rst_ni),
    .din_i(pix_g), .ctrl_i(2'b00), .de_i(active_video),
    .tmds_o(tmds_ch1_10b_o)
  );

  tmds_encoder u_enc_r (
    .clk_pix_i(clk_pix_i), .rst_ni(rst_ni),
    .din_i(pix_r), .ctrl_i(2'b00), .de_i(active_video),
    .tmds_o(tmds_ch2_10b_o)
  );

  assign tmds_clk_10b_o = 10'b00000_11111;

  // 10:1 Bit Serializer (clk_10x_i domain)
  logic [3:0] ser_idx_q;
  logic [9:0] ser_b_q, ser_g_q, ser_r_q, ser_c_q;

  always_ff @(posedge clk_10x_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ser_idx_q <= '0;
      ser_b_q   <= 10'b1101010100;
      ser_g_q   <= 10'b1101010100;
      ser_r_q   <= 10'b1101010100;
      ser_c_q   <= 10'b0000011111;
    end else if (ser_idx_q == 4'd9) begin
      ser_idx_q <= 4'd0;
      ser_b_q   <= tmds_ch0_10b_o;
      ser_g_q   <= tmds_ch1_10b_o;
      ser_r_q   <= tmds_ch2_10b_o;
      ser_c_q   <= 10'b0000011111;
    end else begin
      ser_idx_q <= ser_idx_q + 4'd1;
      ser_b_q   <= {1'b0, ser_b_q[9:1]};
      ser_g_q   <= {1'b0, ser_g_q[9:1]};
      ser_r_q   <= {1'b0, ser_r_q[9:1]};
      ser_c_q   <= {1'b0, ser_c_q[9:1]};
    end
  end

  assign tmds_tx_p_o = {ser_c_q[0], ser_r_q[0], ser_g_q[0], ser_b_q[0]};
  assign tmds_tx_n_o = ~tmds_tx_p_o;

  // Synchronize VSync & Underrun into clk_i domain for CSR/IRQ
  logic underrun_sys_rise;
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_vs_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(vsync_raw), .sync_o(), .rise_o(vsync_sys_rise), .fall_o()
  );
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_ur_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i(underrun_pix_q), .sync_o(), .rise_o(underrun_sys_rise), .fall_o()
  );

  wire [7:0] reg_off = csr_req_i.addr[7:0];
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = fb_base_q;
      8'h08: csr_rsp_o.rdata = h_tim0_q;
      8'h0C: csr_rsp_o.rdata = h_tim1_q;
      8'h10: csr_rsp_o.rdata = v_tim0_q;
      8'h14: csr_rsp_o.rdata = v_tim1_q;
      8'h18: csr_rsp_o.rdata = {30'd0, int_en_q};
      8'h1C: csr_rsp_o.rdata = {30'd0, int_stat_q};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q     <= 32'h0000_0002; // Test pattern ready by default
      fb_base_q  <= '0;
      h_tim0_q   <= {16'd16, 16'd640};
      h_tim1_q   <= {16'd48, 16'd96};
      v_tim0_q   <= {16'd10, 16'd480};
      v_tim1_q   <= {16'd33, 16'd2};
      int_en_q   <= '0;
      int_stat_q <= '0;
    end else begin
      if (vsync_sys_rise)    int_stat_q[0] <= 1'b1;
      if (underrun_sys_rise) int_stat_q[1] <= 1'b1;

      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: ctrl_q     <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
          8'h04: fb_base_q  <= csr_req_i.wdata;
          8'h08: h_tim0_q   <= csr_req_i.wdata;
          8'h0C: h_tim1_q   <= csr_req_i.wdata;
          8'h10: v_tim0_q   <= csr_req_i.wdata;
          8'h14: v_tim1_q   <= csr_req_i.wdata;
          8'h18: int_en_q   <= csr_req_i.wdata[1:0];
          8'h1C: int_stat_q <= int_stat_q & ~csr_req_i.wdata[1:0];
          default: ;
        endcase
      end
    end
  end

  assign irq_o = |(int_en_q & int_stat_q);

endmodule : dvi_tmds_core
