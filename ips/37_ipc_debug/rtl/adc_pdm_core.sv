// SPDX-License-Identifier: Apache-2.0
// adc_pdm_core.sv - SAR-ADC Digital Sequencer & PDM-to-PCM (DFSDM) Sinc3 Decimation Filter
//
// Register Map (12-bit byte offset, word-aligned):
//   0x000 CAPS           (RO)     : [7:0]=16(MAX_SEQ), [15:8]=ADC_FIFO_DEPTH, [23:16]=PDM_FIFO_DEPTH, [31:24]=0x37
//   0x004 IRQ_STAT       (RW1C/RO): [0]=ADC_DONE(W1C), [1]=ADC_OVF(W1C), [2]=AWD_LOW(W1C), [3]=AWD_HIGH(W1C),
//                                   [4]=PDM_OVF(W1C), [8]=ADC_WM(RO), [9]=PDM_WM(RO)
//   0x008 IRQ_EN         (RW)     : Interrupt enable mask matching IRQ_STAT
//   0x020 ADC_CTRL       (RW)     : [0]=EN,[1]=CONT,[2]=EXT_TRG_EN,[3]=SW_START(WO),[7:4]=SEQ_LEN(N-1),
//                                   [10:8]=OVS_RATIO(0..6->1..64x),[14:11]=OVS_SHIFT,[15]=FIFO_FLUSH(WO),
//                                   [19:16]=DMA_WM,[27:20]=SAMPLE_DIV
//   0x024 ADC_STAT       (RO)     : [0]=BUSY,[1]=EMPTY,[2]=FULL,[7:3]=LEVEL,[11:8]=CUR_SLOT,[15:12]=CUR_CH
//   0x028 ADC_SEQ0       (RW)     : Scan slots 0..7 (4b channel ID each)
//   0x02C ADC_SEQ1       (RW)     : Scan slots 8..15 (4b channel ID each)
//   0x030 ADC_AWD_CFG    (RW)     : [0]=AWD_EN, [31:16]=AWD_CH_MASK
//   0x034 ADC_AWD_THRESH (RW)     : [15:0]=AWD_LOW, [31:16]=AWD_HIGH
//   0x038 ADC_DATA       (RO)     : Pop ADC FIFO: [15:0]=RESULT, [19:16]=CH_ID, [20]=AWD_LOW, [21]=AWD_HIGH
//   0x03C ADC_LAST_RAW   (RO)     : Non-destructive last ADC conversion {12'd0, ch_id, result}
//   0x060 PDM_CTRL       (RW)     : [0]=EN,[1]=STEREO_EN,[3:2]=OSR_SEL(0=16,1=32,2=64,3=128),[4]=HPF_EN,
//                                   [8:5]=HPF_SHIFT,[9]=EDGE_POL,[11:10]=GAIN_RSHIFT,[15]=FIFO_FLUSH(WO),
//                                   [19:16]=DMA_WM,[27:20]=CLK_DIV
//   0x064 PDM_STAT       (RO)     : [0]=ACTIVE,[1]=EMPTY,[2]=FULL,[7:3]=LEVEL
//   0x068 PDM_DATA       (RO)     : Pop PDM FIFO: [15:0]=CH0_PCM16, [31:16]=CH1_PCM16
//   0x06C PDM_DC_EST     (RO)     : {dc_est_ch1[15:0], dc_est_ch0[15:0]}

module adc_pdm_core
  import bus_pkg::*;
#(
  parameter int unsigned ADC_FIFO_DEPTH = 16,
  parameter int unsigned PDM_FIFO_DEPTH = 16
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  // External / Foundry SAR-ADC Macro Interface
  output logic        adc_start_o,
  output logic [3:0]  adc_ch_sel_o,
  input  logic        adc_eoc_i,
  input  logic [11:0] adc_data_i,
  input  logic        ext_trgo_i,

  // External MEMS PDM Microphone Interface
  output logic        pdm_clk_o,
  input  logic        pdm_data_i,

  // DMA & Interrupts
  output logic        dma_adc_req_o,
  output logic        dma_pdm_req_o,
  output logic        awd_irq_o,
  output logic        irq_o
);

  localparam int unsigned ADC_CNT_W = $clog2(ADC_FIFO_DEPTH + 1);
  localparam int unsigned PDM_CNT_W = $clog2(PDM_FIFO_DEPTH + 1);

  typedef enum logic [1:0] {
    ADC_IDLE,
    ADC_START,
    ADC_WAIT_EOC,
    ADC_GAP
  } adc_state_e;

  // -------------------------------------------------------------------------
  // Interrupts & CSR Registers
  // -------------------------------------------------------------------------
  logic        irq_adc_done_q, irq_adc_ovf_q, irq_awd_low_q, irq_awd_high_q, irq_pdm_ovf_q;
  logic [31:0] irq_en_q;

  // ADC CSRs
  logic        adc_en_q, adc_cont_q, adc_ext_trg_en_q;
  logic [3:0]  adc_seq_len_q;
  logic [2:0]  adc_ovs_ratio_q;
  logic [3:0]  adc_ovs_shift_q;
  logic [3:0]  adc_dma_wm_q;
  logic [7:0]  adc_sample_div_q;
  logic [31:0] adc_seq0_q, adc_seq1_q;
  logic        awd_en_q;
  logic [15:0] awd_ch_mask_q;
  logic [15:0] awd_low_q, awd_high_q;
  logic [19:0] adc_last_raw_q;

  // ADC Sequencer State
  adc_state_e  adc_state_q;
  logic [3:0]  adc_slot_q;
  logic [6:0]  adc_ovs_cnt_q;
  logic [17:0] adc_acc_q;
  logic [7:0]  adc_gap_cnt_q;

  // PDM CSRs
  logic        pdm_en_q, pdm_stereo_q, pdm_hpf_en_q, pdm_edge_pol_q;
  logic [1:0]  pdm_osr_sel_q, pdm_gain_rshift_q;
  logic [3:0]  pdm_hpf_shift_q, pdm_dma_wm_q;
  logic [7:0]  pdm_clk_div_q;

  // PDM Clock & Capture State
  logic        pdm_clk_q;
  logic [7:0]  pdm_div_cnt_q;
  logic signed [15:0] ch0_pcm_hold_q;

  // External trigger edge detector
  logic ext_trgo_rise;
  edge_detect #(.WIDTH(1), .INIT(1'b0)) u_trgo_ed (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .async_i (ext_trgo_i),
    .sync_o  (),
    .rise_o  (ext_trgo_rise),
    .fall_o  ()
  );

  // -------------------------------------------------------------------------
  // FIFOs
  // -------------------------------------------------------------------------
  logic                   adc_fifo_clr, adc_fifo_push, adc_fifo_pop;
  logic [31:0]            adc_fifo_wdata, adc_fifo_rdata;
  logic                   adc_fifo_full, adc_fifo_empty;
  logic [ADC_CNT_W-1:0]   adc_fifo_level;

  sync_fifo #(
    .DATA_W (32),
    .DEPTH  (ADC_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_adc_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (adc_fifo_clr),
    .push_i   (adc_fifo_push && !adc_fifo_full),
    .wdata_i  (adc_fifo_wdata),
    .full_o   (adc_fifo_full),
    .afull_o  (),
    .pop_i    (adc_fifo_pop),
    .rdata_o  (adc_fifo_rdata),
    .empty_o  (adc_fifo_empty),
    .aempty_o (),
    .level_o  (adc_fifo_level)
  );

  logic                   pdm_fifo_clr, pdm_fifo_push, pdm_fifo_pop;
  logic [31:0]            pdm_fifo_wdata, pdm_fifo_rdata;
  logic                   pdm_fifo_full, pdm_fifo_empty;
  logic [PDM_CNT_W-1:0]   pdm_fifo_level;

  sync_fifo #(
    .DATA_W (32),
    .DEPTH  (PDM_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_pdm_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (pdm_fifo_clr),
    .push_i   (pdm_fifo_push && !pdm_fifo_full),
    .wdata_i  (pdm_fifo_wdata),
    .full_o   (pdm_fifo_full),
    .afull_o  (),
    .pop_i    (pdm_fifo_pop),
    .rdata_o  (pdm_fifo_rdata),
    .empty_o  (pdm_fifo_empty),
    .aempty_o (),
    .level_o  (pdm_fifo_level)
  );

  // -------------------------------------------------------------------------
  // PDM Sinc3 CIC + HPF Channels (Ch0 = Left/Mono, Ch1 = Right)
  // -------------------------------------------------------------------------
  logic               pdm_filter_clr;
  logic               ch0_sample_en, ch1_sample_en;
  logic               ch0_pcm_valid, ch1_pcm_valid;
  logic signed [15:0] ch0_pcm_data, ch1_pcm_data;
  logic signed [15:0] ch0_dc_est, ch1_dc_est;

  pdm_cic_hpf u_pdm_ch0 (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .clr_i         (pdm_filter_clr),
    .sample_en_i   (ch0_sample_en),
    .pdm_bit_i     (pdm_data_i),
    .osr_sel_i     (pdm_osr_sel_q),
    .gain_rshift_i (pdm_gain_rshift_q),
    .hpf_en_i      (pdm_hpf_en_q),
    .hpf_shift_i   (pdm_hpf_shift_q),
    .pcm_valid_o   (ch0_pcm_valid),
    .pcm_data_o    (ch0_pcm_data),
    .dc_est_o      (ch0_dc_est)
  );

  pdm_cic_hpf u_pdm_ch1 (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .clr_i         (pdm_filter_clr),
    .sample_en_i   (ch1_sample_en),
    .pdm_bit_i     (pdm_data_i),
    .osr_sel_i     (pdm_osr_sel_q),
    .gain_rshift_i (pdm_gain_rshift_q),
    .hpf_en_i      (pdm_hpf_en_q),
    .hpf_shift_i   (pdm_hpf_shift_q),
    .pcm_valid_o   (ch1_pcm_valid),
    .pcm_data_o    (ch1_pcm_data),
    .dc_est_o      (ch1_dc_est)
  );

  // -------------------------------------------------------------------------
  // SAR-ADC Channel Slot Mux & Oversampling Calculation
  // -------------------------------------------------------------------------
  logic [3:0] cur_ch_id;
  always_comb begin
    unique case (adc_slot_q)
      4'd0:  cur_ch_id = adc_seq0_q[3:0];
      4'd1:  cur_ch_id = adc_seq0_q[7:4];
      4'd2:  cur_ch_id = adc_seq0_q[11:8];
      4'd3:  cur_ch_id = adc_seq0_q[15:12];
      4'd4:  cur_ch_id = adc_seq0_q[19:16];
      4'd5:  cur_ch_id = adc_seq0_q[23:20];
      4'd6:  cur_ch_id = adc_seq0_q[27:24];
      4'd7:  cur_ch_id = adc_seq0_q[31:28];
      4'd8:  cur_ch_id = adc_seq1_q[3:0];
      4'd9:  cur_ch_id = adc_seq1_q[7:4];
      4'd10: cur_ch_id = adc_seq1_q[11:8];
      4'd11: cur_ch_id = adc_seq1_q[15:12];
      4'd12: cur_ch_id = adc_seq1_q[19:16];
      4'd13: cur_ch_id = adc_seq1_q[23:20];
      4'd14: cur_ch_id = adc_seq1_q[27:24];
      4'd15: cur_ch_id = adc_seq1_q[31:28];
    endcase
  end

  logic [6:0] ovs_max_cnt;
  always_comb begin
    unique case (adc_ovs_ratio_q)
      3'd0:    ovs_max_cnt = 7'd0;  // 1x
      3'd1:    ovs_max_cnt = 7'd1;  // 2x
      3'd2:    ovs_max_cnt = 7'd3;  // 4x
      3'd3:    ovs_max_cnt = 7'd7;  // 8x
      3'd4:    ovs_max_cnt = 7'd15; // 16x
      3'd5:    ovs_max_cnt = 7'd31; // 32x
      default: ovs_max_cnt = 7'd63; // 64x
    endcase
  end

  assign adc_start_o  = (adc_state_q == ADC_START);
  assign adc_ch_sel_o = cur_ch_id;
  assign pdm_clk_o    = pdm_en_q ? pdm_clk_q : 1'b0;

  // Watermarks, DMA, IRQ
  wire [4:0] eff_adc_wm = (adc_dma_wm_q == 4'd0) ? 5'd1 : {1'b0, adc_dma_wm_q};
  wire [4:0] eff_pdm_wm = (pdm_dma_wm_q == 4'd0) ? 5'd1 : {1'b0, pdm_dma_wm_q};

  wire adc_wm_hit = (5'(adc_fifo_level) >= eff_adc_wm);
  wire pdm_wm_hit = (5'(pdm_fifo_level) >= eff_pdm_wm);

  assign dma_adc_req_o = adc_wm_hit;
  assign dma_pdm_req_o = pdm_wm_hit;
  assign awd_irq_o     = irq_awd_low_q || irq_awd_high_q;

  wire [31:0] irq_stat_word = {
    22'd0,
    pdm_wm_hit,
    adc_wm_hit,
    3'd0,
    irq_pdm_ovf_q,
    irq_awd_high_q,
    irq_awd_low_q,
    irq_adc_ovf_q,
    irq_adc_done_q
  };

  assign irq_o = |(irq_stat_word & irq_en_q);

  // CSR Decode
  wire [9:0] word_off  = csr_req_i.addr[11:2];
  wire       bad_align = (csr_req_i.addr[1:0] != 2'b00);

  logic addr_valid;
  always_comb begin
    addr_valid = !bad_align && (
      (word_off == 10'h000) || (word_off == 10'h001) || (word_off == 10'h002) ||
      (word_off >= 10'h008 && word_off <= 10'h00F) ||
      (word_off >= 10'h018 && word_off <= 10'h01B)
    );
  end

  wire csr_wr = csr_req_i.valid && csr_req_i.write && addr_valid;
  wire csr_rd = csr_req_i.valid && !csr_req_i.write && addr_valid;

  wire adc_sw_start_pulse = csr_wr && (word_off == 10'h008) && csr_req_i.wstrb[0] && csr_req_i.wdata[3];
  assign adc_fifo_clr     = csr_wr && (word_off == 10'h008) && csr_req_i.wstrb[1] && csr_req_i.wdata[15];
  assign pdm_fifo_clr     = csr_wr && (word_off == 10'h018) && csr_req_i.wstrb[1] && csr_req_i.wdata[15];
  assign pdm_filter_clr   = pdm_fifo_clr || !pdm_en_q;

  assign adc_fifo_pop     = csr_rd && (word_off == 10'h00E) && !adc_fifo_empty;
  assign pdm_fifo_pop     = csr_rd && (word_off == 10'h01A) && !pdm_fifo_empty;

  // Combinational CSR Read Mux
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = csr_req_i.valid && !addr_valid;

    if (csr_req_i.valid && addr_valid) begin
      unique case (word_off)
        10'h000: csr_rsp_o.rdata = {8'h37, 8'(PDM_FIFO_DEPTH), 8'(ADC_FIFO_DEPTH), 8'd16};
        10'h001: csr_rsp_o.rdata = irq_stat_word;
        10'h002: csr_rsp_o.rdata = irq_en_q;
        10'h008: csr_rsp_o.rdata = {4'd0, adc_sample_div_q, adc_dma_wm_q, 1'b0,
                                    adc_ovs_shift_q, adc_ovs_ratio_q, adc_seq_len_q,
                                    1'b0, adc_ext_trg_en_q, adc_cont_q, adc_en_q};
        10'h009: csr_rsp_o.rdata = {16'd0, cur_ch_id, adc_slot_q,
                                    5'(adc_fifo_level), adc_fifo_full, adc_fifo_empty,
                                    (adc_state_q != ADC_IDLE)};
        10'h00A: csr_rsp_o.rdata = adc_seq0_q;
        10'h00B: csr_rsp_o.rdata = adc_seq1_q;
        10'h00C: csr_rsp_o.rdata = {awd_ch_mask_q, 15'd0, awd_en_q};
        10'h00D: csr_rsp_o.rdata = {awd_high_q, awd_low_q};
        10'h00E: csr_rsp_o.rdata = adc_fifo_rdata;
        10'h00F: csr_rsp_o.rdata = {12'd0, adc_last_raw_q};
        10'h018: csr_rsp_o.rdata = {4'd0, pdm_clk_div_q, pdm_dma_wm_q, 4'd0,
                                    pdm_gain_rshift_q, pdm_edge_pol_q, pdm_hpf_shift_q,
                                    pdm_hpf_en_q, pdm_osr_sel_q, pdm_stereo_q, pdm_en_q};
        10'h019: csr_rsp_o.rdata = {24'd0, 5'(pdm_fifo_level), pdm_fifo_full, pdm_fifo_empty, pdm_en_q};
        10'h01A: csr_rsp_o.rdata = pdm_fifo_rdata;
        10'h01B: csr_rsp_o.rdata = {ch1_dc_est, ch0_dc_est};
        default: csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

  // PDM sample & FIFO push combinational routing
  always_comb begin
    ch0_sample_en  = 1'b0;
    ch1_sample_en  = 1'b0;
    pdm_fifo_push  = 1'b0;
    pdm_fifo_wdata = 32'd0;

    if (pdm_en_q && (pdm_div_cnt_q >= pdm_clk_div_q)) begin
      automatic logic is_ch0_phase = (pdm_clk_q == !pdm_edge_pol_q);
      if (is_ch0_phase) begin
        ch0_sample_en = 1'b1;
      end else if (pdm_stereo_q) begin
        ch1_sample_en = 1'b1;
      end
    end

    if (!pdm_stereo_q && ch0_pcm_valid) begin
      pdm_fifo_push  = 1'b1;
      pdm_fifo_wdata = {16'd0, ch0_pcm_data};
    end else if (pdm_stereo_q && ch1_pcm_valid) begin
      pdm_fifo_push  = 1'b1;
      pdm_fifo_wdata = {ch1_pcm_data, ch0_pcm_hold_q};
    end
  end

  // ADC completion combinational check
  logic        adc_sample_done;
  logic [17:0] adc_acc_next;
  logic [17:0] adc_shifted_val;
  logic [15:0] adc_final_sample;
  logic        awd_low_hit, awd_high_hit;

  always_comb begin
    adc_sample_done  = (adc_state_q == ADC_WAIT_EOC) && adc_eoc_i;
    adc_acc_next     = adc_acc_q + {6'd0, adc_data_i};
    adc_shifted_val  = adc_acc_next >> adc_ovs_shift_q;
    adc_final_sample = (adc_shifted_val > 18'h0FFFF) ? 16'hFFFF : adc_shifted_val[15:0];
    awd_low_hit      = awd_en_q && awd_ch_mask_q[cur_ch_id] && (adc_final_sample < awd_low_q);
    awd_high_hit     = awd_en_q && awd_ch_mask_q[cur_ch_id] && (adc_final_sample > awd_high_q);

    adc_fifo_push    = adc_sample_done && (adc_ovs_cnt_q >= ovs_max_cnt);
    adc_fifo_wdata   = {10'd0, awd_high_hit, awd_low_hit, cur_ch_id, adc_final_sample};
  end

  // Sequential Engine
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      irq_adc_done_q   <= 1'b0;
      irq_adc_ovf_q    <= 1'b0;
      irq_awd_low_q    <= 1'b0;
      irq_awd_high_q   <= 1'b0;
      irq_pdm_ovf_q    <= 1'b0;
      irq_en_q         <= 32'd0;
      adc_en_q         <= 1'b0;
      adc_cont_q       <= 1'b0;
      adc_ext_trg_en_q <= 1'b0;
      adc_seq_len_q    <= 4'd0;
      adc_ovs_ratio_q  <= 3'd0;
      adc_ovs_shift_q  <= 4'd0;
      adc_dma_wm_q     <= 4'd1;
      adc_sample_div_q <= 8'd0;
      adc_seq0_q       <= 32'h7654_3210;
      adc_seq1_q       <= 32'hFEDC_BA98;
      awd_en_q         <= 1'b0;
      awd_ch_mask_q    <= 16'hFFFF;
      awd_low_q        <= 16'h0000;
      awd_high_q       <= 16'h0FFF;
      adc_last_raw_q   <= 20'd0;
      adc_state_q      <= ADC_IDLE;
      adc_slot_q       <= 4'd0;
      adc_ovs_cnt_q    <= 7'd0;
      adc_acc_q        <= 18'd0;
      adc_gap_cnt_q    <= 8'd0;
      pdm_en_q         <= 1'b0;
      pdm_stereo_q     <= 1'b0;
      pdm_osr_sel_q    <= 2'd1;
      pdm_hpf_en_q     <= 1'b0;
      pdm_hpf_shift_q  <= 4'd6;
      pdm_edge_pol_q   <= 1'b0;
      pdm_gain_rshift_q<= 2'd0;
      pdm_dma_wm_q     <= 4'd1;
      pdm_clk_div_q    <= 8'd3;
      pdm_clk_q        <= 1'b0;
      pdm_div_cnt_q    <= 8'd0;
      ch0_pcm_hold_q   <= 16'sd0;
    end else begin
      // CSR writes
      if (csr_wr) begin
        unique case (word_off)
          10'h001: begin // IRQ_STAT W1C
            if (csr_req_i.wstrb[0]) begin
              if (csr_req_i.wdata[0]) irq_adc_done_q <= 1'b0;
              if (csr_req_i.wdata[1]) irq_adc_ovf_q  <= 1'b0;
              if (csr_req_i.wdata[2]) irq_awd_low_q  <= 1'b0;
              if (csr_req_i.wdata[3]) irq_awd_high_q <= 1'b0;
              if (csr_req_i.wdata[4]) irq_pdm_ovf_q  <= 1'b0;
            end
          end
          10'h002: irq_en_q <= ip_common_pkg::apply_wstrb(irq_en_q, csr_req_i.wdata, csr_req_i.wstrb);
          10'h008: begin
            if (csr_req_i.wstrb[0]) begin
              adc_en_q         <= csr_req_i.wdata[0];
              adc_cont_q       <= csr_req_i.wdata[1];
              adc_ext_trg_en_q <= csr_req_i.wdata[2];
              adc_seq_len_q    <= csr_req_i.wdata[7:4];
            end
            if (csr_req_i.wstrb[1]) begin
              adc_ovs_ratio_q <= csr_req_i.wdata[10:8];
              adc_ovs_shift_q <= csr_req_i.wdata[14:11];
            end
            if (csr_req_i.wstrb[2]) begin
              adc_dma_wm_q          <= csr_req_i.wdata[19:16];
              adc_sample_div_q[3:0] <= csr_req_i.wdata[23:20];
            end
            if (csr_req_i.wstrb[3]) begin
              adc_sample_div_q[7:4] <= csr_req_i.wdata[27:24];
            end
          end
          10'h00A: adc_seq0_q <= ip_common_pkg::apply_wstrb(adc_seq0_q, csr_req_i.wdata, csr_req_i.wstrb);
          10'h00B: adc_seq1_q <= ip_common_pkg::apply_wstrb(adc_seq1_q, csr_req_i.wdata, csr_req_i.wstrb);
          10'h00C: begin
            if (csr_req_i.wstrb[0]) awd_en_q             <= csr_req_i.wdata[0];
            if (csr_req_i.wstrb[2]) awd_ch_mask_q[7:0]   <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) awd_ch_mask_q[15:8]  <= csr_req_i.wdata[31:24];
          end
          10'h00D: begin
            if (csr_req_i.wstrb[0]) awd_low_q[7:0]   <= csr_req_i.wdata[7:0];
            if (csr_req_i.wstrb[1]) awd_low_q[15:8]  <= csr_req_i.wdata[15:8];
            if (csr_req_i.wstrb[2]) awd_high_q[7:0]  <= csr_req_i.wdata[23:16];
            if (csr_req_i.wstrb[3]) awd_high_q[15:8] <= csr_req_i.wdata[31:24];
          end
          10'h018: begin
            if (csr_req_i.wstrb[0]) begin
              pdm_en_q            <= csr_req_i.wdata[0];
              pdm_stereo_q        <= csr_req_i.wdata[1];
              pdm_osr_sel_q       <= csr_req_i.wdata[3:2];
              pdm_hpf_en_q        <= csr_req_i.wdata[4];
              pdm_hpf_shift_q[2:0]<= csr_req_i.wdata[7:5];
            end
            if (csr_req_i.wstrb[1]) begin
              pdm_hpf_shift_q[3] <= csr_req_i.wdata[8];
              pdm_edge_pol_q     <= csr_req_i.wdata[9];
              pdm_gain_rshift_q  <= csr_req_i.wdata[11:10];
            end
            if (csr_req_i.wstrb[2]) begin
              pdm_dma_wm_q       <= csr_req_i.wdata[19:16];
              pdm_clk_div_q[3:0] <= csr_req_i.wdata[23:20];
            end
            if (csr_req_i.wstrb[3]) begin
              pdm_clk_div_q[7:4] <= csr_req_i.wdata[27:24];
            end
          end
          default: ;
        endcase
      end

      // SAR-ADC Sequencer FSM
      begin
        automatic logic adc_en_next = (csr_wr && (word_off == 10'h008) && csr_req_i.wstrb[0])
                                      ? csr_req_i.wdata[0] : adc_en_q;
        if (!adc_en_next) begin
          adc_state_q   <= ADC_IDLE;
          adc_slot_q    <= 4'd0;
          adc_ovs_cnt_q <= 7'd0;
          adc_acc_q     <= 18'd0;
          adc_gap_cnt_q <= 8'd0;
        end else begin
          unique case (adc_state_q)
            ADC_IDLE: begin
              if (adc_sw_start_pulse || (adc_ext_trg_en_q && ext_trgo_rise)) begin
                adc_slot_q    <= 4'd0;
                adc_ovs_cnt_q <= 7'd0;
                adc_acc_q     <= 18'd0;
                adc_state_q   <= ADC_START;
              end
            end

            ADC_START: begin
              adc_state_q <= ADC_WAIT_EOC;
            end

            ADC_WAIT_EOC: begin
              if (adc_eoc_i) begin
                if (adc_ovs_cnt_q < ovs_max_cnt) begin
                  adc_acc_q     <= adc_acc_next;
                  adc_ovs_cnt_q <= adc_ovs_cnt_q + 7'd1;
                  if (adc_sample_div_q == 8'd0) begin
                    adc_state_q <= ADC_START;
                  end else begin
                    adc_gap_cnt_q <= adc_sample_div_q - 8'd1;
                    adc_state_q   <= ADC_GAP;
                  end
                end else begin
                  // Oversampling complete for this slot
                  adc_last_raw_q <= {cur_ch_id, adc_final_sample};
                  if (adc_fifo_full) irq_adc_ovf_q <= 1'b1;
                  if (awd_low_hit)   irq_awd_low_q  <= 1'b1;
                  if (awd_high_hit)  irq_awd_high_q <= 1'b1;

                  adc_acc_q     <= 18'd0;
                  adc_ovs_cnt_q <= 7'd0;

                  if (adc_slot_q >= adc_seq_len_q) begin
                    irq_adc_done_q <= 1'b1;
                    adc_slot_q     <= 4'd0;
                    if (adc_cont_q) begin
                      if (adc_sample_div_q == 8'd0) begin
                        adc_state_q <= ADC_START;
                      end else begin
                        adc_gap_cnt_q <= adc_sample_div_q - 8'd1;
                        adc_state_q   <= ADC_GAP;
                      end
                    end else begin
                      adc_state_q <= ADC_IDLE;
                    end
                  end else begin
                    adc_slot_q <= adc_slot_q + 4'd1;
                    if (adc_sample_div_q == 8'd0) begin
                      adc_state_q <= ADC_START;
                    end else begin
                      adc_gap_cnt_q <= adc_sample_div_q - 8'd1;
                      adc_state_q   <= ADC_GAP;
                    end
                  end
                end
              end
            end

            ADC_GAP: begin
              if (adc_gap_cnt_q == 8'd0) begin
                adc_state_q <= ADC_START;
              end else begin
                adc_gap_cnt_q <= adc_gap_cnt_q - 8'd1;
              end
            end

            default: adc_state_q <= ADC_IDLE;
          endcase
        end
      end

      // PDM Clock Divider & Stereo Frame Assembly
      if (!pdm_en_q) begin
        pdm_clk_q      <= 1'b0;
        pdm_div_cnt_q  <= 8'd0;
        ch0_pcm_hold_q <= 16'sd0;
      end else begin
        if (pdm_div_cnt_q >= pdm_clk_div_q) begin
          pdm_div_cnt_q <= 8'd0;
          pdm_clk_q     <= ~pdm_clk_q;
        end else begin
          pdm_div_cnt_q <= pdm_div_cnt_q + 8'd1;
        end
        if (ch0_pcm_valid) begin
          ch0_pcm_hold_q <= ch0_pcm_data;
        end
        if (pdm_fifo_push && pdm_fifo_full) begin
          irq_pdm_ovf_q <= 1'b1;
        end
      end
    end
  end

endmodule : adc_pdm_core
