// SPDX-License-Identifier: Apache-2.0
// sdmmc_core.sv - SD / SDIO / eMMC host controller core (protocol-agnostic CSR + mem master)
//
// Own register map (word offsets, 32-bit registers).  See doc/README.md for bit fields.
//   0x00 ID          0x04 CAPS        0x08 CTRL        0x0C CLK_CTRL
//   0x10 BUS_CTRL    0x14 BLK_CFG     0x18 CMD_ARG     0x1C CMD
//   0x20..0x2C RESP0..RESP3           0x30 RESP_AUTO   0x34 STATUS
//   0x38 FIFO_CTRL   0x3C FIFO_LEVEL  0x40 DATA        0x44 INT_STATUS
//   0x48 INT_ENABLE  0x4C CMD_TOUT    0x50 DATA_TOUT   0x54 BUSY_TOUT
//   0x58 CD_DEB      0x5C VSW_CTRL    0x60 VSW_TIME1   0x64 VSW_TIME2
//   0x68 STOP_ARG    0x6C ACMD_CFG    0x70 DMA_CTRL    0x74 DMA_ADDR
//   0x78 DMA_STAT    0x7C BLK_DONE

module sdmmc_core
  import bus_pkg::*;
  import sdmmc_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned MAX_W        = 8,       // 1, 4 or 8 data lanes
  parameter bit          DDR_EN       = 1'b1,    // implement dual-edge (DDR-style) data slots
  parameter bit          DMA_EN       = 1'b1,    // built-in DMA master
  parameter int unsigned FIFO_DEPTH   = 128,     // data FIFO depth in 32-bit words (>= 8)
  parameter bit          CD_ACTIVE_LOW = 1'b1,   // card-detect pin polarity
  parameter bit          WP_ACTIVE_HIGH = 1'b1,  // write-protect pin polarity
  parameter int unsigned DEB_RST      = 200      // reset value of CD_DEB (x256 core clocks)
) (
  input  logic        clk_i,
  input  logic        rst_ni,

  input  csr_req_t    csr_req_i,
  output csr_rsp_t    csr_rsp_o,

  output mem_req_t    dma_req_o,
  input  mem_rsp_t    dma_rsp_i,

  // Pad ring (split in/out/oe)
  output logic        sd_clk_o,
  output logic        sd_cmd_o,
  output logic        sd_cmd_oe_o,
  input  logic        sd_cmd_i,
  output logic [7:0]  sd_dat_o,
  output logic [7:0]  sd_dat_oe_o,
  input  logic [7:0]  sd_dat_i,
  input  logic        sd_cd_i,
  input  logic        sd_wp_i,
  output logic        sd_vsel_o,
  output logic        sd_rst_o,

  // External DMA request lines (level, drop as the FIFO is serviced)
  output logic        dma_rx_req_o,
  output logic        dma_tx_req_o,

  output logic        irq_o
);

  localparam int unsigned LVL_W = $clog2(FIFO_DEPTH + 1);

  // ------------------------------------------------------------------------
  // Registers
  // ------------------------------------------------------------------------
  logic        rw_assert_q, rst_n_drv_q, vsel_q;
  logic [15:0] div_q;
  logic        clk_en_q, auto_gate_q, out_edge_q;
  logic [3:0]  smp_dly_q;
  logic [1:0]  width_sel_q;
  logic        ddr_q;
  logic [11:0] bs_q;
  logic [15:0] bc_q;
  logic [31:0] arg_q, cmd_reg_q;
  logic [15:0] rx_wm_q, tx_wm_q;
  logic [31:0] int_q, int_en_q;
  logic [15:0] cmd_tout_q, cd_deb_q;
  logic [31:0] data_tout_q, busy_tout_q;
  logic [15:0] vt_stop_q, vt_settle_q, vt_chk_q;
  logic [31:0] stop_arg_q;
  logic [15:0] acmd_hi_q;
  logic        dma_en_q, dma_list_q;
  logic [3:0]  dma_burst_q;
  logic [31:0] dma_addr_q;
  logic [31:0] resp_auto_q;
  logic        vsw_done_q, vsw_fail_q;

  // Command latches (sequencer)
  logic [5:0]  q_idx_q;
  logic [31:0] q_arg_q;
  logic [2:0]  q_rtype_q;
  logic        q_data_q, q_read_q, q_multi_q, q_a12_q, q_a23_q, q_dma_q;
  logic [11:0] q_bs_q;
  logic [15:0] q_bc_q;
  logic        q_cmd_ok_q;

  // ------------------------------------------------------------------------
  // Input registers / card detect
  // ------------------------------------------------------------------------
  logic        cmd_in_q;
  logic [7:0]  dat_in_q;
  logic        dat1_s1_q, dat1_s2_q;
  logic        cd_val, wp_val, cd_raw, wp_raw;
  logic        cd_prev_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cmd_in_q  <= 1'b1;
      dat_in_q  <= 8'hFF;
      dat1_s1_q <= 1'b1;
      dat1_s2_q <= 1'b1;
      cd_prev_q <= 1'b0;
    end else begin
      cmd_in_q  <= sd_cmd_i;
      dat_in_q  <= sd_dat_i;
      dat1_s1_q <= sd_dat_i[1];
      dat1_s2_q <= dat1_s1_q;
      cd_prev_q <= cd_val;
    end
  end

  sdmmc_cd_filter #(.ACTIVE_LOW(CD_ACTIVE_LOW)) u_cd (
    .clk_i(clk_i), .rst_ni(rst_ni), .deb_i(cd_deb_q), .pin_i(sd_cd_i), .val_o(cd_val), .raw_o(cd_raw)
  );
  sdmmc_cd_filter #(.ACTIVE_LOW(!WP_ACTIVE_HIGH)) u_wp (
    .clk_i(clk_i), .rst_ni(rst_ni), .deb_i(cd_deb_q), .pin_i(sd_wp_i), .val_o(wp_val), .raw_o(wp_raw)
  );

  // ------------------------------------------------------------------------
  // Clock generator + run / gating policy
  // ------------------------------------------------------------------------
  logic        tog_rise, tog_fall, smp_rise, smp_fall;
  logic        hold;
  logic        seq_busy, cmd_busy, data_busy;
  logic [4:0]  post_q;
  logic        vsw_clk_off, vsw_force_run;
  logic        busy_any;
  logic        clk_run;

  assign busy_any = seq_busy || cmd_busy || data_busy;
  assign clk_run  = clk_en_q && !vsw_clk_off &&
                    (!auto_gate_q || busy_any || (post_q != 5'd0) || vsw_force_run);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) post_q <= 5'd0;
    else if (busy_any) post_q <= 5'd16;
    else if (tog_rise && post_q != 5'd0) post_q <= post_q - 5'd1;
  end

  sdmmc_clkgen u_clkgen (
    .clk_i(clk_i), .rst_ni(rst_ni), .div_i(div_q), .run_i(clk_run), .hold_i(hold),
    .smp_dly_i(smp_dly_q), .sd_clk_o(sd_clk_o),
    .tog_rise_o(tog_rise), .tog_fall_o(tog_fall), .smp_rise_o(smp_rise), .smp_fall_o(smp_fall)
  );

  wire cmd_lnch = out_edge_q ? tog_rise : tog_fall;

  // ------------------------------------------------------------------------
  // Width clamp
  // ------------------------------------------------------------------------
  logic [3:0] width_eff;
  always_comb begin
    if (width_sel_q == 2'd0 || MAX_W == 1)      width_eff = 4'd1;
    else if (width_sel_q == 2'd1 || MAX_W == 4) width_eff = 4'd4;
    else                                        width_eff = 4'd8;
  end

  // ------------------------------------------------------------------------
  // FIFO
  // ------------------------------------------------------------------------
  logic             f_clr, f_push, f_pop, f_full, f_empty;
  logic [31:0]      f_wdata, f_rdata;
  logic [LVL_W-1:0] f_level;
  logic             f_afull_unused, f_aempty_unused;

  sync_fifo #(.DATA_W(32), .DEPTH(FIFO_DEPTH), .FWFT(1'b1)) u_fifo (
    .clk_i(clk_i), .rst_ni(rst_ni), .clr_i(f_clr),
    .push_i(f_push), .wdata_i(f_wdata), .full_o(f_full), .afull_o(f_afull_unused),
    .pop_i(f_pop), .rdata_o(f_rdata), .empty_o(f_empty), .aempty_o(f_aempty_unused),
    .level_o(f_level)
  );

  wire [31:0] f_level32 = 32'(f_level);
  wire        rx_afull  = (f_level32 + 32'd3 >= 32'(FIFO_DEPTH));

  // ------------------------------------------------------------------------
  // Command engine
  // ------------------------------------------------------------------------
  logic        cmd_start, cmd_abort;
  logic [5:0]  ce_idx;
  logic [31:0] ce_arg;
  logic [2:0]  ce_rtype;
  logic        ce_done, ce_tmo, ce_crc, ce_idxe, ce_end, ce_busye;
  logic [127:0] ce_resp;
  logic [5:0]  ce_resp_idx;
  logic [127:0] resp_q;
  logic [5:0]   resp_idx_q;

  sdmmc_cmd_engine u_cmd (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .lnch_i(cmd_lnch), .smp_i(smp_rise),
    .start_i(cmd_start), .abort_i(cmd_abort),
    .idx_i(ce_idx), .arg_i(ce_arg), .rtype_i(ce_rtype),
    .rsp_tout_i(cmd_tout_q), .busy_tout_i(busy_tout_q),
    .cmd_in_i(cmd_in_q), .dat0_in_i(dat_in_q[0]),
    .cmd_o(sd_cmd_o), .cmd_oe_o(sd_cmd_oe_o),
    .busy_o(cmd_busy), .done_o(ce_done),
    .err_tmo_o(ce_tmo), .err_crc_o(ce_crc), .err_idx_o(ce_idxe), .err_end_o(ce_end), .err_busy_o(ce_busye),
    .resp_o(ce_resp), .resp_idx_o(ce_resp_idx)
  );

  // ------------------------------------------------------------------------
  // Data engine
  // ------------------------------------------------------------------------
  logic        de_start, de_abort;
  logic        de_done, de_blk, de_tmo, de_crc, de_end, de_crcs, de_busye, de_ovf;
  logic [15:0] de_blk_cnt;
  logic        de_tx_pop, de_rx_push;
  logic [31:0] de_rx_data;
  logic [7:0]  de_dat_o, de_dat_oe;
  logic        de_busy;

  sdmmc_data_engine #(.MAX_W(MAX_W), .DDR_EN(DDR_EN)) u_data (
    .clk_i(clk_i), .rst_ni(rst_ni),
    .lnch_rise_i(tog_rise), .lnch_fall_i(tog_fall), .smp_rise_i(smp_rise), .smp_fall_i(smp_fall),
    .out_edge_i(out_edge_q),
    .start_i(de_start), .abort_i(de_abort), .rd_i(q_read_q),
    .bs_i(q_bs_q), .bc_i(q_bc_q), .width_i(width_eff), .ddr_i(ddr_q),
    .dtout_i(data_tout_q), .btout_i(busy_tout_q),
    .dat_in_i(dat_in_q), .dat_o(de_dat_o), .dat_oe_o(de_dat_oe),
    .tx_data_i(f_rdata), .tx_empty_i(f_empty), .tx_pop_o(de_tx_pop),
    .rx_push_o(de_rx_push), .rx_data_o(de_rx_data), .rx_full_i(f_full), .rx_afull_i(rx_afull),
    .hold_o(hold), .busy_o(de_busy), .done_o(de_done), .blk_done_o(de_blk), .blk_cnt_o(de_blk_cnt),
    .err_tmo_o(de_tmo), .err_crc_o(de_crc), .err_end_o(de_end), .err_crcs_o(de_crcs),
    .err_busy_o(de_busye), .err_ovf_o(de_ovf)
  );

  assign data_busy = de_busy;

  // SDIO read-wait: hold DAT2 low while no data transfer is active
  wire rw_active = rw_assert_q && !de_busy;
  assign sd_dat_o    = {de_dat_o[7:3], (rw_active ? 1'b0 : de_dat_o[2]), de_dat_o[1:0]};
  assign sd_dat_oe_o = de_dat_oe | (rw_active ? 8'h04 : 8'h00);

  // ------------------------------------------------------------------------
  // DMA master
  // ------------------------------------------------------------------------
  logic        dma_start, dma_abort;
  logic        dm_busy, dm_done, dm_seg_irq, dm_err, dm_starved, dm_pop, dm_push;
  logic [31:0] dm_wdata, dm_addr;
  logic [31:0] dma_words;

  if (DMA_EN) begin : gen_dma
    sdmmc_dma #(.FIFO_DEPTH(FIFO_DEPTH)) u_dma (
      .clk_i(clk_i), .rst_ni(rst_ni),
      .start_i(dma_start), .abort_i(dma_abort), .to_mem_i(q_read_q), .list_i(dma_list_q),
      .addr_i(dma_addr_q), .words_i(dma_words), .burst_i(dma_burst_q),
      .f_level_i(16'(f_level)), .f_rdata_i(f_rdata), .f_pop_o(dm_pop), .f_push_o(dm_push), .f_wdata_o(dm_wdata),
      .mem_req_o(dma_req_o), .mem_rsp_i(dma_rsp_i),
      .busy_o(dm_busy), .done_o(dm_done), .seg_irq_o(dm_seg_irq), .err_o(dm_err),
      .starved_o(dm_starved), .addr_o(dm_addr)
    );
  end else begin : gen_no_dma
    assign dma_req_o  = '0;
    assign dm_busy    = 1'b0;
    assign dm_done    = 1'b0;
    assign dm_seg_irq = 1'b0;
    assign dm_err     = 1'b0;
    assign dm_starved = 1'b0;
    assign dm_pop     = 1'b0;
    assign dm_push    = 1'b0;
    assign dm_wdata   = '0;
    assign dm_addr    = '0;
  end

  assign dma_words = (q_bc_q == 16'd0) ? 32'd0 : (32'(q_bc_q) * 32'(q_bs_q[11:2]));

  // ------------------------------------------------------------------------
  // CSR decode
  // ------------------------------------------------------------------------
  wire        csr_wr   = csr_req_i.valid && csr_req_i.write;
  wire        csr_rd   = csr_req_i.valid && !csr_req_i.write;
  wire [5:0]  reg_idx  = csr_req_i.addr[7:2];
  wire [31:0] wd       = csr_req_i.wdata;
  wire [3:0]  ws       = csr_req_i.wstrb;

  localparam logic [5:0] R_ID = 6'h00, R_CAPS = 6'h01, R_CTRL = 6'h02, R_CLK = 6'h03, R_BUS = 6'h04,
                         R_BLK = 6'h05, R_ARG = 6'h06, R_CMD = 6'h07, R_RESP0 = 6'h08, R_RESP1 = 6'h09,
                         R_RESP2 = 6'h0A, R_RESP3 = 6'h0B, R_RAUTO = 6'h0C, R_STAT = 6'h0D,
                         R_FCTRL = 6'h0E, R_FLVL = 6'h0F, R_DATA = 6'h10, R_INT = 6'h11, R_INTEN = 6'h12,
                         R_CTMO = 6'h13, R_DTMO = 6'h14, R_BTMO = 6'h15, R_DEB = 6'h16, R_VSW = 6'h17,
                         R_VT1 = 6'h18, R_VT2 = 6'h19, R_STOP = 6'h1A, R_ACFG = 6'h1B, R_DMAC = 6'h1C,
                         R_DMAA = 6'h1D, R_DMAS = 6'h1E, R_BDONE = 6'h1F;

  // Self-clearing control pulses
  wire ctl_wr       = csr_wr && (reg_idx == R_CTRL) && ws[0];
  wire sw_rst       = ctl_wr && wd[0];
  wire fifo_flush_w = ctl_wr && wd[1];
  wire cmd_abort_w  = ctl_wr && wd[2];
  wire data_abort_w = ctl_wr && wd[3];
  wire cmd_wr       = csr_wr && (reg_idx == R_CMD);
  wire vsw_start_w  = csr_wr && (reg_idx == R_VSW) && ws[0] && wd[0];

  // CPU FIFO port
  wire cpu_data_wr  = csr_wr && (reg_idx == R_DATA);
  wire cpu_data_rd  = csr_rd && (reg_idx == R_DATA);
  wire cpu_push     = cpu_data_wr && !q_dma_active_w && !f_full;
  wire cpu_pop      = cpu_data_rd && !q_dma_active_w && !f_empty;
  logic q_dma_active_w;
  wire fifo_err_cpu = (cpu_data_wr && f_full && !q_dma_active_w) ||
                      (cpu_data_rd && f_empty && !q_dma_active_w);

  assign f_push  = de_rx_push || dm_push || cpu_push;
  assign f_wdata = de_rx_push ? de_rx_data : (dm_push ? dm_wdata : wd);
  assign f_pop   = de_tx_pop || dm_pop || cpu_pop;

  // ------------------------------------------------------------------------
  // Sequencer
  // ------------------------------------------------------------------------
  typedef enum logic [3:0] {
    Q_IDLE, Q_C23, Q_C23_W, Q_MAIN, Q_MAIN_W, Q_DSTART, Q_DATA, Q_C12, Q_C12_W, Q_DMAW, Q_FIN
  } qstate_e;
  qstate_e q_st_q;
  logic    q_dma_run_q;     // DMA owns the FIFO for the current transfer
  logic    flush_req_q;
  logic [2:0] settle_q;
  logic    seq_data_phase;
  logic    q_ctx_auto_q;    // last command engine job was an auto command (CMD23/12)
  logic    data_err_q;

  assign q_dma_active_w = q_dma_run_q;
  assign seq_busy = (q_st_q != Q_IDLE);
  assign seq_data_phase = (q_st_q == Q_DSTART) || (q_st_q == Q_DATA) || (q_st_q == Q_DMAW) ||
                          (q_st_q == Q_MAIN_W) || (q_st_q == Q_MAIN);

  wire cmd_wr_ok  = cmd_wr && !seq_busy;
  wire cmd_rejected = cmd_wr && seq_busy;

  wire de_any_err = de_tmo || de_crc || de_end || de_crcs || de_busye || de_ovf;
  wire ce_any_err = ce_tmo || ce_crc || ce_idxe || ce_end || ce_busye;

  // command engine job selection
  always_comb begin
    cmd_start = 1'b0;
    ce_idx    = q_idx_q;
    ce_arg    = q_arg_q;
    ce_rtype  = q_rtype_q;
    unique case (q_st_q)
      Q_C23: begin
        cmd_start = 1'b1;
        ce_idx    = 6'd23;
        ce_arg    = {acmd_hi_q, q_bc_q};
        ce_rtype  = RSP_R48;
      end
      Q_MAIN: cmd_start = 1'b1;
      Q_C12: begin
        cmd_start = 1'b1;
        ce_idx    = 6'd12;
        ce_arg    = stop_arg_q;
        ce_rtype  = RSP_R48B;
      end
      default: ;
    endcase
  end

  wire de_abort_dm  = (q_st_q == Q_DATA) && dm_err;
  wire cmd_abort_dm = (q_st_q == Q_MAIN_W) && dm_err;

  assign dma_start = (q_st_q == Q_MAIN) && q_data_q && q_dma_q && DMA_EN && dma_en_q;
  assign de_start  = (q_st_q == Q_DSTART);
  assign cmd_abort = cmd_abort_w || sw_rst || cmd_abort_dm;
  assign de_abort  = data_abort_w || sw_rst || de_abort_dm;
  assign dma_abort = data_abort_w || sw_rst || dma_abort_seq;
  logic dma_abort_seq;

  // interrupt event vector
  logic [31:0] int_set;
  logic        sdio_lvl;
  logic        fin_data_done;
  assign sdio_lvl = !dat1_s2_q && !de_busy;

  always_comb begin
    int_set = '0;
    if (ce_done) begin
      int_set[0]  = !q_ctx_auto_q;
      int_set[16] = ce_tmo;
      int_set[17] = ce_crc;
      int_set[18] = ce_idxe;
      int_set[19] = ce_end;
      int_set[24] = ce_busye;
      int_set[27] = q_ctx_auto_q && ce_any_err;
    end
    if (de_done) begin
      int_set[20] = de_tmo;
      int_set[21] = de_crc;
      int_set[22] = de_end;
      int_set[23] = de_crcs;
      int_set[24] = int_set[24] || de_busye;
      int_set[26] = de_ovf;
    end
    int_set[1]  = fin_data_done;
    int_set[2]  = de_blk;
    int_set[3]  = dm_done || dm_seg_irq;
    int_set[4]  = cd_val && !cd_prev_q;
    int_set[5]  = !cd_val && cd_prev_q;
    int_set[7]  = vsw_end_ok;
    int_set[25] = dm_err;
    int_set[26] = int_set[26] || fifo_err_cpu;
    int_set[28] = vsw_end_fail;
    int_set[29] = cmd_rejected;
  end

  logic vsw_end_ok, vsw_end_fail;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_st_q       <= Q_IDLE;
      q_idx_q      <= '0;
      q_arg_q      <= '0;
      q_rtype_q    <= '0;
      q_data_q     <= 1'b0;
      q_read_q     <= 1'b0;
      q_multi_q    <= 1'b0;
      q_a12_q      <= 1'b0;
      q_a23_q      <= 1'b0;
      q_dma_q      <= 1'b0;
      q_bs_q       <= 12'd512;
      q_bc_q       <= '0;
      q_cmd_ok_q   <= 1'b0;
      q_dma_run_q  <= 1'b0;
      q_ctx_auto_q <= 1'b0;
      data_err_q   <= 1'b0;
      flush_req_q  <= 1'b0;
      settle_q     <= '0;
      resp_q       <= '0;
      resp_idx_q   <= '0;
      resp_auto_q  <= '0;
      fin_data_done <= 1'b0;
      dma_abort_seq <= 1'b0;
      f_clr        <= 1'b0;
    end else begin
      fin_data_done <= 1'b0;
      dma_abort_seq <= 1'b0;
      f_clr         <= 1'b0;

      // FIFO flush (waits until the DMA master is quiescent)
      if (fifo_flush_w || sw_rst) flush_req_q <= 1'b1;
      if (flush_req_q && !dm_busy && !dma_abort) begin
        f_clr       <= 1'b1;
        flush_req_q <= 1'b0;
      end

      if (ce_done) begin
        resp_idx_q <= ce_resp_idx;
        if (q_ctx_auto_q) resp_auto_q <= ce_resp[31:0];
        else              resp_q      <= ce_resp;
      end

      if (sw_rst || data_abort_w) begin
        q_st_q      <= Q_IDLE;
        q_dma_run_q <= 1'b0;
      end else if (cmd_abort_w && (q_st_q == Q_C23 || q_st_q == Q_C23_W || q_st_q == Q_MAIN ||
                                   q_st_q == Q_MAIN_W || q_st_q == Q_C12 || q_st_q == Q_C12_W)) begin
        q_st_q <= Q_IDLE;
        if (q_dma_run_q) begin
          dma_abort_seq <= 1'b1;
          flush_req_q   <= 1'b1;
          q_dma_run_q   <= 1'b0;
        end
      end else begin
        unique case (q_st_q)
          Q_IDLE: begin
            if (cmd_wr_ok) begin
              q_idx_q     <= wd[5:0];
              q_rtype_q   <= wd[8:6];
              q_arg_q     <= arg_q;
              q_data_q    <= wd[9];
              q_read_q    <= wd[10];
              q_multi_q   <= wd[11];
              q_a12_q     <= wd[12];
              q_a23_q     <= wd[13];
              q_dma_q     <= wd[14] && DMA_EN && dma_en_q && wd[9];
              q_bs_q      <= (bs_q == 12'd0) ? 12'd1 : ((bs_q > 12'd2048) ? 12'd2048 : bs_q);
              q_bc_q      <= wd[11] ? bc_q : 16'd1;
              q_cmd_ok_q  <= 1'b0;
              data_err_q  <= 1'b0;
              q_dma_run_q <= 1'b0;
              q_st_q      <= (wd[13] && wd[9] && wd[11]) ? Q_C23 : Q_MAIN;
            end
          end

          Q_C23: begin
            q_ctx_auto_q <= 1'b1;
            q_st_q       <= Q_C23_W;
          end

          Q_C23_W: begin
            if (ce_done) begin
              q_st_q <= ce_any_err ? Q_FIN : Q_MAIN;
              if (ce_any_err) q_data_q <= 1'b0;   // no data phase
            end
          end

          Q_MAIN: begin
            q_ctx_auto_q <= 1'b0;
            if (dma_start) q_dma_run_q <= 1'b1;
            q_st_q <= Q_MAIN_W;
          end

          Q_MAIN_W: begin
            if (dm_err) begin
              dma_abort_seq <= 1'b1;
              q_st_q        <= Q_FIN;
              q_cmd_ok_q    <= 1'b0;
            end
            if (ce_done) begin
              if (ce_any_err) begin
                q_cmd_ok_q <= 1'b0;
                if (q_dma_run_q) begin
                  dma_abort_seq <= 1'b1;
                  flush_req_q   <= 1'b1;
                end
                q_st_q <= Q_FIN;
              end else begin
                q_cmd_ok_q <= 1'b1;
                q_st_q     <= q_data_q ? Q_DSTART : Q_FIN;
              end
            end
          end

          Q_DSTART: q_st_q <= Q_DATA;

          Q_DATA: begin
            if (dm_err) begin
              dma_abort_seq <= 1'b1;
              flush_req_q   <= 1'b1;
              data_err_q    <= 1'b1;
              q_st_q        <= Q_FIN;
            end else if (de_done) begin
              data_err_q <= de_any_err;
              settle_q   <= 3'd4;
              if (q_a12_q && q_multi_q && !q_a23_q) begin
                q_ctx_auto_q <= 1'b1;
                q_st_q       <= Q_C12;
              end else begin
                q_st_q <= Q_DMAW;
              end
            end
          end

          Q_C12: begin
            q_st_q <= Q_C12_W;
          end

          Q_C12_W: begin
            if (ce_done) q_st_q <= Q_DMAW;
          end

          Q_DMAW: begin
            if (settle_q != 3'd0) settle_q <= settle_q - 3'd1;
            if (!q_dma_run_q || !dm_busy && settle_q == 3'd0) begin
              q_st_q <= Q_FIN;
            end else if (data_err_q || !q_read_q) begin
              // error or card-write: stop the DMA between bursts and discard leftovers
              dma_abort_seq <= 1'b1;
              flush_req_q   <= 1'b1;
              if (!dm_busy) q_st_q <= Q_FIN;
            end else if (settle_q == 3'd0 && dm_starved && f_empty) begin
              // card-read finished but the DMA buffer list wants more data
              dma_abort_seq <= 1'b1;
              flush_req_q   <= 1'b1;
              data_err_q    <= 1'b1;
              q_st_q        <= Q_FIN;
            end
          end

          Q_FIN: begin
            fin_data_done <= q_data_q && q_cmd_ok_q;
            q_dma_run_q   <= 1'b0;
            q_st_q        <= Q_IDLE;
          end

          default: q_st_q <= Q_IDLE;
        endcase
      end
    end
  end

  // ------------------------------------------------------------------------
  // Voltage switch sequencer
  // ------------------------------------------------------------------------
  typedef enum logic [2:0] { V_IDLE, V_STOP, V_SETTLE, V_CHK } vstate_e;
  vstate_e     v_st_q;
  logic [7:0]  vpre_q;
  logic [15:0] vcnt_q;
  wire         vtick = (vpre_q == 8'hFF);

  assign vsw_clk_off   = (v_st_q == V_STOP) || (v_st_q == V_SETTLE);
  assign vsw_force_run = (v_st_q == V_CHK);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      v_st_q       <= V_IDLE;
      vpre_q       <= '0;
      vcnt_q       <= '0;
      vsw_end_ok   <= 1'b0;
      vsw_end_fail <= 1'b0;
      vsw_done_q   <= 1'b0;
      vsw_fail_q   <= 1'b0;
      vsel_q       <= 1'b0;
    end else begin
      vsw_end_ok   <= 1'b0;
      vsw_end_fail <= 1'b0;
      vpre_q       <= vpre_q + 8'd1;
      // sticky flags (W1C through CSR below)
      if (csr_wr && reg_idx == R_VSW && ws[0]) begin
        if (wd[1]) vsw_done_q <= 1'b0;
        if (wd[2]) vsw_fail_q <= 1'b0;
      end
      // CTRL.VSEL written by software
      if (csr_wr && reg_idx == R_CTRL && ws[0]) vsel_q <= wd[6];

      unique case (v_st_q)
        V_IDLE: begin
          if (vsw_start_w && !seq_busy) begin
            vcnt_q     <= '0;
            vsw_done_q <= 1'b0;
            vsw_fail_q <= 1'b0;
            v_st_q     <= V_STOP;
          end
        end
        V_STOP: begin
          if (vtick) begin
            if (vcnt_q >= vt_stop_q) begin
              if (dat_in_q[3:0] == 4'h0) begin
                vsel_q <= 1'b1;
                vcnt_q <= '0;
                v_st_q <= V_SETTLE;
              end else begin
                vsw_fail_q   <= 1'b1;
                vsw_end_fail <= 1'b1;
                v_st_q       <= V_IDLE;
              end
            end else begin
              vcnt_q <= vcnt_q + 16'd1;
            end
          end
        end
        V_SETTLE: begin
          if (vtick) begin
            if (vcnt_q >= vt_settle_q) begin
              vcnt_q <= '0;
              v_st_q <= V_CHK;
            end else begin
              vcnt_q <= vcnt_q + 16'd1;
            end
          end
        end
        V_CHK: begin
          if (dat_in_q[3:0] == 4'hF) begin
            vsw_done_q <= 1'b1;
            vsw_end_ok <= 1'b1;
            v_st_q     <= V_IDLE;
          end else if (vtick) begin
            if (vcnt_q >= vt_chk_q) begin
              vsw_fail_q   <= 1'b1;
              vsw_end_fail <= 1'b1;
              v_st_q       <= V_IDLE;
            end else begin
              vcnt_q <= vcnt_q + 16'd1;
            end
          end
        end
        default: v_st_q <= V_IDLE;
      endcase
    end
  end

  // ------------------------------------------------------------------------
  // CSR registers
  // ------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rw_assert_q <= 1'b0;
      rst_n_drv_q <= 1'b1;
      div_q       <= 16'd255;
      clk_en_q    <= 1'b0;
      auto_gate_q <= 1'b1;
      out_edge_q  <= 1'b0;
      smp_dly_q   <= 4'd0;
      width_sel_q <= 2'd0;
      ddr_q       <= 1'b0;
      bs_q        <= 12'd512;
      bc_q        <= 16'd1;
      arg_q       <= '0;
      cmd_reg_q   <= '0;
      rx_wm_q     <= 16'd1;
      tx_wm_q     <= 16'd1;
      int_q       <= '0;
      int_en_q    <= '0;
      cmd_tout_q  <= 16'd255;
      data_tout_q <= 32'h00FF_FFFF;
      busy_tout_q <= 32'h00FF_FFFF;
      cd_deb_q    <= 16'(DEB_RST);
      vt_stop_q   <= 16'd20;
      vt_settle_q <= 16'd40;
      vt_chk_q    <= 16'd4;
      stop_arg_q  <= '0;
      acmd_hi_q   <= '0;
      dma_en_q    <= 1'b0;
      dma_list_q  <= 1'b0;
      dma_burst_q <= 4'd15;
      dma_addr_q  <= '0;
    end else begin
      int_q <= (int_q & ~(csr_wr && reg_idx == R_INT ? wd : 32'd0)) | int_set;
      if (sw_rst) int_q <= int_set;
      if (csr_wr) begin
        unique case (reg_idx)
          R_CTRL: begin
            if (ws[0]) begin
              rw_assert_q <= wd[4];
              rst_n_drv_q <= wd[5];
            end
          end
          R_CLK: begin
            if (ws[0]) div_q[7:0]  <= wd[7:0];
            if (ws[1]) div_q[15:8] <= wd[15:8];
            if (ws[2]) begin
              clk_en_q    <= wd[16];
              auto_gate_q <= wd[17];
              out_edge_q  <= wd[18];
              smp_dly_q   <= wd[23:20];
            end
          end
          R_BUS: if (ws[0]) begin
            width_sel_q <= wd[1:0];
            ddr_q       <= wd[2];
          end
          R_BLK: begin
            if (ws[0]) bs_q[7:0]  <= wd[7:0];
            if (ws[1]) bs_q[11:8] <= wd[11:8];
            if (ws[2]) bc_q[7:0]  <= wd[23:16];
            if (ws[3]) bc_q[15:8] <= wd[31:24];
          end
          R_ARG:   arg_q <= apply_wstrb(arg_q, wd, ws);
          R_CMD:   cmd_reg_q <= apply_wstrb(cmd_reg_q, wd, ws);
          R_FCTRL: begin
            if (ws[0]) rx_wm_q[7:0]  <= wd[7:0];
            if (ws[1]) rx_wm_q[15:8] <= wd[15:8];
            if (ws[2]) tx_wm_q[7:0]  <= wd[23:16];
            if (ws[3]) tx_wm_q[15:8] <= wd[31:24];
          end
          R_INTEN: int_en_q <= apply_wstrb(int_en_q, wd, ws);
          R_CTMO:  begin
            if (ws[0]) cmd_tout_q[7:0]  <= wd[7:0];
            if (ws[1]) cmd_tout_q[15:8] <= wd[15:8];
          end
          R_DTMO:  data_tout_q <= apply_wstrb(data_tout_q, wd, ws);
          R_BTMO:  busy_tout_q <= apply_wstrb(busy_tout_q, wd, ws);
          R_DEB:   begin
            if (ws[0]) cd_deb_q[7:0]  <= wd[7:0];
            if (ws[1]) cd_deb_q[15:8] <= wd[15:8];
          end
          R_VT1: begin
            if (ws[0]) vt_stop_q[7:0]    <= wd[7:0];
            if (ws[1]) vt_stop_q[15:8]   <= wd[15:8];
            if (ws[2]) vt_settle_q[7:0]  <= wd[23:16];
            if (ws[3]) vt_settle_q[15:8] <= wd[31:24];
          end
          R_VT2: begin
            if (ws[0]) vt_chk_q[7:0]  <= wd[7:0];
            if (ws[1]) vt_chk_q[15:8] <= wd[15:8];
          end
          R_STOP: stop_arg_q <= apply_wstrb(stop_arg_q, wd, ws);
          R_ACFG: begin
            if (ws[2]) acmd_hi_q[7:0]  <= wd[23:16];
            if (ws[3]) acmd_hi_q[15:8] <= wd[31:24];
          end
          R_DMAC: if (ws[0]) begin
            dma_en_q    <= wd[0] && DMA_EN;
            dma_list_q  <= wd[1];
            dma_burst_q <= wd[7:4];
          end
          R_DMAA: dma_addr_q <= apply_wstrb(dma_addr_q, wd, ws);
          default: ;
        endcase
      end
    end
  end

  assign sd_vsel_o = vsel_q;
  assign sd_rst_o  = rst_n_drv_q;

  // ------------------------------------------------------------------------
  // Read mux
  // ------------------------------------------------------------------------
  logic [31:0] int_rd;
  logic [31:0] status_rd;
  wire         rx_wm_hit = q_read_q  && seq_data_phase && !f_empty && (f_level32 >= 32'(rx_wm_q));
  wire         tx_wm_hit = !q_read_q && seq_data_phase && ((32'(FIFO_DEPTH) - f_level32) >= 32'(tx_wm_q));

  always_comb begin
    int_rd    = int_q;
    int_rd[6] = sdio_lvl;
    int_rd[8] = rx_wm_hit;
    int_rd[9] = tx_wm_hit;
    status_rd = '0;
    status_rd[0]     = seq_busy;
    status_rd[1]     = cmd_busy;
    status_rd[2]     = de_busy;
    status_rd[3]     = cd_val;
    status_rd[4]     = wp_val;
    status_rd[5]     = dat_in_q[0];
    status_rd[6]     = cmd_in_q;
    status_rd[7]     = clk_run;
    status_rd[11:8]  = dat_in_q[3:0];
    status_rd[12]    = f_empty;
    status_rd[13]    = f_full;
    status_rd[14]    = dm_busy;
    status_rd[15]    = (v_st_q != V_IDLE);
    status_rd[16]    = sdio_lvl;
    status_rd[17]    = vsel_q;
    status_rd[29:24] = resp_idx_q;
  end

  assign irq_o = |(int_rd & int_en_q);
  assign dma_rx_req_o = rx_wm_hit && !q_dma_run_q;
  assign dma_tx_req_o = tx_wm_hit && !q_dma_run_q;

  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_idx)
      R_ID:    csr_rsp_o.rdata = 32'h5D0C_0100;
      R_CAPS:  csr_rsp_o.rdata = {16'(FIFO_DEPTH), 8'd0, 2'b00, DMA_EN ? 1'b1 : 1'b0, DDR_EN ? 1'b1 : 1'b0,
                                  1'b0, (MAX_W >= 8) ? 1'b1 : 1'b0, (MAX_W >= 4) ? 1'b1 : 1'b0, 1'b1};
      R_CTRL:  csr_rsp_o.rdata = {25'd0, vsel_q, rst_n_drv_q, rw_assert_q, 4'd0};
      R_CLK:   csr_rsp_o.rdata = {8'd0, smp_dly_q, 1'b0, out_edge_q, auto_gate_q, clk_en_q, div_q};
      R_BUS:   csr_rsp_o.rdata = {29'd0, ddr_q, width_sel_q};
      R_BLK:   csr_rsp_o.rdata = {bc_q, 4'd0, bs_q};
      R_ARG:   csr_rsp_o.rdata = arg_q;
      R_CMD:   csr_rsp_o.rdata = cmd_reg_q;
      R_RESP0: csr_rsp_o.rdata = resp_q[31:0];
      R_RESP1: csr_rsp_o.rdata = resp_q[63:32];
      R_RESP2: csr_rsp_o.rdata = resp_q[95:64];
      R_RESP3: csr_rsp_o.rdata = resp_q[127:96];
      R_RAUTO: csr_rsp_o.rdata = resp_auto_q;
      R_STAT:  csr_rsp_o.rdata = status_rd;
      R_FCTRL: csr_rsp_o.rdata = {tx_wm_q, rx_wm_q};
      R_FLVL:  csr_rsp_o.rdata = {14'd0, f_full, f_empty, 16'(f_level)};
      R_DATA:  csr_rsp_o.rdata = (f_empty || q_dma_run_q) ? 32'd0 : f_rdata;
      R_INT:   csr_rsp_o.rdata = int_rd;
      R_INTEN: csr_rsp_o.rdata = int_en_q;
      R_CTMO:  csr_rsp_o.rdata = {16'd0, cmd_tout_q};
      R_DTMO:  csr_rsp_o.rdata = data_tout_q;
      R_BTMO:  csr_rsp_o.rdata = busy_tout_q;
      R_DEB:   csr_rsp_o.rdata = {16'd0, cd_deb_q};
      R_VSW:   csr_rsp_o.rdata = {29'd0, vsw_fail_q, vsw_done_q, (v_st_q != V_IDLE)};
      R_VT1:   csr_rsp_o.rdata = {vt_settle_q, vt_stop_q};
      R_VT2:   csr_rsp_o.rdata = {16'd0, vt_chk_q};
      R_STOP:  csr_rsp_o.rdata = stop_arg_q;
      R_ACFG:  csr_rsp_o.rdata = {acmd_hi_q, 16'd0};
      R_DMAC:  csr_rsp_o.rdata = {24'd0, dma_burst_q, 2'b00, dma_list_q, dma_en_q};
      R_DMAA:  csr_rsp_o.rdata = dm_busy ? dm_addr : dma_addr_q;
      R_DMAS:  csr_rsp_o.rdata = {30'd0, dm_starved, dm_busy};
      R_BDONE: csr_rsp_o.rdata = {16'd0, de_blk_cnt};
      default: ;
    endcase
  end

endmodule : sdmmc_core
