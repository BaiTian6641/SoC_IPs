// SPDX-License-Identifier: Apache-2.0
// trng_cond.sv - Digital TRNG Health-Test (NIST SP 800-90B RCT + APT) & 256-bit Sponge Conditioner
//
// External analog/RO entropy source plugs into (ent_valid_i, ent_data_i[3:0], ent_enable_o).
// When ENT_SOFT_EMU == 1, an optional non-cryptographic LFSR stub (trng_ent_soft) is included
// and selected whenever ext_ent_sel_i == 0.

module trng_cond
  import otp_trng_keymgr_pkg::*;
#(
  parameter bit          ENT_SOFT_EMU   = 1'b1,
  parameter int unsigned RNG_FIFO_DEPTH = 16,
  localparam int unsigned LVL_W         = $clog2(RNG_FIFO_DEPTH + 1)
) (
  input  logic             clk_i,
  input  logic             rst_ni,

  // External Entropy-Source Macro Boundary
  input  logic             ent_valid_i,
  input  logic [3:0]       ent_data_i,
  output logic             ent_enable_o,

  // CSR Control & Config
  input  logic             en_i,
  input  logic             ext_ent_sel_i,
  input  logic             apt_win64_i,    // 0: W=512, 1: W=64
  input  logic [3:0]       absorb_blks_i,  // 64-bit blocks absorbed per 64-bit squeeze (0 -> 1)
  input  logic [3:0]       dma_wm_i,       // FIFO level threshold for dma_req_o (0 -> 1)
  input  logic [15:0]      rct_cutoff_i,
  input  logic [9:0]       apt_cutoff_i,
  input  logic             cmd_clr_health_i,
  input  logic             cmd_flush_i,
  input  logic             wipe_i,
  input  logic             csr_rand_rd_i,

  // CSR Status & Readback
  output logic [31:0]      rand_data_o,
  output logic [LVL_W-1:0] fifo_level_o,
  output logic             fifo_empty_o,
  output logic             fifo_full_o,
  output logic             health_fail_o,
  output logic             rct_fail_o,
  output logic             apt_fail_o,
  output logic             cond_busy_o,
  output logic [7:0]       sample_cnt_o,
  output logic [15:0]      rct_cnt_o,
  output logic [9:0]       apt_cnt_o,
  output logic [9:0]       apt_idx_o,

  // Event Pulses (1 cycle)
  output logic             health_fail_p_o,
  output logic             rand_unf_p_o,

  // Hardware Random Output Port & DMA Request
  output logic             rng_valid_o,
  output logic [31:0]      rng_data_o,
  input  logic             rng_ready_i,
  output logic             dma_req_o
);

  typedef enum logic [1:0] {
    COND_ABSORB  = 2'd0,
    COND_PERMUTE = 2'd1,
    COND_SQUEEZE0= 2'd2,
    COND_SQUEEZE1= 2'd3
  } cond_fsm_e;

  cond_fsm_e cond_st_q;

  // Health test registers
  logic        rct_init_q;
  logic [3:0]  rct_prev_q;
  logic [15:0] rct_cnt_q;
  logic [3:0]  apt_ref_q;
  logic [9:0]  apt_cnt_q;
  logic [9:0]  apt_idx_q;
  logic        rct_fail_q, apt_fail_q;
  logic [7:0]  sample_cnt_q;

  wire health_fail_q = rct_fail_q || apt_fail_q;

  // Sponge conditioner registers
  logic [255:0] sponge_q;
  logic [63:0]  absorb_buf_q;
  logic [3:0]   nib_cnt_q;
  logic [3:0]   blk_cnt_q;
  logic [3:0]   rnd_cnt_q;
  logic [31:0]  reseed_cnt_q;

  // FIFO signals
  logic             fifo_push, fifo_pop, fifo_clr;
  logic [31:0]      fifo_wdata, fifo_rdata;
  logic             fifo_full, fifo_empty;
  logic [LVL_W-1:0] fifo_level;

  // Entropy source selection
  logic       src_valid;
  logic [3:0] src_data;
  logic       soft_en;

  if (ENT_SOFT_EMU) begin : gen_soft_ent
    logic       soft_valid;
    logic [3:0] soft_data;

    assign soft_en = en_i && !ext_ent_sel_i && !health_fail_q && !fifo_full;

    trng_ent_soft u_ent_soft (
      .clk_i        (clk_i),
      .rst_ni       (rst_ni),
      .ent_enable_i (soft_en),
      .ent_valid_o  (soft_valid),
      .ent_data_o   (soft_data)
    );

    assign src_valid    = ext_ent_sel_i ? ent_valid_i : soft_valid;
    assign src_data     = ext_ent_sel_i ? ent_data_i  : soft_data;
    assign ent_enable_o = en_i && ext_ent_sel_i && !health_fail_q && !fifo_full;
  end else begin : gen_ext_ent
    assign soft_en      = 1'b0;
    assign src_valid    = ent_valid_i;
    assign src_data     = ent_data_i;
    assign ent_enable_o = en_i && !health_fail_q && !fifo_full;
    wire unused_ext_sel = ext_ent_sel_i;
  end

  // Health test combinational trip detection (prevents bad samples reaching FIFO)
  wire [15:0] rct_cut_eff = (rct_cutoff_i < 16'd2) ? 16'd2 : rct_cutoff_i;
  wire [9:0]  apt_cut_eff = (apt_cutoff_i < 10'd2) ? 10'd2 : apt_cutoff_i;
  wire [9:0]  apt_win_eff = apt_win64_i ? 10'd64 : 10'd512;

  wire        sample_active = en_i && !health_fail_q && src_valid;
  wire [15:0] next_rct_cnt  = (rct_init_q && (src_data == rct_prev_q)) ? (rct_cnt_q + 16'd1) : 16'd1;
  wire        rct_trip      = sample_active && rct_init_q && (src_data == rct_prev_q) && (next_rct_cnt >= rct_cut_eff);

  wire [9:0]  next_apt_cnt  = (apt_idx_q == 10'd0) ? 10'd1 :
                              (apt_cnt_q + ((src_data == apt_ref_q) ? 10'd1 : 10'd0));
  wire        apt_trip      = sample_active && (apt_idx_q != 10'd0) && (next_apt_cnt >= apt_cut_eff);
  wire        any_trip      = rct_trip || apt_trip;

  // Output FIFO (depth 16, 32-bit FWFT)
  assign fifo_clr = wipe_i || cmd_flush_i || any_trip;

  wire csr_rand_pop = csr_rand_rd_i && !fifo_empty && !health_fail_q;
  assign rng_valid_o = !fifo_empty && !health_fail_q && !csr_rand_rd_i;
  assign rng_data_o  = (!fifo_empty && !health_fail_q) ? fifo_rdata : 32'd0;
  assign fifo_pop    = csr_rand_pop || (rng_valid_o && rng_ready_i);

  assign rand_data_o  = csr_rand_pop ? fifo_rdata : 32'd0;
  assign rand_unf_p_o = csr_rand_rd_i && (fifo_empty || health_fail_q);

  sync_fifo #(
    .DATA_W (32),
    .DEPTH  (RNG_FIFO_DEPTH),
    .FWFT   (1'b1)
  ) u_rng_fifo (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .clr_i    (fifo_clr),
    .push_i   (fifo_push),
    .wdata_i  (fifo_wdata),
    .full_o   (fifo_full),
    .afull_o  (),
    .pop_i    (fifo_pop),
    .rdata_o  (fifo_rdata),
    .empty_o  (fifo_empty),
    .aempty_o (),
    .level_o  (fifo_level)
  );

  // FIFO push during squeeze states
  always_comb begin
    fifo_push  = 1'b0;
    fifo_wdata = 32'd0;
    if (!health_fail_q && !any_trip && !fifo_full) begin
      if (cond_st_q == COND_SQUEEZE0) begin
        fifo_push  = 1'b1;
        fifo_wdata = sponge_q[31:0];
      end else if (cond_st_q == COND_SQUEEZE1) begin
        fifo_push  = 1'b1;
        fifo_wdata = sponge_q[63:32];
      end
    end
  end

  // Status & DMA request
  wire [LVL_W-1:0] dma_wm_eff = (dma_wm_i == 4'd0) ? LVL_W'(1) : LVL_W'(dma_wm_i);
  assign dma_req_o     = !health_fail_q && (fifo_level >= dma_wm_eff);
  assign fifo_level_o  = fifo_level;
  assign fifo_empty_o  = fifo_empty;
  assign fifo_full_o   = fifo_full;
  assign health_fail_o = health_fail_q;
  assign rct_fail_o    = rct_fail_q;
  assign apt_fail_o    = apt_fail_q;
  assign cond_busy_o   = (cond_st_q != COND_ABSORB);
  assign sample_cnt_o  = sample_cnt_q;
  assign rct_cnt_o     = rct_cnt_q;
  assign apt_cnt_o     = apt_cnt_q;
  assign apt_idx_o     = apt_idx_q;

  // Next sponge state helpers
  wire [63:0] absorb_full_c = {src_data, absorb_buf_q[59:0]};
  wire [3:0]  eff_blks_c    = (absorb_blks_i == 4'd0) ? 4'd1 : absorb_blks_i;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      cond_st_q       <= COND_ABSORB;
      rct_init_q      <= 1'b0;
      rct_prev_q      <= 4'd0;
      rct_cnt_q       <= 16'd0;
      apt_ref_q       <= 4'd0;
      apt_cnt_q       <= 10'd0;
      apt_idx_q       <= 10'd0;
      rct_fail_q      <= 1'b0;
      apt_fail_q      <= 1'b0;
      sample_cnt_q    <= 8'd0;
      sponge_q        <= 256'd0;
      absorb_buf_q    <= 64'd0;
      nib_cnt_q       <= 4'd0;
      blk_cnt_q       <= 4'd0;
      rnd_cnt_q       <= 4'd0;
      reseed_cnt_q    <= 32'd0;
      health_fail_p_o <= 1'b0;
    end else begin
      health_fail_p_o <= 1'b0;

      if (wipe_i) begin
        cond_st_q    <= COND_ABSORB;
        rct_init_q   <= 1'b0;
        rct_prev_q   <= 4'd0;
        rct_cnt_q    <= 16'd0;
        apt_ref_q    <= 4'd0;
        apt_cnt_q    <= 10'd0;
        apt_idx_q    <= 10'd0;
        rct_fail_q   <= 1'b0;
        apt_fail_q   <= 1'b0;
        sample_cnt_q <= 8'd0;
        sponge_q     <= 256'd0;
        absorb_buf_q <= 64'd0;
        nib_cnt_q    <= 4'd0;
        blk_cnt_q    <= 4'd0;
        rnd_cnt_q    <= 4'd0;
        reseed_cnt_q <= 32'd0;
      end else begin
        // Health test recovery / FIFO flush
        if (cmd_clr_health_i) begin
          rct_init_q <= 1'b0;
          rct_cnt_q  <= 16'd0;
          apt_cnt_q  <= 10'd0;
          apt_idx_q  <= 10'd0;
          rct_fail_q <= 1'b0;
          apt_fail_q <= 1'b0;
          cond_st_q  <= COND_ABSORB;
          nib_cnt_q  <= 4'd0;
          blk_cnt_q  <= 4'd0;
        end else if (cmd_flush_i) begin
          cond_st_q <= COND_ABSORB;
          nib_cnt_q <= 4'd0;
          blk_cnt_q <= 4'd0;
          sponge_q  <= 256'd0;
        end

        // Continuous NIST SP 800-90B RCT & APT Health Tests
        if (sample_active && !cmd_clr_health_i) begin
          if (sample_cnt_q != 8'hFF) sample_cnt_q <= sample_cnt_q + 8'd1;

          // RCT update
          rct_init_q <= 1'b1;
          rct_prev_q <= src_data;
          rct_cnt_q  <= next_rct_cnt;
          if (rct_trip) rct_fail_q <= 1'b1;

          // APT update
          if (apt_idx_q == 10'd0) begin
            apt_ref_q <= src_data;
            apt_cnt_q <= 10'd1;
            apt_idx_q <= 10'd1;
          end else begin
            if (apt_trip) apt_fail_q <= 1'b1;
            if (apt_idx_q >= (apt_win_eff - 10'd1)) begin
              apt_idx_q <= 10'd0;
              apt_cnt_q <= 10'd0;
            end else begin
              apt_idx_q <= apt_idx_q + 10'd1;
              apt_cnt_q <= next_apt_cnt;
            end
          end

          if (any_trip) begin
            health_fail_p_o <= 1'b1;
            cond_st_q       <= COND_ABSORB;
            nib_cnt_q       <= 4'd0;
            blk_cnt_q       <= 4'd0;
            sponge_q        <= 256'd0;
          end
        end

        // 256-bit Sponge Conditioner FSM
        if (!health_fail_q && !any_trip && !cmd_clr_health_i && !cmd_flush_i) begin
          unique case (cond_st_q)
            COND_ABSORB: begin
              if (sample_active) begin
                absorb_buf_q[int'(nib_cnt_q)*4 +: 4] <= src_data;
                if (nib_cnt_q == 4'd15) begin
                  nib_cnt_q    <= 4'd0;
                  sponge_q[63:0]    <= sponge_q[63:0] ^ absorb_full_c;
                  sponge_q[255:224] <= sponge_q[255:224] ^ reseed_cnt_q ^ 32'h524E_4731;
                  reseed_cnt_q <= reseed_cnt_q + 32'd1;
                  rnd_cnt_q    <= 4'd0;
                  cond_st_q    <= COND_PERMUTE;
                end else begin
                  nib_cnt_q <= nib_cnt_q + 4'd1;
                end
              end
            end

            COND_PERMUTE: begin
              sponge_q <= perm256_round(sponge_q, rnd_cnt_q);
              if (rnd_cnt_q == 4'd11) begin
                rnd_cnt_q <= 4'd0;
                if ((blk_cnt_q + 4'd1) >= eff_blks_c) begin
                  blk_cnt_q <= 4'd0;
                  cond_st_q <= COND_SQUEEZE0;
                end else begin
                  blk_cnt_q <= blk_cnt_q + 4'd1;
                  cond_st_q <= COND_ABSORB;
                end
              end else begin
                rnd_cnt_q <= rnd_cnt_q + 4'd1;
              end
            end

            COND_SQUEEZE0: begin
              if (!fifo_full) begin
                cond_st_q <= COND_SQUEEZE1;
              end
            end

            COND_SQUEEZE1: begin
              if (!fifo_full) begin
                cond_st_q <= COND_ABSORB;
              end
            end

            default: cond_st_q <= COND_ABSORB;
          endcase
        end
      end
    end
  end

endmodule : trng_cond
