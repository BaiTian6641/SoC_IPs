// SPDX-License-Identifier: Apache-2.0
// hub75_scan.sv - BCM bit-plane sequencer + shift engine + panel pad drivers (single clock, tick enables only).
//
// Sequencer (per plane):  WAIT_SH -> PREP(3) -> PRE -> [ADDR] -> LAT -> POST -> ON
// The shifter loads the next plane while the current plane is lit (started when LAT falls).

module hub75_scan
  import hub75_pkg::*;
#(
  parameter int unsigned LANES     = 1,
  parameter int unsigned MAX_CHAIN = 512,
  localparam int unsigned AW       = (MAX_CHAIN > 1) ? $clog2(MAX_CHAIN) : 1
) (
  input  logic                          clk_i,
  input  logic                          rst_ni,

  input  logic                          run_i,
  input  cfg_t                          cfg_f_i,        // latest frame-latched config (from fetch)
  input  logic                          cfg_f_vld_i,

  input  logic [7:0]                    clk_div_i,
  input  logic [7:0]                    lat_w_i,
  input  logic [7:0]                    blank_pre_i,
  input  logic [7:0]                    blank_post_i,
  input  logic [7:0]                    addr_setup_i,
  input  logic [15:0]                   bcm_base_i,
  input  logic [7:0]                    bright_i,
  input  logic                          dither_i,

  // bank handshake
  input  logic [1:0]                    full_i,
  output logic                          rbank_o,
  output logic                          release_o,      // pulse (rbank_o still old value)

  // line buffer read
  output logic                          rd_bank_o,
  output logic [AW-1:0]                 rd_addr_o,
  input  logic [LANES-1:0][1:0][35:0]   rd_data_i,

  // pads (internal polarity: lat/oe active-high here, inverted at top level)
  output logic [LANES-1:0]              r1_o,
  output logic [LANES-1:0]              g1_o,
  output logic [LANES-1:0]              b1_o,
  output logic [LANES-1:0]              r2_o,
  output logic [LANES-1:0]              g2_o,
  output logic [LANES-1:0]              b2_o,
  output logic                          clk_o,
  output logic                          lat_o,
  output logic                          oe_on_o,
  output logic [4:0]                    addr_o,
  input  logic                          addr_serial_unused_i,

  // status / events
  output logic                          idle_o,
  output logic                          frame_done_o,
  output logic                          underrun_o,     // pulse at stall start
  output logic                          stalled_o,
  output logic [4:0]                    row_o,
  output logic [3:0]                    plane_o
);

  // ---------------------------------------------------------------------------
  // Scan-side config
  // ---------------------------------------------------------------------------
  logic [2:0]  sc_log2_q;
  logic [11:0] sc_lc_q;
  logic [3:0]  sc_depth_q;
  logic        sc_msb_q, sc_ser_q;

  wire cfg_eq_w = (cfg_f_i.scan_log2 == sc_log2_q) && (cfg_f_i.lc == sc_lc_q) &&
                  (cfg_f_i.depth == sc_depth_q) && (cfg_f_i.msb_first == sc_msb_q) &&
                  (cfg_f_i.serial_addr == sc_ser_q);

  function automatic logic [3:0] plane_of(input logic [3:0] i, input logic [3:0] depth, input logic msb);
    return msb ? (depth - 4'd1 - i) : i;
  endfunction

  // ---------------------------------------------------------------------------
  // Shifter
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] { SH_IDLE, SH_PRIME, SH_RUN, SH_END } shstate_e;
  shstate_e    sh_state_q;
  logic [AW-1:0] ra_q;
  logic [11:0] sh_idx_q, sh_lc_q;
  logic        sh_ph_q, sh_done_q, prime_q;
  logic [3:0]  sh_plane_q;
  logic [7:0]  tick_cnt_q;
  logic        rbank_q;

  logic        sh_start_w;
  logic [3:0]  sh_start_plane_w;
  logic [11:0] sh_start_lc_w;

  assign rd_bank_o = rbank_q;
  assign rd_addr_o = ra_q;
  assign rbank_o   = rbank_q;

  wire tick_w = (tick_cnt_q == clk_div_i);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sh_state_q <= SH_IDLE;
      ra_q       <= '0;
      sh_idx_q   <= '0;
      sh_lc_q    <= '0;
      sh_ph_q    <= 1'b0;
      sh_done_q  <= 1'b0;
      prime_q    <= 1'b0;
      sh_plane_q <= '0;
      tick_cnt_q <= '0;
      clk_o      <= 1'b0;
      r1_o <= '0; g1_o <= '0; b1_o <= '0; r2_o <= '0; g2_o <= '0; b2_o <= '0;
    end else if (!run_i) begin
      sh_state_q <= SH_IDLE;
      sh_done_q  <= 1'b0;
      clk_o      <= 1'b0;
      r1_o <= '0; g1_o <= '0; b1_o <= '0; r2_o <= '0; g2_o <= '0; b2_o <= '0;
    end else if (sh_start_w) begin
      sh_state_q <= SH_PRIME;
      ra_q       <= '0;
      sh_idx_q   <= '0;
      sh_lc_q    <= sh_start_lc_w;
      sh_ph_q    <= 1'b0;
      sh_done_q  <= 1'b0;
      prime_q    <= 1'b0;
      sh_plane_q <= sh_start_plane_w;
      tick_cnt_q <= '0;
    end else begin
      unique case (sh_state_q)
        SH_PRIME: begin
          if (prime_q) begin
            sh_state_q <= SH_RUN;
            tick_cnt_q <= '0;
          end
          prime_q <= 1'b1;
        end
        SH_RUN: begin
          tick_cnt_q <= tick_w ? 8'd0 : (tick_cnt_q + 8'd1);
          if (tick_w) begin
            if (!sh_ph_q) begin
              clk_o <= 1'b0;
              for (int l = 0; l < LANES; l++) begin
                r1_o[l] <= rd_data_i[l][0][0  + int'(sh_plane_q)];
                g1_o[l] <= rd_data_i[l][0][12 + int'(sh_plane_q)];
                b1_o[l] <= rd_data_i[l][0][24 + int'(sh_plane_q)];
                r2_o[l] <= rd_data_i[l][1][0  + int'(sh_plane_q)];
                g2_o[l] <= rd_data_i[l][1][12 + int'(sh_plane_q)];
                b2_o[l] <= rd_data_i[l][1][24 + int'(sh_plane_q)];
              end
              ra_q    <= ra_q + AW'(1);
              sh_ph_q <= 1'b1;
            end else begin
              clk_o    <= 1'b1;
              sh_ph_q  <= 1'b0;
              sh_idx_q <= sh_idx_q + 12'd1;
              if (sh_idx_q == sh_lc_q - 12'd1) sh_state_q <= SH_END;
            end
          end
        end
        SH_END: begin
          tick_cnt_q <= tick_w ? 8'd0 : (tick_cnt_q + 8'd1);
          if (tick_w) begin
            clk_o      <= 1'b0;
            sh_done_q  <= 1'b1;
            sh_state_q <= SH_IDLE;
          end
        end
        default: ;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Sequencer
  // ---------------------------------------------------------------------------
  typedef enum logic [3:0] { Q_IDLE, Q_WAIT_SH, Q_PREP, Q_PRE, Q_ADDR, Q_LAT, Q_POST, Q_ON } qstate_e;
  qstate_e     q_q;
  logic [4:0]  row_q;
  logic [3:0]  pl_q;
  logic        armed_q;
  logic [31:0] tmr_q;
  logic [1:0]  prep_q;
  logic [31:0] w_q;
  logic [40:0] prod_q;
  logic [31:0] on_q;
  logic [7:0]  acc_q;
  logic        lat_q, oe_q, stalled_q;
  logic [4:0]  addr_q;
  logic [5:0]  ser_cnt_q;
  logic        ser_ph_q;

  assign row_o   = row_q;
  assign plane_o = pl_q;
  assign lat_o   = lat_q;
  assign oe_on_o = oe_q;
  assign addr_o  = addr_q;
  assign idle_o  = (q_q == Q_IDLE) && (sh_state_q == SH_IDLE);
  assign stalled_o = stalled_q;

  wire [7:0]  lat_w_e   = (lat_w_i == 8'd0)       ? 8'd1 : lat_w_i;
  wire [7:0]  pre_e     = (blank_pre_i == 8'd0)   ? 8'd1 : blank_pre_i;
  wire [7:0]  post_e    = (blank_post_i == 8'd0)  ? 8'd1 : blank_post_i;
  wire [7:0]  setup_e   = (addr_setup_i == 8'd0)  ? 8'd1 : addr_setup_i;
  wire [15:0] base_e    = (bcm_base_i == 16'd0)   ? 16'd1 : bcm_base_i;
  wire [8:0]  bright_s  = (bright_i == 8'd255) ? 9'd256 : {1'b0, bright_i};

  wire        last_row_w = ({1'b0, row_q} == ((6'd1 << sc_log2_q) - 6'd1));
  wire        last_pl_w  = (pl_q == sc_depth_q - 4'd1);
  wire [4:0]  next_row_w = last_row_w ? 5'd0 : (row_q + 5'd1);

  // shifter start request (combinational from sequencer state)
  always_comb begin
    sh_start_w       = 1'b0;
    sh_start_plane_w = '0;
    sh_start_lc_w    = sc_lc_q;
    unique case (q_q)
      Q_IDLE: begin
        if (run_i && cfg_f_vld_i && full_i[rbank_q]) begin
          sh_start_w       = 1'b1;
          sh_start_plane_w = plane_of(4'd0, cfg_f_i.depth, cfg_f_i.msb_first);
          sh_start_lc_w    = cfg_f_i.lc;
        end
      end
      Q_WAIT_SH: begin
        if (!armed_q && full_i[rbank_q]) begin
          sh_start_w = 1'b1;
          if (row_q == 5'd0 && pl_q == 4'd0) begin
            sh_start_plane_w = plane_of(4'd0, cfg_f_i.depth, cfg_f_i.msb_first);
            sh_start_lc_w    = cfg_f_i.lc;
          end else begin
            sh_start_plane_w = plane_of(pl_q, sc_depth_q, sc_msb_q);
          end
        end
      end
      Q_LAT: begin
        if (tmr_q == 32'd0) begin
          if (!last_pl_w) begin
            sh_start_w       = 1'b1;
            sh_start_plane_w = plane_of(pl_q + 4'd1, sc_depth_q, sc_msb_q);
          end else if (full_i[rbank_q] && !(last_row_w && !cfg_eq_w) && cfg_f_vld_i) begin
            sh_start_w       = 1'b1;
            sh_start_plane_w = plane_of(4'd0, sc_depth_q, sc_msb_q);
          end
        end
      end
      default: ;
    endcase
  end

  assign release_o = (q_q == Q_WAIT_SH) && armed_q && sh_done_q && last_pl_w;

  // brightness product pipeline helpers
  wire [31:0] w_next_w = 32'(base_e) << plane_of(pl_q, sc_depth_q, sc_msb_q);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      q_q          <= Q_IDLE;
      row_q        <= '0;
      pl_q         <= '0;
      armed_q      <= 1'b0;
      tmr_q        <= '0;
      prep_q       <= '0;
      w_q          <= '0;
      prod_q       <= '0;
      on_q         <= '0;
      acc_q        <= '0;
      lat_q        <= 1'b0;
      oe_q         <= 1'b0;
      stalled_q    <= 1'b0;
      addr_q       <= '0;
      ser_cnt_q    <= '0;
      ser_ph_q     <= 1'b0;
      rbank_q      <= 1'b0;
      sc_log2_q    <= 3'd4;
      sc_lc_q      <= '0;
      sc_depth_q   <= 4'd8;
      sc_msb_q     <= 1'b0;
      sc_ser_q     <= 1'b0;
      frame_done_o <= 1'b0;
      underrun_o   <= 1'b0;
    end else begin
      frame_done_o <= 1'b0;
      underrun_o   <= 1'b0;

      if (!run_i) begin
        q_q       <= Q_IDLE;
        row_q     <= '0;
        pl_q      <= '0;
        armed_q   <= 1'b0;
        lat_q     <= 1'b0;
        oe_q      <= 1'b0;
        stalled_q <= 1'b0;
        addr_q    <= '0;
        rbank_q   <= 1'b0;
        acc_q     <= '0;
      end else begin
        unique case (q_q)
          Q_IDLE: begin
            if (sh_start_w) begin
              sc_log2_q  <= cfg_f_i.scan_log2;
              sc_lc_q    <= cfg_f_i.lc;
              sc_depth_q <= cfg_f_i.depth;
              sc_msb_q   <= cfg_f_i.msb_first;
              sc_ser_q   <= cfg_f_i.serial_addr;
              row_q      <= '0;
              pl_q       <= '0;
              armed_q    <= 1'b1;
              q_q        <= Q_WAIT_SH;
            end
          end

          Q_WAIT_SH: begin
            if (!armed_q) begin
              if (sh_start_w) begin
                armed_q   <= 1'b1;
                stalled_q <= 1'b0;
                if (row_q == 5'd0 && pl_q == 4'd0) begin
                  sc_log2_q  <= cfg_f_i.scan_log2;
                  sc_lc_q    <= cfg_f_i.lc;
                  sc_depth_q <= cfg_f_i.depth;
                  sc_msb_q   <= cfg_f_i.msb_first;
                  sc_ser_q   <= cfg_f_i.serial_addr;
                end
              end else begin
                if (!stalled_q) underrun_o <= 1'b1;
                stalled_q <= 1'b1;
              end
            end else if (sh_done_q) begin
              if (last_pl_w) rbank_q <= ~rbank_q;
              prep_q <= '0;
              q_q    <= Q_PREP;
            end
          end

          Q_PREP: begin
            prep_q <= prep_q + 2'd1;
            if (prep_q == 2'd0) w_q <= w_next_w;
            if (prep_q == 2'd1) prod_q <= 41'(w_q) * 41'(bright_s) + 41'(acc_q);
            if (prep_q == 2'd2) begin
              on_q  <= prod_q[39:8];
              acc_q <= dither_i ? prod_q[7:0] : 8'd0;
              tmr_q <= 32'(pre_e) - 32'd1;
              q_q   <= Q_PRE;
            end
          end

          Q_PRE: begin
            if (tmr_q == 32'd0) begin
              if (pl_q == 4'd0) begin
                q_q       <= Q_ADDR;
                tmr_q     <= 32'(setup_e) - 32'd1;
                ser_ph_q  <= 1'b0;
                ser_cnt_q <= (row_q == 5'd0) ? 6'((6'd1 << sc_log2_q)) : 6'd0;
                if (sc_ser_q) begin
                  addr_q <= {3'd0, 1'b0, (row_q == 5'd0) ? 1'b0 : 1'b0};
                end else begin
                  addr_q <= row_q;
                end
              end else begin
                q_q   <= Q_LAT;
                lat_q <= 1'b1;
                tmr_q <= 32'(lat_w_e) - 32'd1;
              end
            end else begin
              tmr_q <= tmr_q - 32'd1;
            end
          end

          Q_ADDR: begin
            if (tmr_q == 32'd0) begin
              if (!sc_ser_q) begin
                q_q   <= Q_LAT;
                lat_q <= 1'b1;
                tmr_q <= 32'(lat_w_e) - 32'd1;
              end else if (!ser_ph_q) begin
                // end of low phase -> raise row clock
                ser_ph_q <= 1'b1;
                addr_q   <= {3'd0, 1'b1, addr_q[0]};
                tmr_q    <= 32'(setup_e) - 32'd1;
              end else if (ser_cnt_q == 6'd0) begin
                addr_q <= {3'd0, 1'b0, addr_q[0]};
                q_q    <= Q_LAT;
                lat_q  <= 1'b1;
                tmr_q  <= 32'(lat_w_e) - 32'd1;
              end else begin
                ser_cnt_q <= ser_cnt_q - 6'd1;
                ser_ph_q  <= 1'b0;
                addr_q    <= {3'd0, 1'b0, (row_q == 5'd0 && ser_cnt_q == 6'd1) ? 1'b1 : 1'b0};
                tmr_q     <= 32'(setup_e) - 32'd1;
              end
            end else begin
              tmr_q <= tmr_q - 32'd1;
            end
          end

          Q_LAT: begin
            if (tmr_q == 32'd0) begin
              lat_q   <= 1'b0;
              armed_q <= sh_start_w;
              q_q     <= Q_POST;
              tmr_q   <= 32'(post_e) - 32'd1;
            end else begin
              tmr_q <= tmr_q - 32'd1;
            end
          end

          Q_POST: begin
            if (tmr_q == 32'd0) begin
              q_q   <= Q_ON;
              oe_q  <= (on_q != 32'd0);
              tmr_q <= (on_q != 32'd0) ? (on_q - 32'd1) : 32'd0;
            end else begin
              tmr_q <= tmr_q - 32'd1;
            end
          end

          Q_ON: begin
            if (tmr_q == 32'd0) begin
              oe_q <= 1'b0;
              q_q  <= Q_WAIT_SH;
              if (!last_pl_w) begin
                pl_q <= pl_q + 4'd1;
              end else begin
                pl_q  <= '0;
                row_q <= next_row_w;
                if (last_row_w) frame_done_o <= 1'b1;
              end
            end else begin
              tmr_q <= tmr_q - 32'd1;
            end
          end

          default: q_q <= Q_IDLE;
        endcase
      end
    end
  end

  wire _unused_ok = &{1'b0, addr_serial_unused_i};

endmodule : hub75_scan
