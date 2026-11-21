// SPDX-License-Identifier: Apache-2.0
// wdt_rtc_rtcif.sv - Bus-domain side of the RTC: shadow registers, register-channel master,
//                    flag masking, timeout supervision (all clocked by clk_i)
//
//   * Time registers (SUBSEC/TIME/DATE/BIN) are read from a shadow that is refreshed with a
//     coherent snapshot taken in the RTC domain (toggle request / toggle acknowledge, the
//     payload is stable while req != ack).  AUTO=1 refreshes continuously; reading
//     SUBSEC/TIME/BIN_LO locks the shadow until DATE/BIN_HI is read (or SYNC is written).
//   * All other RTC registers are executed inside the RTC domain through a second channel.
//     Writes are posted (one in flight); reads stall until the RTC domain answers.
//     A supervisor timeout (2^TMO_LOG2 cycles) terminates a stalled access with an error so
//     a stopped/powered-down RTC can never hang the bus.

module wdt_rtc_rtcif
  import bus_pkg::*;
  import wdt_rtc_pkg::*;
#(
  parameter int unsigned TMO_LOG2 = 22
) (
  input  logic              clk_i,
  input  logic              rst_ni,

  input  csr_req_t          csr_req_i,   // valid already qualified with the RTC window select
  output logic              ready_o,
  output logic [31:0]       rdata_o,
  output logic              err_o,

  // register channel
  output logic              ch_req_tog_o,
  output logic              ch_rd_o,
  output logic [5:0]        ch_addr_o,
  output logic [31:0]       ch_wdata_o,
  output logic [3:0]        ch_wstrb_o,
  input  logic              ch_ack_tog_i,
  input  logic [31:0]       ch_rdata_i,

  // snapshot channel
  output logic              snap_req_tog_o,
  input  logic              snap_ack_tog_i,
  input  logic [SNAP_W-1:0] snap_i,

  // single-bit levels from the RTC domain
  input  logic [7:0]        flags_i,
  input  logic [7:0]        ien_i,
  input  logic              initf_i,
  input  logic              inits_i,
  input  logic              unlocked_i,

  output logic              rtc_irq_o
);

  // ---------------------------------------------------------------------------
  // Synchronisers
  // ---------------------------------------------------------------------------
  logic ch_ack_s, snap_ack_s;
  edge_detect #(.WIDTH(2), .INIT('0)) u_ack_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i({ch_ack_tog_i, snap_ack_tog_i}),
    .sync_o({ch_ack_s, snap_ack_s}), .rise_o(), .fall_o()
  );

  logic [7:0] flags_s, ien_s;
  logic       initf_s, inits_s, unl_s;
  edge_detect #(.WIDTH(19), .INIT('0)) u_lvl_sync (
    .clk_i(clk_i), .rst_ni(rst_ni), .async_i({flags_i, ien_i, initf_i, inits_i, unlocked_i}),
    .sync_o({flags_s, ien_s, initf_s, inits_s, unl_s}), .rise_o(), .fall_o()
  );

  // ---------------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------------
  logic              ch_req_tog_q, ch_rd_q;
  logic [5:0]        ch_addr_q;
  logic [31:0]       ch_wdata_q;
  logic [3:0]        ch_wstrb_q;
  logic              rd_active_q;
  logic [7:0]        clr_hide_q;
  logic [2:0]        settle_q;
  logic [TMO_LOG2-1:0] tmo_q;
  logic              tout_q;

  logic              snap_req_tog_q, sbusy_q, fresh_q, want_q, rsf_q, lock_q, auto_q;
  logic [SNAP_W-1:0] shadow_q;
  logic              irq_q;

  wire settled  = (settle_q == 3'd4);
  wire busy_raw = (ch_req_tog_q != ch_ack_s);
  wire busy     = busy_raw || !settled;

  assign ch_req_tog_o   = ch_req_tog_q;
  assign ch_rd_o        = ch_rd_q;
  assign ch_addr_o      = ch_addr_q;
  assign ch_wdata_o     = ch_wdata_q;
  assign ch_wstrb_o     = ch_wstrb_q;
  assign snap_req_tog_o = snap_req_tog_q;

  // Shadow slices: {time[22:0], date[31:0], subsec[21:0], bin[63:0]}
  wire [63:0] sh_bin  = shadow_q[63:0];
  wire [21:0] sh_sub  = shadow_q[85:64];
  wire [31:0] sh_date = shadow_q[117:86];
  wire [22:0] sh_time = shadow_q[140:118];

  // ---------------------------------------------------------------------------
  // Access decode
  // ---------------------------------------------------------------------------
  wire       acc = csr_req_i.valid;
  wire       wr  = csr_req_i.write;
  wire [7:0] off = {csr_req_i.addr[7:2], 2'b00};

  logic        nom_ready, launch_rd, launch_wr, sync_wr, first_rd, second_rd, tmo_hit;
  logic [31:0] rdata_c;
  logic        err_c;

  wire [7:0] flags_vis = flags_s & ~(busy_raw ? clr_hide_q : 8'd0);

  always_comb begin
    nom_ready = 1'b1;
    rdata_c   = '0;
    err_c     = 1'b0;
    launch_rd = 1'b0;
    launch_wr = 1'b0;
    sync_wr   = 1'b0;
    first_rd  = 1'b0;
    second_rd = 1'b0;
    if (acc) begin
      if (!wr) begin
        if (is_chan_rd(off)) begin
          nom_ready = rd_active_q && !busy;
          launch_rd = !rd_active_q && !busy;
          rdata_c   = ch_rdata_i;
        end else begin
          unique case (off)
            RTC_O_STAT:   rdata_c = {25'd0, inits_s, unl_s, lock_q, tout_q, busy_raw, initf_s, rsf_q};
            RTC_O_SYNC:   rdata_c = {30'd0, 1'b0, auto_q};
            RTC_O_TIME:   begin rdata_c = {9'd0, sh_time}; first_rd = 1'b1; end
            RTC_O_DATE:   begin rdata_c = sh_date;         second_rd = 1'b1; end
            RTC_O_SUBSEC: begin rdata_c = {10'd0, sh_sub}; first_rd = 1'b1; end
            RTC_O_BIN_LO: begin rdata_c = sh_bin[31:0];    first_rd = 1'b1; end
            RTC_O_BIN_HI: begin rdata_c = sh_bin[63:32];   second_rd = 1'b1; end
            RTC_O_FLAGS:  rdata_c = {24'd0, flags_vis};
            default:      err_c = 1'b1;
          endcase
        end
      end else begin
        if (is_chan_wr(off)) begin
          nom_ready = !busy;
          launch_wr = !busy;
        end else begin
          unique case (off)
            RTC_O_STAT: ;
            RTC_O_SYNC: sync_wr = csr_req_i.wstrb[0] && csr_req_i.wdata[1];
            RTC_O_SUBSEC, RTC_O_WUT_CNT, RTC_O_TS_TIME, RTC_O_TS_DATE, RTC_O_TS_SUB,
            RTC_O_TS_BLO, RTC_O_TS_BHI, RTC_O_TS_SRC: ;
            default: err_c = 1'b1;
          endcase
        end
      end
    end
    tmo_hit = acc && !nom_ready && (&tmo_q);
  end

  assign ready_o = nom_ready || tmo_hit;
  assign err_o   = err_c || tmo_hit;
  assign rdata_o = (acc && nom_ready) ? rdata_c : 32'd0;

  wire acc_done     = acc && ready_o;
  wire lock_set_now = acc_done && !wr && first_rd;
  wire lock_rel_now = acc_done && !wr && second_rd;

  // ---------------------------------------------------------------------------
  // Snapshot / shadow engine (combinational helpers)
  // ---------------------------------------------------------------------------
  wire snap_idle_ok = (snap_ack_s == snap_req_tog_q);
  wire snap_done    = sbusy_q && (snap_ack_s == snap_req_tog_q);
  wire want_n       = want_q || sync_wr;
  wire launch_s     = settled && !sbusy_q && snap_idle_ok && (auto_q || want_n);
  wire blocked      = auto_q && (lock_q || lock_set_now);

  // ---------------------------------------------------------------------------
  // Sequential logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ch_req_tog_q   <= 1'b0;
      ch_rd_q        <= 1'b0;
      ch_addr_q      <= '0;
      ch_wdata_q     <= '0;
      ch_wstrb_q     <= '0;
      rd_active_q    <= 1'b0;
      clr_hide_q     <= '0;
      settle_q       <= '0;
      tmo_q          <= '0;
      tout_q         <= 1'b0;
      snap_req_tog_q <= 1'b0;
      sbusy_q        <= 1'b0;
      fresh_q        <= 1'b0;
      want_q         <= 1'b0;
      rsf_q          <= 1'b0;
      lock_q         <= 1'b0;
      auto_q         <= 1'b1;
      shadow_q       <= '0;
      irq_q          <= 1'b0;
    end else begin
      if (!settled) settle_q <= settle_q + 3'd1;

      // timeout supervisor
      if (acc && !nom_ready) tmo_q <= tmo_q + 1'b1;
      else                   tmo_q <= '0;
      if (tmo_hit) tout_q <= 1'b1;
      if (acc && wr && !tmo_hit && off == RTC_O_STAT && csr_req_i.wstrb[0] && csr_req_i.wdata[3])
        tout_q <= 1'b0;

      // register channel launch
      if (launch_rd || launch_wr) begin
        ch_req_tog_q <= ~ch_req_tog_q;
        ch_rd_q      <= launch_rd;
        ch_addr_q    <= off[7:2];
        ch_wdata_q   <= csr_req_i.wdata;
        ch_wstrb_q   <= csr_req_i.wstrb;
        clr_hide_q   <= (launch_wr && off == RTC_O_FLAGS && csr_req_i.wstrb[0]) ? csr_req_i.wdata[7:0] : 8'd0;
      end
      if (launch_rd) rd_active_q <= 1'b1;
      if (!acc || (acc && ready_o && !wr)) rd_active_q <= 1'b0;

      // SYNC register / lock
      if (acc && wr && off == RTC_O_SYNC && csr_req_i.wstrb[0]) auto_q <= csr_req_i.wdata[0];
      if (lock_rel_now) lock_q <= 1'b0;
      if (lock_set_now) lock_q <= 1'b1;

      // snapshot engine
      if (sync_wr) begin
        rsf_q  <= 1'b0;
        lock_q <= 1'b0;
      end
      if (!sbusy_q) begin
        if (launch_s) begin
          snap_req_tog_q <= ~snap_req_tog_q;
          sbusy_q        <= 1'b1;
          fresh_q        <= 1'b1;
          want_q         <= 1'b0;
        end else begin
          want_q <= want_n;
        end
      end else begin
        if (sync_wr) begin
          fresh_q <= 1'b0;
          want_q  <= 1'b1;
        end
        if (snap_done) begin
          if (!fresh_q || sync_wr) begin
            sbusy_q <= 1'b0;
          end else if (!blocked) begin
            shadow_q <= snap_i;
            rsf_q    <= 1'b1;
            sbusy_q  <= 1'b0;
          end
        end
      end

      irq_q <= |(flags_vis & ien_s);
    end
  end

  assign rtc_irq_o = irq_q;

endmodule : wdt_rtc_rtcif
