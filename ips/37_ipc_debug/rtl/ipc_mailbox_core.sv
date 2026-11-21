// SPDX-License-Identifier: Apache-2.0
// ipc_mailbox_core.sv - Multi-Core IPC Mailbox, Doorbells & Hardware Atomic Spinlocks
//
// Register Map (12-bit byte offset, word-aligned):
//   0x000 CTRL         (RW)  : [0]=EN (default 1), [1]=SW_RST (WO self-clearing)
//   0x004 CAPS         (RO)  : [7:0]=NUM_CORES, [15:8]=NUM_CH, [23:16]=NUM_LOCKS, [31:24]=FIFO_DEPTH
//   0x008 LOCK_STAT_LO (RO)  : [31:0] non-side-effecting bitmap of locks 0..31
//   0x00C LOCK_STAT_HI (RO)  : [31:0] non-side-effecting bitmap of locks 32..63
//   0x040+4*c DB_SET   (W1S) : Set doorbell bits for core c (reads return DB_STAT)
//   0x080+4*c DB_CLR   (W1C) : Clear doorbell bits for core c (reads return DB_STAT)
//   0x0C0+4*c DB_STAT  (RO)  : Pending doorbell bitmask for core c
//   0x100+4*c IRQ_EN   (RW)  : [0]=DB_IRQ_EN, [16+:NUM_CH]=CH_IRQ_EN (default all 1s)
//   0x140+4*c IRQ_STAT (RO)  : [0]=DB_PEND, [16+:NUM_CH]=CH_PEND
//   0x200+16*ch+0x0 MBOX_DATA (RW)     : Push on write, Pop on read
//   0x200+16*ch+0x4 MBOX_STAT (RO/W1C) : [0]=EMPTY,[1]=FULL,[7:2]=LEVEL,[8]=OVF(W1C),[9]=UNF(W1C),[23:16]=SRC_ID
//   0x200+16*ch+0x8 MBOX_CTRL (RW)     : [15:0]=DST_CORE_MASK,[23:16]=SRC_ID,[24]=IRQ_EN,[25]=FLUSH(WO)
//   0x200+16*ch+0xC MBOX_PEEK (RO)     : Non-destructive peek at channel FIFO head
//   0x400+4*k LOCK[k]  (TAS) : Read: returns 0=acquired (atomically locks) or 1=busy;
//                              Write: [0]=0 releases lock, [0]=1 locks with [15:8]=owner
//   0x500+4*k LOCK_OWNER[k] (RW) : [7:0]=owner tag, [31]=lock busy (RO, no test-and-set side effect)

module ipc_mailbox_core
  import bus_pkg::*;
  import ip_common_pkg::*;
#(
  parameter int unsigned NUM_CORES  = 4,
  parameter int unsigned NUM_CH     = 4,
  parameter int unsigned NUM_LOCKS  = 32,
  parameter int unsigned FIFO_DEPTH = 4
) (
  input  logic                 clk_i,
  input  logic                 rst_ni,

  input  csr_req_t             csr_req_i,
  output csr_rsp_t             csr_rsp_o,

  output logic [NUM_CORES-1:0] irq_o
);

  localparam int unsigned CNT_W      = $clog2(FIFO_DEPTH + 1);
  localparam int unsigned CORE_IDX_W = (NUM_CORES <= 1) ? 1 : $clog2(NUM_CORES);
  localparam int unsigned CH_IDX_W   = (NUM_CH    <= 1) ? 1 : $clog2(NUM_CH);
  localparam int unsigned LOCK_IDX_W = (NUM_LOCKS <= 1) ? 1 : $clog2(NUM_LOCKS);

  logic                            en_q;
  logic [NUM_CORES-1:0][31:0]      db_stat_q;
  logic [NUM_CORES-1:0][31:0]      irq_en_q;
  logic [NUM_CORES-1:0][31:0]      core_irq_stat;

  logic [NUM_LOCKS-1:0]            lock_q;
  logic [NUM_LOCKS-1:0][7:0]       lock_owner_q;

  logic [NUM_CH-1:0][15:0]         mbox_dst_q;
  logic [NUM_CH-1:0][7:0]          mbox_src_q;
  logic [NUM_CH-1:0]               mbox_irq_en_q;
  logic [NUM_CH-1:0]               mbox_ovf_q;
  logic [NUM_CH-1:0]               mbox_unf_q;

  logic [NUM_CH-1:0]               fifo_clr;
  logic [NUM_CH-1:0]               fifo_push;
  logic [NUM_CH-1:0]               fifo_pop;
  logic [NUM_CH-1:0]               fifo_full;
  logic [NUM_CH-1:0]               fifo_empty;
  logic [NUM_CH-1:0][31:0]         fifo_rdata;
  logic [NUM_CH-1:0][CNT_W-1:0]    fifo_level;

  logic                            sw_rst_pulse;

  // Per-channel depth-FIFO instances
  for (genvar g_ch = 0; g_ch < NUM_CH; g_ch++) begin : gen_ch_fifo
    sync_fifo #(
      .DATA_W (32),
      .DEPTH  (FIFO_DEPTH),
      .FWFT   (1'b1)
    ) u_mbox_fifo (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .clr_i    (fifo_clr[g_ch]),
      .push_i   (fifo_push[g_ch]),
      .wdata_i  (csr_req_i.wdata),
      .full_o   (fifo_full[g_ch]),
      .afull_o  (),
      .pop_i    (fifo_pop[g_ch]),
      .rdata_o  (fifo_rdata[g_ch]),
      .empty_o  (fifo_empty[g_ch]),
      .aempty_o (),
      .level_o  (fifo_level[g_ch])
    );
  end

  wire [9:0] word_off = csr_req_i.addr[11:2];
  wire       bad_align = (csr_req_i.addr[1:0] != 2'b00);

  // Address region decode
  wire sel_ctrl     = (word_off == 10'h000);
  wire sel_caps     = (word_off == 10'h001);
  wire sel_lstat_lo = (word_off == 10'h002);
  wire sel_lstat_hi = (word_off == 10'h003);

  wire [3:0]            core_idx     = word_off[3:0];
  wire [CORE_IDX_W-1:0] core_idx_eff = core_idx[CORE_IDX_W-1:0];
  wire sel_db_set   = (word_off[9:4] == 6'h01) && (32'(core_idx) < NUM_CORES);
  wire sel_db_clr   = (word_off[9:4] == 6'h02) && (32'(core_idx) < NUM_CORES);
  wire sel_db_stat  = (word_off[9:4] == 6'h03) && (32'(core_idx) < NUM_CORES);
  wire sel_irq_en   = (word_off[9:4] == 6'h04) && (32'(core_idx) < NUM_CORES);
  wire sel_irq_stat = (word_off[9:4] == 6'h05) && (32'(core_idx) < NUM_CORES);

  wire [3:0]          ch_idx     = word_off[5:2];
  wire [CH_IDX_W-1:0] ch_idx_eff = ch_idx[CH_IDX_W-1:0];
  wire [1:0]          ch_sub     = word_off[1:0];
  wire sel_mbox     = (word_off[9:6] == 4'h2) && (32'(ch_idx) < NUM_CH);

  wire [5:0]            lock_idx     = word_off[5:0];
  wire [LOCK_IDX_W-1:0] lock_idx_eff = lock_idx[LOCK_IDX_W-1:0];
  wire sel_lock     = (word_off[9:6] == 4'h4) && (32'(lock_idx) < NUM_LOCKS);
  wire sel_lowner   = (word_off[9:6] == 4'h5) && (32'(lock_idx) < NUM_LOCKS);

  wire addr_valid   = !bad_align && (
                        sel_ctrl || sel_caps || sel_lstat_lo || sel_lstat_hi ||
                        sel_db_set || sel_db_clr || sel_db_stat ||
                        sel_irq_en || sel_irq_stat ||
                        sel_mbox || sel_lock || sel_lowner
                      );

  // Per-core IRQ status & output
  always_comb begin
    for (int c = 0; c < NUM_CORES; c++) begin
      core_irq_stat[c]    = 32'd0;
      core_irq_stat[c][0] = |db_stat_q[c];
      for (int ch = 0; ch < NUM_CH; ch++) begin
        if (ch < 16) begin
          core_irq_stat[c][16 + ch] = !fifo_empty[ch] && mbox_dst_q[ch][c] && mbox_irq_en_q[ch];
        end
      end
      irq_o[c] = en_q && |(core_irq_stat[c] & irq_en_q[c]);
    end
  end

  // Combinational CSR read mux & FIFO strobes
  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = csr_req_i.valid && !addr_valid;

    fifo_clr        = {NUM_CH{sw_rst_pulse}};
    fifo_push       = '0;
    fifo_pop        = '0;

    if (csr_req_i.valid && addr_valid) begin
      if (sel_ctrl) begin
        csr_rsp_o.rdata = {31'd0, en_q};
      end else if (sel_caps) begin
        csr_rsp_o.rdata = {8'(FIFO_DEPTH), 8'(NUM_LOCKS), 8'(NUM_CH), 8'(NUM_CORES)};
      end else if (sel_lstat_lo) begin
        for (int i = 0; i < 32; i++) begin
          if (i < NUM_LOCKS) csr_rsp_o.rdata[i] = lock_q[i];
        end
      end else if (sel_lstat_hi) begin
        for (int i = 0; i < 32; i++) begin
          if ((i + 32) < NUM_LOCKS) csr_rsp_o.rdata[i] = lock_q[i + 32];
        end
      end else if (sel_db_set || sel_db_clr || sel_db_stat) begin
        csr_rsp_o.rdata = db_stat_q[core_idx_eff];
      end else if (sel_irq_en) begin
        csr_rsp_o.rdata = irq_en_q[core_idx_eff];
      end else if (sel_irq_stat) begin
        csr_rsp_o.rdata = core_irq_stat[core_idx_eff];
      end else if (sel_mbox) begin
        unique case (ch_sub)
          2'd0: begin // MBOX_DATA
            csr_rsp_o.rdata = fifo_rdata[ch_idx_eff];
            if (csr_req_i.write && !sw_rst_pulse) begin
              if (!fifo_full[ch_idx_eff]) fifo_push[ch_idx_eff] = 1'b1;
            end else if (!csr_req_i.write && !sw_rst_pulse) begin
              if (!fifo_empty[ch_idx_eff]) fifo_pop[ch_idx_eff] = 1'b1;
            end
          end
          2'd1: begin // MBOX_STAT
            csr_rsp_o.rdata = {8'd0, mbox_src_q[ch_idx_eff], 6'd0, mbox_unf_q[ch_idx_eff], mbox_ovf_q[ch_idx_eff],
                               6'(fifo_level[ch_idx_eff]), fifo_full[ch_idx_eff], fifo_empty[ch_idx_eff]};
          end
          2'd2: begin // MBOX_CTRL
            csr_rsp_o.rdata = {7'd0, mbox_irq_en_q[ch_idx_eff], mbox_src_q[ch_idx_eff], mbox_dst_q[ch_idx_eff]};
            if (csr_req_i.write && csr_req_i.wstrb[3] && csr_req_i.wdata[25]) begin
              fifo_clr[ch_idx_eff] = 1'b1;
            end
          end
          2'd3: begin // MBOX_PEEK
            csr_rsp_o.rdata = fifo_rdata[ch_idx_eff];
          end
          default: ;
        endcase
      end else if (sel_lock) begin
        csr_rsp_o.rdata = {31'd0, lock_q[lock_idx_eff]};
      end else if (sel_lowner) begin
        csr_rsp_o.rdata = {lock_q[lock_idx_eff], 23'd0, lock_owner_q[lock_idx_eff]};
      end
    end
  end

  assign sw_rst_pulse = csr_req_i.valid && csr_req_i.write && sel_ctrl && !bad_align &&
                        csr_req_i.wstrb[0] && csr_req_i.wdata[1];

  // Sequential register & spinlock state
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      en_q          <= 1'b1;
      db_stat_q     <= '0;
      irq_en_q      <= {NUM_CORES{32'hFFFF_FFFF}};
      lock_q        <= '0;
      lock_owner_q  <= '0;
      mbox_irq_en_q <= {NUM_CH{1'b1}};
      mbox_ovf_q    <= '0;
      mbox_unf_q    <= '0;
      mbox_src_q    <= '0;
      for (int ch = 0; ch < NUM_CH; ch++) begin
        mbox_dst_q[ch] <= 16'(1 << (ch % NUM_CORES));
      end
    end else if (sw_rst_pulse) begin
      en_q          <= csr_req_i.wdata[0];
      db_stat_q     <= '0;
      lock_q        <= '0;
      lock_owner_q  <= '0;
      mbox_ovf_q    <= '0;
      mbox_unf_q    <= '0;
    end else if (csr_req_i.valid && addr_valid) begin
      if (csr_req_i.write) begin
        if (sel_ctrl && csr_req_i.wstrb[0]) begin
          en_q <= csr_req_i.wdata[0];
        end else if (sel_db_set) begin
          automatic logic [31:0] mask = apply_wstrb(32'd0, csr_req_i.wdata, csr_req_i.wstrb);
          db_stat_q[core_idx_eff] <= db_stat_q[core_idx_eff] | mask;
        end else if (sel_db_clr) begin
          automatic logic [31:0] mask = apply_wstrb(32'd0, csr_req_i.wdata, csr_req_i.wstrb);
          db_stat_q[core_idx_eff] <= db_stat_q[core_idx_eff] & ~mask;
        end else if (sel_irq_en) begin
          irq_en_q[core_idx_eff] <= apply_wstrb(irq_en_q[core_idx_eff], csr_req_i.wdata, csr_req_i.wstrb);
        end else if (sel_mbox) begin
          unique case (ch_sub)
            2'd0: begin // MBOX_DATA write overflow check
              if (fifo_full[ch_idx_eff]) mbox_ovf_q[ch_idx_eff] <= 1'b1;
            end
            2'd1: begin // MBOX_STAT W1C
              if (csr_req_i.wstrb[1]) begin
                if (csr_req_i.wdata[8]) mbox_ovf_q[ch_idx_eff] <= 1'b0;
                if (csr_req_i.wdata[9]) mbox_unf_q[ch_idx_eff] <= 1'b0;
              end
            end
            2'd2: begin // MBOX_CTRL
              if (csr_req_i.wstrb[0]) mbox_dst_q[ch_idx_eff][7:0]  <= csr_req_i.wdata[7:0];
              if (csr_req_i.wstrb[1]) mbox_dst_q[ch_idx_eff][15:8] <= csr_req_i.wdata[15:8];
              if (csr_req_i.wstrb[2]) mbox_src_q[ch_idx_eff]       <= csr_req_i.wdata[23:16];
              if (csr_req_i.wstrb[3]) begin
                mbox_irq_en_q[ch_idx_eff] <= csr_req_i.wdata[24];
                if (csr_req_i.wdata[25]) begin
                  mbox_ovf_q[ch_idx_eff] <= 1'b0;
                  mbox_unf_q[ch_idx_eff] <= 1'b0;
                end
              end
            end
            default: ;
          endcase
        end else if (sel_lock) begin
          if (csr_req_i.wstrb[0]) begin
            if (!csr_req_i.wdata[0]) begin
              lock_q[lock_idx_eff]       <= 1'b0;
              lock_owner_q[lock_idx_eff] <= 8'd0;
            end else begin
              lock_q[lock_idx_eff]       <= 1'b1;
              if (csr_req_i.wstrb[1]) lock_owner_q[lock_idx_eff] <= csr_req_i.wdata[15:8];
            end
          end
        end else if (sel_lowner) begin
          if (csr_req_i.wstrb[0]) lock_owner_q[lock_idx_eff] <= csr_req_i.wdata[7:0];
        end
      end else begin
        // Read side effects: atomic test-and-set on LOCK[k], underflow on empty MBOX_DATA
        if (sel_lock) begin
          if (!lock_q[lock_idx_eff]) begin
            lock_q[lock_idx_eff] <= 1'b1;
          end
        end else if (sel_mbox && (ch_sub == 2'd0)) begin
          if (fifo_empty[ch_idx_eff]) mbox_unf_q[ch_idx_eff] <= 1'b1;
        end
      end
    end
  end

endmodule : ipc_mailbox_core
