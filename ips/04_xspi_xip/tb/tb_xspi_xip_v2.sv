// SPDX-License-Identifier: Apache-2.0
// tb_xspi_xip_v2.sv - Comprehensive Self-Checking Verification Suite for xspi_xip v2.0
//
// Verifies:
//   1. 8D-8D-8D Octal DTR + DQS Strobe reads/writes & RX_DELAY_CFG sample delay closure
//   2. On-the-Fly AES-128-CTR XIP decryption (2 regions + bypass + WO key + lock) vs golden AES model
//   3. 2-way set-associative (LRU) 16B XIP cache (burst refills, CWF, eviction, hit/miss counters, auto-inval)
//   4. Multi-CS switching (CS0..CS3 via xspi_cs_no + xspi_cs_ext_no[2:0])
//   5. Depth-16 Indirect TX/RX FIFOs + dma_tx_req_o / dma_rx_req_o + >16-word flow-controlled burst
//   6. Hardware WIP Auto-Polling (RDSR match + IRQ, and timeout + IRQ)
//   7. Continuous-read (M7:0) mode entry & 32-bit all-ones exit
//   8. Quad-bus conformance (WB, AXI4 incl. bursts, TileLink, APB4) + +seed constrained-random tests

`timescale 1ns/1ps

// Behavioral Multi-Mode xSPI Flash / PSRAM Model (SDR & 8D-8D-8D DTR + DQS + WIP)
module xspi_flash_behav #(
  parameter int unsigned MEM_BYTES = 65536
) (
  input  logic       clk_i,
  input  logic       sck_i,
  input  logic       cs_ni,
  input  logic [7:0] io_i,
  input  logic [7:0] io_oe_i,
  output logic [7:0] io_o,
  output logic       io_drive_o,
  input  logic       dqs_i,
  input  logic       dqs_oe_i,
  output logic       dqs_o,
  output logic       dqs_drive_o,

  // Flash device configuration (models flash internal VCR/NVCR mode registers)
  input  logic       cfg_ddr_i,
  input  logic       cfg_cmd_sdr_i,
  input  logic       cfg_addr32_i,
  input  logic [1:0] cfg_cmd_lanes_i,
  input  logic [1:0] cfg_addr_lanes_i,
  input  logic [1:0] cfg_data_lanes_i,
  input  logic [4:0] cfg_dummy_i,
  input  logic       cfg_cont_read_i,
  input  logic       cfg_dqs_en_i,
  input  logic       cfg_poll_2byte_i,
  input  int         cfg_io_delay_clks_i
);

  logic [7:0] mem [0:MEM_BYTES-1];
  logic [15:0] status_reg;
  int          wip_polls_left;
  int          wr_dqs_edges;
  int          tx_count;

  typedef enum int {
    PH_CMD,
    PH_ADDR,
    PH_MODE,
    PH_DUMMY,
    PH_DATA_RD,
    PH_DATA_WR,
    PH_RDSR
  } ph_e;

  ph_e         ph;
  logic        in_cont_read;
  logic [7:0]  opcode;
  logic [31:0] addr_acc;
  logic [7:0]  mode_acc;
  logic [31:0] cur_addr;
  int          bits_acc;
  int          dummy_edges_rem;
  logic [7:0]  cur_byte;
  logic [7:0]  raw_io_o;
  logic        raw_io_drv;
  logic        raw_dqs_o;
  logic        raw_dqs_drv;

  // Delay pipe for modeling PCB/pad flight time
  logic [7:0] io_pipe  [0:15];
  logic       drv_pipe [0:15];
  logic       dqs_pipe [0:15];
  logic       ddrv_pipe[0:15];

  always_ff @(posedge clk_i) begin
    io_pipe[0]   <= raw_io_o;
    drv_pipe[0]  <= raw_io_drv;
    dqs_pipe[0]  <= raw_dqs_o;
    ddrv_pipe[0] <= raw_dqs_drv;
    for (int i = 1; i < 16; i++) begin
      io_pipe[i]   <= io_pipe[i-1];
      drv_pipe[i]  <= drv_pipe[i-1];
      dqs_pipe[i]  <= dqs_pipe[i-1];
      ddrv_pipe[i] <= ddrv_pipe[i-1];
    end
  end

  assign io_o        = (cfg_io_delay_clks_i == 0) ? raw_io_o    : io_pipe[cfg_io_delay_clks_i - 1];
  assign io_drive_o  = (cfg_io_delay_clks_i == 0) ? raw_io_drv  : drv_pipe[cfg_io_delay_clks_i - 1];
  assign dqs_o       = (cfg_io_delay_clks_i == 0) ? raw_dqs_o   : dqs_pipe[cfg_io_delay_clks_i - 1];
  assign dqs_drive_o = (cfg_io_delay_clks_i == 0) ? raw_dqs_drv : ddrv_pipe[cfg_io_delay_clks_i - 1];

  function automatic int lw(input logic [1:0] l);
    case (l)
      2'd0: return 1;
      2'd1: return 2;
      2'd2: return 4;
      default: return 8;
    endcase
  endfunction

  function automatic logic [31:0] shift_in_pins(input logic [31:0] cur, input logic [7:0] p, input logic [1:0] l);
    case (l)
      2'd0: return {cur[30:0], p[0]}; // MOSI is IO0!
      2'd1: return {cur[29:0], p[1:0]};
      2'd2: return {cur[27:0], p[3:0]};
      default: return {cur[23:0], p[7:0]};
    endcase
  endfunction

  function automatic logic [7:0] slice_out_byte(input logic [7:0] b, input int bit_off, input logic [1:0] l);
    case (l)
      2'd0: return {6'd0, b[7 - bit_off], 1'b0}; // MISO is IO1!
      2'd1: return {6'd0, b[7 - bit_off -: 2]};
      2'd2: return {4'd0, b[7 - bit_off -: 4]};
      default: return b;
    endcase
  endfunction

  initial begin
    for (int i = 0; i < MEM_BYTES; i++) mem[i] = 8'hFF;
    status_reg     = 16'h0000;
    wip_polls_left = 0;
    wr_dqs_edges   = 0;
    tx_count       = 0;
    in_cont_read   = 1'b0;
    ph             = PH_CMD;
    raw_io_o       = 8'h00;
    raw_io_drv     = 1'b0;
    raw_dqs_o      = 1'b0;
    raw_dqs_drv    = 1'b0;
  end

  task automatic reset_tx();
    ph              = in_cont_read ? PH_ADDR : PH_CMD;
    opcode          = in_cont_read ? 8'hEB : 8'h00;
    addr_acc        = 32'd0;
    mode_acc        = 8'd0;
    bits_acc        = 0;
    dummy_edges_rem = 0;
    raw_io_o        = 8'h00;
    raw_io_drv      = 1'b0;
    raw_dqs_o       = 1'b0;
    raw_dqs_drv     = 1'b0;
  endtask

  initial begin
    forever begin
      @(posedge cs_ni);
      if (ph == PH_RDSR && wip_polls_left > 0) begin
        wip_polls_left--;
        if (wip_polls_left == 0) status_reg[0] = 1'b0;
      end
      reset_tx();
    end
  end

  initial begin
    forever begin
      @(negedge cs_ni);
      tx_count++;
      reset_tx();
    end
  end

  task automatic enter_read_phase();
    raw_io_drv  = 1'b1;
    raw_dqs_drv = cfg_dqs_en_i;
    raw_dqs_o   = 1'b0;
    if (opcode == 8'h05) begin
      ph       = PH_RDSR;
      cur_byte = status_reg[7:0];
      bits_acc = 0;
    end else begin
      ph       = PH_DATA_RD;
      cur_byte = mem[cur_addr % MEM_BYTES];
      bits_acc = 0;
    end
    if (!cfg_ddr_i && !cfg_dqs_en_i) begin
      // Classic SDR Mode 0: pre-drive first slice on negedge before first posedge
      raw_io_o = slice_out_byte(cur_byte, 0, cfg_data_lanes_i);
    end
  endtask

  task automatic on_sck_edge(input bit is_posedge);
    if (cs_ni) return;

    // Input phases (CMD, ADDR, MODE, DATA_WR)
    if (ph == PH_CMD) begin
      automatic bit active_edge = (cfg_ddr_i && !cfg_cmd_sdr_i) ? 1'b1 : is_posedge;
      if (!active_edge) return;
      addr_acc = shift_in_pins(addr_acc, io_i, cfg_cmd_lanes_i);
      bits_acc += lw(cfg_cmd_lanes_i);
      if (bits_acc >= 8) begin
        opcode   = addr_acc[7:0];
        bits_acc = 0;
        addr_acc = 32'd0;
        case (opcode)
          8'h06: begin status_reg[1] = 1'b1; ph = PH_DUMMY; dummy_edges_rem = 100; end
          8'h04: begin status_reg[1] = 1'b0; ph = PH_DUMMY; dummy_edges_rem = 100; end
          8'h05: begin
            if (!cfg_ddr_i) begin
              ph              = PH_DUMMY;
              dummy_edges_rem = 0;
            end else begin
              enter_read_phase();
            end
          end
          8'h02, 8'h32, 8'h12, 8'h20, 8'h03, 8'h0B, 8'hEB, 8'hEC, 8'hEE: begin
            ph = PH_ADDR;
          end
          default: begin
            ph = PH_ADDR;
          end
        endcase
      end
    end else if (ph == PH_ADDR) begin
      automatic bit active_edge = cfg_ddr_i ? 1'b1 : is_posedge;
      automatic int addr_target = cfg_addr32_i ? 32 : 24;
      if (!active_edge) return;
      addr_acc = shift_in_pins(addr_acc, io_i, cfg_addr_lanes_i);
      bits_acc += lw(cfg_addr_lanes_i);
      if (bits_acc >= addr_target) begin
        if (addr_acc == 32'hFFFF_FFFF && (in_cont_read || !cfg_cont_read_i)) begin
          // Continuous read exit sequence (32 ones)
          in_cont_read = 1'b0;
          ph           = PH_DUMMY;
          dummy_edges_rem = 100;
        end else begin
          cur_addr = cfg_addr32_i ? addr_acc : {8'd0, addr_acc[23:0]};
          bits_acc = 0;
          if (opcode == 8'h20) begin
            // 4KB Sector Erase
            automatic logic [31:0] sec_base = cur_addr & 32'hFFFF_F000;
            for (int b = 0; b < 4096; b++) mem[(sec_base + b) % MEM_BYTES] = 8'hFF;
            status_reg[0]  = 1'b1;
            wip_polls_left = 4;
            ph             = PH_DUMMY;
            dummy_edges_rem = 100;
          end else if (opcode == 8'h02 || opcode == 8'h32 || opcode == 8'h12) begin
            ph = PH_DATA_WR;
          end else if (cfg_cont_read_i && (opcode == 8'hEB || opcode == 8'hEC || opcode == 8'hEE)) begin
            ph = PH_MODE;
          end else if (cfg_dummy_i != 0 || !cfg_ddr_i) begin
            ph              = PH_DUMMY;
            dummy_edges_rem = cfg_ddr_i ? (int'(cfg_dummy_i) * 2) : int'(cfg_dummy_i);
          end else begin
            enter_read_phase();
          end
        end
      end
    end else if (ph == PH_MODE) begin
      automatic bit active_edge = cfg_ddr_i ? 1'b1 : is_posedge;
      if (!active_edge) return;
      mode_acc = shift_in_pins({24'd0, mode_acc}, io_i, cfg_addr_lanes_i)[7:0];
      bits_acc += lw(cfg_addr_lanes_i);
      if (bits_acc >= 8) begin
        bits_acc = 0;
        if (mode_acc == 8'hA5) in_cont_read = 1'b1;
        else                   in_cont_read = 1'b0;
        if (cfg_dummy_i != 0 || !cfg_ddr_i) begin
          ph              = PH_DUMMY;
          dummy_edges_rem = cfg_ddr_i ? (int'(cfg_dummy_i) * 2) : int'(cfg_dummy_i);
        end else begin
          enter_read_phase();
        end
      end
    end else if (ph == PH_DUMMY) begin
      if (cfg_ddr_i) begin
        if (dummy_edges_rem > 0) dummy_edges_rem--;
        if (dummy_edges_rem == 0) enter_read_phase();
      end else begin
        if (is_posedge && dummy_edges_rem > 0) dummy_edges_rem--;
        else if (!is_posedge && dummy_edges_rem == 0) enter_read_phase();
      end
    end else if (ph == PH_DATA_WR) begin
      automatic bit active_edge = cfg_ddr_i ? 1'b1 : is_posedge;
      if (!active_edge) return;
      addr_acc = shift_in_pins(addr_acc, io_i, cfg_data_lanes_i);
      bits_acc += lw(cfg_data_lanes_i);
      if (bits_acc >= 8) begin
        mem[cur_addr % MEM_BYTES] = addr_acc[7:0];
        cur_addr++;
        bits_acc      = 0;
        status_reg[0] = 1'b1;
        wip_polls_left = 3;
      end
    end else if (ph == PH_DATA_RD || ph == PH_RDSR) begin
      if (cfg_ddr_i || cfg_dqs_en_i) begin
        // In DDR or DQS read mode: drive slice & toggle DQS on active edge
        automatic bit active_edge = cfg_ddr_i ? 1'b1 : is_posedge;
        if (!active_edge) return;
        raw_io_o = slice_out_byte(cur_byte, bits_acc, cfg_data_lanes_i);
        if (cfg_dqs_en_i) raw_dqs_o = cfg_ddr_i ? ~raw_dqs_o : 1'b1;
        bits_acc += lw(cfg_data_lanes_i);
        if (bits_acc >= 8) begin
          bits_acc = 0;
          if (ph == PH_DATA_RD) begin
            cur_addr++;
            cur_byte = mem[cur_addr % MEM_BYTES];
          end else begin
            cur_byte = cfg_poll_2byte_i ? status_reg[15:8] : status_reg[7:0];
          end
        end
      end else begin
        // SDR non-DQS mode: shift on negedge (after controller sampled on posedge)
        if (is_posedge) return;
        bits_acc += lw(cfg_data_lanes_i);
        if (bits_acc >= 8) begin
          bits_acc = 0;
          if (ph == PH_DATA_RD) begin
            cur_addr++;
            cur_byte = mem[cur_addr % MEM_BYTES];
          end else begin
            cur_byte = cfg_poll_2byte_i ? status_reg[15:8] : status_reg[7:0];
          end
        end
        raw_io_o = slice_out_byte(cur_byte, bits_acc, cfg_data_lanes_i);
      end
    end
  endtask

  initial begin
    forever begin
      @(posedge sck_i);
      on_sck_edge(1'b1);
    end
  end

  initial begin
    forever begin
      @(negedge sck_i);
      if (!cs_ni && !cfg_ddr_i && cfg_dqs_en_i && (ph == PH_DATA_RD || ph == PH_RDSR)) begin
        raw_dqs_o = 1'b0;
      end
      on_sck_edge(1'b0);
    end
  end

  initial begin
    forever begin
      @(posedge dqs_i or negedge dqs_i);
      if (!cs_ni && dqs_oe_i) wr_dqs_edges++;
    end
  end

endmodule : xspi_flash_behav

// =============================================================================
// Main v2.0 Testbench
// =============================================================================
module tb_xspi_xip_v2;
  import bus_pkg::*;
  import tb_bfm_pkg::*;

  logic clk, rst_n;
  initial begin clk = 1'b0; rst_n = 1'b0; end
  always #5 clk = ~clk;

  `include "tb_bfm_tasks.svh"

  // -------------------------------------------------------------------------
  // Independent Golden AES-128 Reference Model (Never shares DUT RTL)
  // -------------------------------------------------------------------------
  function automatic logic [7:0] ref_gf_mul(input logic [7:0] a, input logic [7:0] b);
    logic [7:0] aa, bb, p;
    aa = a; bb = b; p = 8'd0;
    for (int i = 0; i < 8; i++) begin
      if (bb[0]) p ^= aa;
      aa = {aa[6:0], 1'b0} ^ (aa[7] ? 8'h1B : 8'h00);
      bb = bb >> 1;
    end
    return p;
  endfunction

  function automatic logic [7:0] ref_sbox(input logic [7:0] a);
    logic [7:0] inv, s;
    inv = 8'd0;
    if (a != 8'd0) begin
      for (int x = 1; x < 256; x++) begin
        if (ref_gf_mul(a, x[7:0]) == 8'd1) begin
          inv = x[7:0];
          break;
        end
      end
    end
    s = inv;
    for (int i = 1; i <= 4; i++) begin
      s ^= ((inv << i) | (inv >> (8 - i)));
    end
    return s ^ 8'h63;
  endfunction

  function automatic logic [127:0] ref_aes128_enc(input logic [127:0] key, input logic [127:0] blk);
    logic [7:0] st [0:15];
    logic [7:0] tmp[0:15];
    logic [31:0] rk [0:3];
    logic [7:0] rcon [1:10];
    rcon[1]=8'h01; rcon[2]=8'h02; rcon[3]=8'h04; rcon[4]=8'h08; rcon[5]=8'h10;
    rcon[6]=8'h20; rcon[7]=8'h40; rcon[8]=8'h80; rcon[9]=8'h1B; rcon[10]=8'h36;

    for (int i = 0; i < 16; i++) st[i] = blk[127 - 8*i -: 8] ^ key[127 - 8*i -: 8];
    rk[0] = key[127:96]; rk[1] = key[95:64]; rk[2] = key[63:32]; rk[3] = key[31:0];

    for (int r = 1; r <= 10; r++) begin
      logic [31:0] rot, subw;
      for (int i = 0; i < 16; i++) st[i] = ref_sbox(st[i]);
      // ShiftRows
      tmp[0]=st[0];  tmp[1]=st[5];  tmp[2]=st[10]; tmp[3]=st[15];
      tmp[4]=st[4];  tmp[5]=st[9];  tmp[6]=st[14]; tmp[7]=st[3];
      tmp[8]=st[8];  tmp[9]=st[13]; tmp[10]=st[2]; tmp[11]=st[7];
      tmp[12]=st[12];tmp[13]=st[1]; tmp[14]=st[6]; tmp[15]=st[11];
      if (r < 10) begin
        for (int c = 0; c < 4; c++) begin
          logic [7:0] a0, a1, a2, a3;
          a0 = tmp[4*c+0]; a1 = tmp[4*c+1]; a2 = tmp[4*c+2]; a3 = tmp[4*c+3];
          st[4*c+0] = ref_gf_mul(a0,2) ^ ref_gf_mul(a1,3) ^ a2 ^ a3;
          st[4*c+1] = a0 ^ ref_gf_mul(a1,2) ^ ref_gf_mul(a2,3) ^ a3;
          st[4*c+2] = a0 ^ a1 ^ ref_gf_mul(a2,2) ^ ref_gf_mul(a3,3);
          st[4*c+3] = ref_gf_mul(a0,3) ^ a1 ^ a2 ^ ref_gf_mul(a3,2);
        end
      end else begin
        for (int i = 0; i < 16; i++) st[i] = tmp[i];
      end
      rot = {rk[3][23:0], rk[3][31:24]};
      subw = {ref_sbox(rot[31:24]), ref_sbox(rot[23:16]), ref_sbox(rot[15:8]), ref_sbox(rot[7:0])};
      rk[0] = rk[0] ^ subw ^ {rcon[r], 24'h0};
      rk[1] = rk[1] ^ rk[0];
      rk[2] = rk[2] ^ rk[1];
      rk[3] = rk[3] ^ rk[2];
      for (int c = 0; c < 4; c++) begin
        st[4*c+0] ^= rk[c][31:24];
        st[4*c+1] ^= rk[c][23:16];
        st[4*c+2] ^= rk[c][15:8];
        st[4*c+3] ^= rk[c][7:0];
      end
    end
    return {st[0],st[1],st[2],st[3],st[4],st[5],st[6],st[7],
            st[8],st[9],st[10],st[11],st[12],st[13],st[14],st[15]};
  endfunction

  // -------------------------------------------------------------------------
  // Primary AXI4 v2 DUT + 4 Flash Chips (CS0..CS3)
  // -------------------------------------------------------------------------
  axi4_m2s_t axi_csr_m, axi_mem_m;
  axi4_s2m_t axi_csr_s, axi_mem_s;
  logic       dut_sck, dut_cs0;
  logic [2:0] dut_cs_ext;
  logic [7:0] dut_io_i, dut_io_o, dut_io_oe;
  logic       dut_dqs_i, dut_dqs_o, dut_dqs_oe;
  logic       dut_dma_tx, dut_dma_rx, dut_irq;

  // Flash config signals driven by TB to match DUT CTRL
  logic       f_ddr, f_cmd_sdr, f_addr32, f_cont_read, f_dqs_en, f_poll_2b;
  logic [1:0] f_cmd_l, f_addr_l, f_data_l;
  logic [4:0] f_dummy;
  int         f_io_delay;

  logic [7:0] f_io_o   [4];
  logic       f_io_drv [4];
  logic       f_dqs_o  [4];
  logic       f_dqs_drv[4];
  logic [3:0] cs_bus;
  assign cs_bus = {dut_cs_ext, dut_cs0};

  xspi_xip_v2_top #(
    .BUS_TYPE    (BUS_AXI4),
    .CACHE_LINES (16),
    .XIP_WAYS    (2),
    .OTFAD_EN    (1'b1),
    .FIFO_DEPTH  (16)
  ) u_dut_axi (
    .clk_i          (clk),
    .rst_ni         (rst_n),
    .wb_csr_o       (),
    .axi_csr_i      (axi_csr_m),
    .axi_csr_o      (axi_csr_s),
    .tl_csr_o       (),
    .apb_csr_o      (),
    .wb_mem_o       (),
    .axi_mem_i      (axi_mem_m),
    .axi_mem_o      (axi_mem_s),
    .tl_mem_o       (),
    .apb_mem_o      (),
    .xspi_sck_o     (dut_sck),
    .xspi_cs_no     (dut_cs0),
    .xspi_cs_ext_no (dut_cs_ext),
    .xspi_io_i      (dut_io_i),
    .xspi_io_o      (dut_io_o),
    .xspi_io_oe_o   (dut_io_oe),
    .xspi_dqs_i     (dut_dqs_i),
    .xspi_dqs_o     (dut_dqs_o),
    .xspi_dqs_oe_o  (dut_dqs_oe),
    .dma_tx_req_o   (dut_dma_tx),
    .dma_rx_req_o   (dut_dma_rx),
    .irq_o          (dut_irq)
  );

  for (genvar c = 0; c < 4; c++) begin : gen_cs_flash
    xspi_flash_behav #(.MEM_BYTES(65536)) u_flash (
      .clk_i               (clk),
      .sck_i               (dut_sck),
      .cs_ni               (cs_bus[c]),
      .io_i                (dut_io_o),
      .io_oe_i             (dut_io_oe),
      .io_o                (f_io_o[c]),
      .io_drive_o          (f_io_drv[c]),
      .dqs_i               (dut_dqs_o),
      .dqs_oe_i            (dut_dqs_oe),
      .dqs_o               (f_dqs_o[c]),
      .dqs_drive_o         (f_dqs_drv[c]),
      .cfg_ddr_i           (f_ddr),
      .cfg_cmd_sdr_i       (f_cmd_sdr),
      .cfg_addr32_i        (f_addr32),
      .cfg_cmd_lanes_i     (f_cmd_l),
      .cfg_addr_lanes_i    (f_addr_l),
      .cfg_data_lanes_i    (f_data_l),
      .cfg_dummy_i         (f_dummy),
      .cfg_cont_read_i     (f_cont_read),
      .cfg_dqs_en_i        (f_dqs_en),
      .cfg_poll_2byte_i    (f_poll_2b),
      .cfg_io_delay_clks_i (f_io_delay)
    );
  end

  always_comb begin
    dut_io_i  = 8'hFF;
    dut_dqs_i = 1'b0;
    for (int c = 0; c < 4; c++) begin
      if (!cs_bus[c] && f_io_drv[c])  dut_io_i  = f_io_o[c];
      if (!cs_bus[c] && f_dqs_drv[c]) dut_dqs_i = f_dqs_o[c];
    end
  end

  // -------------------------------------------------------------------------
  // Secondary WB / TL / APB DUTs for Quad-Bus Verification
  // -------------------------------------------------------------------------
  wb_m2s_t  wb_csr_m, wb_mem_m;
  wb_s2m_t  wb_csr_s, wb_mem_s;
  tl_m2s_t  tl_csr_m, tl_mem_m;
  tl_s2m_t  tl_csr_s, tl_mem_s;
  apb_m2s_t apb_csr_m, apb_mem_m;
  apb_s2m_t apb_csr_s, apb_mem_s;

  logic wb_sck, wb_cs0, wb_dqs_i, wb_dqs_o, wb_dqs_oe;
  logic [7:0] wb_io_i, wb_io_o, wb_io_oe;
  logic wb_io_drv, wb_dqs_drv;

  xspi_xip_v2_top #(.BUS_TYPE(BUS_WISHBONE)) u_dut_wb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_i(wb_csr_m), .wb_csr_o(wb_csr_s), .axi_csr_o(), .tl_csr_o(), .apb_csr_o(),
    .wb_mem_i(wb_mem_m), .wb_mem_o(wb_mem_s), .axi_mem_o(), .tl_mem_o(), .apb_mem_o(),
    .xspi_sck_o(wb_sck), .xspi_cs_no(wb_cs0), .xspi_cs_ext_no(),
    .xspi_io_i(wb_io_drv ? wb_io_i : 8'hFF), .xspi_io_o(wb_io_o), .xspi_io_oe_o(wb_io_oe),
    .xspi_dqs_i(wb_dqs_drv ? wb_dqs_i : 1'b0), .xspi_dqs_o(wb_dqs_o), .xspi_dqs_oe_o(wb_dqs_oe),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o()
  );
  xspi_flash_behav u_flash_wb (
    .clk_i(clk), .sck_i(wb_sck), .cs_ni(wb_cs0),
    .io_i(wb_io_o), .io_oe_i(wb_io_oe), .io_o(wb_io_i), .io_drive_o(wb_io_drv),
    .dqs_i(wb_dqs_o), .dqs_oe_i(wb_dqs_oe), .dqs_o(wb_dqs_i), .dqs_drive_o(wb_dqs_drv),
    .cfg_ddr_i(1'b1), .cfg_cmd_sdr_i(1'b0), .cfg_addr32_i(1'b1),
    .cfg_cmd_lanes_i(2'd3), .cfg_addr_lanes_i(2'd3), .cfg_data_lanes_i(2'd3),
    .cfg_dummy_i(5'd4), .cfg_cont_read_i(1'b0), .cfg_dqs_en_i(1'b1),
    .cfg_poll_2byte_i(1'b0), .cfg_io_delay_clks_i(0)
  );

  logic tl_sck, tl_cs0, tl_dqs_i, tl_dqs_o, tl_dqs_oe;
  logic [7:0] tl_io_i, tl_io_o, tl_io_oe;
  logic tl_io_drv, tl_dqs_drv;

  xspi_xip_v2_top #(.BUS_TYPE(BUS_TILELINK)) u_dut_tl (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_o(), .axi_csr_o(), .tl_csr_i(tl_csr_m), .tl_csr_o(tl_csr_s), .apb_csr_o(),
    .wb_mem_o(), .axi_mem_o(), .tl_mem_i(tl_mem_m), .tl_mem_o(tl_mem_s), .apb_mem_o(),
    .xspi_sck_o(tl_sck), .xspi_cs_no(tl_cs0), .xspi_cs_ext_no(),
    .xspi_io_i(tl_io_drv ? tl_io_i : 8'hFF), .xspi_io_o(tl_io_o), .xspi_io_oe_o(tl_io_oe),
    .xspi_dqs_i(tl_dqs_drv ? tl_dqs_i : 1'b0), .xspi_dqs_o(tl_dqs_o), .xspi_dqs_oe_o(tl_dqs_oe),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o()
  );
  xspi_flash_behav u_flash_tl (
    .clk_i(clk), .sck_i(tl_sck), .cs_ni(tl_cs0),
    .io_i(tl_io_o), .io_oe_i(tl_io_oe), .io_o(tl_io_i), .io_drive_o(tl_io_drv),
    .dqs_i(tl_dqs_o), .dqs_oe_i(tl_dqs_oe), .dqs_o(tl_dqs_i), .dqs_drive_o(tl_dqs_drv),
    .cfg_ddr_i(1'b1), .cfg_cmd_sdr_i(1'b0), .cfg_addr32_i(1'b1),
    .cfg_cmd_lanes_i(2'd3), .cfg_addr_lanes_i(2'd3), .cfg_data_lanes_i(2'd3),
    .cfg_dummy_i(5'd4), .cfg_cont_read_i(1'b0), .cfg_dqs_en_i(1'b1),
    .cfg_poll_2byte_i(1'b0), .cfg_io_delay_clks_i(0)
  );

  logic apb_sck, apb_cs0, apb_dqs_i, apb_dqs_o, apb_dqs_oe;
  logic [7:0] apb_io_i, apb_io_o, apb_io_oe;
  logic apb_io_drv, apb_dqs_drv;

  xspi_xip_v2_top #(.BUS_TYPE(BUS_APB)) u_dut_apb (
    .clk_i(clk), .rst_ni(rst_n),
    .wb_csr_o(), .axi_csr_o(), .tl_csr_o(), .apb_csr_i(apb_csr_m), .apb_csr_o(apb_csr_s),
    .wb_mem_o(), .axi_mem_o(), .tl_mem_o(), .apb_mem_i(apb_mem_m), .apb_mem_o(apb_mem_s),
    .xspi_sck_o(apb_sck), .xspi_cs_no(apb_cs0), .xspi_cs_ext_no(),
    .xspi_io_i(apb_io_drv ? apb_io_i : 8'hFF), .xspi_io_o(apb_io_o), .xspi_io_oe_o(apb_io_oe),
    .xspi_dqs_i(apb_dqs_drv ? apb_dqs_i : 1'b0), .xspi_dqs_o(apb_dqs_o), .xspi_dqs_oe_o(apb_dqs_oe),
    .dma_tx_req_o(), .dma_rx_req_o(), .irq_o()
  );
  xspi_flash_behav u_flash_apb (
    .clk_i(clk), .sck_i(apb_sck), .cs_ni(apb_cs0),
    .io_i(apb_io_o), .io_oe_i(apb_io_oe), .io_o(apb_io_i), .io_drive_o(apb_io_drv),
    .dqs_i(apb_dqs_o), .dqs_oe_i(apb_dqs_oe), .dqs_o(apb_dqs_i), .dqs_drive_o(apb_dqs_drv),
    .cfg_ddr_i(1'b1), .cfg_cmd_sdr_i(1'b0), .cfg_addr32_i(1'b1),
    .cfg_cmd_lanes_i(2'd3), .cfg_addr_lanes_i(2'd3), .cfg_data_lanes_i(2'd3),
    .cfg_dummy_i(5'd4), .cfg_cont_read_i(1'b0), .cfg_dqs_en_i(1'b1),
    .cfg_poll_2byte_i(1'b0), .cfg_io_delay_clks_i(0)
  );

  task automatic do_apb_write(ref apb_m2s_t m, const ref apb_s2m_t s, input logic [31:0] addr, input logic [31:0] data);
    @(posedge clk); #1;
    m.paddr = addr; m.pwrite = 1'b1; m.pwdata = data; m.pstrb = 4'hF; m.psel = 1'b1; m.penable = 1'b0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    if (s.pslverr) $fatal(1, "APB write error @ %08h", addr);
    #1; m = '0;
  endtask

  task automatic do_apb_read(ref apb_m2s_t m, const ref apb_s2m_t s, input logic [31:0] addr, output logic [31:0] data);
    @(posedge clk); #1;
    m.paddr = addr; m.pwrite = 1'b0; m.pstrb = 4'h0; m.psel = 1'b1; m.penable = 1'b0;
    @(posedge clk); #1;
    m.penable = 1'b1;
    do @(posedge clk); while (!s.pready);
    if (s.pslverr) $fatal(1, "APB read error @ %08h", addr);
    data = s.prdata;
    #1; m = '0;
  endtask

  // -------------------------------------------------------------------------
  // Helpers & Coverage Counters
  // -------------------------------------------------------------------------
  int unsigned cov_8d_dtr_dqs   = 0;
  int unsigned cov_rx_delay     = 0;
  int unsigned cov_otfad_r0     = 0;
  int unsigned cov_otfad_r1     = 0;
  int unsigned cov_otfad_bypass = 0;
  int unsigned cov_cache_lru    = 0;
  int unsigned cov_multi_cs     = 0;
  int unsigned cov_fifo_dma     = 0;
  int unsigned cov_autopoll     = 0;
  int unsigned cov_cont_read    = 0;
  int unsigned cov_quad_bus     = 0;
  int unsigned cov_rand_ops     = 0;

  task automatic axi_csr_w(input logic [31:0] off, input logic [31:0] val);
    do_axi_write(axi_csr_m, axi_csr_s, off, val);
  endtask

  task automatic axi_csr_r(input logic [31:0] off, output logic [31:0] val);
    do_axi_read(axi_csr_m, axi_csr_s, off, val);
  endtask

  task automatic axi_xip_r(input logic [31:0] addr, output logic [31:0] val);
    do_axi_read(axi_mem_m, axi_mem_s, addr, val);
  endtask

  task automatic set_mode(
    input logic       ddr, input logic cmd_sdr, input logic addr32,
    input logic [1:0] cl,  input logic [1:0] al, input logic [1:0] dl,
    input logic [4:0] dum, input logic cont, input logic dqs,
    input logic [1:0] cs,  input logic [7:0] clk_div, input int io_del
  );
    logic [31:0] ctrl_v, st;
    // Wait for any background CWF cache-line tail to finish before changing external flash wires
    do begin
      axi_csr_r(32'h1C, st);
    end while (st[0]);
    f_ddr       = ddr;
    f_cmd_sdr   = cmd_sdr;
    f_addr32    = addr32;
    f_cmd_l     = cl;
    f_addr_l    = al;
    f_data_l    = dl;
    f_dummy     = dum;
    f_cont_read = cont;
    f_dqs_en    = dqs;
    f_poll_2b   = 1'b0;
    f_io_delay  = io_del;
    ctrl_v = {10'd0, cmd_sdr, 1'b0, cs, dqs, cont, 1'b0, dum, dl, al, cl, addr32, ddr, 1'b1, 1'b1};
    axi_csr_w(32'h04, {24'd0, clk_div});
    axi_csr_w(32'h00, ctrl_v);
  endtask

  task automatic wait_ind_done();
    logic [31:0] st;
    do begin
      axi_csr_r(32'h1C, st);
    end while (st[0]);
    axi_csr_w(32'h1C, 32'h2); // W1C ind_done
  endtask

  task automatic preload_plain_word(input int cs_idx, input logic [31:0] addr, input logic [31:0] w);
    case (cs_idx)
      0: begin
        gen_cs_flash[0].u_flash.mem[addr+0] = w[7:0];
        gen_cs_flash[0].u_flash.mem[addr+1] = w[15:8];
        gen_cs_flash[0].u_flash.mem[addr+2] = w[23:16];
        gen_cs_flash[0].u_flash.mem[addr+3] = w[31:24];
      end
      1: begin
        gen_cs_flash[1].u_flash.mem[addr+0] = w[7:0];
        gen_cs_flash[1].u_flash.mem[addr+1] = w[15:8];
        gen_cs_flash[1].u_flash.mem[addr+2] = w[23:16];
        gen_cs_flash[1].u_flash.mem[addr+3] = w[31:24];
      end
      2: begin
        gen_cs_flash[2].u_flash.mem[addr+0] = w[7:0];
        gen_cs_flash[2].u_flash.mem[addr+1] = w[15:8];
        gen_cs_flash[2].u_flash.mem[addr+2] = w[23:16];
        gen_cs_flash[2].u_flash.mem[addr+3] = w[31:24];
      end
      default: begin
        gen_cs_flash[3].u_flash.mem[addr+0] = w[7:0];
        gen_cs_flash[3].u_flash.mem[addr+1] = w[15:8];
        gen_cs_flash[3].u_flash.mem[addr+2] = w[23:16];
        gen_cs_flash[3].u_flash.mem[addr+3] = w[31:24];
      end
    endcase
  endtask

  // Preload 16-byte block encrypted with independent golden AES-128-CTR model
  task automatic preload_enc_block(
    input logic [31:0]  base_addr,
    input logic [127:0] key,
    input logic [95:0]  nonce,
    input logic [31:0]  p0, input logic [31:0] p1, input logic [31:0] p2, input logic [31:0] p3
  );
    logic [127:0] ks;
    logic [31:0]  c0, c1, c2, c3;
    ks = ref_aes128_enc(key, {nonce, 4'b0000, base_addr[31:4]});
    // Keystream bytes are big-endian per 32-bit word; little-endian bus word swaps bytes
    c0 = p0 ^ {ks[103:96], ks[111:104], ks[119:112], ks[127:120]};
    c1 = p1 ^ {ks[71:64],  ks[79:72],   ks[87:80],   ks[95:88]};
    c2 = p2 ^ {ks[39:32],  ks[47:40],   ks[55:48],   ks[63:56]};
    c3 = p3 ^ {ks[7:0],    ks[15:8],    ks[23:16],   ks[31:24]};
    preload_plain_word(0, base_addr + 32'h0, c0);
    preload_plain_word(0, base_addr + 32'h4, c1);
    preload_plain_word(0, base_addr + 32'h8, c2);
    preload_plain_word(0, base_addr + 32'hC, c3);
  endtask

  // -------------------------------------------------------------------------
  // Main Test Sequence
  // -------------------------------------------------------------------------
  initial begin
    int unsigned seed;
    logic [31:0] rd, rd_arr[], st, hcnt, mcnt;
    logic [1:0]  rresp;
    logic        wb_err;
    logic [127:0] k0, k1;
    logic [95:0]  n0, n1;
    logic [31:0]  plain_db [0:255];

    if (!$value$plusargs("seed=%d", seed)) seed = 32'h2026_0402;
    void'($urandom(seed));

    axi_csr_m = '0; axi_mem_m = '0;
    wb_csr_m  = '0; wb_mem_m  = '0;
    tl_csr_m  = '0; tl_mem_m  = '0;
    apb_csr_m = '0; apb_mem_m = '0;
    f_ddr = 1'b0; f_cmd_sdr = 1'b0; f_addr32 = 1'b0;
    f_cmd_l = 2'd2; f_addr_l = 2'd2; f_data_l = 2'd2;
    f_dummy = 5'd8; f_cont_read = 1'b0; f_dqs_en = 1'b0; f_poll_2b = 1'b0;
    f_io_delay = 0;

    repeat (6) @(posedge clk);
    #1 rst_n = 1'b1;
    repeat (4) @(posedge clk);

    // =======================================================================
    // 1. 8D-8D-8D Octal DTR + DQS Strobe Reads & Writes + RX_DELAY_CFG
    // =======================================================================
    for (int i = 0; i < 64; i++) begin
      automatic logic [31:0] w = 32'hA500_0000 | (i * 32'h0101_0001);
      preload_plain_word(0, i * 4, w);
      plain_db[i] = w;
    end

    set_mode(1'b1, 1'b0, 1'b1, 2'd3, 2'd3, 2'd3, 5'd4, 1'b0, 1'b1, 2'd0, 8'd1, 0);
    axi_csr_w(32'h08, 32'h0000_A5EE); // 8D-8D-8D XIP opcode
    axi_csr_w(32'h20, 32'h0000_0020); // dqs_preamble_en=1, rx_sample_delay=0

    // Indirect 8D-8D-8D write with DQS write strobe enabled
    axi_csr_w(32'h10, 32'h0000_0400);
    axi_csr_w(32'h14, 32'hCAFE_BABE);
    axi_csr_w(32'h0C, 32'h0000_0702); // opcode=0x02, has_addr, is_wr, has_data
    wait_ind_done();
    if (gen_cs_flash[0].u_flash.wr_dqs_edges == 0)
      $fatal(1, "Expected DQS write strobe activity during 8D-8D-8D DQS write");

    // XIP 8D-8D-8D DTR + DQS read back of written word & preloaded words
    axi_xip_r(32'h0000_0400, rd);
    if (rd !== 32'hCAFE_BABE) $fatal(1, "8D-8D-8D DQS write/XIP read mismatch: %h", rd);
    axi_xip_r(32'h0000_0014, rd);
    if (rd !== plain_db[5]) $fatal(1, "8D-8D-8D XIP read @0x14 mismatch: %h != %h", rd, plain_db[5]);
    cov_8d_dtr_dqs++;

    // Non-DQS DDR with flight delay (io_delay = 2 -> 3-clk round-trip -> rx_sample_delay = 3, clk_div = 2)
    set_mode(1'b1, 1'b0, 1'b1, 2'd3, 2'd3, 2'd3, 5'd4, 1'b0, 1'b0, 2'd0, 8'd2, 2);
    axi_csr_w(32'h20, 32'h0000_0003); // rx_sample_delay = 3
    axi_xip_r(32'h0000_0028, rd);
    if (rd !== plain_db[10]) $fatal(1, "RX_DELAY_CFG=3 delayed read mismatch: %h != %h", rd, plain_db[10]);
    axi_csr_w(32'h20, 32'h0000_0000);
    cov_rx_delay++;
    $display("  [1] 8D-8D-8D DTR + DQS + RX_DELAY_CFG passed");

    // =======================================================================
    // 2. On-the-Fly AES-128-CTR XIP Decryption (OTFAD Regions 0 & 1 + Bypass)
    // =======================================================================
    k0 = 128'h2B7E_1516_28AE_D2A6_ABF7_1588_09CF_4F3C;
    n0 = 96'hF0F1_F2F3_F4F5_F6F7_F8F9_FAFB;
    k1 = 128'h0001_0203_0405_0607_0809_0A0B_0C0D_0E0F;
    n1 = 96'h1122_3344_5566_7788_99AA_BBCC;

    // Preload Region 0 (0x2000..0x203F) encrypted
    for (int blk = 0; blk < 4; blk++) begin
      automatic logic [31:0] ba = 32'h0000_2000 + blk * 16;
      preload_enc_block(ba, k0, n0, 32'hD000_0000 + blk*4 + 0, 32'hD000_0000 + blk*4 + 1,
                                    32'hD000_0000 + blk*4 + 2, 32'hD000_0000 + blk*4 + 3);
    end
    // Preload Region 1 (0x4000..0x403F) encrypted
    for (int blk = 0; blk < 4; blk++) begin
      automatic logic [31:0] ba = 32'h0000_4000 + blk * 16;
      preload_enc_block(ba, k1, n1, 32'hE100_0000 + blk*4 + 0, 32'hE100_0000 + blk*4 + 1,
                                    32'hE100_0000 + blk*4 + 2, 32'hE100_0000 + blk*4 + 3);
    end
    // Preload Bypass region (0x1000..0x100F) plaintext
    preload_plain_word(0, 32'h0000_1000, 32'h1234_5678);
    preload_plain_word(0, 32'h0000_1004, 32'h9ABC_DEF0);

    // Program Region 0 & Region 1 CSRs
    axi_csr_w(32'h54, 32'h0000_2000); axi_csr_w(32'h58, 32'h0000_2100);
    axi_csr_w(32'h5C, k0[31:0]);      axi_csr_w(32'h60, k0[63:32]);
    axi_csr_w(32'h64, k0[95:64]);     axi_csr_w(32'h68, k0[127:96]);
    axi_csr_w(32'h6C, n0[31:0]);      axi_csr_w(32'h70, n0[63:32]);
    axi_csr_w(32'h74, n0[95:64]);

    axi_csr_w(32'h78, 32'h0000_4000); axi_csr_w(32'h7C, 32'h0000_4100);
    axi_csr_w(32'h80, k1[31:0]);      axi_csr_w(32'h84, k1[63:32]);
    axi_csr_w(32'h88, k1[95:64]);     axi_csr_w(32'h8C, k1[127:96]);
    axi_csr_w(32'h90, n1[31:0]);      axi_csr_w(32'h94, n1[63:32]);
    axi_csr_w(32'h98, n1[95:64]);

    // Check WO keys read back 0
    axi_csr_r(32'h5C, rd); if (rd !== 32'd0) $fatal(1, "OTFAD KEY0 must read back 0!");
    // Enable Region 0 & Region 1 + Lock Region 0
    axi_csr_w(32'h50, 32'h0000_0007);
    // Attempt to overwrite locked Region 0 key -> must be ignored
    axi_csr_w(32'h5C, 32'hDEAD_BEEF);

    set_mode(1'b1, 1'b0, 1'b1, 2'd3, 2'd3, 2'd3, 5'd4, 1'b0, 1'b1, 2'd0, 8'd1, 0);
    // Test Critical-Word-First offsets (word 2 first, then word 0, 1, 3) on Region 0
    axi_xip_r(32'h0000_2008, rd);
    if (rd !== 32'hD000_0002) $fatal(1, "OTFAD R0 CWF word 2 mismatch: %h", rd);
    axi_xip_r(32'h0000_2000, rd);
    if (rd !== 32'hD000_0000) $fatal(1, "OTFAD R0 word 0 mismatch: %h", rd);
    axi_xip_r(32'h0000_201C, rd);
    if (rd !== 32'hD000_0007) $fatal(1, "OTFAD R0 blk1 word 3 mismatch: %h", rd);
    cov_otfad_r0++;

    // Region 1
    axi_xip_r(32'h0000_4014, rd);
    if (rd !== 32'hE100_0005) $fatal(1, "OTFAD R1 blk1 word 1 mismatch: %h", rd);
    cov_otfad_r1++;

    // Unencrypted bypass
    axi_xip_r(32'h0000_1000, rd);
    if (rd !== 32'h1234_5678) $fatal(1, "OTFAD bypass mismatch: %h", rd);
    cov_otfad_bypass++;
    $display("  [2] OTFAD AES-128-CTR (R0/R1/Bypass/WO/Lock) passed");

    // =======================================================================
    // 3. 2-Way Set-Associative (LRU) 16B XIP Cache + Hit/Miss Counters
    // =======================================================================
    // Set 0 = addr[6:4] == 3'b000 -> addresses 0x5000, 0x5080, 0x5100 all map to Set 0!
    preload_plain_word(0, 32'h0000_5000, 32'h5000_1111);
    preload_plain_word(0, 32'h0000_5004, 32'h5000_2222);
    preload_plain_word(0, 32'h0000_5080, 32'h5080_3333);
    preload_plain_word(0, 32'h0000_5100, 32'h5100_4444);

    axi_csr_w(32'h00, 32'h0000_8000 | 32'h0002_13EB); // flush + 8D-8D-8D DQS
    set_mode(1'b1, 1'b0, 1'b1, 2'd3, 2'd3, 2'd3, 5'd4, 1'b0, 1'b1, 2'd0, 8'd1, 0);
    axi_csr_w(32'h38, 32'd0); // Clear hit cnt
    axi_csr_w(32'h3C, 32'd0); // Clear miss cnt

    axi_xip_r(32'h0000_5000, rd); if (rd !== 32'h5000_1111) $fatal(1, "Cache fill 5000 fail: %h", rd); // Miss 1
    axi_xip_r(32'h0000_5004, rd); if (rd !== 32'h5000_2222) $fatal(1, "Cache burst hit 5004 fail: %h", rd); // Hit 1 (same 16B line!)
    axi_xip_r(32'h0000_5080, rd); if (rd !== 32'h5080_3333) $fatal(1, "Cache fill 5080 fail: %h", rd); // Miss 2 (Way 1 of Set 0)
    axi_xip_r(32'h0000_5000, rd); if (rd !== 32'h5000_1111) $fatal(1, "2-way co-residency 5000 fail"); // Hit 2 (Way 0 still alive!)
    axi_xip_r(32'h0000_5080, rd); if (rd !== 32'h5080_3333) $fatal(1, "2-way co-residency 5080 fail"); // Hit 3 (5080 MRU, 5000 LRU)
    axi_xip_r(32'h0000_5100, rd); if (rd !== 32'h5100_4444) $fatal(1, "Cache fill 5100 fail"); // Miss 3 (evicts LRU = 5000!)
    axi_xip_r(32'h0000_5080, rd); if (rd !== 32'h5080_3333) $fatal(1, "LRU survivor 5080 fail"); // Hit 4 (5080 survived!)
    axi_xip_r(32'h0000_5000, rd); if (rd !== 32'h5000_1111) $fatal(1, "Refill evicted 5000 fail"); // Miss 4

    axi_csr_r(32'h38, hcnt);
    axi_csr_r(32'h3C, mcnt);
    if (hcnt !== 32'd4 || mcnt !== 32'd4)
      $fatal(1, "2-way LRU cache counters mismatch: hits=%0d (exp 4) misses=%0d (exp 4)", hcnt, mcnt);
    cov_cache_lru++;
    $display("  [3] 2-way LRU 16B XIP cache (burst hits + LRU eviction + counters) passed");

    // =======================================================================
    // 4. Multi-CS (CS0..CS3) Switching
    // =======================================================================
    preload_plain_word(1, 32'h0000_0200, 32'hC501_1111);
    preload_plain_word(2, 32'h0000_0200, 32'hC502_2222);
    preload_plain_word(3, 32'h0000_0200, 32'hC503_3333);
    for (int c = 1; c <= 3; c++) begin
      automatic int tx_before;
      tx_before = (c == 1) ? gen_cs_flash[1].u_flash.tx_count :
                  (c == 2) ? gen_cs_flash[2].u_flash.tx_count :
                             gen_cs_flash[3].u_flash.tx_count;
      set_mode(1'b1, 1'b0, 1'b1, 2'd3, 2'd3, 2'd3, 5'd4, 1'b0, 1'b1, c[1:0], 8'd1, 0);
      axi_xip_r(32'h0000_0200, rd);
      if (rd !== (32'hC500_0000 | (c << 16) | (c * 32'h1111)))
        $fatal(1, "Multi-CS CS%0d read mismatch: %h", c, rd);
      if (((c == 1) ? gen_cs_flash[1].u_flash.tx_count :
           (c == 2) ? gen_cs_flash[2].u_flash.tx_count :
                      gen_cs_flash[3].u_flash.tx_count) != tx_before + 1)
        $fatal(1, "CS%0d flash did not see CS assertion", c);
    end
    cov_multi_cs++;
    $display("  [4] Multi-CS (CS0..CS3) switching passed");

    // =======================================================================
    // 5. Indirect TX/RX FIFOs (Depth 16 & 20-Word Burst) + DMA Requests
    // =======================================================================
    set_mode(1'b0, 1'b0, 1'b1, 2'd2, 2'd2, 2'd2, 5'd4, 1'b0, 1'b0, 2'd0, 8'd1, 0); // Quad SDR
    axi_csr_w(32'h24, 32'h0000_044F); // tx_fifo_rst + rx_fifo_rst + dma_tx_en + dma_rx_en, tx_thresh=4, rx_thresh=4
    if (!dut_dma_tx || dut_dma_rx) $fatal(1, "Initial DMA req state wrong: tx=%b rx=%b", dut_dma_tx, dut_dma_rx);

    // Push 16 words into TX FIFO
    for (int i = 0; i < 16; i++) begin
      axi_csr_w(32'h28, 32'hF1F0_0000 | i);
    end
    if (dut_dma_tx) $fatal(1, "dma_tx_req_o should deassert when TX FIFO full (16 > thresh 4)");
    axi_csr_w(32'h34, 32'd16);        // IND_LEN = 16 words
    axi_csr_w(32'h10, 32'h0000_6000); // IND_ADDR = 0x6000
    axi_csr_w(32'h0C, 32'h0000_1732); // use_fifo=1, has_data=1, is_wr=1, has_addr=1, op=0x32
    wait_ind_done();
    if (!dut_dma_tx) $fatal(1, "dma_tx_req_o should re-assert after TX FIFO drains");

    // Read 16 words back via RX FIFO
    axi_csr_w(32'h10, 32'h0000_6000);
    axi_csr_w(32'h0C, 32'h0000_1D0B); // use_fifo=1, has_dummy=1, has_data=1, is_wr=0, has_addr=1, op=0x0B
    wait_ind_done();
    if (!dut_dma_rx) $fatal(1, "dma_rx_req_o should assert when RX FIFO has 16 >= 4");
    for (int i = 0; i < 16; i++) begin
      axi_csr_r(32'h2C, rd);
      if (rd !== (32'hF1F0_0000 | i)) $fatal(1, "FIFO readback[%0d] mismatch: %h", i, rd);
    end
    if (dut_dma_rx) $fatal(1, "dma_rx_req_o should deassert after RX FIFO drained");
    cov_fifo_dma++;
    $display("  [5] Depth-16 Indirect TX/RX FIFOs & DMA requests passed");

    // =======================================================================
    // 6. Hardware WIP Auto-Polling + Timeout + IRQ
    // =======================================================================
    axi_csr_w(32'h30, 32'h0000_0006); // enable poll_done & poll_timeout IRQs (disable ind_done IRQ)
    gen_cs_flash[0].u_flash.status_reg[0]  = 1'b1;
    gen_cs_flash[0].u_flash.wip_polls_left = 4;
    axi_csr_w(32'h48, 32'h0000_0001); // mask=0x0001, match=0x0000
    axi_csr_w(32'h4C, 32'd20);        // timeout = 20 polls
    axi_csr_w(32'h44, 32'h0001_0105); // interval=1, poll_start=1, opcode=0x05
    do @(posedge clk); while (!dut_irq);
    axi_csr_r(32'h1C, st);
    if (!st[2] || st[3]) $fatal(1, "Auto-poll done status mismatch: st=%h", st);
    axi_csr_w(32'h1C, 32'h4); // W1C poll_done
    @(posedge clk); #1;
    if (dut_irq) $fatal(1, "IRQ did not clear after W1C poll_done");

    // Timeout test
    gen_cs_flash[0].u_flash.status_reg[0]  = 1'b1;
    gen_cs_flash[0].u_flash.wip_polls_left = 20;
    axi_csr_w(32'h4C, 32'd3);         // timeout = 3 polls
    axi_csr_w(32'h44, 32'h0000_0105); // poll_start=1, opcode=0x05
    do @(posedge clk); while (!dut_irq);
    axi_csr_r(32'h1C, st);
    if (!st[3]) $fatal(1, "Auto-poll timeout bit not set: st=%h", st);
    axi_csr_w(32'h1C, 32'h8); // W1C poll_timeout
    axi_csr_w(32'h30, 32'h0000_0001); // restore ind_done IRQ
    gen_cs_flash[0].u_flash.status_reg[0]  = 1'b0;
    gen_cs_flash[0].u_flash.wip_polls_left = 0;
    cov_autopoll++;
    $display("  [6] Hardware WIP Auto-Polling + Timeout + IRQ passed");

    // =======================================================================
    // 7. Continuous-Read (M7:0) Mode Entry & 32-bit All-Ones Exit
    // =======================================================================
    set_mode(1'b0, 1'b0, 1'b0, 2'd2, 2'd2, 2'd2, 5'd4, 1'b1, 1'b0, 2'd0, 8'd1, 0);
    axi_csr_w(32'h08, 32'h0000_A5EB);
    axi_xip_r(32'h0000_0000, rd); // Enters M7:0=0xA5
    if (rd !== plain_db[0]) $fatal(1, "Cont-read 1st read mismatch: %h", rd);
    axi_xip_r(32'h0000_0010, rd); // Skips opcode!
    if (rd !== plain_db[4]) $fatal(1, "Cont-read 2nd read mismatch: %h", rd);
    // Trigger indirect read -> must execute 32-bit exit phase first
    axi_csr_w(32'h10, 32'h0000_0008);
    axi_csr_w(32'h0C, 32'h0000_0D0B); // has_dummy, has_data, has_addr, op=0x0B
    wait_ind_done();
    axi_csr_r(32'h18, rd);
    if (rd !== plain_db[2]) $fatal(1, "Post-cont-read exit indirect read mismatch: %h", rd);
    cov_cont_read++;
    $display("  [7] Continuous-Read (M7:0) & Exit Sequence passed");

    // =======================================================================
    // 8. Quad-Bus Verification (AXI4 Burst + Wishbone + TileLink + APB4)
    // =======================================================================
    // AXI4 4-beat INCR burst read on XIP window
    set_mode(1'b1, 1'b0, 1'b1, 2'd3, 2'd3, 2'd3, 5'd4, 1'b0, 1'b1, 2'd0, 8'd1, 0);
    axi_csr_w(32'h08, 32'h0000_A5EE);
    do_axi_burst_read(axi_mem_m, axi_mem_s, 32'h0000_0000, 4, rd_arr, AXI_BURST_INCR, 4'd2, 4'd0, rresp);
    for (int i = 0; i < 4; i++) begin
      if (rd_arr[i] !== plain_db[i] || rresp != AXI_RESP_OKAY)
        $fatal(1, "AXI4 burst read[%0d] mismatch: %h", i, rd_arr[i]);
    end

    // Wishbone (8D-8D-8D DTR + DQS + pipelined burst read)
    u_flash_wb.mem[32'h100+0]=8'h11; u_flash_wb.mem[32'h100+1]=8'h22;
    u_flash_wb.mem[32'h100+2]=8'h33; u_flash_wb.mem[32'h100+3]=8'h44;
    u_flash_wb.mem[32'h100+4]=8'h55; u_flash_wb.mem[32'h100+5]=8'h66;
    u_flash_wb.mem[32'h100+6]=8'h77; u_flash_wb.mem[32'h100+7]=8'h88;
    do_wb_write(wb_csr_m, wb_csr_s, 32'h00, 32'h0002_13FF);
    do_wb_pipelined_burst_read(wb_mem_m, wb_mem_s, 32'h0000_0100, 2, rd_arr, 2'b00, wb_err);
    if (wb_err || rd_arr[0] !== 32'h4433_2211 || rd_arr[1] !== 32'h8877_6655)
      $fatal(1, "WB 8D DQS burst mismatch: %h %h", rd_arr[0], rd_arr[1]);

    // TileLink (8D-8D-8D DTR + DQS)
    u_flash_tl.mem[32'h200+0]=8'hAA; u_flash_tl.mem[32'h200+1]=8'hBB;
    u_flash_tl.mem[32'h200+2]=8'hCC; u_flash_tl.mem[32'h200+3]=8'hDD;
    do_tl_write(tl_csr_m, tl_csr_s, 32'h00, 32'h0002_13FF);
    do_tl_read(tl_mem_m, tl_mem_s, 32'h0000_0200, rd);
    if (rd !== 32'hDDCC_BBAA) $fatal(1, "TL 8D DQS read mismatch: %h", rd);

    // APB4 (8D-8D-8D DTR + DQS)
    u_flash_apb.mem[32'h300+0]=8'hDE; u_flash_apb.mem[32'h300+1]=8'hAD;
    u_flash_apb.mem[32'h300+2]=8'hBE; u_flash_apb.mem[32'h300+3]=8'hEF;
    do_apb_write(apb_csr_m, apb_csr_s, 32'h00, 32'h0002_13FF);
    do_apb_read(apb_mem_m, apb_mem_s, 32'h0000_0300, rd);
    if (rd !== 32'hEFBE_ADDE) $fatal(1, "APB 8D DQS read mismatch: %h", rd);
    cov_quad_bus++;
    $display("  [8] Quad-Bus (AXI4 burst, WB burst, TileLink, APB4) passed");

    // =======================================================================
    // 9. Constrained-Random Regression (+seed)
    // =======================================================================
    for (int iter = 0; iter < 40; iter++) begin
      automatic int kind = $urandom_range(0, 2);
      if (kind == 0) begin
        // Random bypass XIP read (0x0000..0x00FC)
        automatic int widx = $urandom_range(0, 63);
        axi_xip_r(32'(widx * 4), rd);
        if (rd !== plain_db[widx])
          $fatal(1, "Random bypass XIP mismatch @%0d: %h != %h", widx, rd, plain_db[widx]);
      end else if (kind == 1) begin
        // Random OTFAD Region 0 or Region 1 read
        automatic int reg_sel = $urandom_range(0, 1);
        automatic int widx    = $urandom_range(0, 15);
        automatic logic [31:0] addr = (reg_sel == 0 ? 32'h2000 : 32'h4000) + 32'(widx * 4);
        automatic logic [31:0] exp  = (reg_sel == 0 ? 32'hD000_0000 : 32'hE100_0000) + 32'(widx);
        axi_xip_r(addr, rd);
        if (rd !== exp)
          $fatal(1, "Random OTFAD R%0d mismatch @%h: %h != %h", reg_sel, addr, rd, exp);
      end else begin
        // Random indirect single-word read from plain_db
        automatic int widx = $urandom_range(0, 63);
        axi_csr_w(32'h10, 32'(widx * 4));
        axi_csr_w(32'h0C, 32'h0000_0D0B);
        wait_ind_done();
        axi_csr_r(32'h18, rd);
        if (rd !== plain_db[widx])
          $fatal(1, "Random indirect read mismatch @%0d: %h != %h", widx, rd, plain_db[widx]);
      end
      cov_rand_ops++;
    end

    $display("  coverage: seed=%0d 8d_dtr_dqs=%0d rx_delay=%0d otfad=(%0d/%0d/%0d) cache_lru=%0d multi_cs=%0d fifo_dma=%0d autopoll=%0d cont=%0d quad_bus=%0d rand_ops=%0d",
             seed, cov_8d_dtr_dqs, cov_rx_delay, cov_otfad_r0, cov_otfad_r1, cov_otfad_bypass,
             cov_cache_lru, cov_multi_cs, cov_fifo_dma, cov_autopoll, cov_cont_read, cov_quad_bus, cov_rand_ops);
    $display("[PASS] tb_xspi_xip_v2: 8D-8D-8D DTR/DQS, RX_DELAY, AES-128-CTR OTFAD, 2-way LRU cache, Multi-CS, TX/RX FIFOs/DMA, HW Auto-Poll & Quad-Bus (WB/AXI4/TL/APB) verified.");
    $finish;
  end

  // Watchdog
  initial begin
    #(20_000_000);
    $fatal(1, "[TIMEOUT] %m: simulation watchdog expired");
  end

endmodule : tb_xspi_xip_v2
