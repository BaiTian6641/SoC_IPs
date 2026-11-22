// SPDX-License-Identifier: Apache-2.0
// sdram_hp_pkg.sv - Package for High-Throughput 4-Bank Open-Page SDR SDRAM Controller

package sdram_hp_pkg;

  // JEDEC SDR SDRAM Command Encodings {CS#, RAS#, CAS#, WE#}
  typedef enum logic [3:0] {
    SDR_CMD_LMR      = 4'b0000, // Load Mode Register
    SDR_CMD_REFRESH  = 4'b0001, // Auto-Refresh / Self-Refresh (with CKE=0)
    SDR_CMD_PRE      = 4'b0010, // Precharge (A10=0: bank, A10=1: all banks)
    SDR_CMD_ACTIVE   = 4'b0011, // Bank/Row Activate
    SDR_CMD_WRITE    = 4'b0100, // Column Write
    SDR_CMD_READ     = 4'b0101, // Column Read
    SDR_CMD_BST      = 4'b0110, // Burst Terminate
    SDR_CMD_NOP      = 4'b0111, // No Operation
    SDR_CMD_DESELECT = 4'b1111  // Device Deselect
  } sdr_cmd_e;

  // JEDEC Mode Register Burst Length Encodings (MR[2:0])
  typedef enum logic [2:0] {
    SDR_BL_1    = 3'b000,
    SDR_BL_2    = 3'b001,
    SDR_BL_4    = 3'b010,
    SDR_BL_8    = 3'b011,
    SDR_BL_FULL = 3'b111
  } sdr_bl_e;

  // CSR Byte Offsets
  localparam logic [7:0] REG_CTRL       = 8'h00;
  localparam logic [7:0] REG_CFG        = 8'h04;
  localparam logic [7:0] REG_STATUS     = 8'h08;
  localparam logic [7:0] REG_TIMING0    = 8'h0C;
  localparam logic [7:0] REG_TIMING1    = 8'h10;
  localparam logic [7:0] REG_TIMING2    = 8'h14;
  localparam logic [7:0] REG_PWR_CTRL   = 8'h18;
  localparam logic [7:0] REG_IRQ_STATUS = 8'h1C;
  localparam logic [7:0] REG_PERF_HITS  = 8'h20;
  localparam logic [7:0] REG_PERF_MISS  = 8'h24;
  localparam logic [7:0] REG_PERF_REFS  = 8'h28;
  localparam logic [7:0] REG_PERF_WORDS = 8'h2C;
  localparam logic [7:0] REG_IP_ID      = 8'h30;

  localparam logic [31:0] SDRAM_HP_IP_ID = 32'h5344_4850; // "SDHP"

  // Internal Scheduler Command Packet (64 bits)
  typedef struct packed {
    logic        is_write;
    logic [31:0] addr;       // Byte address (32-bit word aligned)
    logic [3:0]  words_m1;   // Host 32-bit words in chunk minus 1 (0..15)
    logic        last_chunk; // 1 on final chunk of host transaction
  } sdr_sched_cmd_t;

  // Configuration Bundle passed from Core CSRs to Scheduler
  typedef struct packed {
    logic        enable;
    logic        auto_ref_en;
    logic        open_page_en;
    logic        addr_map_brc; // 0: RBC (Row-Bank-Col), 1: BRC (Bank-Row-Col)
    logic [2:0]  burst_len;    // sdr_bl_e
    logic [1:0]  cas_lat;      // 2 or 3
    logic [1:0]  col_bits;     // 0:8b, 1:9b, 2:10b, 3:11b
    logic [1:0]  row_bits;     // 0:11b, 1:12b, 2:13b
    logic [3:0]  ref_urg_wm;
    logic [1:0]  rd_lat_adj;
    logic        wb_single_wr; // MR[9] Write Burst Mode (0=burst, 1=single)
    logic [3:0]  t_rp;
    logic [3:0]  t_rcd;
    logic [3:0]  t_ras;
    logic [4:0]  t_rc;
    logic [3:0]  t_rrd;
    logic [3:0]  t_wr;
    logic [4:0]  t_rfc;
    logic [3:0]  t_mrd;
    logic [3:0]  t_wtr;
    logic [3:0]  t_xsr;
    logic [15:0] t_refi;
    logic [15:0] t_init;
    logic        self_ref_req;
    logic        clk_susp_req;
    logic        auto_pd_en;
    logic [11:0] pd_thres;
  } sdr_cfg_t;

  // Decode {ba, row, col} from 32-bit byte address
  function automatic void decode_sdr_addr(
    input  logic [31:0] addr,
    input  logic        is_32b_sdram,
    input  logic        addr_map_brc,
    input  logic [1:0]  col_bits_cfg,
    input  logic [1:0]  row_bits_cfg,
    output logic [1:0]  ba_o,
    output logic [12:0] row_o,
    output logic [10:0] col_o
  );
    logic [31:0] shifted, row_raw;
    logic [10:0] col_mask;
    logic [12:0] row_mask;
    int unsigned col_w, row_w;

    // 32-bit word-aligned host address -> SDR element index
    // 16-bit SDRAM: 2B per SDR column -> shift by 1 (col[0] always 0 at word start)
    // 32-bit SDRAM: 4B per SDR column -> shift by 2
    shifted = is_32b_sdram ? {2'b00, addr[31:2]} : {1'b0, addr[31:2], 1'b0};

    unique case (col_bits_cfg)
      2'd0: begin col_w = 8;  col_mask = 11'h0FF; end
      2'd1: begin col_w = 9;  col_mask = 11'h1FF; end
      2'd2: begin col_w = 10; col_mask = 11'h3FF; end
      2'd3: begin col_w = 11; col_mask = 11'h7FF; end
      default: begin col_w = 9; col_mask = 11'h1FF; end
    endcase

    unique case (row_bits_cfg)
      2'd0: begin row_w = 11; row_mask = 13'h07FF; end
      2'd1: begin row_w = 12; row_mask = 13'h0FFF; end
      2'd2: begin row_w = 13; row_mask = 13'h1FFF; end
      default: begin row_w = 13; row_mask = 13'h1FFF; end
    endcase

    col_o   = shifted[10:0] & col_mask;
    row_raw = shifted >> col_w;

    if (addr_map_brc) begin
      // Bank - Row - Col
      row_o = row_raw[12:0] & row_mask;
      ba_o  = 2'((row_raw >> row_w) & 32'h3);
    end else begin
      // Row - Bank - Col (interleaves across banks on every row boundary)
      ba_o  = row_raw[1:0];
      row_o = row_raw[14:2] & row_mask;
    end
  endfunction

  // Compute Mode Register value A[12:0]
  function automatic logic [12:0] encode_sdr_mr(
    input logic [2:0] burst_len,
    input logic [1:0] cas_lat,
    input logic       wb_single_wr
  );
    logic [2:0] bl_field;
    logic [2:0] cl_field;
    unique case (burst_len)
      SDR_BL_1:    bl_field = 3'b000;
      SDR_BL_2:    bl_field = 3'b001;
      SDR_BL_4:    bl_field = 3'b010;
      SDR_BL_8:    bl_field = 3'b011;
      SDR_BL_FULL: bl_field = 3'b111;
      default:     bl_field = 3'b010;
    endcase
    cl_field = (cas_lat == 2'd3) ? 3'b011 : 3'b010;
    // A[12:10]=0, A[9]=wb_single_wr, A[8:7]=00, A[6:4]=CL, A[3]=0(Sequential), A[2:0]=BL
    return {3'b000, wb_single_wr, 2'b00, cl_field, 1'b0, bl_field};
  endfunction

endpackage : sdram_hp_pkg
