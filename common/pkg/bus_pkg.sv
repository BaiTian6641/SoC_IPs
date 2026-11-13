// SPDX-License-Identifier: Apache-2.0
// bus_pkg.sv - Unified Definitions for Wishbone B4, AXI4/AXI4-Lite, TileLink (TL-UL/TL-UH),
//              Internal CSR Bus, and Generic Burst Memory Interface.

package bus_pkg;

  // Bus Architecture Selector for Unified *_top Wrappers
  typedef enum int unsigned {
    BUS_WISHBONE = 0,
    BUS_AXI4     = 1,
    BUS_TILELINK = 2
  } bus_proto_e;

  // -------------------------------------------------------------------------
  // Generic Internal CSR Interface (Single-Beat Slave Register Access)
  // Handshake completes when (req.valid && rsp.ready).
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic        valid;
    logic        write;
    logic [31:0] addr;
    logic [31:0] wdata;
    logic [3:0]  wstrb;
  } csr_req_t;

  typedef struct packed {
    logic        ready;
    logic [31:0] rdata;
    logic        err;
  } csr_rsp_t;

  // -------------------------------------------------------------------------
  // Generic Burst Memory Interface (Used by DMA/XIP/Memory Controllers)
  // Unidirectional structs: mem_req_t (Initiator->Target), mem_rsp_t (Target->Initiator)
  // -------------------------------------------------------------------------
  typedef struct packed {
    logic        valid;
    logic        write;
    logic [31:0] addr;
    logic [31:0] wdata;
    logic [3:0]  wstrb;
    logic [7:0]  len;        // Beats - 1 (0 = single beat)
    logic [2:0]  size;       // 2^size bytes per beat (3'd2 = 4B)
    logic        burst_wrap; // 0 = INCR, 1 = WRAP
    logic        last;       // Asserted on final write beat
    logic [3:0]  id;
    logic [3:0]  qos;
    logic        resp_ready; // Initiator ready to accept read/write response
  } mem_req_t;

  typedef struct packed {
    logic        req_ready;
    logic        resp_valid;
    logic [31:0] rdata;
    logic        err;
    logic        last;
    logic [3:0]  id;
  } mem_rsp_t;

  // -------------------------------------------------------------------------
  // Wishbone B4 (Classic & Pipelined) 32-bit Standard Bundles
  // CTI: 000=Classic, 010=Inc Burst, 111=End-of-Burst
  // BTE: 00=Linear, 01=4-beat wrap, 10=8-beat wrap, 11=16-beat wrap
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    WB_CTI_CLASSIC = 3'b000,
    WB_CTI_CONST   = 3'b001,
    WB_CTI_INC     = 3'b010,
    WB_CTI_EOB     = 3'b111
  } wb_cti_e;

  typedef struct packed {
    logic        cyc;
    logic        stb;
    logic        we;
    logic [31:0] adr;
    logic [31:0] dat;
    logic [3:0]  sel;
    logic [2:0]  cti;
    logic [1:0]  bte;
  } wb_m2s_t;

  typedef struct packed {
    logic        ack;
    logic        stall;
    logic        err;
    logic        rty;
    logic [31:0] dat;
  } wb_s2m_t;

  // -------------------------------------------------------------------------
  // AXI4 & AXI4-Lite 32-bit Standard Bundles
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    AXI_BURST_FIXED = 2'b00,
    AXI_BURST_INCR  = 2'b01,
    AXI_BURST_WRAP  = 2'b10
  } axi_burst_e;

  typedef enum logic [1:0] {
    AXI_RESP_OKAY   = 2'b00,
    AXI_RESP_EXOKAY = 2'b01,
    AXI_RESP_SLVERR = 2'b10,
    AXI_RESP_DECERR = 2'b11
  } axi_resp_e;

  typedef struct packed {
    // AW Channel
    logic [3:0]  awid;
    logic [31:0] awaddr;
    logic [7:0]  awlen;
    logic [2:0]  awsize;
    logic [1:0]  awburst;
    logic [2:0]  awprot;
    logic [3:0]  awqos;
    logic        awvalid;
    // W Channel
    logic [31:0] wdata;
    logic [3:0]  wstrb;
    logic        wlast;
    logic        wvalid;
    // B Channel
    logic        bready;
    // AR Channel
    logic [3:0]  arid;
    logic [31:0] araddr;
    logic [7:0]  arlen;
    logic [2:0]  arsize;
    logic [1:0]  arburst;
    logic [2:0]  arprot;
    logic [3:0]  arqos;
    logic        arvalid;
    // R Channel
    logic        rready;
  } axi4_m2s_t;

  typedef struct packed {
    // AW Channel
    logic        awready;
    // W Channel
    logic        wready;
    // B Channel
    logic [3:0]  bid;
    logic [1:0]  bresp;
    logic        bvalid;
    // AR Channel
    logic        arready;
    // R Channel
    logic [3:0]  rid;
    logic [31:0] rdata;
    logic [1:0]  rresp;
    logic        rlast;
    logic        rvalid;
  } axi4_s2m_t;

  // -------------------------------------------------------------------------
  // TileLink (TL-UL & TL-UH) 32-bit Bundles (Channel A & Channel D)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    TL_OP_PUT_FULL_DATA    = 3'd0,
    TL_OP_PUT_PARTIAL_DATA = 3'd1,
    TL_OP_ARITHMETIC_DATA  = 3'd2,
    TL_OP_LOGICAL_DATA     = 3'd3,
    TL_OP_GET              = 3'd4,
    TL_OP_INTENT           = 3'd5
  } tl_a_op_e;

  typedef enum logic [2:0] {
    TL_OP_ACCESS_ACK      = 3'd0,
    TL_OP_ACCESS_ACK_DATA = 3'd1,
    TL_OP_HINT_ACK        = 3'd2
  } tl_d_op_e;

  typedef struct packed {
    logic        a_valid;
    logic [2:0]  a_opcode;
    logic [2:0]  a_param;
    logic [3:0]  a_size;    // 2^a_size total bytes in message (2 = 4B single beat)
    logic [7:0]  a_source;
    logic [31:0] a_address;
    logic [3:0]  a_mask;
    logic [31:0] a_data;
    logic        a_corrupt;
    logic        d_ready;
  } tl_m2s_t;

  typedef struct packed {
    logic        a_ready;
    logic        d_valid;
    logic [2:0]  d_opcode;
    logic [1:0]  d_param;
    logic [3:0]  d_size;
    logic [7:0]  d_source;
    logic [3:0]  d_sink;
    logic        d_denied;
    logic [31:0] d_data;
    logic        d_corrupt;
  } tl_s2m_t;

  // Address Map Entry for Crossbars & Smart Switch
  typedef struct packed {
    logic [31:0] base_addr;
    logic [31:0] addr_mask; // 1 bits indicate address bits compared against base_addr
  } addr_map_rule_t;

  // -------------------------------------------------------------------------
  // Protocol Address Calculation & Boundary Helpers
  // -------------------------------------------------------------------------
  function automatic logic [31:0] axi_next_addr(
    input logic [31:0] addr,
    input logic [2:0]  size,
    input logic [1:0]  burst,
    input logic [7:0]  len
  );
    logic [31:0] step, aligned, inc_addr, wrap_mask;
    step     = 32'd1 << size;
    aligned  = addr & ~(step - 32'd1);
    inc_addr = aligned + step;
    if (burst == AXI_BURST_FIXED) begin
      return addr;
    end else if (burst == AXI_BURST_WRAP) begin
      wrap_mask = ((32'(len) + 32'd1) << size) - 32'd1;
      return (addr & ~wrap_mask) | (inc_addr & wrap_mask);
    end else begin
      return inc_addr;
    end
  endfunction

  function automatic logic axi_crosses_4kb(
    input logic [31:0] addr,
    input logic [7:0]  len,
    input logic [2:0]  size,
    input logic [1:0]  burst
  );
    logic [15:0] step_mask, start_off, end_off;
    step_mask = (16'd1 << size) - 16'd1;
    start_off = {4'd0, addr[11:0]} & ~step_mask;
    end_off   = start_off + ({8'd0, len} << size);
    return (burst == AXI_BURST_INCR) && (end_off >= 16'h1000);
  endfunction

  function automatic logic [31:0] wb_next_addr(
    input logic [31:0] addr,
    input logic [2:0]  cti,
    input logic [1:0]  bte,
    input logic [2:0]  size
  );
    logic [31:0] step, inc_addr, wrap_mask;
    step     = 32'd1 << size;
    inc_addr = addr + step;
    if (cti == WB_CTI_CONST) begin
      return addr;
    end else if (cti == WB_CTI_INC) begin
      unique case (bte)
        2'b01:   wrap_mask = (32'd4  << size) - 32'd1; // 4-beat wrap
        2'b10:   wrap_mask = (32'd8  << size) - 32'd1; // 8-beat wrap
        2'b11:   wrap_mask = (32'd16 << size) - 32'd1; // 16-beat wrap
        default: wrap_mask = '0;
      endcase
      if (bte == 2'b00) return inc_addr;
      else              return (addr & ~wrap_mask) | (inc_addr & wrap_mask);
    end else begin
      return inc_addr;
    end
  endfunction

endpackage : bus_pkg
