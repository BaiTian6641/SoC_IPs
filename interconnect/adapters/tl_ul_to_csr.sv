// SPDX-License-Identifier: Apache-2.0
// tl_ul_to_csr.sv - TileLink TL-UL / TL-UH Slave to Internal CSR Bus Adapter
// Supports PutFullData, PutPartialData, Get, and TL-UH Atomic Arithmetic/Logical RMW!

module tl_ul_to_csr
  import bus_pkg::*;
  import ip_common_pkg::*;
(
  input  logic     clk_i,
  input  logic     rst_ni,

  input  tl_m2s_t  tl_i,
  output tl_s2m_t  tl_o,

  output csr_req_t csr_req_o,
  input  csr_rsp_t csr_rsp_i
);

  typedef enum logic [1:0] {
    ST_REQ,
    ST_ATOMIC_WRITE,
    ST_RESP
  } state_e;

  state_e      state_q;
  logic [2:0]  d_opcode_q;
  logic [3:0]  d_size_q;
  logic [7:0]  d_source_q;
  logic [31:0] d_data_q;
  logic        d_err_q;

  logic [2:0]  amo_opcode_q;
  logic [2:0]  amo_param_q;
  logic [31:0] amo_addr_q;
  logic [31:0] amo_operand_q;
  logic [3:0]  amo_mask_q;

  wire is_put    = (tl_i.a_opcode == TL_OP_PUT_FULL_DATA) || (tl_i.a_opcode == TL_OP_PUT_PARTIAL_DATA);
  wire is_get    = (tl_i.a_opcode == TL_OP_GET);
  wire is_atomic = (tl_i.a_opcode == TL_OP_ARITHMETIC_DATA) || (tl_i.a_opcode == TL_OP_LOGICAL_DATA);
  wire is_intent = (tl_i.a_opcode == TL_OP_INTENT);

  function automatic logic [31:0] calc_amo(
    input logic [2:0]  op,
    input logic [2:0]  param,
    input logic [31:0] old_v,
    input logic [31:0] rhs_v
  );
    if (op == TL_OP_ARITHMETIC_DATA) begin
      unique case (param)
        3'd0: return ($signed(old_v) < $signed(rhs_v)) ? old_v : rhs_v; // MIN
        3'd1: return ($signed(old_v) > $signed(rhs_v)) ? old_v : rhs_v; // MAX
        3'd2: return (old_v < rhs_v) ? old_v : rhs_v;                   // MINU
        3'd3: return (old_v > rhs_v) ? old_v : rhs_v;                   // MAXU
        default: return old_v + rhs_v;                                  // ADD
      endcase
    end else begin
      unique case (param)
        3'd0: return old_v ^ rhs_v; // XOR
        3'd1: return old_v | rhs_v; // OR
        3'd2: return old_v & rhs_v; // AND
        default: return rhs_v;      // SWAP
      endcase
    end
  endfunction

  always_comb begin
    tl_o      = '0;
    csr_req_o = '0;

    unique case (state_q)
      ST_REQ: begin
        if (tl_i.a_valid) begin
          if (is_intent) begin
            tl_o.a_ready = 1'b1;
          end else begin
            csr_req_o.valid = 1'b1;
            csr_req_o.write = is_put;
            csr_req_o.addr  = tl_i.a_address;
            csr_req_o.wdata = tl_i.a_data;
            csr_req_o.wstrb = tl_i.a_mask;
            tl_o.a_ready    = csr_rsp_i.ready;
          end
        end
      end

      ST_ATOMIC_WRITE: begin
        csr_req_o.valid = 1'b1;
        csr_req_o.write = 1'b1;
        csr_req_o.addr  = amo_addr_q;
        csr_req_o.wdata = calc_amo(amo_opcode_q, amo_param_q, d_data_q, amo_operand_q);
        csr_req_o.wstrb = amo_mask_q;
      end

      ST_RESP: begin
        tl_o.d_valid   = 1'b1;
        tl_o.d_opcode  = d_opcode_q;
        tl_o.d_size    = d_size_q;
        tl_o.d_source  = d_source_q;
        tl_o.d_data    = d_data_q;
        tl_o.d_denied  = d_err_q;
        tl_o.d_corrupt = d_err_q && (d_opcode_q == TL_OP_ACCESS_ACK_DATA);
      end

      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q       <= ST_REQ;
      d_opcode_q    <= TL_OP_ACCESS_ACK;
      d_size_q      <= '0;
      d_source_q    <= '0;
      d_data_q      <= '0;
      d_err_q       <= 1'b0;
      amo_opcode_q  <= '0;
      amo_param_q   <= '0;
      amo_addr_q    <= '0;
      amo_operand_q <= '0;
      amo_mask_q    <= '0;
    end else begin
      unique case (state_q)
        ST_REQ: begin
          if (tl_i.a_valid && tl_o.a_ready) begin
            d_size_q   <= tl_i.a_size;
            d_source_q <= tl_i.a_source;
            if (is_intent) begin
              d_opcode_q <= TL_OP_HINT_ACK;
              d_data_q   <= '0;
              d_err_q    <= 1'b0;
              state_q    <= ST_RESP;
            end else if (is_atomic) begin
              d_opcode_q    <= TL_OP_ACCESS_ACK_DATA;
              d_data_q      <= csr_rsp_i.rdata;
              d_err_q       <= csr_rsp_i.err || tl_i.a_corrupt;
              amo_opcode_q  <= tl_i.a_opcode;
              amo_param_q   <= tl_i.a_param;
              amo_addr_q    <= tl_i.a_address;
              amo_operand_q <= tl_i.a_data;
              amo_mask_q    <= tl_i.a_mask;
              state_q       <= (csr_rsp_i.err || tl_i.a_corrupt) ? ST_RESP : ST_ATOMIC_WRITE;
            end else begin
              d_opcode_q <= is_get ? TL_OP_ACCESS_ACK_DATA : TL_OP_ACCESS_ACK;
              d_data_q   <= is_get ? csr_rsp_i.rdata : '0;
              d_err_q    <= csr_rsp_i.err || tl_i.a_corrupt;
              state_q    <= ST_RESP;
            end
          end
        end

        ST_ATOMIC_WRITE: begin
          if (csr_rsp_i.ready) begin
            d_err_q <= d_err_q | csr_rsp_i.err;
            state_q <= ST_RESP;
          end
        end

        ST_RESP: begin
          if (tl_i.d_ready) begin
            state_q <= ST_REQ;
          end
        end

        default: state_q <= ST_REQ;
      endcase
    end
  end

endmodule : tl_ul_to_csr
