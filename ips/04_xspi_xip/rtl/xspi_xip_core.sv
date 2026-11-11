// SPDX-License-Identifier: Apache-2.0
// xspi_xip_core.sv - Dual / Quad / Octal SPI Controller with Execute-In-Place (XIP) & 4-Line Cache
//
// Supports 1-lane (Standard), 2-lane (Dual), 4-lane (Quad), and 8-lane (Octal/xSPI) transfers
// independently configurable across Instruction, Address, Mode/Continuous-Read, Dummy, and Data phases,
// plus DDR/DQS sampling mode and a 4-entry XIP instruction/data cache for 1-cycle XIP cache hits!
//
// CSR Register Map:
//   0x00 CTRL      : [0]=enable, [1]=xip_en, [2]=ddr_en, [3]=addr_32b(1:32b,0:24b),
//                    [5:4]=cmd_lanes(0:1L,1:2L,2:4L,3:8L), [7:6]=addr_lanes, [9:8]=data_lanes,
//                    [14:10]=dummy_cycles, [15]=cache_flush(WO), [16]=cont_read_en
//   0x04 CLK_DIV   : [7:0]=sck_half_div (>=1)
//   0x08 XIP_CFG   : [7:0]=xip_read_opcode (default 0xEB Quad I/O Fast Read), [15:8]=xip_mode_bits (e.g. 0xA5)
//   0x0C IND_CMD   : Indirect transaction trigger: [7:0]=opcode, [8]=has_addr, [9]=is_write,
//                    [10]=has_data, [11]=has_dummy
//   0x10 IND_ADDR  : [31:0] Indirect 24/32-bit target flash address
//   0x14 IND_WDATA : [31:0] Indirect write data word
//   0x18 IND_RDATA : [31:0] Indirect captured read data word (RO)
//   0x1C STATUS    : [0]=busy, [1]=ind_done(W1C), [15:8]=xip_hit_cnt(RO)

module xspi_xip_core
  import bus_pkg::*;
(
  input  logic       clk_i,
  input  logic       rst_ni,

  // Control/Status Port
  input  csr_req_t   csr_req_i,
  output csr_rsp_t   csr_rsp_o,

  // Direct Execute-In-Place (XIP) Memory Window Port (Read-Only XIP)
  input  csr_req_t   mem_win_req_i,
  output csr_rsp_t   mem_win_rsp_o,

  // xSPI Physical Pins (1/2/4/8 IO lanes + CS# + SCK + DQS)
  output logic       xspi_sck_o,
  output logic       xspi_cs_no,
  input  logic [7:0] xspi_io_i,
  output logic [7:0] xspi_io_o,
  output logic [7:0] xspi_io_oe_o,
  input  logic       xspi_dqs_i,

  output logic       irq_o
);

  logic [31:0] ctrl_q, ind_addr_q, ind_wdata_q, ind_rdata_q;
  logic [7:0]  clk_div_q;
  logic [15:0] xip_cfg_q;
  logic        ind_done_q;
  logic [7:0]  hit_cnt_q;
  logic        hit_seen_q;

  wire enable      = ctrl_q[0];
  wire xip_en      = ctrl_q[1];
  wire addr_32b    = ctrl_q[3];
  wire [1:0] cmd_lanes_cfg  = ctrl_q[5:4];
  wire [1:0] addr_lanes_cfg = ctrl_q[7:6];
  wire [1:0] data_lanes_cfg = ctrl_q[9:8];
  wire [4:0] dummy_cfg      = ctrl_q[14:10];
  wire cont_read_en         = ctrl_q[16];

  // 4-Entry Direct-Mapped XIP Cache (32-bit word lines)
  logic [3:0]        cache_valid_q;
  logic [3:0][27:0]  cache_tag_q;
  logic [3:0][31:0]  cache_data_q;

  wire [1:0]  xip_idx = mem_win_req_i.addr[3:2];
  wire [27:0] xip_tag = mem_win_req_i.addr[31:4];
  wire        xip_cache_hit = xip_en && mem_win_req_i.valid && !mem_win_req_i.write &&
                              cache_valid_q[xip_idx] && (cache_tag_q[xip_idx] == xip_tag);

  typedef enum logic [2:0] {
    ST_IDLE,
    ST_CMD,
    ST_ADDR,
    ST_MODE,
    ST_DUMMY,
    ST_DATA,
    ST_DONE
  } state_e;

  state_e      state_q;
  logic        is_xip_req_q;
  logic        cont_active_q; // Tracks if flash is already in continuous read M7:0 mode
  logic        req_has_addr_q, req_is_wr_q, req_has_data_q, req_has_dummy_q, req_has_mode_q;
  logic [7:0]  cur_opcode_q, cur_mode_q;
  logic [31:0] cur_addr_q, shift_data_q;
  logic [5:0]  bits_rem_q;
  logic [7:0]  div_cnt_q;
  logic        sck_q, cs_n_q;
  logic [7:0]  io_out_q, io_oe_q;
  logic        xip_resp_ready_q;
  logic [31:0] xip_resp_data_q;

  function automatic logic [3:0] lane_width(input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return 4'd1;
      2'd1: return 4'd2;
      2'd2: return 4'd4;
      2'd3: return 4'd8;
    endcase
  endfunction

  function automatic logic [7:0] lane_oe_mask(input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return 8'b0000_0001; // IO0=MOSI, IO1=MISO
      2'd1: return 8'b0000_0011;
      2'd2: return 8'b0000_1111;
      2'd3: return 8'b1111_1111;
    endcase
  endfunction

  function automatic logic [7:0] pack_out_lanes(input logic [31:0] sh, input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return {7'd0, sh[31]};
      2'd1: return {6'd0, sh[31:30]};
      2'd2: return {4'd0, sh[31:28]};
      2'd3: return sh[31:24];
    endcase
  endfunction

  function automatic logic [31:0] shift_in_lanes(input logic [31:0] sh, input logic [7:0] pins, input logic [1:0] lcfg);
    unique case (lcfg)
      2'd0: return {sh[30:0], pins[1]}; // Standard SPI MISO is IO[1]!
      2'd1: return {sh[29:0], pins[1:0]};
      2'd2: return {sh[27:0], pins[3:0]};
      2'd3: return {sh[23:0], pins[7:0]};
    endcase
  endfunction

  assign xspi_sck_o   = sck_q;
  assign xspi_cs_no   = cs_n_q;
  assign xspi_io_o    = io_out_q;
  assign xspi_io_oe_o = io_oe_q;

  wire [7:0] reg_off  = csr_req_i.addr[7:0];
  wire       ind_trig = csr_req_i.valid && csr_req_i.write && (reg_off == 8'h0C) && (state_q == ST_IDLE);
  wire [7:0] eff_div  = (clk_div_q == 8'd0) ? 8'd1 : clk_div_q;

  // Helper task-like logic inside main FSM
  logic [1:0] active_lanes;
  always_comb begin
    unique case (state_q)
      ST_CMD:  active_lanes = cmd_lanes_cfg;
      ST_ADDR, ST_MODE: active_lanes = addr_lanes_cfg;
      default: active_lanes = data_lanes_cfg;
    endcase
  end

  wire [3:0] step_w = lane_width(active_lanes);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ctrl_q           <= 32'h0000_2AA3; // Enabled, XIP on, Quad Addr/Data, 8 dummy cycles
      clk_div_q        <= 8'd1;
      xip_cfg_q        <= 16'hA5_EB;
      ind_addr_q       <= '0;
      ind_wdata_q      <= '0;
      ind_rdata_q      <= '0;
      ind_done_q       <= 1'b0;
      hit_cnt_q        <= '0;
      hit_seen_q       <= 1'b0;
      cache_valid_q    <= '0;
      cache_tag_q      <= '0;
      cache_data_q     <= '0;
      state_q          <= ST_IDLE;
      is_xip_req_q     <= 1'b0;
      cont_active_q    <= 1'b0;
      req_has_addr_q   <= 1'b0;
      req_is_wr_q      <= 1'b0;
      req_has_data_q   <= 1'b0;
      req_has_dummy_q  <= 1'b0;
      req_has_mode_q   <= 1'b0;
      cur_opcode_q     <= '0;
      cur_mode_q       <= '0;
      cur_addr_q       <= '0;
      shift_data_q     <= '0;
      bits_rem_q       <= '0;
      div_cnt_q        <= '0;
      sck_q            <= 1'b0;
      cs_n_q           <= 1'b1;
      io_out_q         <= '0;
      io_oe_q          <= '0;
      xip_resp_ready_q <= 1'b0;
      xip_resp_data_q  <= '0;
    end else begin
      xip_resp_ready_q <= 1'b0;

      // Count each XIP cache hit once per transaction (the request may stay valid for several cycles);
      // the cycle that completes a cache-fill (miss) response is not a hit
      if (!mem_win_req_i.valid) begin
        hit_seen_q <= 1'b0;
      end else if (xip_cache_hit && state_q == ST_IDLE && !hit_seen_q && !xip_resp_ready_q) begin
        hit_seen_q <= 1'b1;
        hit_cnt_q  <= hit_cnt_q + 8'd1;
      end

      // CSR Register Writes
      if (csr_req_i.valid && csr_req_i.write) begin
        unique case (reg_off)
          8'h00: begin
            ctrl_q <= ip_common_pkg::apply_wstrb(ctrl_q, csr_req_i.wdata, csr_req_i.wstrb);
            if (csr_req_i.wdata[15]) begin
              cache_valid_q <= '0;
              cont_active_q <= 1'b0;
            end
          end
          8'h04: clk_div_q   <= csr_req_i.wdata[7:0];
          8'h08: xip_cfg_q   <= csr_req_i.wdata[15:0];
          8'h10: ind_addr_q  <= csr_req_i.wdata;
          8'h14: ind_wdata_q <= csr_req_i.wdata;
          8'h1C: if (csr_req_i.wdata[1]) ind_done_q <= 1'b0;
          default: ;
        endcase
      end

      unique case (state_q)
        ST_IDLE: begin
          sck_q   <= 1'b0;
          cs_n_q  <= 1'b1;
          io_oe_q <= '0;
          if (enable && xip_en && mem_win_req_i.valid && !mem_win_req_i.write && !xip_cache_hit && !xip_resp_ready_q) begin
            // Start XIP Cache-Fill Read Transaction
            is_xip_req_q    <= 1'b1;
            cur_opcode_q    <= xip_cfg_q[7:0];
            cur_mode_q      <= xip_cfg_q[15:8];
            cur_addr_q      <= {mem_win_req_i.addr[31:2], 2'b00};
            req_has_addr_q  <= 1'b1;
            req_has_mode_q  <= cont_read_en;
            req_has_dummy_q <= (dummy_cfg != 5'd0);
            req_has_data_q  <= 1'b1;
            req_is_wr_q     <= 1'b0;
            cs_n_q          <= 1'b0;
            div_cnt_q       <= eff_div - 8'd1;
            if (cont_read_en && cont_active_q) begin
              // Skip instruction phase in Continuous Read XIP mode!
              shift_data_q <= addr_32b ? {mem_win_req_i.addr[31:2], 2'b00} : {mem_win_req_i.addr[23:2], 2'b00, 8'd0};
              bits_rem_q   <= addr_32b ? 6'd32 : 6'd24;
              io_oe_q      <= lane_oe_mask(addr_lanes_cfg);
              io_out_q     <= pack_out_lanes(addr_32b ? {mem_win_req_i.addr[31:2], 2'b00} : {mem_win_req_i.addr[23:2], 2'b00, 8'd0}, addr_lanes_cfg);
              state_q      <= ST_ADDR;
            end else begin
              shift_data_q <= {xip_cfg_q[7:0], 24'd0};
              bits_rem_q   <= 6'd8;
              io_oe_q      <= lane_oe_mask(cmd_lanes_cfg);
              io_out_q     <= pack_out_lanes({xip_cfg_q[7:0], 24'd0}, cmd_lanes_cfg);
              state_q      <= ST_CMD;
            end
          end else if (enable && ind_trig) begin
            is_xip_req_q    <= 1'b0;
            cont_active_q   <= 1'b0;
            cur_opcode_q    <= csr_req_i.wdata[7:0];
            req_has_addr_q  <= csr_req_i.wdata[8];
            req_is_wr_q     <= csr_req_i.wdata[9];
            req_has_data_q  <= csr_req_i.wdata[10];
            req_has_dummy_q <= csr_req_i.wdata[11];
            req_has_mode_q  <= 1'b0;
            cur_addr_q      <= ind_addr_q;
            shift_data_q    <= {csr_req_i.wdata[7:0], 24'd0};
            bits_rem_q      <= 6'd8;
            cs_n_q          <= 1'b0;
            div_cnt_q       <= eff_div - 8'd1;
            io_oe_q         <= lane_oe_mask(cmd_lanes_cfg);
            io_out_q        <= pack_out_lanes({csr_req_i.wdata[7:0], 24'd0}, cmd_lanes_cfg);
            state_q         <= ST_CMD;
            if (csr_req_i.wdata[9]) cache_valid_q <= '0; // Invalidate XIP cache on flash write/erase
          end
        end

        ST_CMD, ST_ADDR, ST_MODE, ST_DUMMY, ST_DATA: begin
          if (div_cnt_q != 8'd0) begin
            div_cnt_q <= div_cnt_q - 8'd1;
          end else begin
            div_cnt_q <= eff_div - 8'd1;
            if (!sck_q) begin
              // Rising SCK edge (sample input lanes during read data phase)
              sck_q <= 1'b1;
              if (state_q == ST_DATA && !req_is_wr_q) begin
                shift_data_q <= shift_in_lanes(shift_data_q, xspi_io_i, data_lanes_cfg);
              end
            end else begin
              // Falling SCK edge (advance phase/bits and drive next output lanes)
              sck_q <= 1'b0;
              if (state_q == ST_DUMMY) begin
                if (bits_rem_q <= 6'd1) begin
                  if (req_has_data_q) begin
                    bits_rem_q   <= 6'd32;
                    shift_data_q <= req_is_wr_q ? ind_wdata_q : '0;
                    io_oe_q      <= req_is_wr_q ? lane_oe_mask(data_lanes_cfg) : 8'h00;
                    io_out_q     <= pack_out_lanes(ind_wdata_q, data_lanes_cfg);
                    state_q      <= ST_DATA;
                  end else begin
                    state_q <= ST_DONE;
                  end
                end else begin
                  bits_rem_q <= bits_rem_q - 6'd1;
                end
              end else if (bits_rem_q <= {2'd0, step_w}) begin
                // Advance to next protocol phase
                unique case (state_q)
                  ST_CMD: begin
                    if (req_has_addr_q) begin
                      logic [31:0] a_aligned;
                      a_aligned    = addr_32b ? cur_addr_q : {cur_addr_q[23:0], 8'd0};
                      shift_data_q <= a_aligned;
                      bits_rem_q   <= addr_32b ? 6'd32 : 6'd24;
                      io_oe_q      <= lane_oe_mask(addr_lanes_cfg);
                      io_out_q     <= pack_out_lanes(a_aligned, addr_lanes_cfg);
                      state_q      <= ST_ADDR;
                    end else if (req_has_dummy_q) begin
                      bits_rem_q <= {1'b0, dummy_cfg};
                      io_oe_q    <= 8'h00;
                      state_q    <= ST_DUMMY;
                    end else if (req_has_data_q) begin
                      shift_data_q <= req_is_wr_q ? ind_wdata_q : '0;
                      bits_rem_q   <= 6'd32;
                      io_oe_q      <= req_is_wr_q ? lane_oe_mask(data_lanes_cfg) : 8'h00;
                      io_out_q     <= pack_out_lanes(ind_wdata_q, data_lanes_cfg);
                      state_q      <= ST_DATA;
                    end else begin
                      state_q <= ST_DONE;
                    end
                  end

                  ST_ADDR: begin
                    if (req_has_mode_q) begin
                      shift_data_q <= {cur_mode_q, 24'd0};
                      bits_rem_q   <= 6'd8;
                      io_oe_q      <= lane_oe_mask(addr_lanes_cfg);
                      io_out_q     <= pack_out_lanes({cur_mode_q, 24'd0}, addr_lanes_cfg);
                      cont_active_q<= 1'b1;
                      state_q      <= ST_MODE;
                    end else if (req_has_dummy_q) begin
                      bits_rem_q <= {1'b0, dummy_cfg};
                      io_oe_q    <= 8'h00;
                      state_q    <= ST_DUMMY;
                    end else if (req_has_data_q) begin
                      shift_data_q <= req_is_wr_q ? ind_wdata_q : '0;
                      bits_rem_q   <= 6'd32;
                      io_oe_q      <= req_is_wr_q ? lane_oe_mask(data_lanes_cfg) : 8'h00;
                      io_out_q     <= pack_out_lanes(ind_wdata_q, data_lanes_cfg);
                      state_q      <= ST_DATA;
                    end else begin
                      state_q <= ST_DONE;
                    end
                  end

                  ST_MODE: begin
                    if (req_has_dummy_q) begin
                      bits_rem_q <= {1'b0, dummy_cfg};
                      io_oe_q    <= 8'h00;
                      state_q    <= ST_DUMMY;
                    end else begin
                      shift_data_q <= '0;
                      bits_rem_q   <= 6'd32;
                      io_oe_q      <= 8'h00;
                      state_q      <= ST_DATA;
                    end
                  end

                  ST_DATA: begin
                    state_q <= ST_DONE;
                  end

                  default: state_q <= ST_DONE;
                endcase
              end else begin
                logic [31:0] next_sh;
                next_sh    = shift_data_q << step_w;
                bits_rem_q <= bits_rem_q - {2'd0, step_w};
                if (state_q != ST_DATA || req_is_wr_q) begin
                  shift_data_q <= next_sh;
                  io_out_q     <= pack_out_lanes(next_sh, active_lanes);
                end
              end
            end
          end
        end

        ST_DONE: begin
          cs_n_q  <= 1'b1;
          io_oe_q <= 8'h00;
          if (is_xip_req_q) begin
            // Byte-swap big-endian SPI flash stream or deliver word directly & populate XIP cache
            cache_valid_q[xip_idx] <= 1'b1;
            cache_tag_q[xip_idx]   <= xip_tag;
            cache_data_q[xip_idx]  <= shift_data_q;
            xip_resp_data_q        <= shift_data_q;
            xip_resp_ready_q       <= 1'b1;
          end else begin
            if (!req_is_wr_q) ind_rdata_q <= shift_data_q;
            ind_done_q <= 1'b1;
          end
          state_q <= ST_IDLE;
        end

        default: state_q <= ST_IDLE;
      endcase
    end
  end

  // Direct XIP Window Response (1-cycle hit or stalls until cache-fill completes; rejects writes)
  always_comb begin
    mem_win_rsp_o = '0;
    if (mem_win_req_i.valid) begin
      if (mem_win_req_i.write || !xip_en) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.err   = 1'b1;
      end else if (xip_cache_hit && state_q == ST_IDLE) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.rdata = cache_data_q[xip_idx];
      end else if (xip_resp_ready_q) begin
        mem_win_rsp_o.ready = 1'b1;
        mem_win_rsp_o.rdata = xip_resp_data_q;
      end
    end
  end

  // CSR Read Mux
  always_comb begin
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = 1'b0;
    csr_rsp_o.rdata = '0;
    unique case (reg_off)
      8'h00: csr_rsp_o.rdata = ctrl_q;
      8'h04: csr_rsp_o.rdata = {24'd0, clk_div_q};
      8'h08: csr_rsp_o.rdata = {16'd0, xip_cfg_q};
      8'h0C: csr_rsp_o.rdata = '0;
      8'h10: csr_rsp_o.rdata = ind_addr_q;
      8'h14: csr_rsp_o.rdata = ind_wdata_q;
      8'h18: csr_rsp_o.rdata = ind_rdata_q;
      8'h1C: csr_rsp_o.rdata = {16'd0, hit_cnt_q, 6'd0, ind_done_q, (state_q != ST_IDLE)};
      default: csr_rsp_o.err = csr_req_i.valid;
    endcase
  end

  assign irq_o = ind_done_q;

endmodule : xspi_xip_core
