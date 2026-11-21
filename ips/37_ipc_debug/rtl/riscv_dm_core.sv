// SPDX-License-Identifier: Apache-2.0
// riscv_dm_core.sv - RISC-V Debug Specification (DM 0.13 / 1.0) Debug Module Core
//
// Features:
//   - Standard DMI request/response interface (connects to ips/11_jtag DTM or external bridge)
//   - Multi-hart run control (NUM_HARTS=4 default): dmcontrol, dmstatus, hartinfo, hawindowsel/hawindow, haltsum0
//   - Abstract commands: Access Register (cmdtype=0, 32-bit, aarpostincrement), data0..3, abstractauto, progbuf0..3
//   - System Bus Access (SBA): sbcs, sbaddress0, sbdata0 (8/16/32-bit, sbreadonaddr, sbautoincrement, sbreadondata)
//     driving a real mem_req_t/mem_rsp_t master port
//   - Read-only CSR monitor/status & progbuf aperture for SoC inspection

module riscv_dm_core
  import bus_pkg::*;
#(
  parameter int unsigned NUM_HARTS = 4,
  localparam int unsigned HARTSEL_W = (NUM_HARTS <= 1) ? 1 : $clog2(NUM_HARTS)
) (
  input  logic                  clk_i,
  input  logic                  rst_ni,

  // Debug Module Interface (DMI)
  input  logic                  dmi_req_valid_i,
  output logic                  dmi_req_ready_o,
  input  logic [6:0]            dmi_req_addr_i,
  input  logic [1:0]            dmi_req_op_i,
  input  logic [31:0]           dmi_req_data_i,

  output logic                  dmi_rsp_valid_o,
  input  logic                  dmi_rsp_ready_i,
  output logic [31:0]           dmi_rsp_data_o,
  output logic [1:0]            dmi_rsp_op_o,

  // Read-only CSR Monitor / Status / Progbuf Aperture
  input  csr_req_t              csr_req_i,
  output csr_rsp_t              csr_rsp_o,

  // System Bus Access (SBA) Master Port
  output mem_req_t              sba_req_o,
  input  mem_rsp_t              sba_rsp_i,

  // Hart Run-Control & Reset Sideband
  input  logic [NUM_HARTS-1:0]  halted_i,
  input  logic [NUM_HARTS-1:0]  resume_ack_i,
  output logic [NUM_HARTS-1:0]  halt_req_o,
  output logic [NUM_HARTS-1:0]  resume_req_o,
  output logic                  ndmreset_o,

  // Hart Abstract Register Access Interface
  output logic                  abs_req_valid_o,
  output logic                  abs_req_write_o,
  output logic [HARTSEL_W-1:0]  abs_req_hartsel_o,
  output logic [15:0]           abs_req_regno_o,
  output logic [31:0]           abs_req_wdata_o,
  input  logic                  abs_rsp_valid_i,
  input  logic [31:0]           abs_rsp_rdata_i,
  input  logic                  abs_rsp_err_i,

  output logic                  irq_o
);

  // DMI Operations
  localparam logic [1:0] DMI_OP_NOP    = 2'd0;
  localparam logic [1:0] DMI_OP_READ   = 2'd1;
  localparam logic [1:0] DMI_OP_WRITE  = 2'd2;

  localparam logic [1:0] DMI_RSP_OK    = 2'd0;
  localparam logic [1:0] DMI_RSP_FAIL  = 2'd2;

  // DMI Register Addresses
  localparam logic [6:0] DMI_DATA0       = 7'h04;
  localparam logic [6:0] DMI_DATA1       = 7'h05;
  localparam logic [6:0] DMI_DATA2       = 7'h06;
  localparam logic [6:0] DMI_DATA3       = 7'h07;
  localparam logic [6:0] DMI_DMCONTROL   = 7'h10;
  localparam logic [6:0] DMI_DMSTATUS    = 7'h11;
  localparam logic [6:0] DMI_HARTINFO    = 7'h12;
  localparam logic [6:0] DMI_HAWINDOWSEL = 7'h14;
  localparam logic [6:0] DMI_HAWINDOW    = 7'h15;
  localparam logic [6:0] DMI_ABSTRACTCS  = 7'h16;
  localparam logic [6:0] DMI_COMMAND     = 7'h17;
  localparam logic [6:0] DMI_ABSTRACTAUTO= 7'h18;
  localparam logic [6:0] DMI_PROGBUF0    = 7'h20;
  localparam logic [6:0] DMI_PROGBUF3    = 7'h23;
  localparam logic [6:0] DMI_SBCS        = 7'h38;
  localparam logic [6:0] DMI_SBADDRESS0  = 7'h39;
  localparam logic [6:0] DMI_SBDATA0     = 7'h3C;
  localparam logic [6:0] DMI_HALTSUM0    = 7'h40;

  // SBA FSM
  typedef enum logic [1:0] {
    SBA_IDLE,
    SBA_REQ,
    SBA_WAIT_RSP
  } sba_state_e;

  // DM Control & Run-Control State
  logic                  dmactive_q;
  logic                  ndmreset_q;
  logic [9:0]            hartsel_q;
  logic                  hasel_q;
  logic [NUM_HARTS-1:0]  hawindow_q;
  logic [NUM_HARTS-1:0]  halt_req_q;
  logic [NUM_HARTS-1:0]  resume_req_q;
  logic [NUM_HARTS-1:0]  resume_ack_q;
  logic [NUM_HARTS-1:0]  resethaltreq_q;
  logic [NUM_HARTS-1:0]  havereset_q;

  // Abstract Command State
  logic [3:0][31:0]      data_q;
  logic [3:0][31:0]      progbuf_q;
  logic [31:0]           command_q;
  logic [31:0]           abstractauto_q;
  logic [2:0]            cmderr_q;
  logic                  abs_busy_q;
  logic                  abs_write_q;
  logic                  abs_postinc_q;
  logic [HARTSEL_W-1:0]  abs_hartsel_q;
  logic [15:0]           abs_regno_q;

  // System Bus Access (SBA) State
  sba_state_e            sba_state_q;
  logic                  sbbusyerror_q;
  logic                  sbreadonaddr_q;
  logic [2:0]            sbaccess_q;
  logic                  sbautoincrement_q;
  logic                  sbreadondata_q;
  logic [2:0]            sberror_q;
  logic [31:0]           sbaddress0_q;
  logic [31:0]           sbdata0_q;
  logic                  sba_write_q;
  logic [31:0]           sba_addr_q;
  logic [31:0]           sba_wdata_q;
  logic [2:0]            sba_size_q;

  // DMI Response Holding Register
  logic                  dmi_rsp_valid_q;
  logic [31:0]           dmi_rsp_data_q;
  logic [1:0]            dmi_rsp_op_q;

  assign dmi_req_ready_o = !dmi_rsp_valid_q;
  assign dmi_rsp_valid_o = dmi_rsp_valid_q;
  assign dmi_rsp_data_o  = dmi_rsp_data_q;
  assign dmi_rsp_op_o    = dmi_rsp_op_q;

  assign halt_req_o        = halt_req_q;
  assign resume_req_o      = resume_req_q;
  assign ndmreset_o        = ndmreset_q;
  assign abs_req_valid_o   = abs_busy_q;
  assign abs_req_write_o   = abs_write_q;
  assign abs_req_hartsel_o = abs_hartsel_q;
  assign abs_req_regno_o   = abs_regno_q;
  assign abs_req_wdata_o   = data_q[0];

  wire sbbusy = (sba_state_q != SBA_IDLE);
  assign irq_o = dmactive_q && ((sberror_q != 3'd0) || (cmderr_q != 3'd0) || (|halt_req_q));

  // Selected harts mask & dmstatus aggregation
  logic [NUM_HARTS-1:0] sel_harts_mask;
  logic anyhalted, allhalted;
  logic anyrunning, allrunning;
  logic anynonexistent, allnonexistent;
  logic anyresumeack, allresumeack;
  logic anyhavereset, allhavereset;

  always_comb begin
    automatic logic [NUM_HARTS-1:0] existent_sel;
    automatic logic                 base_exists;

    base_exists = (32'(hartsel_q) < NUM_HARTS);
    for (int h = 0; h < NUM_HARTS; h++) begin
      sel_harts_mask[h] = (10'(h) == hartsel_q) || (hasel_q && hawindow_q[h]);
    end
    existent_sel = sel_harts_mask;

    if (!base_exists && !(hasel_q && (|hawindow_q))) begin
      anynonexistent = 1'b1;
      allnonexistent = 1'b1;
      anyhalted      = 1'b0;
      allhalted      = 1'b0;
      anyrunning     = 1'b0;
      allrunning     = 1'b0;
      anyresumeack   = 1'b0;
      allresumeack   = 1'b0;
      anyhavereset   = 1'b0;
      allhavereset   = 1'b0;
    end else begin
      anynonexistent = !base_exists;
      allnonexistent = 1'b0;
      anyhalted      = |(existent_sel & halted_i);
      allhalted      = (existent_sel != '0) && ((existent_sel & halted_i) == existent_sel);
      anyrunning     = |(existent_sel & ~halted_i);
      allrunning     = (existent_sel != '0) && ((existent_sel & ~halted_i) == existent_sel);
      anyresumeack   = |(existent_sel & resume_ack_q);
      allresumeack   = (existent_sel != '0) && ((existent_sel & resume_ack_q) == existent_sel);
      anyhavereset   = |(existent_sel & havereset_q);
      allhavereset   = (existent_sel != '0) && ((existent_sel & havereset_q) == existent_sel);
    end
  end

  wire [31:0] dmcontrol_rd = {
    5'd0,
    hasel_q,
    hartsel_q,
    14'd0,
    ndmreset_q,
    dmactive_q
  };

  wire [31:0] dmstatus_rd = {
    9'd0,
    1'b0, // impebreak
    2'd0,
    allhavereset,
    anyhavereset,
    allresumeack,
    anyresumeack,
    allnonexistent,
    anynonexistent,
    1'b0, // allunavail
    1'b0, // anyunavail
    allrunning,
    anyrunning,
    allhalted,
    anyhalted,
    1'b1, // authenticated
    1'b0, // authbusy
    1'b1, // hasresethaltreq
    1'b0, // confstrptrvalid
    4'd2  // version = 2 (DM 0.13 / 1.0)
  };

  wire [31:0] hartinfo_rd   = {8'd0, 4'd2, 4'd0, 4'd0, 12'h000};
  wire [31:0] abstractcs_rd = {3'd0, 5'd4, 11'd0, abs_busy_q, 1'b0, cmderr_q, 4'd0, 4'd4};

  wire [31:0] sbcs_rd = {
    3'd1,              // sbversion = 1
    6'd0,
    sbbusyerror_q,
    sbbusy,
    sbreadonaddr_q,
    sbaccess_q,
    sbautoincrement_q,
    sbreadondata_q,
    sberror_q,
    7'd32,             // sbasize = 32
    5'b00111           // sbaccess32/16/8 supported
  };

  // Combinational DMI read mux
  logic [31:0] dmi_rdata_comb;
  logic        dmi_addr_known;

  always_comb begin
    dmi_rdata_comb = 32'd0;
    dmi_addr_known = 1'b1;

    unique case (dmi_req_addr_i)
      DMI_DATA0:        dmi_rdata_comb = data_q[0];
      DMI_DATA1:        dmi_rdata_comb = data_q[1];
      DMI_DATA2:        dmi_rdata_comb = data_q[2];
      DMI_DATA3:        dmi_rdata_comb = data_q[3];
      DMI_DMCONTROL:    dmi_rdata_comb = dmcontrol_rd;
      DMI_DMSTATUS:     dmi_rdata_comb = dmstatus_rd;
      DMI_HARTINFO:     dmi_rdata_comb = hartinfo_rd;
      DMI_HAWINDOWSEL:  dmi_rdata_comb = 32'd0;
      DMI_HAWINDOW:     dmi_rdata_comb = {{(32-NUM_HARTS){1'b0}}, hawindow_q};
      DMI_ABSTRACTCS:   dmi_rdata_comb = abstractcs_rd;
      DMI_COMMAND:      dmi_rdata_comb = command_q;
      DMI_ABSTRACTAUTO: dmi_rdata_comb = abstractauto_q;
      7'h20:            dmi_rdata_comb = progbuf_q[0];
      7'h21:            dmi_rdata_comb = progbuf_q[1];
      7'h22:            dmi_rdata_comb = progbuf_q[2];
      7'h23:            dmi_rdata_comb = progbuf_q[3];
      DMI_SBCS:         dmi_rdata_comb = sbcs_rd;
      DMI_SBADDRESS0:   dmi_rdata_comb = sbaddress0_q;
      DMI_SBDATA0:      dmi_rdata_comb = sbdata0_q;
      DMI_HALTSUM0:     dmi_rdata_comb = {{(32-NUM_HARTS){1'b0}}, halted_i};
      default:          dmi_addr_known = 1'b0;
    endcase
  end

  // System Bus Access (SBA) Master Output Signals
  always_comb begin
    sba_req_o = '0;
    if (sba_state_q == SBA_REQ) begin
      sba_req_o.valid      = 1'b1;
      sba_req_o.write      = sba_write_q;
      sba_req_o.addr       = sba_addr_q;
      sba_req_o.size       = sba_size_q;
      sba_req_o.len        = 8'd0;
      sba_req_o.last       = sba_write_q;
      sba_req_o.resp_ready = 1'b1;
      unique case (sba_size_q)
        3'd0: begin
          sba_req_o.wstrb = 4'b0001 << sba_addr_q[1:0];
          sba_req_o.wdata = {4{sba_wdata_q[7:0]}};
        end
        3'd1: begin
          sba_req_o.wstrb = sba_addr_q[1] ? 4'b1100 : 4'b0011;
          sba_req_o.wdata = {2{sba_wdata_q[15:0]}};
        end
        default: begin
          sba_req_o.wstrb = 4'b1111;
          sba_req_o.wdata = sba_wdata_q;
        end
      endcase
    end else if (sba_state_q == SBA_WAIT_RSP) begin
      sba_req_o.resp_ready = 1'b1;
    end
  end

  // Helper task to launch an SBA transaction with size/alignment checks
  task automatic launch_sba(
    input logic        is_write,
    input logic [31:0] addr,
    input logic [31:0] wdata
  );
    if ((sberror_q == 3'd0) && !sbbusyerror_q) begin
      if (sbaccess_q > 3'd2) begin
        sberror_q <= 3'd4; // Unsupported size
      end else if (((sbaccess_q == 3'd1) && addr[0]) ||
                   ((sbaccess_q == 3'd2) && (addr[1:0] != 2'b00))) begin
        sberror_q <= 3'd3; // Alignment error
      end else begin
        sba_state_q <= SBA_REQ;
        sba_write_q <= is_write;
        sba_addr_q  <= addr;
        sba_wdata_q <= wdata;
        sba_size_q  <= sbaccess_q;
      end
    end
  endtask

  // Helper task to launch an Abstract Command
  task automatic launch_abs_cmd(input logic [31:0] cmd_val);
    if (abs_busy_q) begin
      cmderr_q <= 3'd1;
    end else if (cmderr_q == 3'd0) begin
      automatic logic [7:0]  cmdtype = cmd_val[31:24];
      automatic logic [2:0]  aarsize = cmd_val[22:20];
      automatic logic        postinc = cmd_val[19];
      automatic logic        xfer    = cmd_val[17];
      automatic logic        wr      = cmd_val[16];
      automatic logic [15:0] regno   = cmd_val[15:0];
      automatic logic        hart_ok = (32'(hartsel_q) < NUM_HARTS);
      automatic logic [HARTSEL_W-1:0] hidx = hartsel_q[HARTSEL_W-1:0];

      if ((cmdtype != 8'h00) || (aarsize != 3'd2)) begin
        cmderr_q <= 3'd2; // Not supported
      end else if (!hart_ok || (xfer && !halted_i[hidx])) begin
        cmderr_q <= 3'd4; // Halt/Resume error
      end else if (xfer) begin
        abs_busy_q    <= 1'b1;
        abs_write_q   <= wr;
        abs_postinc_q <= postinc;
        abs_hartsel_q <= hidx;
        abs_regno_q   <= regno;
      end else if (postinc) begin
        command_q[15:0] <= cmd_val[15:0] + 16'd1;
      end
    end
  endtask

  // DMI & DM Sequential Logic
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      dmactive_q        <= 1'b0;
      ndmreset_q        <= 1'b0;
      hartsel_q         <= '0;
      hasel_q           <= 1'b0;
      hawindow_q        <= '0;
      halt_req_q        <= '0;
      resume_req_q      <= '0;
      resume_ack_q      <= '0;
      resethaltreq_q    <= '0;
      havereset_q       <= {NUM_HARTS{1'b1}};
      data_q            <= '0;
      progbuf_q         <= '0;
      command_q         <= '0;
      abstractauto_q    <= '0;
      cmderr_q          <= 3'd0;
      abs_busy_q        <= 1'b0;
      abs_write_q       <= 1'b0;
      abs_postinc_q     <= 1'b0;
      abs_hartsel_q     <= '0;
      abs_regno_q       <= '0;
      sba_state_q       <= SBA_IDLE;
      sbbusyerror_q     <= 1'b0;
      sbreadonaddr_q    <= 1'b0;
      sbaccess_q        <= 3'd2;
      sbautoincrement_q <= 1'b0;
      sbreadondata_q    <= 1'b0;
      sberror_q         <= 3'd0;
      sbaddress0_q      <= 32'd0;
      sbdata0_q         <= 32'd0;
      sba_write_q       <= 1'b0;
      sba_addr_q        <= 32'd0;
      sba_wdata_q       <= 32'd0;
      sba_size_q        <= 3'd2;
      dmi_rsp_valid_q   <= 1'b0;
      dmi_rsp_data_q    <= 32'd0;
      dmi_rsp_op_q      <= DMI_RSP_OK;
    end else begin
      // Consume DMI response
      if (dmi_rsp_valid_q && dmi_rsp_ready_i) begin
        dmi_rsp_valid_q <= 1'b0;
      end

      // Hart resume_ack and ndmreset tracking (when dmactive)
      if (dmactive_q) begin
        for (int h = 0; h < NUM_HARTS; h++) begin
          if (ndmreset_q) begin
            havereset_q[h] <= 1'b1;
            if (resethaltreq_q[h]) halt_req_q[h] <= 1'b1;
          end
          if (resume_req_q[h] && resume_ack_i[h]) begin
            resume_req_q[h] <= 1'b0;
            resume_ack_q[h] <= 1'b1;
          end
        end

        // Abstract command completion
        if (abs_busy_q && abs_rsp_valid_i) begin
          abs_busy_q <= 1'b0;
          if (abs_rsp_err_i) begin
            cmderr_q <= 3'd3; // Exception
          end else begin
            if (!abs_write_q) data_q[0] <= abs_rsp_rdata_i;
            if (abs_postinc_q) command_q[15:0] <= command_q[15:0] + 16'd1;
          end
        end

        // SBA master FSM
        unique case (sba_state_q)
          SBA_REQ: begin
            if (sba_rsp_i.req_ready) begin
              sba_state_q <= SBA_WAIT_RSP;
            end
          end
          SBA_WAIT_RSP: begin
            if (sba_rsp_i.resp_valid) begin
              sba_state_q <= SBA_IDLE;
              if (sba_rsp_i.err) begin
                sberror_q <= 3'd2;
              end else begin
                if (!sba_write_q) begin
                  unique case (sba_size_q)
                    3'd0: begin
                      unique case (sba_addr_q[1:0])
                        2'd0: sbdata0_q <= {24'd0, sba_rsp_i.rdata[7:0]};
                        2'd1: sbdata0_q <= {24'd0, sba_rsp_i.rdata[15:8]};
                        2'd2: sbdata0_q <= {24'd0, sba_rsp_i.rdata[23:16]};
                        2'd3: sbdata0_q <= {24'd0, sba_rsp_i.rdata[31:24]};
                      endcase
                    end
                    3'd1: begin
                      sbdata0_q <= sba_addr_q[1] ? {16'd0, sba_rsp_i.rdata[31:16]}
                                                 : {16'd0, sba_rsp_i.rdata[15:0]};
                    end
                    default: begin
                      sbdata0_q <= sba_rsp_i.rdata;
                    end
                  endcase
                end
                if (sbautoincrement_q) begin
                  sbaddress0_q <= sbaddress0_q + (32'd1 << sba_size_q);
                end
              end
            end
          end
          default: ;
        endcase
      end else begin
        // dmactive == 0 keeps DM state reset
        ndmreset_q        <= 1'b0;
        hartsel_q         <= '0;
        hasel_q           <= 1'b0;
        hawindow_q        <= '0;
        halt_req_q        <= '0;
        resume_req_q      <= '0;
        resume_ack_q      <= '0;
        resethaltreq_q    <= '0;
        data_q            <= '0;
        progbuf_q         <= '0;
        command_q         <= '0;
        abstractauto_q    <= '0;
        cmderr_q          <= 3'd0;
        abs_busy_q        <= 1'b0;
        sba_state_q       <= SBA_IDLE;
        sbbusyerror_q     <= 1'b0;
        sbreadonaddr_q    <= 1'b0;
        sbaccess_q        <= 3'd2;
        sbautoincrement_q <= 1'b0;
        sbreadondata_q    <= 1'b0;
        sberror_q         <= 3'd0;
        sbaddress0_q      <= 32'd0;
        sbdata0_q         <= 32'd0;
      end

      // Handle new DMI request
      if (dmi_req_valid_i && dmi_req_ready_o) begin
        dmi_rsp_valid_q <= 1'b1;

        if (dmi_req_op_i == DMI_OP_NOP) begin
          dmi_rsp_data_q <= 32'd0;
          dmi_rsp_op_q   <= DMI_RSP_OK;
        end else if ((dmi_req_op_i != DMI_OP_READ) && (dmi_req_op_i != DMI_OP_WRITE)) begin
          dmi_rsp_data_q <= 32'd0;
          dmi_rsp_op_q   <= DMI_RSP_FAIL;
        end else if (dmi_req_op_i == DMI_OP_READ) begin
          dmi_rsp_data_q <= dmi_rdata_comb;
          dmi_rsp_op_q   <= dmi_addr_known ? DMI_RSP_OK : DMI_RSP_FAIL;

          if (dmactive_q) begin
            if ((dmi_req_addr_i >= DMI_DATA0) && (dmi_req_addr_i <= DMI_DATA3)) begin
              automatic logic [1:0] di = dmi_req_addr_i[1:0];
              if (abstractauto_q[{3'd0, di}]) launch_abs_cmd(command_q);
            end else if ((dmi_req_addr_i >= DMI_PROGBUF0) && (dmi_req_addr_i <= DMI_PROGBUF3)) begin
              automatic logic [1:0] pi = dmi_req_addr_i[1:0];
              if (abstractauto_q[{1'b1, 2'd0, pi}]) launch_abs_cmd(command_q);
            end else if (dmi_req_addr_i == DMI_SBDATA0) begin
              if (sbbusy) begin
                sbbusyerror_q <= 1'b1;
              end else if (sbreadondata_q) begin
                launch_sba(1'b0, sbaddress0_q, 32'd0);
              end
            end
          end
        end else begin
          // DMI_OP_WRITE
          dmi_rsp_data_q <= 32'd0;
          dmi_rsp_op_q   <= dmi_addr_known ? DMI_RSP_OK : DMI_RSP_FAIL;

          if (dmi_req_addr_i == DMI_DMCONTROL) begin
            dmactive_q <= dmi_req_data_i[0];
            if (dmi_req_data_i[0]) begin
              automatic logic [9:0] new_hsel  = dmi_req_data_i[25:16];
              automatic logic       new_hasel = dmi_req_data_i[26];
              ndmreset_q <= dmi_req_data_i[1];
              hartsel_q  <= new_hsel;
              hasel_q    <= new_hasel;
              for (int h = 0; h < NUM_HARTS; h++) begin
                if ((10'(h) == new_hsel) || (new_hasel && hawindow_q[h])) begin
                  halt_req_q[h] <= dmi_req_data_i[31];
                  if (dmi_req_data_i[31]) begin
                    resume_req_q[h] <= 1'b0;
                  end else if (dmi_req_data_i[30]) begin
                    resume_req_q[h] <= 1'b1;
                    resume_ack_q[h] <= 1'b0;
                  end
                  if (dmi_req_data_i[3])  resethaltreq_q[h] <= 1'b1;
                  if (dmi_req_data_i[2])  resethaltreq_q[h] <= 1'b0;
                  if (dmi_req_data_i[28]) havereset_q[h]    <= 1'b0;
                end
              end
            end
          end else if (dmactive_q) begin
            if ((dmi_req_addr_i >= DMI_DATA0) && (dmi_req_addr_i <= DMI_DATA3)) begin
              automatic logic [1:0] di = dmi_req_addr_i[1:0];
              if (abs_busy_q) begin
                cmderr_q <= 3'd1;
              end else begin
                data_q[di] <= dmi_req_data_i;
                if (abstractauto_q[{3'd0, di}]) launch_abs_cmd(command_q);
              end
            end else if ((dmi_req_addr_i >= DMI_PROGBUF0) && (dmi_req_addr_i <= DMI_PROGBUF3)) begin
              automatic logic [1:0] pi = dmi_req_addr_i[1:0];
              if (abs_busy_q) begin
                cmderr_q <= 3'd1;
              end else begin
                progbuf_q[pi] <= dmi_req_data_i;
                if (abstractauto_q[{1'b1, 2'd0, pi}]) launch_abs_cmd(command_q);
              end
            end else begin
              unique case (dmi_req_addr_i)
                DMI_HAWINDOW: begin
                  hawindow_q <= dmi_req_data_i[NUM_HARTS-1:0];
                end
                DMI_ABSTRACTCS: begin
                  if (abs_busy_q) begin
                    cmderr_q <= 3'd1;
                  end else begin
                    cmderr_q <= cmderr_q & ~dmi_req_data_i[10:8];
                  end
                end
                DMI_COMMAND: begin
                  if (!abs_busy_q && (cmderr_q == 3'd0)) begin
                    command_q <= dmi_req_data_i;
                  end
                  launch_abs_cmd(dmi_req_data_i);
                end
                DMI_ABSTRACTAUTO: begin
                  if (abs_busy_q) begin
                    cmderr_q <= 3'd1;
                  end else begin
                    abstractauto_q <= {12'd0, dmi_req_data_i[19:16], 12'd0, dmi_req_data_i[3:0]};
                  end
                end
                DMI_SBCS: begin
                  if (dmi_req_data_i[22]) sbbusyerror_q <= 1'b0;
                  sberror_q         <= sberror_q & ~dmi_req_data_i[14:12];
                  if (!sbbusy) begin
                    sbreadonaddr_q    <= dmi_req_data_i[20];
                    sbaccess_q        <= dmi_req_data_i[19:17];
                    sbautoincrement_q <= dmi_req_data_i[16];
                    sbreadondata_q    <= dmi_req_data_i[15];
                  end
                end
                DMI_SBADDRESS0: begin
                  if (sbbusy) begin
                    sbbusyerror_q <= 1'b1;
                  end else begin
                    sbaddress0_q <= dmi_req_data_i;
                    if (sbreadonaddr_q) begin
                      launch_sba(1'b0, dmi_req_data_i, 32'd0);
                    end
                  end
                end
                DMI_SBDATA0: begin
                  if (sbbusy) begin
                    sbbusyerror_q <= 1'b1;
                  end else begin
                    sbdata0_q <= dmi_req_data_i;
                    launch_sba(1'b1, sbaddress0_q, dmi_req_data_i);
                  end
                end
                default: ;
              endcase
            end
          end
        end
      end
    end
  end

  // Quad-Bus Read-Only CSR Monitor / Status & Progbuf Aperture
  wire [5:0] csr_word_off = csr_req_i.addr[7:2];
  wire       csr_bad_addr = (csr_req_i.addr[1:0] != 2'b00) || (csr_word_off > 6'h0C);

  always_comb begin
    csr_rsp_o       = '0;
    csr_rsp_o.ready = 1'b1;
    csr_rsp_o.err   = csr_req_i.valid && (csr_req_i.write || csr_bad_addr);

    if (csr_req_i.valid && !csr_req_i.write && !csr_bad_addr) begin
      unique case (csr_word_off)
        6'h00: begin // DM_CSR_STAT
          csr_rsp_o.rdata[0]     = dmactive_q;
          csr_rsp_o.rdata[1]     = ndmreset_q;
          csr_rsp_o.rdata[4:2]   = sberror_q;
          csr_rsp_o.rdata[5]     = sbbusy;
          csr_rsp_o.rdata[8:6]   = cmderr_q;
          csr_rsp_o.rdata[9]     = abs_busy_q;
          csr_rsp_o.rdata[15:12] = hartsel_q[3:0];
          for (int h = 0; h < NUM_HARTS; h++) begin
            if (h < 4) begin
              csr_rsp_o.rdata[16 + h] = halted_i[h];
              csr_rsp_o.rdata[20 + h] = halt_req_q[h];
              csr_rsp_o.rdata[24 + h] = resume_req_q[h];
              csr_rsp_o.rdata[28 + h] = havereset_q[h];
            end
          end
        end
        6'h01:   csr_rsp_o.rdata = sbaddress0_q;
        6'h02:   csr_rsp_o.rdata = sbdata0_q;
        6'h03:   csr_rsp_o.rdata = command_q;
        6'h04:   csr_rsp_o.rdata = data_q[0];
        6'h05:   csr_rsp_o.rdata = data_q[1];
        6'h06:   csr_rsp_o.rdata = data_q[2];
        6'h07:   csr_rsp_o.rdata = data_q[3];
        6'h08:   csr_rsp_o.rdata = progbuf_q[0];
        6'h09:   csr_rsp_o.rdata = progbuf_q[1];
        6'h0A:   csr_rsp_o.rdata = progbuf_q[2];
        6'h0B:   csr_rsp_o.rdata = progbuf_q[3];
        6'h0C:   csr_rsp_o.rdata = {{(32-NUM_HARTS){1'b0}}, halted_i};
        default: csr_rsp_o.rdata = 32'd0;
      endcase
    end
  end

endmodule : riscv_dm_core
