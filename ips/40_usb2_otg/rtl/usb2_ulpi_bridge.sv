// SPDX-License-Identifier: Apache-2.0
// usb2_ulpi_bridge.sv - UTMI+ (8-bit/60MHz) <-> ULPI 1.1 SDR (8-bit/60MHz) Bridge
//
// Features:
//   - Split pad I/O (`ulpi_data_i/o/oe_o`, `ulpi_dir_i`, `ulpi_nxt_i`, `ulpi_stp_o`, `ulpi_rst_no`)
//   - Full ULPI 1.1 SDR turnaround handling, RX CMD decode (LineState/VbusState/RxEvent/IdDig)
//   - TX framing (`TXCMD` PID encoding + `STP` termination)
//   - Software-initiated ULPI register read/write + automatic sync of Function/OTG Control

module usb2_ulpi_bridge
  import usb2_pkg::*;
(
  input  logic       phy_clk_i,
  input  logic       phy_rst_ni,

  // Internal UTMI+ Interface (Controller Side)
  input  logic [7:0] utmi_data_i,
  input  logic       utmi_txvalid_i,
  output logic       utmi_txready_o,

  output logic [7:0] utmi_data_o,
  output logic       utmi_rxvalid_o,
  output logic       utmi_rxactive_o,
  output logic       utmi_rxerror_o,

  output logic [1:0] utmi_linestate_o,
  output logic       utmi_vbusvalid_o,
  output logic       utmi_sessend_o,
  output logic       utmi_iddig_o,

  input  logic [1:0] utmi_opmode_i,
  input  logic [1:0] utmi_xcvrselect_i,
  input  logic       utmi_termselect_i,
  input  logic       utmi_dppulldown_i,
  input  logic       utmi_dmpulldown_i,
  input  logic       utmi_suspendm_ni,
  input  logic       utmi_reset_i,

  // Software ULPI Register Access Interface (phy_clk_i domain)
  input  logic       reg_req_i,
  input  logic       reg_wr_i,
  input  logic [5:0] reg_addr_i,
  input  logic [7:0] reg_wdata_i,
  output logic       reg_done_o,
  output logic [7:0] reg_rdata_o,
  output logic       ulpi_busy_o,

  // External ULPI 1.1 SDR PHY Pads
  input  logic [7:0] ulpi_data_i,
  output logic [7:0] ulpi_data_o,
  output logic       ulpi_data_oe_o,
  input  logic       ulpi_dir_i,
  input  logic       ulpi_nxt_i,
  output logic       ulpi_stp_o,
  output logic       ulpi_rst_no
);

  typedef enum logic [3:0] {
    ST_IDLE,
    ST_TX_CMD,
    ST_TX_DATA,
    ST_REG_WR_CMD,
    ST_REG_WR_DATA,
    ST_REG_WR_STP,
    ST_REG_RD_CMD,
    ST_REG_RD_TA,
    ST_REG_RD_DATA
  } state_e;

  state_e     st_q;
  logic       dir_prev_q;
  logic       turn_drop_q;     // 1 cycle cooldown after DIR 1->0

  // UTMI+ decoded line/OTG status
  logic [1:0] linestate_q;
  logic       vbusvalid_q;
  logic       sessend_q;
  logic       iddig_q;
  logic       rxactive_q;
  logic       rxerror_q;

  // Pending software or auto-sync register transactions
  logic       sw_pend_q;
  logic       sw_wr_q;
  logic [5:0] sw_addr_q;
  logic [7:0] sw_wdata_q;
  logic       cur_is_sw_q;
  logic       cur_wr_q;
  logic [5:0] cur_addr_q;
  logic [7:0] cur_wdata_q;

  logic [7:0] shadow_func_q;
  logic [7:0] shadow_otg_q;

  wire [7:0] target_func = {1'b0, utmi_suspendm_ni, utmi_reset_i,
                            utmi_opmode_i, utmi_termselect_i, utmi_xcvrselect_i};
  wire [7:0] target_otg  = {5'b00000, utmi_dmpulldown_i, utmi_dppulldown_i, 1'b0};

  wire func_dirty = (target_func != shadow_func_q);
  wire otg_dirty  = (target_otg  != shadow_otg_q);

  assign ulpi_rst_no      = phy_rst_ni && !utmi_reset_i;
  assign utmi_linestate_o = linestate_q;
  assign utmi_vbusvalid_o = vbusvalid_q;
  assign utmi_sessend_o   = sessend_q;
  assign utmi_iddig_o     = iddig_q;
  assign ulpi_busy_o      = sw_pend_q || (st_q != ST_IDLE);

  // Combinational RX decode when PHY owns bus (after 0->1 turnaround cycle)
  wire dir_rising   = !dir_prev_q && ulpi_dir_i;
  wire dir_falling  = dir_prev_q  && !ulpi_dir_i;
  wire rx_bus_valid = dir_prev_q && ulpi_dir_i && (st_q != ST_REG_RD_TA) && (st_q != ST_REG_RD_DATA);

  always_comb begin
    ulpi_data_o    = 8'h00;
    ulpi_data_oe_o = 1'b0;
    ulpi_stp_o     = 1'b0;
    utmi_txready_o = 1'b0;

    utmi_data_o     = ulpi_data_i;
    utmi_rxvalid_o  = rx_bus_valid && ulpi_nxt_i;
    utmi_rxactive_o = rxactive_q && ulpi_dir_i;
    utmi_rxerror_o  = rxerror_q  && ulpi_dir_i;

    if (!ulpi_dir_i && !turn_drop_q) begin
      unique case (st_q)
        ST_TX_CMD: begin
          ulpi_data_oe_o = 1'b1;
          ulpi_data_o    = (utmi_opmode_i == UTMI_OPMODE_DIS_STUFF) ? 8'h40 : {4'b0100, utmi_data_i[3:0]};
          utmi_txready_o = ulpi_nxt_i && (utmi_opmode_i != UTMI_OPMODE_DIS_STUFF);
        end
        ST_TX_DATA: begin
          if (utmi_txvalid_i) begin
            ulpi_data_oe_o = 1'b1;
            ulpi_data_o    = utmi_data_i;
            utmi_txready_o = ulpi_nxt_i;
          end else begin
            ulpi_data_oe_o = 1'b1;
            ulpi_data_o    = 8'h00;
            ulpi_stp_o     = 1'b1;
          end
        end
        ST_REG_WR_CMD: begin
          ulpi_data_oe_o = 1'b1;
          ulpi_data_o    = {2'b10, cur_addr_q};
        end
        ST_REG_WR_DATA: begin
          ulpi_data_oe_o = 1'b1;
          ulpi_data_o    = cur_wdata_q;
        end
        ST_REG_WR_STP: begin
          ulpi_data_oe_o = 1'b1;
          ulpi_data_o    = 8'h00;
          ulpi_stp_o     = 1'b1;
        end
        ST_REG_RD_CMD: begin
          ulpi_data_oe_o = 1'b1;
          ulpi_data_o    = {2'b11, cur_addr_q};
        end
        default: ;
      endcase
    end
  end

  always_ff @(posedge phy_clk_i or negedge phy_rst_ni) begin
    if (!phy_rst_ni) begin
      st_q          <= ST_IDLE;
      dir_prev_q    <= 1'b0;
      turn_drop_q   <= 1'b0;
      linestate_q   <= LINE_SE0;
      vbusvalid_q   <= 1'b1;
      sessend_q     <= 1'b0;
      iddig_q       <= 1'b1;
      rxactive_q    <= 1'b0;
      rxerror_q     <= 1'b0;
      sw_pend_q     <= 1'b0;
      sw_wr_q       <= 1'b0;
      sw_addr_q     <= '0;
      sw_wdata_q    <= '0;
      cur_is_sw_q   <= 1'b0;
      cur_wr_q      <= 1'b0;
      cur_addr_q    <= '0;
      cur_wdata_q   <= '0;
      shadow_func_q <= 8'h41; // ULPI default: SuspendM=1, XcvrSelect=01 (FS)
      shadow_otg_q  <= 8'h06; // ULPI default: DpPulldown=1, DmPulldown=1
      reg_done_o    <= 1'b0;
      reg_rdata_o   <= '0;
    end else begin
      dir_prev_q  <= ulpi_dir_i;
      turn_drop_q <= dir_falling;
      reg_done_o  <= 1'b0;

      // Latch software ULPI register request
      if (reg_req_i) begin
        sw_pend_q  <= 1'b1;
        sw_wr_q    <= reg_wr_i;
        sw_addr_q  <= reg_addr_i;
        sw_wdata_q <= reg_wdata_i;
      end

      // RX tracking whenever PHY owns the bus
      if (dir_rising) begin
        if (ulpi_nxt_i) begin
          rxactive_q <= 1'b1;
          rxerror_q  <= 1'b0;
        end
      end else if (rx_bus_valid) begin
        if (!ulpi_nxt_i) begin
          // RX CMD byte
          linestate_q <= ulpi_data_i[1:0];
          vbusvalid_q <= (ulpi_data_i[3:2] == 2'b11);
          sessend_q   <= (ulpi_data_i[3:2] == 2'b00);
          iddig_q     <= ulpi_data_i[6];
          rxactive_q  <= ulpi_data_i[4];
          rxerror_q   <= (ulpi_data_i[5:4] == 2'b11);
        end else begin
          rxactive_q  <= 1'b1;
          rxerror_q   <= 1'b0;
        end
      end else if (!ulpi_dir_i) begin
        rxactive_q <= 1'b0;
        rxerror_q  <= 1'b0;
      end

      // Preemption by PHY DIR=1 during Link-owned states
      if (ulpi_dir_i) begin
        if (st_q == ST_REG_RD_TA) begin
          if (ulpi_nxt_i) begin
            // Preempted by RX packet before register read completed
            st_q <= ST_IDLE;
          end else begin
            st_q <= ST_REG_RD_DATA;
          end
        end else if (st_q == ST_REG_RD_DATA) begin
          if (!ulpi_nxt_i) begin
            reg_rdata_o <= ulpi_data_i;
            reg_done_o  <= 1'b1;
            if (cur_is_sw_q) sw_pend_q <= 1'b0;
            st_q <= ST_IDLE;
          end else begin
            st_q <= ST_IDLE;
          end
        end else begin
          st_q <= ST_IDLE;
        end
      end else if (!turn_drop_q) begin
        unique case (st_q)
          ST_IDLE: begin
            if (utmi_txvalid_i) begin
              st_q <= ST_TX_CMD;
            end else if (sw_pend_q) begin
              cur_is_sw_q <= 1'b1;
              cur_wr_q    <= sw_wr_q;
              cur_addr_q  <= sw_addr_q;
              cur_wdata_q <= sw_wdata_q;
              st_q        <= sw_wr_q ? ST_REG_WR_CMD : ST_REG_RD_CMD;
            end else if (func_dirty) begin
              cur_is_sw_q <= 1'b0;
              cur_wr_q    <= 1'b1;
              cur_addr_q  <= ULPI_REG_FUNC_CTRL;
              cur_wdata_q <= target_func;
              st_q        <= ST_REG_WR_CMD;
            end else if (otg_dirty) begin
              cur_is_sw_q <= 1'b0;
              cur_wr_q    <= 1'b1;
              cur_addr_q  <= ULPI_REG_OTG_CTRL;
              cur_wdata_q <= target_otg;
              st_q        <= ST_REG_WR_CMD;
            end
          end

          ST_TX_CMD: begin
            if (ulpi_nxt_i) begin
              st_q <= ST_TX_DATA;
            end
          end

          ST_TX_DATA: begin
            if (!utmi_txvalid_i) begin
              // STP asserted combinationally this cycle
              st_q <= ST_IDLE;
            end
          end

          ST_REG_WR_CMD: begin
            if (ulpi_nxt_i) begin
              st_q <= ST_REG_WR_DATA;
            end
          end

          ST_REG_WR_DATA: begin
            if (ulpi_nxt_i) begin
              st_q <= ST_REG_WR_STP;
            end
          end

          ST_REG_WR_STP: begin
            if (cur_is_sw_q) begin
              sw_pend_q  <= 1'b0;
              reg_done_o <= 1'b1;
            end else if (cur_addr_q == ULPI_REG_FUNC_CTRL) begin
              shadow_func_q <= cur_wdata_q;
            end else if (cur_addr_q == ULPI_REG_OTG_CTRL) begin
              shadow_otg_q <= cur_wdata_q;
            end
            st_q <= ST_IDLE;
          end

          ST_REG_RD_CMD: begin
            if (ulpi_nxt_i) begin
              st_q <= ST_REG_RD_TA;
            end
          end

          ST_REG_RD_TA,
          ST_REG_RD_DATA: begin
            // Stay waiting for PHY to assert DIR=1; if PHY didn't yet, stay here
          end

          default: st_q <= ST_IDLE;
        endcase
      end
    end
  end

endmodule : usb2_ulpi_bridge
