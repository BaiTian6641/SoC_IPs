// SPDX-License-Identifier: Apache-2.0
// usb2_pkg.sv - Shared Types, USB 2.0 PIDs, ULPI Constants, CDC Commands/Events, and CSR Map

package usb2_pkg;

  // -------------------------------------------------------------------------
  // USB 2.0 Packet Identifiers (4-bit PID, wire byte = {~pid, pid})
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    PID_OUT   = 4'b0001, // 4'h1
    PID_IN    = 4'b1001, // 4'h9
    PID_SOF   = 4'b0101, // 4'h5
    PID_SETUP = 4'b1101, // 4'hD
    PID_DATA0 = 4'b0011, // 4'h3
    PID_DATA1 = 4'b1011, // 4'hB
    PID_DATA2 = 4'b0111, // 4'h7
    PID_MDATA = 4'b1111, // 4'hF
    PID_ACK   = 4'b0010, // 4'h2
    PID_NAK   = 4'b1010, // 4'hA
    PID_STALL = 4'b1110, // 4'hE
    PID_NYET  = 4'b0110, // 4'h6
    PID_PING  = 4'b0100, // 4'h4
    PID_PRE   = 4'b1100  // 4'hC
  } usb_pid_e;

  function automatic logic [7:0] pid_to_wire(input logic [3:0] pid);
    return {~pid, pid};
  endfunction

  function automatic logic pid_is_valid(input logic [7:0] wire_byte);
    return (wire_byte[7:4] == ~wire_byte[3:0]);
  endfunction

  function automatic logic pid_is_token(input logic [3:0] pid);
    return (pid == PID_OUT) || (pid == PID_IN) || (pid == PID_SOF) ||
           (pid == PID_SETUP) || (pid == PID_PING);
  endfunction

  function automatic logic pid_is_data(input logic [3:0] pid);
    return (pid == PID_DATA0) || (pid == PID_DATA1) ||
           (pid == PID_DATA2) || (pid == PID_MDATA);
  endfunction

  function automatic logic pid_is_hs(input logic [3:0] pid);
    return (pid == PID_ACK) || (pid == PID_NAK) ||
           (pid == PID_STALL) || (pid == PID_NYET);
  endfunction

  // -------------------------------------------------------------------------
  // Endpoint Types
  // -------------------------------------------------------------------------
  typedef enum logic [1:0] {
    EP_TYPE_CTRL = 2'd0,
    EP_TYPE_ISO  = 2'd1,
    EP_TYPE_BULK = 2'd2,
    EP_TYPE_INT  = 2'd3
  } usb_ep_type_e;

  // -------------------------------------------------------------------------
  // UTMI+ Line / Mode Constants
  // -------------------------------------------------------------------------
  localparam logic [1:0] UTMI_OPMODE_NORMAL     = 2'b00;
  localparam logic [1:0] UTMI_OPMODE_NONDRV     = 2'b01;
  localparam logic [1:0] UTMI_OPMODE_DIS_STUFF  = 2'b10;

  localparam logic [1:0] UTMI_XCVR_HS           = 2'b00;
  localparam logic [1:0] UTMI_XCVR_FS           = 2'b01;
  localparam logic [1:0] UTMI_XCVR_LS           = 2'b10;

  localparam logic [1:0] LINE_SE0               = 2'b00;
  localparam logic [1:0] LINE_J                 = 2'b01;
  localparam logic [1:0] LINE_K                 = 2'b10;
  localparam logic [1:0] LINE_SE1               = 2'b11;

  // -------------------------------------------------------------------------
  // ULPI 1.1 Register Map Constants
  // -------------------------------------------------------------------------
  localparam logic [5:0] ULPI_REG_VID_LO        = 6'h00;
  localparam logic [5:0] ULPI_REG_VID_HI        = 6'h01;
  localparam logic [5:0] ULPI_REG_PID_LO        = 6'h02;
  localparam logic [5:0] ULPI_REG_PID_HI        = 6'h03;
  localparam logic [5:0] ULPI_REG_FUNC_CTRL     = 6'h04;
  localparam logic [5:0] ULPI_REG_IFACE_CTRL    = 6'h07;
  localparam logic [5:0] ULPI_REG_OTG_CTRL      = 6'h0A;
  localparam logic [5:0] ULPI_REG_USB_INT_EN_R  = 6'h0D;
  localparam logic [5:0] ULPI_REG_USB_INT_EN_F  = 6'h10;
  localparam logic [5:0] ULPI_REG_USB_INT_STAT  = 6'h13;
  localparam logic [5:0] ULPI_REG_USB_INT_LATCH = 6'h14;
  localparam logic [5:0] ULPI_REG_SCRATCH       = 6'h16;

  // -------------------------------------------------------------------------
  // Host-Lite Transaction Result Status Codes
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    HOST_ST_OK      = 3'd0,
    HOST_ST_NAK     = 3'd1,
    HOST_ST_STALL   = 3'd2,
    HOST_ST_NYET    = 3'd3,
    HOST_ST_TIMEOUT = 3'd4,
    HOST_ST_CRC_ERR = 3'd5,
    HOST_ST_PID_ERR = 3'd6
  } host_xact_stat_e;

  // -------------------------------------------------------------------------
  // CDC Command Queue (clk_i -> phy_clk_i, packed 40-bit)
  // -------------------------------------------------------------------------
  typedef enum logic [2:0] {
    CMD_NOP        = 3'd0,
    CMD_SET_CFG    = 3'd1, // payload: [0]=core_en,[1]=host_mode,[2]=connect,[3]=hs_en,[4]=sof_en,[5]=fast_tmr,[6]=phy_ovr,[8:7]=opmode,[10:9]=xcvr,[11]=term,[12]=dp_pd,[13]=dm_pd,[14]=susp_n,[15]=phy_rst,[16]=host_rst,[23:17]=dev_addr,[35:24]=reserved
    CMD_SET_SOF    = 3'd2, // payload: [15:0]=sof_period
    CMD_EP_CFG     = 3'd3, // payload: [2:0]=ep,[3]=tx_en,[4]=rx_en,[6:5]=type,[17:7]=max_pkt,[18]=ping_pong
    CMD_EP_ACT     = 3'd4, // payload: [2:0]=ep,[3]=stall_in_set,[4]=stall_in_clr,[5]=stall_out_set,[6]=stall_out_clr,[7]=tog_in_rst,[8]=tog_in_set1,[9]=tog_out_rst,[10]=tog_out_set1
    CMD_TX_COMMIT  = 3'd5, // payload: [2:0]=ep,[3]=bank,[14:4]=byte_len
    CMD_RX_FREE    = 3'd6, // payload: [2:0]=ep,[3]=bank
    CMD_HOST_XACT  = 3'd7  // payload: [6:0]=dev_addr,[10:7]=ep,[14:11]=pid,[15]=toggle,[16]=is_iso,[17]=bank,[28:18]=byte_len
  } usb_cmd_op_e;

  typedef struct packed {
    usb_cmd_op_e op;
    logic [31:0] arg;
  } usb_cmd_t;

  // Separate ULPI Reg command (packed 16-bit) or multiplexed: let's keep usb_cmd_t 36-bit
  // and add CMD_ULPI_REG via dedicated 4-bit op! Let's use 4-bit op so ULPI fits cleanly.
  typedef enum logic [3:0] {
    UCMD_NOP       = 4'd0,
    UCMD_SET_CFG   = 4'd1,
    UCMD_SET_SOF   = 4'd2,
    UCMD_EP_CFG    = 4'd3,
    UCMD_EP_ACT    = 4'd4,
    UCMD_TX_COMMIT = 4'd5,
    UCMD_RX_FREE   = 4'd6,
    UCMD_HOST_XACT = 4'd7,
    UCMD_ULPI_REG  = 4'd8, // arg: [5:0]=addr,[6]=write,[15:8]=wdata
    UCMD_SOFT_RST  = 4'd9
  } usb_ucmd_op_e;

  typedef struct packed {
    usb_ucmd_op_e op;
    logic [31:0]  arg;
  } usb_ucmd_t;

  // -------------------------------------------------------------------------
  // CDC Event Queue (phy_clk_i -> clk_i, packed 36-bit)
  // -------------------------------------------------------------------------
  typedef enum logic [3:0] {
    UEVT_NOP       = 4'd0,
    UEVT_TX_DONE   = 4'd1, // arg: [2:0]=ep,[3]=bank
    UEVT_RX_DONE   = 4'd2, // arg: [2:0]=ep,[3]=bank,[14:4]=byte_len
    UEVT_SETUP     = 4'd3, // arg: [2:0]=ep (actual 8 bytes pushed to u_setup_fifo)
    UEVT_HOST_DONE = 4'd4, // arg: [2:0]=status,[6:3]=rx_pid,[7]=next_toggle,[18:8]=rx_len
    UEVT_ULPI_DONE = 4'd5, // arg: [7:0]=rdata
    UEVT_BUS_RST   = 4'd6, // USB bus reset detected
    UEVT_SUSPEND   = 4'd7, // USB suspend detected
    UEVT_RESUME    = 4'd8, // USB resume detected
    UEVT_CRC_ERR   = 4'd9  // Bad CRC-16 packet dropped
  } usb_uevt_op_e;

  typedef struct packed {
    usb_uevt_op_e op;
    logic [31:0]  arg;
  } usb_uevt_t;

  // -------------------------------------------------------------------------
  // CSR Register Address Offsets (32-bit byte offset within IP aperture)
  // -------------------------------------------------------------------------
  localparam logic [31:0] REG_ID             = 32'h000;
  localparam logic [31:0] REG_CAP            = 32'h004;
  localparam logic [31:0] REG_CTRL           = 32'h008;
  localparam logic [31:0] REG_DEV_ADDR       = 32'h00C;
  localparam logic [31:0] REG_STATUS         = 32'h010;
  localparam logic [31:0] REG_IRQ_STAT       = 32'h014;
  localparam logic [31:0] REG_IRQ_EN         = 32'h018;
  localparam logic [31:0] REG_SOF_PERIOD     = 32'h01C;
  localparam logic [31:0] REG_ULPI_REG       = 32'h020;
  localparam logic [31:0] REG_EP0_SETUP_LO   = 32'h024;
  localparam logic [31:0] REG_EP0_SETUP_HI   = 32'h028;
  localparam logic [31:0] REG_FIFO_PTR       = 32'h030;
  localparam logic [31:0] REG_TX_FIFO_DATA   = 32'h034;
  localparam logic [31:0] REG_RX_FIFO_DATA   = 32'h038;
  localparam logic [31:0] REG_DMA_ADDR       = 32'h040;
  localparam logic [31:0] REG_DMA_CTRL       = 32'h044;
  localparam logic [31:0] REG_DMA_STAT       = 32'h048;
  localparam logic [31:0] REG_DMA_CUR_ADDR   = 32'h04C;
  localparam logic [31:0] REG_EP_TX_STAT     = 32'h050;
  localparam logic [31:0] REG_EP_RX_STAT     = 32'h054;
  localparam logic [31:0] REG_HOST_XACT_CTRL = 32'h060;
  localparam logic [31:0] REG_HOST_XACT_STAT = 32'h064;

  // Per-endpoint base = 12'h100 + ep * 12'h010
  //   +0x00: EPn_CFG
  //   +0x04: EPn_CTRL
  //   +0x08: EPn_STAT

endpackage : usb2_pkg
