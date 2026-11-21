// SPDX-License-Identifier: Apache-2.0
// timer_group.sv - Daisy-chains NUM_TIM timer_pwm_core instances (master/slave synchronisation helper)
//
// ITR wiring for timer k (TS selects ITR0..ITR3 in SMCR):
//   ITR0 = TRGO of timer k-1   (k = 0: ext_trg_i)
//   ITR1 = TRGO of timer k-2   (k < 2: ext_trg_i)
//   ITR2 = TRGO of timer k-3   (k < 3: ext_trg_i)
//   ITR3 = ext_trg_i           (common external / system trigger)
// Each instance has its own CSR port (decode externally, e.g. one timer_pwm_<bus> per slot or an address split in
// the SoC); pads are packed [timer][channel].  All timers share clk_i/rst_ni, so ITR links need no synchroniser.
// Typical use: timer 0 = master (MMS=enable/update), timers 1..n slaves (SMS=trigger/gated/reset, TS=ITR0).
// irq_o is the OR of all timers; irq_vec_o[k] is the per-timer aggregate.

module timer_group
  import bus_pkg::*;
#(
  parameter int unsigned NUM_TIM = 3,
  parameter int unsigned CNT_W   = 16,
  parameter int unsigned NUM_CH  = 4
) (
  input  logic                clk_i,
  input  logic                rst_ni,

  input  csr_req_t            csr_req_i [NUM_TIM],
  output csr_rsp_t            csr_rsp_o [NUM_TIM],

  input  logic                ext_trg_i,
  input  logic                dbg_freeze_i,
  input  logic [NUM_TIM-1:0][NUM_CH-1:0] ti_i,
  input  logic [NUM_TIM-1:0]  etr_i,
  input  logic [NUM_TIM-1:0][1:0] brk_i,
  input  logic [NUM_TIM-1:0]  index_i,

  output logic [NUM_TIM-1:0][NUM_CH-1:0] pwm_o,
  output logic [NUM_TIM-1:0][NUM_CH-1:0] pwm_oe_o,
  output logic [NUM_TIM-1:0][NUM_CH-1:0] pwm_n_o,
  output logic [NUM_TIM-1:0][NUM_CH-1:0] pwm_n_oe_o,
  output logic [NUM_TIM-1:0]  trgo_o,
  output logic [NUM_TIM-1:0][1:0] adc_trg_o,
  output logic [NUM_TIM-1:0]  brk_active_o,
  output logic [NUM_TIM-1:0][NUM_CH+2:0] dma_req_o,
  input  logic [NUM_TIM-1:0][NUM_CH+2:0] dma_ack_i,
  output logic [NUM_TIM-1:0]  irq_vec_o,
  output logic                irq_o
);

  for (genvar k = 0; k < int'(NUM_TIM); k++) begin : g_tim
    logic [3:0] itr;
    if (k >= 1) begin : g_i0
      assign itr[0] = trgo_o[(k >= 1) ? k - 1 : 0];
    end else begin : g_i0e
      assign itr[0] = ext_trg_i;
    end
    if (k >= 2) begin : g_i1
      assign itr[1] = trgo_o[(k >= 2) ? k - 2 : 0];
    end else begin : g_i1e
      assign itr[1] = ext_trg_i;
    end
    if (k >= 3) begin : g_i2
      assign itr[2] = trgo_o[(k >= 3) ? k - 3 : 0];
    end else begin : g_i2e
      assign itr[2] = ext_trg_i;
    end
    assign itr[3] = ext_trg_i;

    logic [3:0] vec;
    timer_pwm_core #(.CNT_W(CNT_W), .NUM_CH(NUM_CH)) u_tim (
      .clk_i, .rst_ni,
      .csr_req_i(csr_req_i[k]), .csr_rsp_o(csr_rsp_o[k]),
      .ti_i(ti_i[k]), .etr_i(etr_i[k]), .brk_i(brk_i[k]), .index_i(index_i[k]),
      .itr_i(itr), .dbg_freeze_i(dbg_freeze_i),
      .pwm_o(pwm_o[k]), .pwm_oe_o(pwm_oe_o[k]), .pwm_n_o(pwm_n_o[k]), .pwm_n_oe_o(pwm_n_oe_o[k]),
      .trgo_o(trgo_o[k]), .adc_trg_o(adc_trg_o[k]), .brk_active_o(brk_active_o[k]),
      .dma_req_o(dma_req_o[k]), .dma_ack_i(dma_ack_i[k]),
      .irq_o(irq_vec_o[k]), .irq_vec_o(vec)
    );
  end

  assign irq_o = |irq_vec_o;

endmodule : timer_group
