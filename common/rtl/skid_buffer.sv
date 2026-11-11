// SPDX-License-Identifier: Apache-2.0
// skid_buffer.sv - Zero-Bubble Valid/Ready Skid Buffer for Timing Closure

module skid_buffer #(
  parameter type T = logic [31:0]
) (
  input  logic clk_i,
  input  logic rst_ni,

  input  logic s_valid_i,
  output logic s_ready_o,
  input  T     s_data_i,

  output logic m_valid_o,
  input  logic m_ready_i,
  output T     m_data_o
);

  logic full_q;
  T     skid_q, out_q;
  logic valid_q;

  assign s_ready_o = !full_q;
  assign m_valid_o = valid_q;
  assign m_data_o  = out_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      valid_q <= 1'b0;
      full_q  <= 1'b0;
      out_q   <= '0;
      skid_q  <= '0;
    end else begin
      if (full_q) begin
        if (m_ready_i) begin
          out_q   <= skid_q;
          full_q  <= 1'b0;
          valid_q <= 1'b1;
        end
      end else begin
        if (s_valid_i && (!valid_q || m_ready_i)) begin
          out_q   <= s_data_i;
          valid_q <= 1'b1;
        end else if (s_valid_i && valid_q && !m_ready_i) begin
          skid_q  <= s_data_i;
          full_q  <= 1'b1;
        end else if (!s_valid_i && valid_q && m_ready_i) begin
          valid_q <= 1'b0;
        end
      end
    end
  end

endmodule : skid_buffer
