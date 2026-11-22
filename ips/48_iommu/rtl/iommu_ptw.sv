// SPDX-License-Identifier: Apache-2.0
// iommu_ptw.sv - Sv32 2-Level Hardware Page-Table Walker over AXI4 Read Master

module iommu_ptw
  import bus_pkg::*;
  import iommu_pkg::*;
(
  input  logic         clk_i,
  input  logic         rst_ni,

  // Walk request interface
  input  logic         start_i,
  input  logic         is_write_i,
  input  logic         ad_check_en_i,
  input  logic [21:0]  root_ppn_i,
  input  logic [19:0]  vpn_i,

  // Walk response interface
  output logic         busy_o,
  output logic         done_o,
  output logic         fault_o,
  output fault_cause_e fault_cause_o,
  output logic         is_superpage_o,
  output logic [21:0]  ppn_o,
  output logic         perm_r_o,
  output logic         perm_w_o,
  output logic         flag_a_o,
  output logic         flag_d_o,

  // Dedicated AXI4 Read Master for PTE fetches
  output axi4_m2s_t    m_axi_ptw_o,
  input  axi4_s2m_t    m_axi_ptw_i
);

  typedef enum logic [2:0] {
    PTW_IDLE,
    PTW_L1_AR,
    PTW_L1_R,
    PTW_L2_AR,
    PTW_L2_R
  } ptw_state_e;

  ptw_state_e  state_q;
  logic        is_write_q;
  logic        ad_check_q;
  logic [21:0] root_ppn_q;
  logic [19:0] vpn_q;
  logic [21:0] l1_ppn_q;

  assign busy_o = (state_q != PTW_IDLE);

  // Check leaf PTE permissions & A/D flags
  function automatic void eval_leaf_pte(
    input  logic [31:0]  pte,
    input  logic         is_wr,
    input  logic         ad_chk,
    input  logic         is_l1,
    output logic         ok_o,
    output fault_cause_e cause_o
  );
    logic v, r, w, a, d;
    logic [21:0] ppn;
    v   = pte[PTE_BIT_V];
    r   = pte[PTE_BIT_R];
    w   = pte[PTE_BIT_W];
    a   = pte[PTE_BIT_A];
    d   = pte[PTE_BIT_D];
    ppn = pte[31:10];

    ok_o    = 1'b0;
    cause_o = FAULT_NONE;

    if (is_l1 && (ppn[9:0] != 10'd0)) begin
      // Misaligned 4MB superpage PTE
      cause_o = FAULT_PTE_INVALID;
    end else if (!is_wr && !r) begin
      cause_o = FAULT_PERM_RD;
    end else if (is_wr && !w) begin
      cause_o = FAULT_PERM_WR;
    end else if (ad_chk && !a) begin
      cause_o = FAULT_PTE_INVALID;
    end else if (ad_chk && is_wr && !d) begin
      cause_o = FAULT_PERM_WR;
    end else begin
      ok_o = 1'b1;
    end
  endfunction

  always_comb begin
    m_axi_ptw_o = '0;

    unique case (state_q)
      PTW_L1_AR: begin
        m_axi_ptw_o.arvalid = 1'b1;
        m_axi_ptw_o.araddr  = {root_ppn_q[19:0], vpn_q[19:10], 2'b00};
        m_axi_ptw_o.arlen   = 8'd0;
        m_axi_ptw_o.arsize  = 3'd2;
        m_axi_ptw_o.arburst = AXI_BURST_INCR;
        m_axi_ptw_o.arid    = 4'd0;
      end

      PTW_L1_R: begin
        m_axi_ptw_o.rready = 1'b1;
      end

      PTW_L2_AR: begin
        m_axi_ptw_o.arvalid = 1'b1;
        m_axi_ptw_o.araddr  = {l1_ppn_q[19:0], vpn_q[9:0], 2'b00};
        m_axi_ptw_o.arlen   = 8'd0;
        m_axi_ptw_o.arsize  = 3'd2;
        m_axi_ptw_o.arburst = AXI_BURST_INCR;
        m_axi_ptw_o.arid    = 4'd0;
      end

      PTW_L2_R: begin
        m_axi_ptw_o.rready = 1'b1;
      end

      default: ;
    endcase
  end

  wire unused_ptw_in = &{1'b0, m_axi_ptw_i.awready, m_axi_ptw_i.wready,
                         m_axi_ptw_i.bid, m_axi_ptw_i.bresp, m_axi_ptw_i.bvalid,
                         m_axi_ptw_i.rid, m_axi_ptw_i.rlast};

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      state_q        <= PTW_IDLE;
      is_write_q     <= 1'b0;
      ad_check_q     <= 1'b1;
      root_ppn_q     <= '0;
      vpn_q          <= '0;
      l1_ppn_q       <= '0;
      done_o         <= 1'b0;
      fault_o        <= 1'b0;
      fault_cause_o  <= FAULT_NONE;
      is_superpage_o <= 1'b0;
      ppn_o          <= '0;
      perm_r_o       <= 1'b0;
      perm_w_o       <= 1'b0;
      flag_a_o       <= 1'b0;
      flag_d_o       <= 1'b0;
    end else begin
      done_o <= 1'b0;

      unique case (state_q)
        PTW_IDLE: begin
          if (start_i) begin
            is_write_q <= is_write_i;
            ad_check_q <= ad_check_en_i;
            root_ppn_q <= root_ppn_i;
            vpn_q      <= vpn_i;
            state_q    <= PTW_L1_AR;
          end
        end

        PTW_L1_AR: begin
          if (m_axi_ptw_i.arready) begin
            state_q <= PTW_L1_R;
          end
        end

        PTW_L1_R: begin
          if (m_axi_ptw_i.rvalid) begin
            automatic logic        bus_err;
            automatic logic [31:0] pte;
            automatic logic        v, r, w, x, a, d;
            bus_err = (m_axi_ptw_i.rresp != AXI_RESP_OKAY) &&
                      (m_axi_ptw_i.rresp != AXI_RESP_EXOKAY);
            pte     = m_axi_ptw_i.rdata;
            v       = pte[PTE_BIT_V];
            r       = pte[PTE_BIT_R];
            w       = pte[PTE_BIT_W];
            x       = pte[PTE_BIT_X];
            a       = pte[PTE_BIT_A];
            d       = pte[PTE_BIT_D];

            if (bus_err) begin
              done_o        <= 1'b1;
              fault_o       <= 1'b1;
              fault_cause_o <= FAULT_PTW_BUS_ERR;
              state_q       <= PTW_IDLE;
            end else if (!v || (!r && w)) begin
              done_o        <= 1'b1;
              fault_o       <= 1'b1;
              fault_cause_o <= FAULT_PTE_INVALID;
              state_q       <= PTW_IDLE;
            end else if (r || x) begin
              // L1 Leaf (4MB superpage)
              automatic logic         leaf_ok;
              automatic fault_cause_e leaf_cause;
              eval_leaf_pte(pte, is_write_q, ad_check_q, 1'b1, leaf_ok, leaf_cause);
              done_o         <= 1'b1;
              fault_o        <= !leaf_ok;
              fault_cause_o  <= leaf_cause;
              is_superpage_o <= 1'b1;
              ppn_o          <= pte[31:10];
              perm_r_o       <= r && (!ad_check_q || a);
              perm_w_o       <= w && (!ad_check_q || (a && d));
              flag_a_o       <= a;
              flag_d_o       <= d;
              state_q        <= PTW_IDLE;
            end else begin
              // L1 Non-leaf pointer to L2 table
              l1_ppn_q <= pte[31:10];
              state_q  <= PTW_L2_AR;
            end
          end
        end

        PTW_L2_AR: begin
          if (m_axi_ptw_i.arready) begin
            state_q <= PTW_L2_R;
          end
        end

        PTW_L2_R: begin
          if (m_axi_ptw_i.rvalid) begin
            automatic logic        bus_err;
            automatic logic [31:0] pte;
            automatic logic        v, r, w, x, a, d;
            bus_err = (m_axi_ptw_i.rresp != AXI_RESP_OKAY) &&
                      (m_axi_ptw_i.rresp != AXI_RESP_EXOKAY);
            pte     = m_axi_ptw_i.rdata;
            v       = pte[PTE_BIT_V];
            r       = pte[PTE_BIT_R];
            w       = pte[PTE_BIT_W];
            x       = pte[PTE_BIT_X];
            a       = pte[PTE_BIT_A];
            d       = pte[PTE_BIT_D];

            if (bus_err) begin
              done_o        <= 1'b1;
              fault_o       <= 1'b1;
              fault_cause_o <= FAULT_PTW_BUS_ERR;
              state_q       <= PTW_IDLE;
            end else if (!v || (!r && w) || (!r && !w && !x)) begin
              // L2 must be a valid leaf PTE (non-leaf at L2 is invalid in Sv32)
              done_o        <= 1'b1;
              fault_o       <= 1'b1;
              fault_cause_o <= FAULT_PTE_INVALID;
              state_q       <= PTW_IDLE;
            end else begin
              automatic logic         leaf_ok;
              automatic fault_cause_e leaf_cause;
              eval_leaf_pte(pte, is_write_q, ad_check_q, 1'b0, leaf_ok, leaf_cause);
              done_o         <= 1'b1;
              fault_o        <= !leaf_ok;
              fault_cause_o  <= leaf_cause;
              is_superpage_o <= 1'b0;
              ppn_o          <= pte[31:10];
              perm_r_o       <= r && (!ad_check_q || a);
              perm_w_o       <= w && (!ad_check_q || (a && d));
              flag_a_o       <= a;
              flag_d_o       <= d;
              state_q        <= PTW_IDLE;
            end
          end
        end

        default: state_q <= PTW_IDLE;
      endcase
    end
  end

endmodule : iommu_ptw
