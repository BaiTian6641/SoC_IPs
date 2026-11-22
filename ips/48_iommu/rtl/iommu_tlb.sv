// SPDX-License-Identifier: Apache-2.0
// iommu_tlb.sv - Parameterized Fully-Associative Sv32 TLB with Targeted Invalidation

module iommu_tlb
  import iommu_pkg::*;
#(
  parameter int unsigned ENTRIES = IOMMU_TLB_ENTRIES
) (
  input  logic       clk_i,
  input  logic       rst_ni,

  // Combinational lookup port
  input  logic [3:0]  lookup_stream_id_i,
  input  logic [7:0]  lookup_asid_i,
  input  logic [19:0] lookup_vpn_i,
  output logic        hit_o,
  output tlb_entry_t  hit_entry_o,

  // Synchronous fill port
  input  logic        insert_valid_i,
  input  tlb_entry_t  insert_entry_i,

  // Maintenance / invalidation ports
  input  logic        inv_all_i,
  input  logic        inv_stream_i,
  input  logic [3:0]  inv_stream_id_i,
  input  logic        inv_iova_i,
  input  logic [19:0] inv_vpn_i
);

  localparam int unsigned PTR_W = (ENTRIES <= 1) ? 1 : $clog2(ENTRIES);

  tlb_entry_t [ENTRIES-1:0] entries_q;
  logic [PTR_W-1:0]         rr_ptr_q;

  logic [ENTRIES-1:0] hit_vec;
  logic [ENTRIES-1:0] dup_vec;
  logic               has_free;
  logic [PTR_W-1:0]   free_idx;
  logic               has_dup;
  logic [PTR_W-1:0]   dup_idx;

  // Combinational CAM lookup and fill-slot selection
  always_comb begin
    hit_vec     = '0;
    dup_vec     = '0;
    hit_entry_o = '0;
    has_free    = 1'b0;
    free_idx    = '0;
    has_dup     = 1'b0;
    dup_idx     = '0;

    for (int i = 0; i < int'(ENTRIES); i++) begin
      automatic logic vpn_match_lkp;
      automatic logic vpn_match_ins;
      automatic logic [PTR_W-1:0] idx;
      idx = PTR_W'(i);

      vpn_match_lkp = entries_q[idx].is_superpage
                    ? (entries_q[idx].vpn[19:10] == lookup_vpn_i[19:10])
                    : (entries_q[idx].vpn == lookup_vpn_i);

      if (entries_q[idx].valid &&
          (entries_q[idx].stream_id == lookup_stream_id_i) &&
          (entries_q[idx].asid == lookup_asid_i) &&
          vpn_match_lkp) begin
        hit_vec[idx] = 1'b1;
        hit_entry_o  = entries_q[idx];
      end

      vpn_match_ins = (entries_q[idx].is_superpage == insert_entry_i.is_superpage) &&
                      (insert_entry_i.is_superpage
                       ? (entries_q[idx].vpn[19:10] == insert_entry_i.vpn[19:10])
                       : (entries_q[idx].vpn == insert_entry_i.vpn));

      if (entries_q[idx].valid &&
          (entries_q[idx].stream_id == insert_entry_i.stream_id) &&
          (entries_q[idx].asid == insert_entry_i.asid) &&
          vpn_match_ins) begin
        dup_vec[idx] = 1'b1;
        if (!has_dup) begin
          has_dup = 1'b1;
          dup_idx = idx;
        end
      end

      if (!entries_q[idx].valid && !has_free) begin
        has_free = 1'b1;
        free_idx = idx;
      end
    end

    hit_o = |hit_vec;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      entries_q <= '0;
      rr_ptr_q  <= '0;
    end else if (inv_all_i) begin
      entries_q <= '0;
      rr_ptr_q  <= '0;
    end else begin
      // Targeted invalidations
      for (int i = 0; i < int'(ENTRIES); i++) begin
        automatic logic [PTR_W-1:0] idx;
        idx = PTR_W'(i);
        if (entries_q[idx].valid && (entries_q[idx].stream_id == inv_stream_id_i)) begin
          if (inv_stream_i) begin
            entries_q[idx].valid <= 1'b0;
          end else if (inv_iova_i) begin
            automatic logic inv_match;
            inv_match = entries_q[idx].is_superpage
                      ? (entries_q[idx].vpn[19:10] == inv_vpn_i[19:10])
                      : (entries_q[idx].vpn == inv_vpn_i);
            if (inv_match) begin
              entries_q[idx].valid <= 1'b0;
            end
          end
        end
      end

      // Insert (update matching slot if present, else first free, else round-robin victim)
      if (insert_valid_i) begin
        automatic logic [PTR_W-1:0] dst_idx;
        if (has_dup) begin
          dst_idx = dup_idx;
        end else if (has_free) begin
          dst_idx = free_idx;
        end else begin
          dst_idx  = rr_ptr_q;
          rr_ptr_q <= (rr_ptr_q == PTR_W'(ENTRIES - 1)) ? '0 : (rr_ptr_q + 1'b1);
        end
        entries_q[dst_idx]       <= insert_entry_i;
        entries_q[dst_idx].valid <= 1'b1;
      end
    end
  end

endmodule : iommu_tlb
