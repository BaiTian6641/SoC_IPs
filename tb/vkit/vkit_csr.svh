// SPDX-License-Identifier: Apache-2.0
// vkit_csr.svh - Reusable generic CSR verification engine (`csr_random_test`).
//
// Any IP testbench can plug in a register table + one of the protocol BFM adapters
// (WB / AXI4 / TL / APB, or a custom vkit_csr_bfm subclass) to get:
//   1. Reset-value verification against the register table
//   2. Directed RW / RO / W1C / WO / byte-strobe (wstrb) tests
//   3. Constrained-random W/R regression with shadow model and partial strobes
//   4. Optional unmapped-offset bus error verification
//   5. Functional coverage bins logged to vkit_cov
//
// UVM mapping: csr_random_test <-> uvm_reg_hw_reset_seq + uvm_reg_bit_bash_seq +
//              random uvm_reg_sequence; vkit_csr_bfm <-> uvm_reg_adapter + predictor.

typedef struct {
  string     name;
  bit [31:0] offset;
  bit [31:0] reset_val;
  bit [31:0] rw_mask;
  bit [31:0] ro_mask;
  bit [31:0] w1c_mask;
  bit [31:0] wo_mask;
  bit [31:0] volatile_mask;
  bit        wo_reads_zero;
  bit        err_on_write;
} vkit_csr_reg_t;

// ---- Protocol-agnostic CSR transport interface ------------------------------------
virtual class vkit_csr_bfm;
  virtual task write(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    err = 1'b1;
  endtask
  virtual task read(bit [31:0] addr, output bit [31:0] data, output bit err);
    data = 32'd0;
    err  = 1'b1;
  endtask
endclass

class vkit_csr_wb_bfm extends vkit_csr_bfm;
  vkit_wb_master drv;
  function new(vkit_wb_master drv_); drv = drv_; endfunction
  virtual task write(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    drv.write32(addr, data, strb, err);
  endtask
  virtual task read(bit [31:0] addr, output bit [31:0] data, output bit err);
    drv.read32(addr, data, err);
  endtask
endclass

class vkit_csr_axi_bfm extends vkit_csr_bfm;
  vkit_axi_master drv;
  function new(vkit_axi_master drv_); drv = drv_; endfunction
  virtual task write(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    drv.write32(addr, data, strb, err);
  endtask
  virtual task read(bit [31:0] addr, output bit [31:0] data, output bit err);
    drv.read32(addr, data, err);
  endtask
endclass

class vkit_csr_tl_bfm extends vkit_csr_bfm;
  vkit_tl_master drv;
  function new(vkit_tl_master drv_); drv = drv_; endfunction
  virtual task write(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    drv.write32(addr, data, strb, err);
  endtask
  virtual task read(bit [31:0] addr, output bit [31:0] data, output bit err);
    drv.read32(addr, data, err);
  endtask
endclass

class vkit_csr_apb_bfm extends vkit_csr_bfm;
  vkit_apb_master drv;
  function new(vkit_apb_master drv_); drv = drv_; endfunction
  virtual task write(bit [31:0] addr, bit [31:0] data, bit [3:0] strb, output bit err);
    drv.write32(addr, data, strb, err);
  endtask
  virtual task read(bit [31:0] addr, output bit [31:0] data, output bit err);
    drv.read32(addr, data, err);
  endtask
endclass

// ===================================================================================
class csr_random_test;
  string         name;
  vkit_csr_bfm   bfm;
  vkit_rng       rng;
  bit [31:0]     base_addr = 32'h0;
  bit            supports_byte_strobes = 1'b1;   // set 0 if DUT ignores partial wstrb
  vkit_csr_reg_t regs[$];
  bit [31:0]     shadow[$];
  bit [31:0]     unmapped_offs[$];
  int unsigned   n_reads  = 0;
  int unsigned   n_writes = 0;

  function new(string name_, vkit_csr_bfm bfm_, bit [31:0] base_addr_ = 32'h0);
    name = name_; bfm = bfm_; base_addr = base_addr_;
    rng = new({"csr_", name_});
    vkit_cov::declare({"csr.", name}, '{"reset", "rw", "ro", "w1c", "wo", "partial_strb", "rand_wr", "rand_rd", "unmapped"});
  endfunction

  function void add_reg(
    string     rname,
    bit [31:0] offset,
    bit [31:0] reset_val,
    bit [31:0] rw_mask       = 32'h0,
    bit [31:0] ro_mask       = 32'h0,
    bit [31:0] w1c_mask      = 32'h0,
    bit [31:0] wo_mask       = 32'h0,
    bit [31:0] volatile_mask = 32'h0,
    bit        wo_reads_zero = 1'b1,
    bit        err_on_write  = 1'b0
  );
    vkit_csr_reg_t r;
    r = '{name: rname, offset: offset, reset_val: reset_val, rw_mask: rw_mask,
          ro_mask: ro_mask, w1c_mask: w1c_mask, wo_mask: wo_mask,
          volatile_mask: volatile_mask, wo_reads_zero: wo_reads_zero,
          err_on_write: err_on_write};
    regs.push_back(r);
    shadow.push_back(reset_val);
  endfunction

  function void add_unmapped(bit [31:0] offset);
    unmapped_offs.push_back(offset);
  endfunction

  // Notify the shadow model when external stimulus sets W1C status bits in hardware
  function void notify_hw_set(bit [31:0] offset, bit [31:0] set_mask);
    foreach (regs[i]) begin
      if (regs[i].offset == offset) begin
        shadow[i] = shadow[i] | (set_mask & regs[i].w1c_mask);
        return;
      end
    end
  endfunction

  static function bit [31:0] strb_mask(bit [3:0] s);
    bit [31:0] m;
    for (int k = 0; k < 4; k++) m[8*k +: 8] = {8{s[k]}};
    return m;
  endfunction

  function bit [31:0] readable_mask(int idx);
    vkit_csr_reg_t r;
    r = regs[idx];
    if (r.wo_reads_zero)
      return (r.rw_mask | r.ro_mask | r.w1c_mask | r.wo_mask) & ~r.volatile_mask;
    else
      return (r.rw_mask | r.ro_mask | r.w1c_mask) & ~r.volatile_mask;
  endfunction

  function void apply_shadow_write(int idx, bit [31:0] wdata, bit [3:0] strb);
    bit [31:0] sm, rw_eff, w1c_eff;
    sm      = supports_byte_strobes ? strb_mask(strb) : 32'hFFFF_FFFF;
    rw_eff  = regs[idx].rw_mask  & sm;
    w1c_eff = regs[idx].w1c_mask & sm;
    shadow[idx] = (shadow[idx] & ~rw_eff) | (wdata & rw_eff);
    shadow[idx] = shadow[idx] & ~(wdata & w1c_eff);
    if (regs[idx].wo_reads_zero) shadow[idx] = shadow[idx] & ~regs[idx].wo_mask;
  endfunction

  task do_write_idx(int idx, bit [31:0] wdata, bit [3:0] strb);
    bit err;
    bfm.write(base_addr + regs[idx].offset, wdata, strb, err);
    n_writes++;
    if (err != regs[idx].err_on_write) begin
      vkit_report::error({"CSR.", name},
        $sformatf("write %s (@+%04h) strb=%h err=%0b expected %0b",
                  regs[idx].name, regs[idx].offset, strb, err, regs[idx].err_on_write));
    end else begin
      vkit_report::pass_count++;
    end
    if (!err) apply_shadow_write(idx, wdata, strb);
  endtask

  task do_read_check_idx(int idx, string phase);
    bit [31:0] got, exp, cmask;
    bit        err;
    bfm.read(base_addr + regs[idx].offset, got, err);
    n_reads++;
    if (err) begin
      vkit_report::error({"CSR.", name},
        $sformatf("[%s] unexpected read error on %s (@+%04h)", phase, regs[idx].name, regs[idx].offset));
      return;
    end
    cmask = readable_mask(idx);
    exp   = shadow[idx];
    if (regs[idx].wo_reads_zero) exp = exp & ~regs[idx].wo_mask;
    if (((got ^ exp) & cmask) !== 32'd0) begin
      vkit_report::error({"CSR.", name},
        $sformatf("[%s] %s (@+%04h) mismatch: got=%08h exp=%08h mask=%08h",
                  phase, regs[idx].name, regs[idx].offset, got, exp, cmask));
    end else begin
      vkit_report::pass_count++;
    end
  endtask

  // ---- 1. Reset value check ----
  task check_reset();
    vkit_report::info({"CSR.", name}, $sformatf("checking reset values for %0d registers", regs.size()), 1);
    foreach (regs[i]) begin
      shadow[i] = regs[i].reset_val;
      if (regs[i].wo_reads_zero) shadow[i] = shadow[i] & ~regs[i].wo_mask;
      do_read_check_idx(i, "RESET");
      vkit_cov::hit({"csr.", name, ".reset"});
    end
  endtask

  // ---- 2. Directed attribute checks (RW, RO, W1C, WO, byte strobes, unmapped) ----
  task check_directed();
    bit [31:0] pats[5];
    bit [3:0]  strbs[6];
    pats[0] = 32'h0000_0000;
    pats[1] = 32'hFFFF_FFFF;
    pats[2] = 32'hA5A5_5A5A;
    pats[3] = 32'h5A5A_A5A5;
    pats[4] = 32'h1234_5678;
    strbs[0] = 4'b0001; strbs[1] = 4'b0010; strbs[2] = 4'b0100;
    strbs[3] = 4'b1000; strbs[4] = 4'b0011; strbs[5] = 4'b1100;

    foreach (regs[i]) begin
      // RW / RO patterns
      if (regs[i].rw_mask != 32'd0 || regs[i].ro_mask != 32'd0) begin
        foreach (pats[p]) begin
          do_write_idx(i, pats[p], 4'hF);
          do_read_check_idx(i, "DIR-PAT");
        end
        if (regs[i].rw_mask != 32'd0) vkit_cov::hit({"csr.", name, ".rw"});
        if (regs[i].ro_mask != 32'd0) vkit_cov::hit({"csr.", name, ".ro"});
      end

      // Walking-1 across RW bits (sample up to 32 single-bit masks)
      if (regs[i].rw_mask != 32'd0) begin
        for (int b = 0; b < 32; b++) begin
          if (regs[i].rw_mask[b]) begin
            do_write_idx(i, 32'd1 << b, 4'hF);
            do_read_check_idx(i, "WALK1");
          end
        end
      end

      // Partial byte-strobe tests
      if (supports_byte_strobes && regs[i].rw_mask != 32'd0 && !regs[i].err_on_write) begin
        do_write_idx(i, 32'hA5A5_A5A5, 4'hF);
        foreach (strbs[k]) begin
          do_write_idx(i, rng.u32(), strbs[k]);
          do_read_check_idx(i, "STRB");
          vkit_cov::hit({"csr.", name, ".partial_strb"});
        end
      end

      // W1C check: write 0 preserves, write 1 clears
      if (regs[i].w1c_mask != 32'd0) begin
        do_write_idx(i, ~regs[i].w1c_mask, 4'hF);
        do_read_check_idx(i, "W1C-HOLD");
        do_write_idx(i, regs[i].w1c_mask, 4'hF);
        do_read_check_idx(i, "W1C-CLR");
        vkit_cov::hit({"csr.", name, ".w1c"});
      end

      // WO check: write random data, verify readback & no corruption
      if (regs[i].wo_mask != 32'd0) begin
        do_write_idx(i, rng.u32(), 4'hF);
        do_read_check_idx(i, "WO");
        vkit_cov::hit({"csr.", name, ".wo"});
      end
    end

    // Verify no cross-register aliasing after directed writes
    foreach (regs[i]) do_read_check_idx(i, "POST-DIR");

    // Unmapped offsets
    foreach (unmapped_offs[u]) begin
      bit [31:0] rdat;
      bit        err;
      bfm.read(base_addr + unmapped_offs[u], rdat, err);
      n_reads++;
      if (!err) vkit_report::error({"CSR.", name}, $sformatf("[UNMAP] read @+%04h did not assert error", unmapped_offs[u]));
      else vkit_report::pass_count++;
      bfm.write(base_addr + unmapped_offs[u], rng.u32(), 4'hF, err);
      n_writes++;
      if (!err) vkit_report::error({"CSR.", name}, $sformatf("[UNMAP] write @+%04h did not assert error", unmapped_offs[u]));
      else vkit_report::pass_count++;
      vkit_cov::hit({"csr.", name, ".unmapped"});
    end
  endtask

  // ---- 3. Constrained-random W/R sequence ----
  task run_random(int unsigned n_ops = 200);
    int unsigned n;
    n = vkit_cfg::scale(int'(n_ops));
    if (regs.size() == 0) return;
    for (int unsigned k = 0; k < n; k++) begin
      int idx;
      idx = int'(rng.range(0, regs.size() - 1));
      if (rng.chance(55)) begin
        bit [3:0] st;
        st = (supports_byte_strobes && rng.chance(40)) ? 4'(rng.range(1, 14)) : 4'hF;
        do_write_idx(idx, rng.u32(), st);
        vkit_cov::hit({"csr.", name, ".rand_wr"});
        if (st != 4'hF) vkit_cov::hit({"csr.", name, ".partial_strb"});
      end else begin
        do_read_check_idx(idx, "RAND");
        vkit_cov::hit({"csr.", name, ".rand_rd"});
      end
    end
    // Final sweep across every register
    foreach (regs[i]) do_read_check_idx(i, "FINAL");
    vkit_report::info({"CSR.", name}, $sformatf("completed %0d writes, %0d reads across %0d regs",
                      n_writes, n_reads, regs.size()), 1);
  endtask

  task run_all(int unsigned n_random = 200);
    check_reset();
    check_directed();
    run_random(n_random);
  endtask
endclass

typedef csr_random_test vkit_csr_random_test;
