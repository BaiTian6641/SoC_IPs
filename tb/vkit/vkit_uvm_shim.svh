// SPDX-License-Identifier: Apache-2.0
// vkit_uvm_shim.svh - Optional thin UVM 1.2 / IEEE 1800.2 migration shim sketch.
//
// STATUS: COMPILE-GUARDED & UNTESTED ON THIS MACHINE (Verilator 5.054 only; no UVM
//         library, QuestaSim, Xcelium, or VCS installed here).
//
// Purpose: Demonstrates how vkit's library-free classes map directly onto standard UVM
//          base classes when porting to QuestaSim / Xcelium / VCS with +define+VKIT_USE_UVM.
//          See docs/10_verification_methodology.md for the full migration guide.

`ifdef VKIT_USE_UVM
  // Requires: `import uvm_pkg::*;` and `` `include "uvm_macros.svh" `` compiled before vkit_pkg.

  class vkit_uvm_seq_item extends uvm_sequence_item;
    `uvm_object_utils(vkit_uvm_seq_item)
    vkit_txn payload;

    function new(string name = "vkit_uvm_seq_item");
      super.new(name);
      payload = new();
    endfunction

    virtual function string convert2string();
      return (payload != null) ? payload.convert2string() : "<null>";
    endfunction
  endclass

  // Generic uvm_driver wrapper delegating bus pinning to a vkit_*_master object
  class vkit_uvm_wb_driver extends uvm_driver #(vkit_uvm_seq_item);
    `uvm_component_utils(vkit_uvm_wb_driver)
    virtual vkit_wb_if vif;
    vkit_wb_master     impl;
    int                master_idx = 0;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual vkit_wb_if)::get(this, "", "vif", vif))
        `uvm_fatal("NO_VIF", "vkit_wb_if not set in uvm_config_db")
      impl = new(get_full_name(), master_idx, vif);
    endfunction

    virtual task run_phase(uvm_phase phase);
      vkit_uvm_seq_item req_item;
      impl.start();
      forever begin
        seq_item_port.get_next_item(req_item);
        impl.do_txn(req_item.payload);
        seq_item_port.item_done();
      end
    endtask
  endclass

  // Bridge from vkit_subscriber -> uvm_analysis_port #(vkit_uvm_seq_item)
  class vkit_uvm_ap_bridge extends vkit_subscriber;
    uvm_analysis_port #(vkit_uvm_seq_item) uvm_ap;
    function new(uvm_analysis_port #(vkit_uvm_seq_item) ap_);
      uvm_ap = ap_;
    endfunction
    virtual function void write(vkit_txn t);
      vkit_uvm_seq_item wrap;
      wrap = vkit_uvm_seq_item::type_id::create("mon_item");
      wrap.payload = t.clone();
      uvm_ap.write(wrap);
    endfunction
  endclass

  class vkit_uvm_wb_monitor extends uvm_monitor;
    `uvm_component_utils(vkit_uvm_wb_monitor)
    virtual vkit_wb_if                     vif;
    uvm_analysis_port #(vkit_uvm_seq_item) ap;
    vkit_wb_monitor                        impl;
    vkit_uvm_ap_bridge                     bridge;
    int                                    port_idx = 0;
    bit                                    group_bursts = 1'b1;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual vkit_wb_if)::get(this, "", "vif", vif))
        `uvm_fatal("NO_VIF", "vkit_wb_if not set in uvm_config_db")
      impl   = new(get_full_name(), port_idx, vif, group_bursts);
      bridge = new(ap);
      impl.ap.connect(bridge);
    endfunction

    virtual task run_phase(uvm_phase phase);
      impl.run();
    endtask
  endclass

  class vkit_uvm_fabric_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(vkit_uvm_fabric_scoreboard)
    uvm_analysis_imp #(vkit_uvm_seq_item, vkit_uvm_fabric_scoreboard) mst_imp;
    vkit_fabric_sb impl;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    virtual function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      mst_imp = new("mst_imp", this);
    endfunction

    virtual function void write(vkit_uvm_seq_item item);
      if (impl != null && item != null) impl.write_mst(item.payload);
    endfunction

    virtual function void check_phase(uvm_phase phase);
      super.check_phase(phase);
      if (impl != null) impl.check_final();
    endfunction
  endclass
`endif
