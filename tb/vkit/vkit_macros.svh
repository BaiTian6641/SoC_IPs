// SPDX-License-Identifier: Apache-2.0
// vkit_macros.svh - report macros (UVM-flavoured, library-free).
// Include in a TB with:  `include "vkit_macros.svh"   (needs +incdir+tb/vkit)
`ifndef VKIT_MACROS_SVH
`define VKIT_MACROS_SVH

// `VKIT_INFO(id, msg)        verbosity LOW  (default visible, +vkit_verbosity>=1)
// `VKIT_MEDIUM(id, msg)      verbosity MEDIUM (>=2)
// `VKIT_DEBUG(id, msg)       verbosity DEBUG  (>=3)
// `VKIT_WARN / `VKIT_ERROR / `VKIT_FATAL    counted; ERROR is non-fatal until max errors
// msg is any string expression, typically $sformatf(...)
`ifdef VKIT_USE_UVM
  // Optional UVM mapping (compile-guarded, untested on this Verilator-only host)
  `define VKIT_INFO(ID, MSG)   `uvm_info((ID), (MSG), UVM_LOW)
  `define VKIT_MEDIUM(ID, MSG) `uvm_info((ID), (MSG), UVM_MEDIUM)
  `define VKIT_DEBUG(ID, MSG)  `uvm_info((ID), (MSG), UVM_HIGH)
  `define VKIT_WARN(ID, MSG)   `uvm_warning((ID), (MSG))
  `define VKIT_ERROR(ID, MSG)  `uvm_error((ID), (MSG))
  `define VKIT_FATAL(ID, MSG)  `uvm_fatal((ID), (MSG))
`else
  `define VKIT_INFO(ID, MSG)   vkit_pkg::vkit_report::info((ID), (MSG), 1)
  `define VKIT_MEDIUM(ID, MSG) vkit_pkg::vkit_report::info((ID), (MSG), 2)
  `define VKIT_DEBUG(ID, MSG)  vkit_pkg::vkit_report::info((ID), (MSG), 3)
  `define VKIT_WARN(ID, MSG)   vkit_pkg::vkit_report::warning((ID), (MSG))
  `define VKIT_ERROR(ID, MSG)  vkit_pkg::vkit_report::error((ID), (MSG))
  `define VKIT_FATAL(ID, MSG)  vkit_pkg::vkit_report::fatal((ID), (MSG))
`endif

// Check helpers: `VKIT_CHECK(cond, id, msg) counts an error when cond is false.
`define VKIT_CHECK(COND, ID, MSG) \
  do begin if (!(COND)) vkit_pkg::vkit_report::error((ID), (MSG)); \
           else vkit_pkg::vkit_report::pass_count++; end while (0)

`endif
