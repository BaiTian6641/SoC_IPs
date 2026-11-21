// SPDX-License-Identifier: Apache-2.0
// vkit_pkg.sv - library-free, UVM-flavoured verification kit (see docs/10_verification_methodology.md)
//
// Compile order: bus_pkg.sv, vkit_if.sv, vkit_pkg.sv   (tb/vkit/filelist.f does this)
`timescale 1ns/1ps

package vkit_pkg;
  import bus_pkg::*;
`ifdef VKIT_USE_UVM
  import uvm_pkg::*;
  `include "uvm_macros.svh"
`endif

  `include "vkit_report.svh"
  `include "vkit_rng.svh"
  `include "vkit_cov.svh"
  `include "vkit_txn.svh"
  `include "vkit_mem.svh"
  `include "vkit_sb.svh"
  `include "vkit_gen.svh"
  `include "vkit_wb.svh"
  `include "vkit_axi.svh"
  `include "vkit_tl.svh"
  `include "vkit_apb.svh"
  `include "vkit_csr.svh"
  `include "vkit_uvm_shim.svh"
endpackage : vkit_pkg
