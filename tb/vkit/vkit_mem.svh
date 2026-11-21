// SPDX-License-Identifier: Apache-2.0
// vkit_mem.svh - sparse byte-addressed reference memory, address map, slave behaviour config.
//
// UVM mapping: vkit_mem <-> a plain SV class used as ref-model storage (uvm_mem is a RAL
//              object and unrelated); vkit_addr_map <-> uvm_reg_map-lite for decode.

class vkit_mem;
  bit [7:0]   bytes[bit [31:0]];
  string      name;
  int unsigned salt = 32'h5A5A_1234;

  function new(string name_ = "mem", int unsigned salt_ = 32'h5A5A_1234);
    name = name_;
    salt = salt_;
  endfunction

  // Deterministic "power-up" content so reads of never-written locations are checkable
  function bit [7:0] init_byte(bit [31:0] a);
    bit [31:0] h;
    h = (a ^ salt) * 32'h9E37_79B1;
    h = h ^ (h >> 15);
    h = h * 32'h85EB_CA6B;
    return h[31:24];
  endfunction

  function bit [7:0] read_byte(bit [31:0] a);
    if (bytes.exists(a)) return bytes[a];
    return init_byte(a);
  endfunction

  function void write_byte(bit [31:0] a, bit [7:0] d);
    bytes[a] = d;
  endfunction

  function bit is_written(bit [31:0] a);
    return bytes.exists(a);
  endfunction

  // 32-bit word at word-aligned address (little endian)
  function bit [31:0] read_word(bit [31:0] a);
    bit [31:0] w, wa;
    wa = a & 32'hFFFF_FFFC;
    for (int i = 0; i < 4; i++) w[8*i +: 8] = read_byte(wa + 32'(i));
    return w;
  endfunction

  function void write_word(bit [31:0] a, bit [31:0] d, bit [3:0] strb);
    bit [31:0] wa;
    wa = a & 32'hFFFF_FFFC;
    for (int i = 0; i < 4; i++) if (strb[i]) write_byte(wa + 32'(i), d[8*i +: 8]);
  endfunction

  function int unsigned num_written();
    return bytes.num();
  endfunction
endclass

// Address map with first-match priority; same rule convention as the RTL crossbars
// ((addr & mask) == (base & mask)) but implemented independently.
class vkit_addr_map;
  bit [31:0] base[$];
  bit [31:0] mask[$];

  function void add(bit [31:0] b, bit [31:0] m);
    base.push_back(b);
    mask.push_back(m);
  endfunction

  function int n();
    return base.size();
  endfunction

  // slave index or -1 when unmapped
  function int decode(bit [31:0] a);
    foreach (base[i]) if ((a & mask[i]) == (base[i] & mask[i])) return i;
    return -1;
  endfunction

  // packed rule vector for the DUT address-map port (bus_pkg::addr_map_rule_t has base then mask)
  function bit [63:0] rule(int i);
    return {base[i], mask[i]};
  endfunction
endclass

// Behaviour knobs of a reactive slave (responder)
class vkit_slave_cfg;
  vkit_dist  lat;                 // response latency in cycles (>=0) after request acceptance
  int        stall_pct = 0;       // % of cycles the slave refuses a new request (ready/stall)
  bit [31:0] err_lo = 32'h1;      // error window (absolute addresses); accesses answered with error
  bit [31:0] err_hi = 32'h0;      // err_lo > err_hi  => no window
  bit        err_decerr = 0;      // AXI: error code DECERR instead of SLVERR
  function new();
    lat = new("slave_lat");
    void'(lat.add(0, 0, 55));
    void'(lat.add(1, 3, 35));
    void'(lat.add(4, 12, 10));
  endfunction
  function bit in_err_window(bit [31:0] a);
    return (a >= err_lo) && (a <= err_hi);
  endfunction
endclass
