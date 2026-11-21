// SPDX-License-Identifier: Apache-2.0
// vkit_txn.svh - protocol-neutral transaction (uvm_sequence_item analogue), analysis port,
//                in-flight hazard tracker.
//
// One transaction class serves every protocol; bus agents translate to/from signals.
// Data convention: 32-bit bus, data/strobes sit on ABSOLUTE byte lanes (lane = addr[1:0]).
//
// UVM mapping: vkit_txn <-> uvm_sequence_item;  vkit_ap <-> uvm_analysis_port #(vkit_txn);
//              vkit_subscriber <-> uvm_subscriber / analysis_imp

typedef enum int { VK_FIXED = 0, VK_INCR = 1, VK_WRAP = 2 } vkit_burst_e;
typedef enum int { VK_OKAY = 0, VK_EXOKAY = 1, VK_SLVERR = 2, VK_DECERR = 3 } vkit_resp_e;
// TL-UH / misc opcode kinds (VK_K_PLAIN for WB/AXI/APB)
typedef enum int { VK_K_PLAIN = 0, VK_K_PUT_FULL, VK_K_PUT_PART, VK_K_GET, VK_K_ARITH, VK_K_LOGIC,
                   VK_K_INTENT } vkit_kind_e;

class vkit_item;
  static int unsigned next_uid = 1;
  int unsigned uid;
  time         t_start = 0;
  time         t_end   = 0;
  function new();
    uid = next_uid++;
  endfunction
  virtual function string convert2string();
    return $sformatf("item#%0d", uid);
  endfunction
endclass

class vkit_txn extends vkit_item;
  // --- identity / routing -------------------------------------------------
  int            master = 0;          // issuing master port
  int            id     = 0;          // AXI id / TL source (protocol-local)
  int            qos    = 0;
  // --- request ------------------------------------------------------------
  bit            write  = 0;
  bit [31:0]     addr   = 0;
  int unsigned   size   = 2;          // log2(bytes per beat), 0..2
  int unsigned   len    = 0;          // beats-1
  vkit_burst_e   burst  = VK_INCR;
  vkit_kind_e    kind   = VK_K_PLAIN;
  bit [2:0]      param  = 0;          // TL atomic param
  bit            pipelined = 0;       // WB: use pipelined (stall/cti) cycle
  bit [31:0]     wdata[$];
  bit [3:0]      wstrb[$];
  // --- response -----------------------------------------------------------
  bit [31:0]     rdata[$];
  vkit_resp_e    resp[$];             // per beat (one entry for write responses)
  // --- stimulus shaping (driver-side only) --------------------------------
  int unsigned   pre_gap = 0;         // idle cycles before the request
  int unsigned   beat_gap[$];         // idle cycles before each data beat (may be shorter)
  int unsigned   rdy_delay = 0;       // cycles before accepting the response (back-pressure)

  function new();
    super.new();
  endfunction

  function int unsigned beats();
    return len + 1;
  endfunction

  function int unsigned nbytes();
    return 1 << size;
  endfunction

  // Address of beat i - independent re-implementation of the AXI4 burst rules.
  function bit [31:0] beat_addr(int unsigned i);
    bit [31:0] nb, aligned, total, lower, off;
    nb      = 32'd1 << size;
    aligned = addr & ~(nb - 32'd1);
    total   = nb * (len + 32'd1);
    unique case (burst)
      VK_FIXED: return addr;
      VK_WRAP: begin
        lower = addr & ~(total - 32'd1);
        off   = (addr - lower + nb * i) % total;
        return lower + off;
      end
      default: return (i == 0) ? addr : (aligned + nb * i);
    endcase
  endfunction

  // Active byte lanes of beat i inside its 32-bit word (bit k = lane k)
  function bit [3:0] lanes(int unsigned i);
    bit [31:0] a, nb;
    bit [3:0]  m;
    bit [1:0]  base;
    a    = beat_addr(i);
    nb   = 32'd1 << size;
    base = 2'(a[1:0] & ~(nb[1:0] - 2'd1));
    m    = 4'((32'd1 << nb) - 32'd1) << base;
    if (i == 0 || burst == VK_FIXED) m = m & (4'hF << a[1:0]);   // unaligned first beat
    return m;
  endfunction

  // slave-side word address of beat i
  function bit [31:0] beat_word(int unsigned i);
    return beat_addr(i) & 32'hFFFF_FFFC;
  endfunction

  function bit any_err();
    foreach (resp[i]) if (resp[i] != VK_OKAY && resp[i] != VK_EXOKAY) return 1'b1;
    return 1'b0;
  endfunction

  virtual function string convert2string();
    string s;
    s = $sformatf("%s m%0d id%0d @%08h sz%0d len%0d %s", write ? "WR" : "RD", master, id, addr, size, len,
                  (burst == VK_FIXED) ? "FIXED" : (burst == VK_WRAP) ? "WRAP" : "INCR");
    if (write) foreach (wdata[i]) s = {s, $sformatf(" [%0d]%08h/%h", i, wdata[i], (i < wstrb.size()) ? wstrb[i] : 4'hx)};
    else       foreach (rdata[i]) s = {s, $sformatf(" [%0d]%08h", i, rdata[i])};
    foreach (resp[i]) if (resp[i] != VK_OKAY) s = {s, $sformatf(" resp[%0d]=%s", i, resp[i].name())};
    return s;
  endfunction

  function vkit_txn clone();
    vkit_txn c;
    c = new();
    c.master = master; c.id = id; c.qos = qos; c.write = write; c.addr = addr; c.size = size;
    c.len = len; c.burst = burst; c.kind = kind; c.param = param; c.pipelined = pipelined;
    c.wdata = wdata; c.wstrb = wstrb; c.rdata = rdata; c.resp = resp;
    c.pre_gap = pre_gap; c.beat_gap = beat_gap; c.rdy_delay = rdy_delay;
    c.t_start = t_start; c.t_end = t_end;
    return c;
  endfunction
endclass

// ---- analysis plumbing -------------------------------------------------------
virtual class vkit_subscriber;
  pure virtual function void write(vkit_txn t);
endclass

class vkit_ap;
  vkit_subscriber subs[$];
  int unsigned    n_written = 0;
  function void connect(vkit_subscriber s);
    subs.push_back(s);
  endfunction
  function void write(vkit_txn t);
    n_written++;
    foreach (subs[i]) subs[i].write(t);
  endfunction
endclass

// collects transactions into a queue (handy in directed tests)
class vkit_collector extends vkit_subscriber;
  vkit_txn q[$];
  virtual function void write(vkit_txn t);
    q.push_back(t);
  endfunction
endclass

// ---- in-flight hazard tracker --------------------------------------------------
// A driver with several outstanding transactions must not overlap a write with any other
// access to the same 8-byte granule (the reference memory model is updated at completion).
class vkit_inflight;
  vkit_txn live[$];

  function bit conflicts(vkit_txn t);
    foreach (live[i]) begin
      if (!(live[i].write || t.write)) continue;
      for (int unsigned a = 0; a <= t.len; a++)
        for (int unsigned b = 0; b <= live[i].len; b++)
          if ((t.beat_addr(a) >> 3) == (live[i].beat_addr(b) >> 3)) return 1'b1;
    end
    return 1'b0;
  endfunction

  function void add(vkit_txn t);
    live.push_back(t);
  endfunction

  function void remove(vkit_txn t);
    foreach (live[i]) if (live[i] == t) begin live.delete(i); return; end
  endfunction

  function int count();
    return live.size();
  endfunction
endclass
