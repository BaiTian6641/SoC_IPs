// SPDX-License-Identifier: Apache-2.0
// iommu_top.sv - Parameterized Quad-Bus Hardware IOMMU / SMMU-Lite Top Wrapper

module iommu_top
  import bus_pkg::*;
  import iommu_pkg::*;
#(
  parameter bus_pkg::bus_proto_e BUS_TYPE             = bus_pkg::BUS_AXI4,
  parameter int unsigned         NUM_STREAMS          = IOMMU_NUM_STREAMS,
  parameter int unsigned         TLB_ENTRIES          = IOMMU_TLB_ENTRIES,
  parameter int unsigned         FAULT_FIFO_DEPTH     = IOMMU_FAULT_DEPTH,
  parameter bit                  USE_AXI_ID_AS_STREAM = 1'b0
) (
  input  logic       clk_i,
  input  logic       rst_ni,

  // Selectable CSR Control Slave Ports (tie unused inputs to '0)
  input  wb_m2s_t    wb_i,
  output wb_s2m_t    wb_o,
  input  axi4_m2s_t  axi_i,
  output axi4_s2m_t  axi_o,
  input  tl_m2s_t    tl_i,
  output tl_s2m_t    tl_o,
  input  apb_m2s_t   apb_i,
  output apb_s2m_t   apb_o,

  // Inline AXI4 Data Path: Upstream Device Master -> IOMMU -> Downstream Memory
  input  axi4_m2s_t  s_axi_dev_i,
  output axi4_s2m_t  s_axi_dev_o,
  input  logic [3:0] aw_stream_id_i,
  input  logic [3:0] ar_stream_id_i,

  output axi4_m2s_t  m_axi_mem_o,
  input  axi4_s2m_t  m_axi_mem_i,

  output logic       irq_o
);

  if (BUS_TYPE == BUS_WISHBONE) begin : gen_wb
    assign axi_o = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, axi_i, tl_i, apb_i};

    iommu_wb #(
      .NUM_STREAMS          (NUM_STREAMS),
      .TLB_ENTRIES          (TLB_ENTRIES),
      .FAULT_FIFO_DEPTH     (FAULT_FIFO_DEPTH),
      .USE_AXI_ID_AS_STREAM (USE_AXI_ID_AS_STREAM)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .wb_i           (wb_i),
      .wb_o           (wb_o),
      .s_axi_dev_i    (s_axi_dev_i),
      .s_axi_dev_o    (s_axi_dev_o),
      .aw_stream_id_i (aw_stream_id_i),
      .ar_stream_id_i (ar_stream_id_i),
      .m_axi_mem_o    (m_axi_mem_o),
      .m_axi_mem_i    (m_axi_mem_i),
      .irq_o          (irq_o)
    );
  end else if (BUS_TYPE == BUS_TILELINK) begin : gen_tl
    assign wb_o  = '0;
    assign axi_o = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, axi_i, apb_i};

    iommu_tl #(
      .NUM_STREAMS          (NUM_STREAMS),
      .TLB_ENTRIES          (TLB_ENTRIES),
      .FAULT_FIFO_DEPTH     (FAULT_FIFO_DEPTH),
      .USE_AXI_ID_AS_STREAM (USE_AXI_ID_AS_STREAM)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .tl_i           (tl_i),
      .tl_o           (tl_o),
      .s_axi_dev_i    (s_axi_dev_i),
      .s_axi_dev_o    (s_axi_dev_o),
      .aw_stream_id_i (aw_stream_id_i),
      .ar_stream_id_i (ar_stream_id_i),
      .m_axi_mem_o    (m_axi_mem_o),
      .m_axi_mem_i    (m_axi_mem_i),
      .irq_o          (irq_o)
    );
  end else if (BUS_TYPE == BUS_APB) begin : gen_apb
    assign wb_o  = '0;
    assign axi_o = '0;
    assign tl_o  = '0;
    wire unused_bus = &{1'b0, wb_i, axi_i, tl_i};

    iommu_apb #(
      .NUM_STREAMS          (NUM_STREAMS),
      .TLB_ENTRIES          (TLB_ENTRIES),
      .FAULT_FIFO_DEPTH     (FAULT_FIFO_DEPTH),
      .USE_AXI_ID_AS_STREAM (USE_AXI_ID_AS_STREAM)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .apb_i          (apb_i),
      .apb_o          (apb_o),
      .s_axi_dev_i    (s_axi_dev_i),
      .s_axi_dev_o    (s_axi_dev_o),
      .aw_stream_id_i (aw_stream_id_i),
      .ar_stream_id_i (ar_stream_id_i),
      .m_axi_mem_o    (m_axi_mem_o),
      .m_axi_mem_i    (m_axi_mem_i),
      .irq_o          (irq_o)
    );
  end else begin : gen_axi
    assign wb_o  = '0;
    assign tl_o  = '0;
    assign apb_o = APB_S2M_IDLE;
    wire unused_bus = &{1'b0, wb_i, tl_i, apb_i};

    iommu_axi #(
      .NUM_STREAMS          (NUM_STREAMS),
      .TLB_ENTRIES          (TLB_ENTRIES),
      .FAULT_FIFO_DEPTH     (FAULT_FIFO_DEPTH),
      .USE_AXI_ID_AS_STREAM (USE_AXI_ID_AS_STREAM)
    ) u_imp (
      .clk_i          (clk_i),
      .rst_ni         (rst_ni),
      .axi_i          (axi_i),
      .axi_o          (axi_o),
      .s_axi_dev_i    (s_axi_dev_i),
      .s_axi_dev_o    (s_axi_dev_o),
      .aw_stream_id_i (aw_stream_id_i),
      .ar_stream_id_i (ar_stream_id_i),
      .m_axi_mem_o    (m_axi_mem_o),
      .m_axi_mem_i    (m_axi_mem_i),
      .irq_o          (irq_o)
    );
  end

endmodule : iommu_top
