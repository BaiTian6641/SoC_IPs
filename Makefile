# SPDX-License-Identifier: Apache-2.0
# Top-Level Build, Lint & Simulation Makefile (Verilator / QuestaSim / VCS / Xcelium)

VERILATOR ?= verilator
VLOG      ?= vlog
VSIM      ?= vsim

VERILATOR_FLAGS := --binary -j 0 -Wall -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-TIMESCALEMOD --timing
IPS_TB_DIRS := -y ips/17_mini_tpu/tb

ALL_TBS := \
  tb_interconnect_bridges \
  tb_crossbars_smart_switch \
  tb_soc_subsystem \
  tb_uart_usart \
  tb_spi \
  tb_i2c_i3c \
  tb_xspi_xip \
  tb_hyperram_ctrl \
  tb_ext_mem_ctrl \
  tb_nvpsram_ctrl \
  tb_gpio \
  tb_i2s \
  tb_ledc \
  tb_jtag \
  tb_can \
  tb_eth_mac \
  tb_dvi_tmds \
  tb_gdma \
  tb_gfx2d_dma \
  tb_mini_tpu

LINT_FLAGS := --lint-only -Wall -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY

LINT_TOPS := \
  wb_xbar \
  axi4_xbar \
  tl_xbar \
  smart_switch_interconnect \
  width_upsizer_downsizer \
  width_upsizer_32to64 \
  wb_reg_slice \
  axi4_reg_slice \
  tl_reg_slice \
  wb_to_axi4 \
  axi4_to_wb \
  wb_to_tl \
  tl_to_wb \
  axi4_to_tl \
  tl_to_axi4 \
  uart_usart_top \
  spi_top \
  i2c_i3c_top \
  xspi_xip_top \
  hyperram_ctrl_top \
  ext_mem_ctrl_top \
  nvpsram_ctrl_top \
  gpio_top \
  i2s_top \
  ledc_top \
  jtag_top \
  can_top \
  eth_mac_top \
  dvi_tmds_top \
  gdma_top \
  gfx2d_dma_top \
  mini_tpu_top

.PHONY: all lint sim clean $(ALL_TBS)

all: lint sim

lint:
	@for top in $(LINT_TOPS); do \
	  echo "=== Linting $$top ==="; \
	  $(VERILATOR) $(LINT_FLAGS) -f filelists/peripherals_all.f --top-module $$top || exit 1; \
	done

sim: $(ALL_TBS)

$(ALL_TBS):
	@echo "=== Running $@ ==="
	$(VERILATOR) $(VERILATOR_FLAGS) $(IPS_TB_DIRS) -f filelists/sim_all.f ips/17_mini_tpu/tb/tb_mini_tpu.sv --top-module $@ -o sim_$@
	./obj_dir/sim_$@

questa:
	vlib work
	$(VLOG) -sv $(IPS_TB_DIRS) +libext+.sv -f filelists/sim_all.f ips/17_mini_tpu/tb/tb_mini_tpu.sv
	for tb in $(ALL_TBS); do $(VSIM) -c -do "run -all; quit -f" work.$$tb; done

clean:
	rm -rf obj_dir work transcript *.vcd *.fst
