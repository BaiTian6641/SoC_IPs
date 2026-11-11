# SPDX-License-Identifier: Apache-2.0
# Top-Level Build, Lint & Simulation Makefile (Verilator / QuestaSim / VCS / Xcelium)

VERILATOR ?= verilator
VLOG      ?= vlog
VSIM      ?= vsim

VERILATOR_FLAGS := --binary -j 0 -Wall -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-TIMESCALEMOD --timing

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
  tb_gfx2d_dma

.PHONY: all lint sim clean $(ALL_TBS)

all: lint sim

lint:
	$(VERILATOR) --lint-only -Wall -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -f filelists/peripherals_all.f --top-module smart_switch_interconnect

sim: $(ALL_TBS)

$(ALL_TBS):
	@echo "=== Running $@ ==="
	$(VERILATOR) $(VERILATOR_FLAGS) -f filelists/sim_all.f --top-module $@ -o sim_$@
	./obj_dir/sim_$@

questa:
	vlib work
	$(VLOG) -sv -f filelists/sim_all.f
	for tb in $(ALL_TBS); do $(VSIM) -c -do "run -all; quit -f" work.$$tb; done

clean:
	rm -rf obj_dir work transcript *.vcd *.fst
