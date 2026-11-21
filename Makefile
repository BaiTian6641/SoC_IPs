# SPDX-License-Identifier: Apache-2.0
# =============================================================================
# Top-level build / lint / simulation Makefile for the SystemVerilog IP collection.
#
# Primary flow : Verilator >= 5.0 (--binary --timing).  Best-effort hooks for
#                QuestaSim (`make questa`), VCS (`make vcs`), Xcelium (`make xrun`).
#
#   make help                 list every target
#   make lint                 lint every top, one verilator run per top
#   make lint_fast            same, parallel (JOBS=N)
#   make lint_<top>           lint a single top             (e.g. make lint_gpio_top)
#   make sim                  build + run every testbench
#   make sim_jobs             same, testbenches in parallel (JOBS=N)
#   make <tb_name>            build + run a single testbench (e.g. make tb_gpio)
#   make list                 print LINT_TOPS and ALL_TBS
#   make check_lists          consistency check of filelists / FuseSoC core vs. tables below
#   make core                 regenerate sv_ip_library.core (scripts/gen_fusesoc_core.py)
#   make clean
#
# Each testbench is compiled from ONLY the files it needs:
#     common.f + interconnect.f + ips/<NN>/filelist.f (for its TB_IPS) + BFMs + TB.
# All outputs live under $(BUILD) (default ./build), one directory per target, so
# parallel runs never collide.  Options:  SIMARGS="+seed=7"  VJ=<verilator -j>  V=1
# =============================================================================

VERILATOR ?= verilator
VLOG      ?= vlog
VSIM      ?= vsim
BUILD     ?= build
SIMARGS   ?=
JOBS      ?= $(shell sysctl -n hw.ncpu 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)
VJ        ?= 0

ifeq ($(V),1)
  Q :=
else
  Q := @
endif

# -----------------------------------------------------------------------------
# Tool flags (do NOT add new -Wno-* flags: the repo is warning-clean)
# -----------------------------------------------------------------------------
WARN_FLAGS      := -Wall -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY
VERILATOR_FLAGS := --binary --timing $(WARN_FLAGS) -Wno-TIMESCALEMOD
LINT_FLAGS      := --lint-only $(WARN_FLAGS)

LIB_F      := -f filelists/interconnect.f   # interconnect.f already pulls in common.f
SIM_INC    := +incdir+tb/common
SIM_BFM    := tb/common/tb_bfm_pkg.sv tb/common/tb_sim_ram.sv
ip_f        = $(foreach d,$(1),-f ips/$(d)/filelist.f)

# -----------------------------------------------------------------------------
# Registration helpers
#   $(eval $(call LINT,<top>,<ip dirs under ips/>))
#   $(eval $(call TB,<tb module>,<tb source>,<ip dirs>,<extra files>))
# -----------------------------------------------------------------------------
LINT_TOPS :=
ALL_TBS   :=

define LINT
LINT_TOPS += $(1)
LINT_IPS_$(1) := $(2)
endef

define TB
ALL_TBS += $(1)
TB_SRC_$(1)   := $(2)
TB_IPS_$(1)   := $(3)
TB_EXTRA_$(1) := $(4)
endef

# ---- IP directories ---------------------------------------------------------
IP_UART   := 01_uart_usart
IP_SPI    := 02_spi
IP_I2C    := 03_i2c_i3c
IP_XSPI   := 04_xspi_xip
IP_HYPER  := 05_hyperram
IP_EXTMEM := 06_sdram_sram_nvsram
IP_NVPSRAM:= 07_nvpsram
IP_GPIO   := 08_gpio
IP_I2S    := 09_i2s
IP_LEDC   := 10_ledc
IP_JTAG   := 11_jtag
IP_CAN    := 12_can
IP_ETH    := 13_eth_mac
IP_DVI    := 14_dvi_tmds
IP_GDMA   := 15_dma_gdma
IP_GFX    := 16_gfx2d_dma
IP_TPU    := 17_mini_tpu
IP_RMT    := 18_rmt
IP_IR     := 19_ir_txrx
IP_HUB75  := 20_hub75
IP_PARIF  := 21_par_if
IP_TIMERS := 24_timer_pwm
IP_FLEXBUS:= 22_flex_ext_bus
IP_PIO    := 23_pio_engine
IP_INTC   := 25_plic_clint
IP_WDTRTC := 26_wdt_rtc
IP_SDMMC  := 27_sdmmc
IP_CRYPTO := 28_crypto
IP_CORDIC   := 29_cordic
IP_FFT      := 30_fft
IP_DSP_FILT := 31_dsp_filter
ALL_IPS   := $(IP_UART) $(IP_SPI) $(IP_I2C) $(IP_XSPI) $(IP_HYPER) $(IP_EXTMEM) \
             $(IP_NVPSRAM) $(IP_GPIO) $(IP_I2S) $(IP_LEDC) $(IP_JTAG) $(IP_CAN) \
             $(IP_ETH) $(IP_DVI) $(IP_GDMA) $(IP_GFX) $(IP_TPU) $(IP_RMT) $(IP_IR) $(IP_HUB75) $(IP_PARIF) $(IP_FLEXBUS) $(IP_PIO) $(IP_TIMERS) $(IP_INTC) $(IP_WDTRTC) $(IP_SDMMC) $(IP_CRYPTO) $(IP_CORDIC) $(IP_FFT) $(IP_DSP_FILT)

# ---- Lint tops: interconnect / common (no IP needed) --------------------------
$(eval $(call LINT,wb_xbar,))
$(eval $(call LINT,axi4_xbar,))
$(eval $(call LINT,tl_xbar,))
$(eval $(call LINT,smart_switch_interconnect,))
$(eval $(call LINT,width_upsizer_downsizer,))
$(eval $(call LINT,width_upsizer_32to64,))
$(eval $(call LINT,wb_reg_slice,))
$(eval $(call LINT,axi4_reg_slice,))
$(eval $(call LINT,tl_reg_slice,))
$(eval $(call LINT,wb_to_axi4,))
$(eval $(call LINT,axi4_to_wb,))
$(eval $(call LINT,wb_to_tl,))
$(eval $(call LINT,tl_to_wb,))
$(eval $(call LINT,axi4_to_tl,))
$(eval $(call LINT,tl_to_axi4,))
$(eval $(call LINT,apb_xbar,))
$(eval $(call LINT,apb_reg_slice,))
$(eval $(call LINT,apb_decoder,))
$(eval $(call LINT,apb_mux,))
$(eval $(call LINT,apb_to_csr,))
$(eval $(call LINT,csr_to_apb,))
$(eval $(call LINT,axi4_to_apb,))
$(eval $(call LINT,wb_to_apb,))
$(eval $(call LINT,tl_to_apb,))
$(eval $(call LINT,apb_to_axi4,))
$(eval $(call LINT,apb_to_wb,))
$(eval $(call LINT,axis_register,))
$(eval $(call LINT,axis_fifo,))
$(eval $(call LINT,axis_async_fifo,))
$(eval $(call LINT,axis_width_converter,))
$(eval $(call LINT,axis_arb_mux,))
$(eval $(call LINT,axis_demux,))
$(eval $(call LINT,axi4_to_axis,))
$(eval $(call LINT,axis_to_axi4,))
$(eval $(call LINT,axi4_to_axil,))
$(eval $(call LINT,axil_to_axi4,))
$(eval $(call LINT,axi4_burst_splitter,))
$(eval $(call LINT,axi4_id_remap,))

# ---- Lint tops: IPs -----------------------------------------------------------
$(eval $(call LINT,uart_usart_top,$(IP_UART)))
$(eval $(call LINT,spi_top,$(IP_SPI)))
$(eval $(call LINT,i2c_i3c_top,$(IP_I2C)))
$(eval $(call LINT,xspi_xip_top,$(IP_XSPI)))
$(eval $(call LINT,hyperram_ctrl_top,$(IP_HYPER)))
$(eval $(call LINT,ext_mem_ctrl_top,$(IP_EXTMEM)))
$(eval $(call LINT,nvpsram_ctrl_top,$(IP_NVPSRAM)))
$(eval $(call LINT,gpio_top,$(IP_GPIO)))
$(eval $(call LINT,i2s_top,$(IP_I2S)))
$(eval $(call LINT,ledc_top,$(IP_LEDC)))
$(eval $(call LINT,jtag_top,$(IP_JTAG)))
$(eval $(call LINT,can_top,$(IP_CAN)))
$(eval $(call LINT,eth_mac_top,$(IP_ETH)))
$(eval $(call LINT,dvi_tmds_top,$(IP_DVI)))
$(eval $(call LINT,gdma_top,$(IP_GDMA)))
$(eval $(call LINT,gfx2d_dma_top,$(IP_GFX)))
$(eval $(call LINT,mini_tpu_top,$(IP_TPU)))
$(eval $(call LINT,rmt_top,$(IP_RMT)))
$(eval $(call LINT,ir_txrx_top,$(IP_IR)))
$(eval $(call LINT,hub75_top,$(IP_HUB75)))
$(eval $(call LINT,par_if_top,$(IP_PARIF)))
$(eval $(call LINT,timer_pwm_top,$(IP_TIMERS)))
$(eval $(call LINT,timer_group,$(IP_TIMERS)))
$(eval $(call LINT,flex_ext_bus_top,$(IP_FLEXBUS)))
$(eval $(call LINT,pio_top,$(IP_PIO)))
$(eval $(call LINT,plic_clint_top,$(IP_INTC)))
$(eval $(call LINT,wdt_rtc_top,$(IP_WDTRTC)))
$(eval $(call LINT,sdmmc_top,$(IP_SDMMC)))
$(eval $(call LINT,crypto_top,$(IP_CRYPTO)))
$(eval $(call LINT,irq_aggregator,$(IP_INTC)))
$(eval $(call LINT,cordic_top,$(IP_CORDIC)))
$(eval $(call LINT,fft_top,$(IP_FFT)))
$(eval $(call LINT,dsp_filter_top,$(IP_DSP_FILT)))

# ---- Testbenches ----------------------------------------------------------------
$(eval $(call TB,tb_interconnect_bridges,tb/tb_interconnect_bridges.sv,,))
$(eval $(call TB,tb_crossbars_smart_switch,tb/tb_crossbars_smart_switch.sv,,))
$(eval $(call TB,tb_axis_utils,tb/tb_axis_utils.sv,,))
$(eval $(call TB,tb_audit_regress,tb/tb_audit_regress.sv,,))
$(eval $(call TB,tb_soc_subsystem,tb/tb_soc_subsystem.sv,$(IP_GPIO) $(IP_GDMA) $(IP_GFX) $(IP_TPU),))
$(eval $(call TB,tb_uart_usart,ips/01_uart_usart/tb/tb_uart_usart.sv,$(IP_UART),))
$(eval $(call TB,tb_spi,ips/02_spi/tb/tb_spi.sv,$(IP_SPI),))
$(eval $(call TB,tb_i2c_i3c,ips/03_i2c_i3c/tb/tb_i2c_i3c.sv,$(IP_I2C),))
$(eval $(call TB,tb_xspi_xip,ips/04_xspi_xip/tb/tb_xspi_xip.sv,$(IP_XSPI),))
$(eval $(call TB,tb_hyperram_ctrl,ips/05_hyperram/tb/tb_hyperram_ctrl.sv,$(IP_HYPER),))
$(eval $(call TB,tb_ext_mem_ctrl,ips/06_sdram_sram_nvsram/tb/tb_ext_mem_ctrl.sv,$(IP_EXTMEM),))
$(eval $(call TB,tb_nvpsram_ctrl,ips/07_nvpsram/tb/tb_nvpsram_ctrl.sv,$(IP_NVPSRAM),))
$(eval $(call TB,tb_gpio,ips/08_gpio/tb/tb_gpio.sv,$(IP_GPIO),))
$(eval $(call TB,tb_i2s,ips/09_i2s/tb/tb_i2s.sv,$(IP_I2S),))
$(eval $(call TB,tb_ledc,ips/10_ledc/tb/tb_ledc.sv,$(IP_LEDC),))
$(eval $(call TB,tb_jtag,ips/11_jtag/tb/tb_jtag.sv,$(IP_JTAG),))
$(eval $(call TB,tb_can,ips/12_can/tb/tb_can.sv,$(IP_CAN),))
$(eval $(call TB,tb_eth_mac,ips/13_eth_mac/tb/tb_eth_mac.sv,$(IP_ETH),))
$(eval $(call TB,tb_dvi_tmds,ips/14_dvi_tmds/tb/tb_dvi_tmds.sv,$(IP_DVI),))
$(eval $(call TB,tb_gdma,ips/15_dma_gdma/tb/tb_gdma.sv,$(IP_GDMA),))
$(eval $(call TB,tb_gdma_rand,ips/15_dma_gdma/tb/tb_gdma_rand.sv,$(IP_GDMA),))
$(eval $(call TB,tb_gfx2d_dma,ips/16_gfx2d_dma/tb/tb_gfx2d_dma.sv,$(IP_GFX),))
$(eval $(call TB,tb_gfx2d_rand,ips/16_gfx2d_dma/tb/tb_gfx2d_rand.sv,$(IP_GFX),))
$(eval $(call TB,tb_mini_tpu,ips/17_mini_tpu/tb/tb_mini_tpu.sv,$(IP_TPU),))
$(eval $(call TB,tb_rmt,ips/18_rmt/tb/tb_rmt.sv,$(IP_RMT),))
$(eval $(call TB,tb_ir_txrx,ips/19_ir_txrx/tb/tb_ir_txrx.sv,$(IP_IR),))
$(eval $(call TB,tb_hub75,ips/20_hub75/tb/tb_hub75.sv,$(IP_HUB75),ips/20_hub75/tb/tb_hub75_panel.sv))
$(eval $(call TB,tb_par_if,ips/21_par_if/tb/tb_par_if.sv,$(IP_PARIF),))
$(eval $(call TB,tb_timer_pwm_wave,ips/24_timer_pwm/tb/tb_timer_pwm_wave.sv,$(IP_TIMERS),))
$(eval $(call TB,tb_timer_pwm_feat,ips/24_timer_pwm/tb/tb_timer_pwm_feat.sv,$(IP_TIMERS),))
$(eval $(call TB,tb_flex_ext_bus,ips/22_flex_ext_bus/tb/tb_flex_ext_bus.sv,$(IP_FLEXBUS),))
$(eval $(call TB,tb_pio_engine,ips/23_pio_engine/tb/tb_pio_engine.sv,$(IP_PIO),ips/23_pio_engine/tb/pio_tb_pkg.sv))
$(eval $(call TB,tb_pio_examples,ips/23_pio_engine/tb/tb_pio_examples.sv,$(IP_PIO),ips/23_pio_engine/tb/pio_tb_pkg.sv))
$(eval $(call TB,tb_plic_clint,ips/25_plic_clint/tb/tb_plic_clint.sv,$(IP_INTC),))
$(eval $(call TB,tb_wdt_rtc,ips/26_wdt_rtc/tb/tb_wdt_rtc.sv,$(IP_WDTRTC),))
$(eval $(call TB,tb_sdmmc,ips/27_sdmmc/tb/tb_sdmmc.sv,$(IP_SDMMC),ips/27_sdmmc/tb/tb_sd_card.sv))
$(eval $(call TB,tb_sd_card_selftest,ips/27_sdmmc/tb/tb_sd_card_selftest.sv,,ips/27_sdmmc/tb/tb_sd_card.sv))
$(eval $(call TB,tb_crypto,ips/28_crypto/tb/tb_crypto.sv,$(IP_CRYPTO),))
$(eval $(call TB,tb_cordic,ips/29_cordic/tb/tb_cordic.sv,$(IP_CORDIC),))
$(eval $(call TB,tb_fft,ips/30_fft/tb/tb_fft.sv,$(IP_FFT),))
$(eval $(call TB,tb_dsp_filter,ips/31_dsp_filter/tb/tb_dsp_filter.sv,$(IP_DSP_FILT),))

# ---- APB fabric / retrofit testbenches (owner: apb) ----------------------------
$(eval $(call TB,tb_apb_fabric,tb/tb_apb_fabric.sv,,))
$(eval $(call TB,tb_apb_peripherals,tb/tb_apb_peripherals.sv,$(IP_UART) $(IP_SPI) $(IP_XSPI) $(IP_HYPER) $(IP_EXTMEM) $(IP_NVPSRAM) $(IP_GPIO) $(IP_LEDC) $(IP_TPU),))

# ---- UVM-style verification kit regressions (owner: vkit) ---------------------
VKIT_F := -f tb/vkit/filelist.f
$(eval $(call TB,tb_vkit_csr_selftest,tb/tb_vkit_csr_selftest.sv,,$(VKIT_F)))
$(eval $(call TB,tb_vkit_axi_xbar_rand,tb/tb_vkit_axi_xbar_rand.sv,,$(VKIT_F)))
$(eval $(call TB,tb_vkit_apb_xbar_rand,tb/tb_vkit_apb_xbar_rand.sv,,$(VKIT_F)))
$(eval $(call TB,tb_vkit_wb_xbar_rand,tb/tb_vkit_wb_xbar_rand.sv,,$(VKIT_F)))
$(eval $(call TB,tb_vkit_tl_xbar_rand,tb/tb_vkit_tl_xbar_rand.sv,,$(VKIT_F)))
$(eval $(call TB,tb_vkit_smart_switch_rand,tb/tb_vkit_smart_switch_rand.sv,,$(VKIT_F)))

define newline


endef

# =============================================================================
# Targets
# =============================================================================
LINT_TARGETS := $(addprefix lint_,$(LINT_TOPS))

.PHONY: all help list tables core lint lint_fast sim sim_jobs summary check_lists clean \
        questa vcs xrun $(LINT_TARGETS) $(ALL_TBS)

all: lint sim

help:
	@sed -n '3,22p' Makefile | sed 's/^# \{0,1\}//'

list:
	@echo "LINT_TOPS ($(words $(LINT_TOPS))):"; for t in $(LINT_TOPS); do echo "  $$t"; done
	@echo "ALL_TBS ($(words $(ALL_TBS))):"; for t in $(ALL_TBS); do echo "  $$t"; done

# machine-readable dump used by scripts/gen_fusesoc_core.py
tables:
	@echo "IPS|$(ALL_IPS)"
	$(foreach t,$(LINT_TOPS),@echo "LINT|$(t)|$(LINT_IPS_$(t))"$(newline))
	$(foreach t,$(ALL_TBS),@echo "TB|$(t)|$(TB_SRC_$(t))|$(TB_IPS_$(t))|$(TB_EXTRA_$(t))"$(newline))

# ---- Lint -------------------------------------------------------------------
lint: $(LINT_TARGETS)
	@echo "[LINT OK] $(words $(LINT_TOPS)) tops clean"

lint_fast:
	$(MAKE) -j$(JOBS) lint

$(LINT_TARGETS): lint_%:
	@mkdir -p $(BUILD)/lint_$*
	$(Q)echo "=== Linting $* ===" ; \
	$(VERILATOR) $(LINT_FLAGS) -Mdir $(BUILD)/lint_$* $(LIB_F) $(call ip_f,$(LINT_IPS_$*)) \
	  --top-module $* > $(BUILD)/lint_$*/lint.log 2>&1 \
	  || { cat $(BUILD)/lint_$*/lint.log; echo "[LINT FAIL] $*"; exit 1; }

# ---- Simulation ---------------------------------------------------------------
sim: $(ALL_TBS)
	@$(MAKE) --no-print-directory summary

sim_jobs:
	$(MAKE) -j$(JOBS) VJ=1 sim

summary:
	@pass=0; fail=0; for t in $(ALL_TBS); do \
	  if [ -f $(BUILD)/$$t/PASS ]; then pass=$$((pass+1)); else fail=$$((fail+1)); echo "[NOT PASSED] $$t"; fi; \
	done; echo "[SUMMARY] $$pass/$(words $(ALL_TBS)) testbenches passed"; [ $$fail -eq 0 ]

$(ALL_TBS):
	@mkdir -p $(BUILD)/$@
	@rm -f $(BUILD)/$@/PASS
	$(Q)echo "=== Building $@ ===" ; \
	$(VERILATOR) $(VERILATOR_FLAGS) -j $(VJ) -Mdir $(BUILD)/$@ $(SIM_INC) $(LIB_F) \
	  $(call ip_f,$(TB_IPS_$@)) $(SIM_BFM) $(TB_EXTRA_$@) $(TB_SRC_$@) \
	  --top-module $@ -o sim_$@ > $(BUILD)/$@/build.log 2>&1 \
	  || { cat $(BUILD)/$@/build.log; echo "[BUILD FAIL] $@"; exit 1; }
	$(Q)echo "=== Running $@ ===" ; \
	./$(BUILD)/$@/sim_$@ $(SIMARGS) > $(BUILD)/$@/run.log 2>&1 ; rc=$$? ; \
	grep -E '\[(PASS|FAIL)\]|%Error|%Warning' $(BUILD)/$@/run.log | tail -n 3 ; \
	if [ $$rc -eq 0 ] && grep -q '\[PASS\]' $(BUILD)/$@/run.log && ! grep -q '\[FAIL\]' $(BUILD)/$@/run.log ; then \
	  touch $(BUILD)/$@/PASS ; \
	else echo "[SIM FAIL] $@ (rc=$$rc, full log: $(BUILD)/$@/run.log)" ; exit 1 ; fi

# ---- Consistency check of hand-maintained lists --------------------------------
check_lists:
	@rc=0; \
	for d in $(ALL_IPS); do \
	  [ -f ips/$$d/filelist.f ] || { echo "missing ips/$$d/filelist.f"; rc=1; }; \
	  grep -q "ips/$$d/filelist.f" filelists/peripherals_all.f || { echo "ips/$$d/filelist.f not in peripherals_all.f"; rc=1; }; \
	done; \
	for f in $(foreach t,$(ALL_TBS),$(TB_SRC_$(t))); do \
	  [ -f $$f ] || { echo "missing TB source $$f"; rc=1; }; \
	  grep -q "$$f" filelists/sim_all.f || { echo "$$f not in filelists/sim_all.f"; rc=1; }; \
	done; \
	for d in ips/*/; do n=$$(basename $$d); \
	  case " $(ALL_IPS) " in *" $$n "*) ;; *) echo "ips/$$n is not registered in Makefile ALL_IPS"; rc=1;; esac; \
	done; \
	python3 scripts/gen_fusesoc_core.py --check || { echo "sv_ip_library.core is stale: run 'make core'"; rc=1; }; \
	if [ $$rc -eq 0 ]; then echo "[CHECK OK] filelists + FuseSoC core consistent"; fi; exit $$rc

# regenerate the FuseSoC core from filelists + the tables above
core:
	python3 scripts/gen_fusesoc_core.py

# ---- Other simulators (best effort, not runnable on the reference machine) -----
# All TB sources are compiled together; tops are run one by one.
questa:
	vlib work
	$(VLOG) -sv +libext+.sv -f filelists/sim_all.f
	for tb in $(ALL_TBS); do $(VSIM) -c -do "run -all; quit -f" work.$$tb; done

# Synopsys VCS (untested): one elaboration per TB
vcs:
	for tb in $(ALL_TBS); do \
	  vcs -full64 -sverilog -timescale=1ns/1ps -f filelists/sim_all.f -top $$tb -o $(BUILD)/simv_$$tb && $(BUILD)/simv_$$tb || exit 1; \
	done

# Cadence Xcelium (untested)
xrun:
	for tb in $(ALL_TBS); do \
	  xrun -64bit -sv -timescale 1ns/1ps -f filelists/sim_all.f -top $$tb -access +r || exit 1; \
	done

clean:
	rm -rf $(BUILD) obj_dir work transcript *.vcd *.fst
