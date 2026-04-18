# ==============================================================================
# Makefile — Supply Chain Security 3PIP FPGA Simulation
# Author   : Kaustubh Pandey (230959054) | MIT Manipal ECE 4424
#
# Targets:
#   make            — Verilator compile + run full integration test
#   make unit       — Icarus unit tests (TMR + CRC, fast)
#   make icarus     — Icarus full top-level simulation
#   make lint       — Verilator lint only (no simulation)
#   make waves      — Open GTKWave on last VCD
#   make clean      — Remove all build artefacts
#   make help       — Print this message
# ==============================================================================

TOP        = security_top
TB_CPP     = tb/tb_verilator.cpp

RTL_FILES  = rtl/mac_core.sv         \
             rtl/mac_trojan.sv        \
             rtl/tmr_wrapper.sv       \
             rtl/crc16_monitor.sv     \
             rtl/ro_sensor_array.sv   \
             rtl/alarm_aggregator.sv  \
             rtl/pr_controller.sv     \
             rtl/security_top.sv

VER_DIR    = build_verilator
ICA_DIR    = build_iverilog
VCD_TOP    = security_top_verilator.vcd
VCD_ICA    = security_top_sim.vcd

# Verilator flags:
#   --sv            : enable SystemVerilog 2012
#   --cc            : generate C++ model
#   --exe           : build executable
#   --trace         : enable VCD tracing
#   --timing        : model timing constructs (needed for timescale)
#   -Wall           : enable all lint warnings
#   -Wno-fatal      : warnings don't abort (see UNOPTFLAT note in guide)
VER_FLAGS  = --sv --cc --exe --trace --timing  \
             -Wall -Wno-fatal                   \
             --top-module $(TOP)                \
             -CFLAGS "-O2 -std=c++17"

# ── Default target ────────────────────────────────────────────────
.PHONY: all
all: $(VER_DIR)/V$(TOP)
	@echo ""
	@echo ">>> Running Verilator simulation..."
	@echo ""
	./$(VER_DIR)/V$(TOP)
	@echo ""
	@echo ">>> Done. Open waveform with: make waves"

# ── Verilator: generate C++ model and compile ────────────────────
$(VER_DIR)/V$(TOP): $(RTL_FILES) $(TB_CPP)
	@mkdir -p $(VER_DIR)
	@echo ">>> Generating C++ model from SystemVerilog..."
	verilator $(VER_FLAGS)              \
	    -o V$(TOP)                      \
	    --Mdir $(VER_DIR)               \
	    $(RTL_FILES) $(TB_CPP)
	@echo ">>> Compiling C++ model..."
	$(MAKE) -C $(VER_DIR) -f V$(TOP).mk V$(TOP)

# ── Lint only — syntax + type check, no simulation ───────────────
.PHONY: lint
lint:
	@echo ">>> Running Verilator lint..."
	verilator --sv --lint-only -Wall $(RTL_FILES)
	@echo "Lint passed — no fatal errors."

# ── Icarus: unit tests ────────────────────────────────────────────
.PHONY: unit
unit: unit_tmr unit_crc
	@echo ""
	@echo ">>> All unit tests complete."

.PHONY: unit_tmr
unit_tmr:
	@mkdir -p $(ICA_DIR)
	@echo "--- Unit test: tmr_wrapper ---"
	iverilog -g2012 -o $(ICA_DIR)/sim_tmr.out \
	    rtl/mac_core.sv rtl/mac_trojan.sv rtl/tmr_wrapper.sv \
	    tb/tb_tmr_wrapper.sv
	vvp $(ICA_DIR)/sim_tmr.out

.PHONY: unit_crc
unit_crc:
	@mkdir -p $(ICA_DIR)
	@echo "--- Unit test: crc16_monitor ---"
	iverilog -g2012 -o $(ICA_DIR)/sim_crc.out \
	    rtl/crc16_monitor.sv \
	    tb/tb_crc16_monitor.sv
	vvp $(ICA_DIR)/sim_crc.out

# ── Icarus: full top-level simulation ────────────────────────────
.PHONY: icarus
icarus:
	@mkdir -p $(ICA_DIR)
	@echo ">>> Running Icarus Verilog full simulation..."
	iverilog -g2012 -o $(ICA_DIR)/sim_top.out \
	    $(RTL_FILES) \
	    tb/tb_security_top.sv
	vvp $(ICA_DIR)/sim_top.out
	@echo ">>> VCD written to: $(VCD_ICA)"

# ── Open waveform viewer ─────────────────────────────────────────
.PHONY: waves
waves:
	@if [ -f $(VCD_TOP) ]; then \
	    echo ">>> Opening $(VCD_TOP) in GTKWave..."; \
	    gtkwave $(VCD_TOP) & \
	elif [ -f $(VCD_ICA) ]; then \
	    echo ">>> Opening $(VCD_ICA) in GTKWave..."; \
	    gtkwave $(VCD_ICA) & \
	else \
	    echo "No VCD file found. Run 'make' or 'make icarus' first."; \
	fi

# ── Clean ────────────────────────────────────────────────────────
.PHONY: clean
clean:
	rm -rf $(VER_DIR) $(ICA_DIR) *.vcd *.wdb *.jou *.log

# ── Help ─────────────────────────────────────────────────────────
.PHONY: help
help:
	@echo ""
	@echo "  Supply Chain Security 3PIP FPGA — Simulation Makefile"
	@echo "  ──────────────────────────────────────────────────────"
	@echo "  make             Verilator compile + run full test"
	@echo "  make unit        Icarus unit tests (TMR + CRC)"
	@echo "  make icarus      Icarus full top-level simulation"
	@echo "  make lint        Verilator lint only (fast)"
	@echo "  make waves       Open GTKWave on last VCD"
	@echo "  make clean       Remove all build artefacts"
	@echo ""
