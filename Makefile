# =============================================================================
#  Makefile — Verilator Lane Detection Verification
#  Usage:
#    make               — compile only
#    make run IMG=road.png
#    make run IMG=road.png THRESH=48 VCD=1
#    make clean
# =============================================================================

# ---------------------------------------------------------------------------
#  Tool paths (override if needed, e.g. VERILATOR=/opt/verilator/bin/verilator)
# ---------------------------------------------------------------------------
VERILATOR ?= verilator
CXX       ?= g++

# ---------------------------------------------------------------------------
#  DUT source files (relative path — copy all .sv here or adjust SV_DIR)
# ---------------------------------------------------------------------------
SV_DIR    ?= ../sv        # directory containing all .sv files
TOP       := lane_detection_top

SV_SRCS   := $(SV_DIR)/rgb_to_gray.sv          \
             $(SV_DIR)/gaussian_blur.sv         \
             $(SV_DIR)/sobel_edge_detector.sv   \
             $(SV_DIR)/lane_roi_filter.sv        \
             $(SV_DIR)/hough_transform.sv        \
             $(SV_DIR)/$(TOP).sv

# ---------------------------------------------------------------------------
#  DUT parameters  (must match what you synthesise)
# ---------------------------------------------------------------------------
IMG_WIDTH       ?= 640
IMG_HEIGHT      ?= 480
EDGE_THRESH     ?= 64
ROI_ROW_TOP     ?= 288
ROI_ROW_BOT     ?= 479
THETA_BINS      ?= 180
RHO_BINS        ?= 1024
HT_THRESHOLD    ?= 50

# Verilator parameter overrides
VPARAMS := -GIMG_WIDTH=$(IMG_WIDTH)       \
           -GIMG_HEIGHT=$(IMG_HEIGHT)      \
           -GEDGE_THRESH=$(EDGE_THRESH)    \
           -GROI_ROW_TOP=$(ROI_ROW_TOP)   \
           -GROI_ROW_BOT=$(ROI_ROW_BOT)   \
           -GTHETA_BINS=$(THETA_BINS)      \
           -GRHO_BINS=$(RHO_BINS)          \
           -GHT_THRESHOLD=$(HT_THRESHOLD)

# C++ compile-time definitions (so sim_main.cpp knows param values at build)
CDEFS   := -DIMG_WIDTH=$(IMG_WIDTH)        \
           -DIMG_HEIGHT=$(IMG_HEIGHT)       \
           -DRHO_BINS_VAL=$(RHO_BINS)       \
           -DHT_THRESHOLD_VAL=$(HT_THRESHOLD)

# ---------------------------------------------------------------------------
#  Verilator flags
# ---------------------------------------------------------------------------
VFLAGS  := --cc                     \
           --exe                     \
           --build                   \
           --trace                   \
           -Wall                     \
           --assert                  \
           --timing                  \
           --top-module $(TOP)       \
           $(VPARAMS)                \
           --Mdir obj_dir

# VCD size guard: disable tracing by default unless VCD=1 is passed
VCD ?= 0

# ---------------------------------------------------------------------------
#  STB headers (single-file image library, header-only, no deps)
# ---------------------------------------------------------------------------
STB_URL := https://raw.githubusercontent.com/nothings/stb/master

CPP_DIR := cpp
OBJ_DIR := obj_dir

.PHONY: all run clean stb help

# Default target
all: stb obj_dir/V$(TOP)
	@echo ""
	@echo "Build complete. Run with:"
	@echo "  make run IMG=<path/to/image.png>"

# ---------------------------------------------------------------------------
#  Download STB image headers (only if missing)
# ---------------------------------------------------------------------------
stb: $(CPP_DIR)/stb_image.h $(CPP_DIR)/stb_image_write.h

$(CPP_DIR)/stb_image.h:
	@echo "Downloading stb_image.h..."
	@mkdir -p $(CPP_DIR)
	curl -fsSL $(STB_URL)/stb_image.h -o $@

$(CPP_DIR)/stb_image_write.h:
	@echo "Downloading stb_image_write.h..."
	@mkdir -p $(CPP_DIR)
	curl -fsSL $(STB_URL)/stb_image_write.h -o $@

# ---------------------------------------------------------------------------
#  Verilate + compile
# ---------------------------------------------------------------------------
obj_dir/V$(TOP): stb $(SV_SRCS) $(CPP_DIR)/sim_main.cpp
	@echo "Verilating $(TOP)..."
	$(VERILATOR) $(VFLAGS)                     \
	    $(SV_SRCS)                              \
	    $(CPP_DIR)/sim_main.cpp                 \
	    -CFLAGS "$(CDEFS) -I$(CPP_DIR) -O2 -std=c++17" \
	    -o V$(TOP)
	@echo "Build OK → obj_dir/V$(TOP)"

# ---------------------------------------------------------------------------
#  Run simulation with a real image
# ---------------------------------------------------------------------------
run: all
ifndef IMG
	$(error IMG is required. Usage: make run IMG=path/to/image.png [THRESH=64] [VCD=0])
endif
	./obj_dir/V$(TOP) "$(IMG)" $(EDGE_THRESH) $(VCD)

# ---------------------------------------------------------------------------
#  Run with a generated test pattern (no real image needed — for smoke test)
# ---------------------------------------------------------------------------
smoke: all
	@python3 scripts/gen_test_pattern.py $(IMG_WIDTH) $(IMG_HEIGHT) /tmp/test_pattern.png
	./obj_dir/V$(TOP) /tmp/test_pattern.png $(EDGE_THRESH) 0
	@echo "Smoke test complete. Check /tmp/test_pattern.png_edges.png"

# ---------------------------------------------------------------------------
#  Lint only (no C++ compilation)
# ---------------------------------------------------------------------------
lint:
	$(VERILATOR) --lint-only -Wall --timing $(VPARAMS) --top-module $(TOP) $(SV_SRCS)

# ---------------------------------------------------------------------------
#  Clean
# ---------------------------------------------------------------------------
clean:
	rm -rf obj_dir lane_detection.vcd lane_results.csv
	@echo "Cleaned."

help:
	@echo "Targets:"
	@echo "  make              — build only"
	@echo "  make run IMG=x.png              — run with image"
	@echo "  make run IMG=x.png THRESH=48    — custom Sobel threshold"
	@echo "  make run IMG=x.png VCD=1        — enable waveform dump"
	@echo "  make smoke                      — test with synthetic pattern"
	@echo "  make lint                       — lint SV files only"
	@echo "  make clean                      — remove build artefacts"
	@echo ""
	@echo "Parameter overrides (default shown):"
	@echo "  IMG_WIDTH=640 IMG_HEIGHT=480 EDGE_THRESH=64"
	@echo "  ROI_ROW_TOP=288 ROI_ROW_BOT=479"
	@echo "  THETA_BINS=180 RHO_BINS=1024 HT_THRESHOLD=50"
