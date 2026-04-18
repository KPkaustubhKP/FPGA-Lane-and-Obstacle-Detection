# ==============================================================================
# File        : security_top.xdc
# Project     : Supply Chain Security in 3PIP FPGA
# Author      : Kaustubh Pandey (230959054)
# Description : Vivado implementation constraints for security_top on
#               Artix-7 xc7a35tcpg236-1 (Basys3 / Arty A7-35).
#               Enforces physical isolation between TMR voter copies and
#               places RO sensors adjacent to the protected IP region.
# ==============================================================================

# ==============================================================================
# CLOCK CONSTRAINTS
# ==============================================================================

# 100 MHz system clock (Basys3 onboard oscillator)
create_clock -name sys_clk -period 10.0 [get_ports clk]

# False paths across resets (asynchronous deassert, synchronous assert)
set_false_path -from [get_cells {rst_n}]

# ==============================================================================
# I/O CONSTRAINTS
# ==============================================================================

set_property PACKAGE_PIN W5  [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]

set_property PACKAGE_PIN V17 [get_ports rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n]

set_property PACKAGE_PIN U16 [get_ports global_alarm]
set_property IOSTANDARD LVCMOS33 [get_ports global_alarm]

set_property PACKAGE_PIN E19 [get_ports cal_en]
set_property IOSTANDARD LVCMOS33 [get_ports cal_en]

# ==============================================================================
# PHYSICAL REGION: 3PIP PBLOCKS (TMR Instances)
# ==============================================================================
# Each MAC instance is confined to a separate Pblock to:
# (a) Prevent Vivado from merging structurally identical logic (optimisation)
# (b) Ensure the RO sensors are physically adjacent to distinct logic regions
# (c) Satisfy the "physical separation = independent fault model" assumption

# --- MAC Instance 0 (potentially Trojan-infected) ---
create_pblock pblock_mac0
add_cells_to_pblock [get_pblocks pblock_mac0] \
    [get_cells -hierarchical -filter { NAME =~ "*trojan_inst*" || NAME =~ "*clean_inst0*" }]
resize_pblock [get_pblocks pblock_mac0] -add {SLICE_X0Y0:SLICE_X15Y24}

# --- MAC Instance 1 (clean) ---
create_pblock pblock_mac1
add_cells_to_pblock [get_pblocks pblock_mac1] \
    [get_cells -hierarchical -filter { NAME =~ "*u_mac1*" }]
resize_pblock [get_pblocks pblock_mac1] -add {SLICE_X16Y0:SLICE_X31Y24}

# --- MAC Instance 2 (clean) ---
create_pblock pblock_mac2
add_cells_to_pblock [get_pblocks pblock_mac2] \
    [get_cells -hierarchical -filter { NAME =~ "*u_mac2*" }]
resize_pblock [get_pblocks pblock_mac2] -add {SLICE_X32Y0:SLICE_X47Y24}

# --- TMR Voter (adjacent to all three) ---
create_pblock pblock_tmr
add_cells_to_pblock [get_pblocks pblock_tmr] \
    [get_cells -hierarchical -filter { NAME =~ "*u_tmr*" }]
resize_pblock [get_pblocks pblock_tmr] -add {SLICE_X0Y25:SLICE_X47Y34}

# ==============================================================================
# DONT_TOUCH: Preserve security-critical cells from optimisation
# ==============================================================================
# Without DONT_TOUCH, Vivado may merge identical TMR copies or remove
# "redundant" logic, completely defeating the security architecture.

# TMR instances — must not be merged with each other
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*trojan_inst*" }]
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*clean_inst0*" }]
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*u_mac1*" }]
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*u_mac2*" }]

# TMR voter — must not be duplicated or pipelined by Vivado
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*u_tmr*" }]

# RO sensors — must not be absorbed into adjacent logic or clock-balanced
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*u_ro*" }]

# PR controller — prevent decomposition of state machine
set_property DONT_TOUCH TRUE [get_cells -hierarchical -filter { NAME =~ "*u_pr*" }]

# ==============================================================================
# PHYSICAL REGION: RO SENSOR ARRAY
# ==============================================================================
# Place 8 ROs in a grid around the MAC0 Pblock boundary where localized
# Vdd droop from a triggered Trojan is most detectable.

create_pblock pblock_ro_sensors
add_cells_to_pblock [get_pblocks pblock_ro_sensors] \
    [get_cells -hierarchical -filter { NAME =~ "*u_ro*ro_gen*" }]
resize_pblock [get_pblocks pblock_ro_sensors] -add {SLICE_X0Y26:SLICE_X15Y35}

# ==============================================================================
# PARTIAL RECONFIGURATION REGION
# ==============================================================================
# The PR reconfigurable partition covers the MAC0 Pblock. On Trojan detection,
# a blank/trusted bitstream is loaded over this region via ICAP.
# Requires Vivado DFX (Dynamic Function eXchange) licence.

# Mark mac0 region as reconfigurable partition
#   (Uncomment and assign to RP in DFX flow):
# set_property HD.RECONFIGURABLE TRUE [get_cells -hierarchical -filter { NAME =~ "*trojan_inst*" }]

# ==============================================================================
# TIMING EXCEPTIONS
# ==============================================================================

# RO sensors are asynchronous paths — exempt from setup/hold analysis
set_false_path -to [get_cells -hierarchical -filter { NAME =~ "*ro_sim_reg*" }]

# CRC monitor: data_in to crc_reg is registered; relax if needed for -1 grade
# set_multicycle_path -setup 2 -from [get_cells "*u_crc*crc_reg*"]

# ==============================================================================
# BITSTREAM PROPERTIES (for PR flow)
# ==============================================================================

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4  [current_design]
set_property CONFIG_MODE SPIx4                [current_design]
