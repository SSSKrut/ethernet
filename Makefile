IVERILOG  ?= iverilog -g2012
VVP       ?= vvp
VERILATOR ?= verilator

SRC = crc32.v eth_mac_tx.v tb_eth_mac_tx.v

all: lint test

lint:
	$(VERILATOR) --lint-only -Wall --top-module eth_mac_tx crc32.v eth_mac_tx.v

# --- Icarus Verilog ---
test: $(SRC)
	$(IVERILOG) -o sim_eth $(SRC)
	$(VVP) sim_eth

# осциллограмма: make wave && gtkwave tb_eth_mac_tx.vcd
wave: $(SRC)
	$(IVERILOG) -o sim_wave $(SRC)
	$(VVP) sim_wave +dump

clean:
	rm -f sim_eth sim_wave *.vcd
	rm -rf obj_dir

.PHONY: all lint test wave clean
