### ------------------------------------------------------------------------------------------
##
## Brief: makefile file for vcs and verdi
## Author: Macro
## Email: makermuyi@gmail.com
## Run:
##
### ------------------------------------------------------------------------------------------
# Project output target
OUTPUT = macan
TOP_MODULE = tb_top

# Set the debug GUI (verdi or dve)
DEBUG_GUI = verdi

# module file list
MODULE_FLIES = module.list

# output log file
COM_LOG_FILE = complier_log
SIM_LOG_FILE = simlution_log
COV_LOG_FILE = coverage_log

# fsdb dump file for verdi
WAVEFORM = $(OUTPUT).fsdb

# Coverage command and flags
COV_FLAGS = -cm line+cond+fsm+tgl+branch+assert
COV_NAME = -cm_name $(OUTPUT).cov
COV_DIR = -cm_dir ./$(OUTPUT).vdb

# Dve and Verdi Coverage command
DVE_COV_CMD = dve -full64 -covdir $(OUTPUT).vdb &

# Compile command and flags
VCS_FLAGS = -f $(MODULE_FLIES) -sverilog \
            -full64                      \
            +lint=all                    \
            +warn=noVPI-CT-NS            \
            +v2k                         \
            -override_timescale=1ns/1ns  \
            -debug_access+all            \
            -debug_acc+dmptf             \
            -debug_region+cell+encrypt   \
            -cpp g++-4.8                 \
            -cc  gcc-4.8 			     \
            -LDFLAGS -Wl,--no-as-needed  \
            -P $(VERDI_HOME)/share/PLI/VCS/LINUX64/novas.tab                         \
            $(VERDI_HOME)/share/PLI/VCS/LINUX64/pli.a -load libnovas.so:FSDBDumpCmd  \
            $(COV_FLAGS)                 \
            $(COV_DIR)                   \
			+incdir+${PWD}/rtl           \
            -Mupdate                     \
            -o $(OUTPUT)                 \
            -l $(COM_LOG_FILE).log

# Simulation command
SIM_FLAGS = +fsdb+autoflush              \
            $(COV_FLAGS)                 \
            -cm_log $(COV_LOG_FILE).log  \
            -l $(SIM_LOG_FILE).log

# Debug command and flags
DEBUG_FLAGS = -f $(MODULE_FLIES)         \
		      +incdir+${PWD}/rtl         \
              -top $(TOP_MODULE)         \
              -ssf $(WAVEFORM)           \
              -nologo

# Lint Flags
LINT_FLAGS = --lint-only        \
             -Wall              \
             -f $(MODULE_FLIES)

# Formality Flags

# PeimrTime Flags

# Spyglass Flags

# Design Compiler Flags

# Synthesis Flags

### ------------------------------------------------------------------------------------------
# all run
all: com sim wave

# install file
VSIM_DIR = rtl
STAGE = if

install: clean install_$(STAGE) com_sim

install_if:
	mkdir -p ${PWD}/${VSIM_DIR}
	cp ${PWD}/../rtl/core/macan_fetch.v     ${VSIM_DIR}/macan_fetch.v
	cp ${PWD}/../rtl/core/macan_control_unit.v     ${VSIM_DIR}/macan_control_unit.v
	cp ${PWD}/../rtl/core/macan_decode.v    ${VSIM_DIR}/macan_decode.v
	cp ${PWD}/../rtl/core/register_file.v   ${VSIM_DIR}/register_file.v
	cp ${PWD}/../rtl/memory/mem_rom.v       ${VSIM_DIR}/mem_rom.v
	cp ${PWD}/../rtl/config/macan_defines.v ${VSIM_DIR}/macan_defines.v
	cp ${PWD}/../tb/stages_if_tb.v          ${VSIM_DIR}/testbench.v
	cp ${PWD}/../tb/list/stages_if.list     module.list
	cp ${PWD}/../tb/inst.hex                inst.hex

# only com and sim
com_sim: com sim

# lint the verilog design code
# verilator --lint-only -Wall [source_files.v]...
lint:
	verilator $(LINT_FLAGS)

# compiler verilog module file
com:
	vcs $(VCS_FLAGS)

# VCS -R FLAG will start simv sutomatic
sim:
	./$(OUTPUT) $(SIM_FLAGS)

# coverage the verilog module
cov:
	$(DVE_COV_CMD)

# debug the model via verdi
wave:
	verdi $(DEBUG_FLAGS) &

# show coverage by html
html:
	urg -dir $(OUTPUT).vdb -dbname merge &

# clean the compiler-time file
.PHONY: clean all com_sim lint com sim cov wave html install
clean:
	rm -rf csrc *simv *.daidir *.log *.key $(OUTPUT) *.vpd DVEfiles *.fsdb \
	verdiLog novas.* *.vdb urgReport vdCovLog output \
	build usage_statistics_webtalk.* *.jou *.log rtl module.list inst.hex

### ------------------------------------------------------------------------------------------
# display the help information
help:
	@echo
	@echo --------------------------------- Help  ---------------------------------
	@echo "command     : descript"
	@echo "make clean  : clean the temp file."
	@echo "make install STAGE= [ID EX MEM WB]: install file for stage x simulator."
	@echo "make lint   : lint the verilog format and pre-check."
	@echo "make all    : execute the compile simluation and open verdi debug."
	@echo "make com    : only execute compile the verilog module."
	@echo "make sim    : only execute simulation the model."
	@echo "make cov    : only execute coverage."
	@echo "make wave   : start the verdi and execute debug the waveform."
	@echo "make html   : view coverage report by html file."
	@echo "make help   : print the all command and use method."
	@echo --------------------------------- Help  ---------------------------------
### ------------------------------------------------------------------------------------------
