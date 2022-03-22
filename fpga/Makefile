#--------------------------------------------------------------------
# @brief: no-project synplify and implement wujian_100
# @author: macro
# @date: 2022-03-18
#
#--------------------------------------------------------------------
base_dir := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
INSTALL_RTL ?= $(base_dir)/rtl

# install files
install:
	mkdir -p ${PWD}/rtl
	cp ${PWD}/../soc/*.v ${INSTALL_RTL} -rf
	cp ${PWD}/../soc/sim_lib ${INSTALL_RTL}/sim_lib -rf
	cp ${PWD}/../soc/params ${INSTALL_RTL}/params -rf

.PHONY: install clean bit mcs

bit:
	vivado -mode tcl -source script/nonproject_mode_build.tcl

mcs:
	vivado -mode tcl -source script/buildfile.tcl

clean:
	rm -rf rtl *.log *.pb xsim.dir *.html *.xml *.jou output .Xil *.txt build

#--------------------------------------------------------------------
