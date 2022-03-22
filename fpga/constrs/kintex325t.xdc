#Copyright (c) 2019 Alibaba Group Holding Limited
#
#Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
#
#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#===========================================================
# Macro
# Xilinx Kinext 325t, system clock 50M
# Pin assignment constraint file
#===========================================================

#===========================================
# Create clock
#===========================================
create_clock -period 10.000 -name PIN_EHS [get_ports PIN_EHS]
create_clock -period 500.000 -name PAD_JTAG_TCLK [get_ports PAD_JTAG_TCLK]
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets PAD_JTAG_TCLK]
set_false_path -from [get_pins u_clk_wiz_0/inst/mmcm_adv_inst/CLKOUT0] -to [get_pins x_cpu_top/CPU/x_cr_tcipif_top/x_cr_coretim_top/refclk_ff1_reg/D]

#===========================================
# Global reset source
#===========================================
set_property PACKAGE_PIN G22   [get_ports PIN_EHS]
set_property PACKAGE_PIN D26   [get_ports PAD_MCURST];
set_property PACKAGE_PIN M22   [get_ports SYS_CLK_TEST];

#===========================================
# C-SKY  JTAG interface: J8
#===========================================
set_property PACKAGE_PIN H14   [get_ports PAD_JTAG_TCLK]
set_property PACKAGE_PIN H12   [get_ports PAD_JTAG_TMS]

#===========================================
# YOC SOCKET 1
#===========================================
set_property PACKAGE_PIN H11   [get_ports PAD_GPIO_0]
set_property PACKAGE_PIN F14   [get_ports PAD_GPIO_1]
set_property PACKAGE_PIN G14   [get_ports PAD_GPIO_2]
set_property PACKAGE_PIN F13   [get_ports PAD_GPIO_3]
set_property PACKAGE_PIN G12   [get_ports PAD_GPIO_4]
set_property PACKAGE_PIN F12   [get_ports PAD_GPIO_5]
set_property PACKAGE_PIN G10   [get_ports PAD_GPIO_6]
set_property PACKAGE_PIN F10   [get_ports PAD_GPIO_7]
set_property PACKAGE_PIN F8    [get_ports PAD_GPIO_8]
set_property PACKAGE_PIN A8    [get_ports PAD_GPIO_9]
set_property PACKAGE_PIN F9    [get_ports PAD_GPIO_10]
set_property PACKAGE_PIN B9    [get_ports PAD_GPIO_11]
set_property PACKAGE_PIN D8    [get_ports PAD_GPIO_12]
set_property PACKAGE_PIN A9    [get_ports PAD_GPIO_13]
set_property PACKAGE_PIN E13   [get_ports PAD_GPIO_14]
set_property PACKAGE_PIN C9    [get_ports PAD_GPIO_15]
set_property PACKAGE_PIN E10   [get_ports PAD_GPIO_16]
set_property PACKAGE_PIN B11   [get_ports PAD_GPIO_17]
set_property PACKAGE_PIN E11   [get_ports PAD_GPIO_18]

#===========================================
# YOC SOCKET 2
#===========================================
set_property PACKAGE_PIN C11   [get_ports PAD_GPIO_19]
set_property PACKAGE_PIN D10   [get_ports PAD_GPIO_20]
set_property PACKAGE_PIN C13   [get_ports PAD_GPIO_21]
set_property PACKAGE_PIN D9    [get_ports PAD_GPIO_22]
set_property PACKAGE_PIN A14   [get_ports PAD_GPIO_23]
set_property PACKAGE_PIN D11   [get_ports PAD_GPIO_24]
set_property PACKAGE_PIN D14   [get_ports PAD_GPIO_25]
set_property PACKAGE_PIN E12   [get_ports PAD_GPIO_26]
set_property PACKAGE_PIN B12   [get_ports PAD_GPIO_27]
set_property PACKAGE_PIN C12   [get_ports PAD_GPIO_28]
set_property PACKAGE_PIN C14   [get_ports PAD_GPIO_29]
set_property PACKAGE_PIN D13   [get_ports PAD_GPIO_30]
set_property PACKAGE_PIN B14   [get_ports PAD_GPIO_31]

#===========================================
# YOC SOCKET 3
#===========================================
set_property PACKAGE_PIN N21   [get_ports PAD_PWM_CH0]
set_property PACKAGE_PIN N22   [get_ports PAD_PWM_CH1]
set_property PACKAGE_PIN P23   [get_ports PAD_PWM_CH2]
set_property PACKAGE_PIN T24   [get_ports PAD_PWM_CH3]
set_property PACKAGE_PIN N23   [get_ports PAD_PWM_CH4]
set_property PACKAGE_PIN T25   [get_ports PAD_PWM_CH5]
set_property PACKAGE_PIN P24   [get_ports PAD_PWM_CH6]
set_property PACKAGE_PIN T22   [get_ports PAD_PWM_CH7]
set_property PACKAGE_PIN N24   [get_ports PAD_PWM_CH8]
set_property PACKAGE_PIN T23   [get_ports PAD_PWM_CH9]
set_property PACKAGE_PIN R22   [get_ports PAD_PWM_CH10]
set_property PACKAGE_PIN T20   [get_ports PAD_PWM_CH11]
set_property PACKAGE_PIN R23   [get_ports PAD_PWM_FAULT]

#===========================================
# YOC SOCKET 4
#===========================================
set_property PACKAGE_PIN R20   [get_ports PAD_USI0_NSS]
#set_property PACKAGE_PIN R21   [get_ports PAD_USI0_SCLK]
#set_property PACKAGE_PIN P19   [get_ports PAD_USI0_SD0]
# SCLK TX; SD0 RX
set_property PACKAGE_PIN A17   [get_ports PAD_USI0_SCLK]
set_property PACKAGE_PIN B17   [get_ports PAD_USI0_SD0]
set_property PACKAGE_PIN P21   [get_ports PAD_USI0_SD1]
set_property PACKAGE_PIN P20   [get_ports PAD_USI1_NSS]
set_property PACKAGE_PIN R25   [get_ports PAD_USI1_SCLK]
set_property PACKAGE_PIN N26   [get_ports PAD_USI1_SD0]
set_property PACKAGE_PIN P25   [get_ports PAD_USI1_SD1]
set_property PACKAGE_PIN M26   [get_ports PAD_USI2_NSS]
set_property PACKAGE_PIN R26   [get_ports PAD_USI2_SCLK]
set_property PACKAGE_PIN N19   [get_ports PAD_USI2_SD0]
set_property PACKAGE_PIN P26   [get_ports PAD_USI2_SD1]

#===========================================
# LED
#===========================================
set_property PACKAGE_PIN A23  [get_ports POUT_EHS]

#===========================================
# set io standard
#===========================================
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_0]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_1]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_2]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_3]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_4]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_5]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_6]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_7]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_8]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_9]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_10]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_11]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_12]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_13]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_14]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_15]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_16]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_17]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_18]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_19]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_20]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_21]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_22]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_23]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_24]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_25]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_26]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_27]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_28]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_29]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_30]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_GPIO_31]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH0]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH1]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH2]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH3]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH4]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH5]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH6]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH7]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH8]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH9]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH10]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_CH11]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_PWM_FAULT]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI0_NSS]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI0_SCLK]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI0_SD0]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI0_SD1]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI1_NSS]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI1_SCLK]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI1_SD0]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI1_SD1]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI2_NSS]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI2_SCLK]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI2_SD0]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_USI2_SD1]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_JTAG_TCLK]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_JTAG_TMS]
set_property IOSTANDARD LVCMOS33 [get_ports PAD_MCURST]
set_property IOSTANDARD LVCMOS33 [get_ports PIN_EHS]
set_property IOSTANDARD LVCMOS33 [get_ports POUT_EHS]
set_property IOSTANDARD LVCMOS33 [get_ports SYS_CLK_TEST]

#===========================================
# FPGA configuration properties
#===========================================
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 50 [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE Yes [current_design]
set_property BITSTREAM.GENERAL.COMPRESS true [current_design]
