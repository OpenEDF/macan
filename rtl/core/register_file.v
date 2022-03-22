//--------------------------------------------------------------------------
//                         RISC-V Core
//                            V1.0.1
//                         openedf.com
//                     Copyright 2020-2021
//
//                     makermuyi@gmail.com
//
//                       License: BSD
//--------------------------------------------------------------------------
//
// Copyright (c) 2020-2021, openedf.com
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
// THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// SUCH DAMAGE.

//--------------------------------------------------------------------------
// Designer: Macro
// Brief: RISC-V register file
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module register_file
//--------------------------------------------------------------------------
// Ports
//--------------------------------------------------------------------------
(
    // Inputs
    input clk,
    input rst_n,
    input [5:0]  read_scr1_idx,
    input [5:0]  read_scr2_idx,
    input [5:0]  wbck_dest_idx, // index for register
    input [31:0] wbck_dest_dat, // write data

    // Outputs
    output [31:0] read_src1_dat,
    output [31:0] read_src2_dat
);

// risc-v has 32 registers
reg [31:0] reg_x0_q;
reg [31:0] reg_x1_q;
reg [31:0] reg_x2_q;
reg [31:0] reg_x3_q;
reg [31:0] reg_x4_q;
reg [31:0] reg_x5_q;
reg [31:0] reg_x6_q;
reg [31:0] reg_x7_q;
reg [31:0] reg_x8_q;
reg [31:0] reg_x9_q;
reg [31:0] reg_x10_q;
reg [31:0] reg_x11_q;
reg [31:0] reg_x12_q;
reg [31:0] reg_x13_q;
reg [31:0] reg_x14_q;
reg [31:0] reg_x15_q;
reg [31:0] reg_x16_q;
reg [31:0] reg_x17_q;
reg [31:0] reg_x18_q;
reg [31:0] reg_x19_q;
reg [31:0] reg_x20_q;
reg [31:0] reg_x21_q;
reg [31:0] reg_x22_q;
reg [31:0] reg_x23_q;
reg [31:0] reg_x24_q;
reg [31:0] reg_x25_q;
reg [31:0] reg_x26_q;
reg [31:0] reg_x27_q;
reg [31:0] reg_x28_q;
reg [31:0] reg_x29_q;
reg [31:0] reg_x30_q;
reg [31:0] reg_x31_q;

`ifdef SIMULATION_DEBUG
    wire [31:0] x0_zero_w = 32'h0000_0000;   // hardwires zero
    wire [31:0] x1_ra_w =  reg_x1_q;         // return address
    wire [31:0] x2_sp_w =  reg_x2_q;         // stack pointer
    wire [31:0] x3_gp_w =  reg_x3_q;         // global pointer
    wire [31:0] x4_tp_w =  reg_x4_q;         // thread pointer
    wire [31:0] x5_t0_w =  reg_x5_q;         // x5 - x7 temporary registers
    wire [31:0] x6_t1_w =  reg_x6_q;
    wire [31:0] x7_t2_w =  reg_x7_q;
    wire [31:0] x8_s0_w =  reg_x8_q;         // saved register / frame poinyer
    wire [31:0] x9_s1_w =  reg_x9_q;         // saved register
    wire [31:0] x10_a0_w =  reg_x10_q;       // a0 - 1 function arguments / return values
    wire [31:0] x11_a1_w =  reg_x11_q;
    wire [31:0] x12_a2_w =  reg_x12_q;       // a2 - 7 function arguments
    wire [31:0] x13_a3_w =  reg_x13_q;
    wire [31:0] x14_a4_w =  reg_x14_q;
    wire [31:0] x15_a5_w =  reg_x15_q;
    wire [31:0] x16_a6_w =  reg_x16_q;
    wire [31:0] x17_a7_w =  reg_x17_q;
    wire [31:0] x18_s2_w =  reg_x18_q;       // s2 - 11 saved registers
    wire [31:0] x19_s3_w =  reg_x19_q;
    wire [31:0] x20_s5_w =  reg_x20_q;
    wire [31:0] x21_s6_w =  reg_x21_q;
    wire [31:0] x22_s7_w =  reg_x22_q;
    wire [31:0] x23_s8_w =  reg_x23_q;
    wire [31:0] x24_s9_w =  reg_x24_q;
    wire [31:0] x25_s9_w =  reg_x25_q;
    wire [31:0] x26_s10_w =  reg_x26_q;
    wire [31:0] x27_s11_w =  reg_x27_q;
    wire [31:0] x28_t3_w =  reg_x28_q;       // t3 -6 temporary registers
    wire [31:0] x29_t4_w =  reg_x29_q;
    wire [31:0] x30_t5_w =  reg_x30_q;
    wire [31:0] x31_t6_w =  reg_x31_q;
`endif

// register wirte
always @(posedge clk or posedge rst_n) begin
    if (!rst_n) begin
        reg_x0_q  <= 32'h0000_0000;
        reg_x1_q  <= 32'h0000_0000;
        reg_x2_q  <= 32'h0000_0000;
        reg_x3_q  <= 32'h0000_0000;
        reg_x4_q  <= 32'h0000_0000;
        reg_x5_q  <= 32'h0000_0000;
        reg_x6_q  <= 32'h0000_0000;
        reg_x7_q  <= 32'h0000_0000;
        reg_x8_q  <= 32'h0000_0000;
        reg_x9_q  <= 32'h0000_0000;
        reg_x10_q <= 32'h0000_0000;
        reg_x11_q <= 32'h0000_0000;
        reg_x12_q <= 32'h0000_0000;
        reg_x13_q <= 32'h0000_0000;
        reg_x14_q <= 32'h0000_0000;
        reg_x15_q <= 32'h0000_0000;
        reg_x16_q <= 32'h0000_0000;
        reg_x17_q <= 32'h0000_0000;
        reg_x18_q <= 32'h0000_0000;
        reg_x19_q <= 32'h0000_0000;
        reg_x20_q <= 32'h0000_0000;
        reg_x21_q <= 32'h0000_0000;
        reg_x22_q <= 32'h0000_0000;
        reg_x23_q <= 32'h0000_0000;
        reg_x24_q <= 32'h0000_0000;
        reg_x25_q <= 32'h0000_0000;
        reg_x26_q <= 32'h0000_0000;
        reg_x27_q <= 32'h0000_0000;
        reg_x28_q <= 32'h0000_0000;
        reg_x29_q <= 32'h0000_0000;
        reg_x30_q <= 32'h0000_0000;
        reg_x31_q <= 32'h0000_0000;
    end else begin
        if (5'd1 == wbck_dest_idx) reg_x1_q <= wbck_dest_dat;
        else reg_x1_q <= reg_x1_q;

        if (5'd2 == wbck_dest_idx) reg_x2_q <= wbck_dest_dat;
        else reg_x2_q <= reg_x2_q;

        if (5'd3 == wbck_dest_idx) reg_x3_q <= wbck_dest_dat;
        else reg_x3_q <= reg_x3_q;

        if (5'd4 == wbck_dest_idx) reg_x4_q <= wbck_dest_dat;
        else reg_x4_q <= reg_x4_q;

        if (5'd5 == wbck_dest_idx) reg_x5_q <= wbck_dest_dat;
        else reg_x5_q <= reg_x5_q;

        if (5'd6 == wbck_dest_idx) reg_x6_q <= wbck_dest_dat;
        else reg_x6_q <= reg_x6_q;

        if (5'd7 == wbck_dest_idx) reg_x7_q <= wbck_dest_dat;
        else reg_x7_q <= reg_x7_q;

        if (5'd8 == wbck_dest_idx) reg_x8_q <= wbck_dest_dat;
        else reg_x8_q <= reg_x8_q;

        if (5'd9 == wbck_dest_idx) reg_x9_q <= wbck_dest_dat;
        else reg_x9_q <= reg_x9_q;
        
        if (5'd10 == wbck_dest_idx) reg_x10_q <= wbck_dest_dat;
        else reg_x10_q <= reg_x10_q;

        if (5'd11 == wbck_dest_idx) reg_x11_q <= wbck_dest_dat;
        else reg_x11_q <= reg_x11_q;

        if (5'd12 == wbck_dest_idx) reg_x12_q <= wbck_dest_dat;
        else reg_x12_q <= reg_x12_q;

        if (5'd13 == wbck_dest_idx) reg_x13_q <= wbck_dest_dat;
        else reg_x13_q <= reg_x13_q;

        if (5'd14 == wbck_dest_idx) reg_x14_q <= wbck_dest_dat;
        else reg_x14_q <= reg_x14_q;

        if (5'd15 == wbck_dest_idx) reg_x15_q <= wbck_dest_dat;
        else reg_x15_q <= reg_x15_q;

        if (5'd16 == wbck_dest_idx) reg_x16_q <= wbck_dest_dat;
        else reg_x16_q <= reg_x16_q;

        if (5'd17 == wbck_dest_idx) reg_x17_q <= wbck_dest_dat;
        else reg_x17_q <= reg_x17_q;

        if (5'd18 == wbck_dest_idx) reg_x18_q <= wbck_dest_dat;
        else reg_x18_q <= reg_x18_q;

        if (5'd19 == wbck_dest_idx) reg_x19_q <= wbck_dest_dat;
        else reg_x19_q <= reg_x19_q;

        if (5'd20 == wbck_dest_idx) reg_x20_q <= wbck_dest_dat;
        else reg_x20_q <= reg_x20_q;

        if (5'd21 == wbck_dest_idx) reg_x21_q <= wbck_dest_dat;
        else reg_x21_q <= reg_x21_q;

        if (5'd22 == wbck_dest_idx) reg_x22_q <= wbck_dest_dat;
        else reg_x22_q <= reg_x22_q;

        if (5'd23 == wbck_dest_idx) reg_x23_q <= wbck_dest_dat;
        else reg_x23_q <= reg_x23_q;

        if (5'd24 == wbck_dest_idx) reg_x24_q <= wbck_dest_dat;
        else reg_x24_q <= reg_x24_q;

        if (5'd25 == wbck_dest_idx) reg_x25_q <= wbck_dest_dat;
        else reg_x25_q <= reg_x25_q;

        if (5'd26 == wbck_dest_idx) reg_x26_q <= wbck_dest_dat;
        else reg_x26_q <= reg_x26_q;

        if (5'd27 == wbck_dest_idx) reg_x27_q <= wbck_dest_dat;
        else reg_x27_q <= reg_x27_q;

        if (5'd28 == wbck_dest_idx) reg_x28_q <= wbck_dest_dat;
        else reg_x28_q <= reg_x28_q;

        if (5'd29 == wbck_dest_idx) reg_x29_q <= wbck_dest_dat;
        else reg_x29_q <= reg_x29_q;

        if (5'd30 == wbck_dest_idx) reg_x30_q <= wbck_dest_dat;
        else reg_x30_q <= reg_x30_q;

        if (5'd31 == wbck_dest_idx) reg_x31_q <= wbck_dest_dat;
        else reg_x1_q <= reg_x1_q;
    end 
end

reg [31:0] read_scr1_dat_temp;
reg [31:0] read_scr2_dat_temp;

// read register
always @ (clk posedge)
    case (read_scr1_idx)
        5'd1 :  read_scr1_dat_temp  <= reg_x1_q;
        5'd2 :  read_scr1_dat_temp  <= reg_x2_q;
        5'd3 :  read_scr1_dat_temp  <= reg_x3_q;
        5'd4 :  read_scr1_dat_temp  <= reg_x4_q;
        5'd5 :  read_scr1_dat_temp  <= reg_x5_q;
        5'd6 :  read_scr1_dat_temp  <= reg_x6_q;
        5'd7 :  read_scr1_dat_temp  <= reg_x7_q;
        5'd8 :  read_scr1_dat_temp  <= reg_x8_q;
        5'd9 :  read_scr1_dat_temp  <= reg_x9_q;
        5'd10 : read_scr1_dat_temp  <= reg_x10_q;
        5'd11 : read_scr1_dat_temp  <= reg_x11_q;
        5'd12 : read_scr1_dat_temp  <= reg_x12_q;
        5'd13 : read_scr1_dat_temp  <= reg_x13_q;
        5'd14 : read_scr1_dat_temp  <= reg_x14_q;
        5'd15 : read_scr1_dat_temp  <= reg_x15_q;
        5'd16 : read_scr1_dat_temp  <= reg_x16_q;
        5'd17 : read_scr1_dat_temp  <= reg_x17_q;
        5'd18 : read_scr1_dat_temp  <= reg_x18_q;
        5'd19 : read_scr1_dat_temp  <= reg_x19_q;
        5'd20 : read_scr1_dat_temp  <= reg_x20_q;
        5'd21 : read_scr1_dat_temp  <= reg_x21_q;
        5'd22 : read_scr1_dat_temp  <= reg_x22_q;
        5'd23 : read_scr1_dat_temp  <= reg_x23_q;
        5'd24 : read_scr1_dat_temp  <= reg_x24_q;
        5'd25 : read_scr1_dat_temp  <= reg_x25_q;
        5'd26 : read_scr1_dat_temp  <= reg_x26_q;
        5'd27 : read_scr1_dat_temp  <= reg_x27_q;
        5'd28 : read_scr1_dat_temp  <= reg_x28_q;
        5'd29 : read_scr1_dat_temp  <= reg_x29_q;
        5'd30 : read_scr1_dat_temp  <= reg_x30_q;
        5'd31 : read_scr1_dat_temp  <= reg_x31_q;
        default read_scr1_dat_temp  <= 32'h0000_0000;
    endcase
    
    case (read_scr2_idx)
        5'd1 :  read_scr2_dat_temp <= reg_x1_q;
        5'd2 :  read_scr2_dat_temp <= reg_x2_q;
        5'd3 :  read_scr2_dat_temp <= reg_x3_q;
        5'd4 :  read_scr2_dat_temp <= reg_x4_q;
        5'd5 :  read_scr2_dat_temp <= reg_x5_q;
        5'd6 :  read_scr2_dat_temp <= reg_x6_q;
        5'd7 :  read_scr2_dat_temp <= reg_x7_q;
        5'd8 :  read_scr2_dat_temp <= reg_x8_q;
        5'd9 :  read_scr2_dat_temp <= reg_x9_q;
        5'd10 : read_scr2_dat_temp <= reg_x10_q;
        5'd11 : read_scr2_dat_temp <= reg_x11_q;
        5'd12 : read_scr2_dat_temp <= reg_x12_q;
        5'd13 : read_scr2_dat_temp <= reg_x13_q;
        5'd14 : read_scr2_dat_temp <= reg_x14_q;
        5'd15 : read_scr2_dat_temp <= reg_x15_q;
        5'd16 : read_scr2_dat_temp <= reg_x16_q;
        5'd17 : read_scr2_dat_temp <= reg_x17_q;
        5'd18 : read_scr2_dat_temp <= reg_x18_q;
        5'd19 : read_scr2_dat_temp <= reg_x19_q;
        5'd20 : read_scr2_dat_temp <= reg_x20_q;
        5'd21 : read_scr2_dat_temp <= reg_x21_q;
        5'd22 : read_scr2_dat_temp <= reg_x22_q;
        5'd23 : read_scr2_dat_temp <= reg_x23_q;
        5'd24 : read_scr2_dat_temp <= reg_x24_q;
        5'd25 : read_scr2_dat_temp <= reg_x25_q;
        5'd26 : read_scr2_dat_temp <= reg_x26_q;
        5'd27 : read_scr2_dat_temp <= reg_x27_q;
        5'd28 : read_scr2_dat_temp <= reg_x28_q;
        5'd29 : read_scr2_dat_temp <= reg_x29_q;
        5'd30 : read_scr2_dat_temp <= reg_x30_q;
        5'd31 : read_scr2_dat_temp <= reg_x31_q;
        default read_scr2_dat_temp <= 32'h0000_0000;
    endcase

// assign register to wire
assign read_src1_dat = read_scr1_dat_temp;
assign read_sec2_dat = read_scr2_dat_temp;

endmodule

//--------------------------------------------------------------------------
