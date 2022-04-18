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
    input wire [4:0]  read_scr1_idx,
    input wire [4:0]  read_scr2_idx,
    input wire [4:0]  wbck_dest_idx, // index for register
    input wire [31:0] wbck_dest_dat, // write data

    // Outputs
    output wire [31:0] read_src1_data,
    output wire [31:0] read_src2_data
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
always @(posedge clk or posedge rst_n) begin
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
end

// assign register to wire
assign read_src1_data = read_scr1_dat_temp;
assign read_src2_data = read_scr2_dat_temp;

endmodule
//--------------------------------------------------------------------------
