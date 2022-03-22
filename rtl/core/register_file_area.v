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
module register_file_area
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
reg [31:0] reg_xx_q [31:0];

`ifdef SIMULATION_DEBUG
    wire [31:0] x0_zero_w = 32'h0000_0000;        // hardwires zero
    wire [31:0] x1_ra_w   =  reg_xx_q[1];         // return address
    wire [31:0] x2_sp_w   =  reg_xx_q[2];         // stack pointer
    wire [31:0] x3_gp_w   =  reg_xx_q[3];         // global pointer
    wire [31:0] x4_tp_w   =  reg_xx_q[4];         // thread pointer
    wire [31:0] x5_t0_w   =  reg_xx_q[5];         // x5 - x7 temporary registers
    wire [31:0] x6_t1_w   =  reg_xx_q[6];
    wire [31:0] x7_t2_w   =  reg_xx_q[7];
    wire [31:0] x8_s0_w   =  reg_xx_q[8];         // saved register / frame poinyer
    wire [31:0] x9_s1_w   =  reg_xx_q[9];         // saved register
    wire [31:0] x10_a0_w  =  reg_xx_q[10];        // a0 - 1 function arguments / return values
    wire [31:0] x11_a1_w  =  reg_xx_q[11];
    wire [31:0] x12_a2_w  =  reg_xx_q[12];        // a2 - 7 function arguments
    wire [31:0] x13_a3_w  =  reg_xx_q[13];
    wire [31:0] x14_a4_w  =  reg_xx_q[14];
    wire [31:0] x15_a5_w  =  reg_xx_q[15];
    wire [31:0] x16_a6_w  =  reg_xx_q[16];
    wire [31:0] x17_a7_w  =  reg_xx_q[17];
    wire [31:0] x18_s2_w  =  reg_xx_q[18];        // s2 - 11 saved registers
    wire [31:0] x19_s3_w  =  reg_xx_q[19];
    wire [31:0] x20_s5_w  =  reg_xx_q[20];
    wire [31:0] x21_s6_w  =  reg_xx_q[21];
    wire [31:0] x22_s7_w  =  reg_xx_q[22];
    wire [31:0] x23_s8_w  =  reg_xx_q[23];
    wire [31:0] x24_s9_w  =  reg_xx_q[24];
    wire [31:0] x25_s9_w  =  reg_xx_q[25];
    wire [31:0] x26_s10_w =  reg_xx_q[26];
    wire [31:0] x27_s11_w =  reg_xx_q[27];
    wire [31:0] x28_t3_w  =  reg_xx_q[28];       // t3 -6 temporary registers
    wire [31:0] x29_t4_w  =  reg_xx_q[29];
    wire [31:0] x30_t5_w  =  reg_xx_q[30];
    wire [31:0] x31_t6_w  =  reg_xx_q[31];
`endif

// register wirte
genvar index;
always @(posedge clk or posedge rst_n) begin
    if (!rst_n) begin
        generate
            for (index = 0; index < 32; index = index + 1) begin: regx
                reg_xx_q[index] <= 32'h0000_0000;
            end
        endgenerate
    end else begin
        reg_xx_q[wbck_dest_idx] <= wbck_dest_dat;
    end
end

reg [31:0] read_scr1_dat_temp;
reg [31:0] read_scr2_dat_temp;

// read register
always @ (posedge clk) begin
    read_scr1_dat_temp <= reg_xx_q[read_scr1_idx];
    read_scr2_dat_temp <= reg_xx_q[read_scr2_idx];
end

// assign register to wire
assign read_src1_dat = read_scr1_dat_temp;
assign read_src2_dat = read_scr2_dat_temp;

endmodule

//--------------------------------------------------------------------------
