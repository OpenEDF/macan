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
// Brief: RISC-V Instruction Decode file: understand waht instruction means
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module macan_decode
//--------------------------------------------------------------------------
// Params
//--------------------------------------------------------------------------
#(
    parameter MACAN_START_PC = `DEF_START_MACAN_PC //start pc
)
//--------------------------------------------------------------------------
// Ports
//--------------------------------------------------------------------------
(
    // Inputs
    input clk,
    input rst_n,

    // Input from IF/ID
    input wire [31:0]  if_inst_in,
    input wire [31:0]  if_pc_in,
    input wire [2:0]   imm_src,

    // register file write from WB
    input wire         reg_write_in,
    input wire [4:0]   write_rd_idx_in,
    input wire [31:0]  write_rd_data_in,

    // Interface for register file
    output wire [4:0]  read_rs1_idx,
    output wire [4:0]  read_rs2_idx,
    output wire [4:0]  write_rd_idx,
    output wire [31:0] write_rd_data,

    input wire [31:0]  read_rs1_data,
    input wire [31:0]  read_rs2_data,

    // Outputs to ID/EX
    output reg [31:0]  read_rs1_data_o,
    output reg [31:0]  read_rs2_data_o,
    output reg [31:0]  sign_immediate,
    output reg [4:0]   inst_rd,
    output reg [31:0]  if_pc_o
);

wire signed [31:0] i_immediate;
wire signed [31:0] s_immediate;
wire signed [31:0] b_immediate;
wire signed [31:0] u_immediate;
wire signed [31:0] j_immediate;
reg  signed [31:0] immediate;

// immediate generation uniti, sign-Extend
assign i_immediate = {{21{if_inst_in[31]}}, if_inst_in[30:25], if_inst_in[24:21], if_inst_in[20]};
assign s_immediate = {{21{if_inst_in[31]}}, if_inst_in[30:25], if_inst_in[11:8], if_inst_in[7]};
assign b_immediate = {{20{if_inst_in[31]}}, if_inst_in[7], if_inst_in[30:25], if_inst_in[11:8], 1'b0};
assign u_immediate = {if_inst_in[31], if_inst_in[30:20], if_inst_in[19:12], {12{1'b0}}};
assign j_immediate = {{12{if_inst_in[31]}}, if_inst_in[19:12], if_inst_in[20], if_inst_in[30:25], if_inst_in[24:21], 1'b0};

// produce immedicates by base instruction format
always @(*) begin
    case (imm_src)
        `I_FORMAT_INST: immediate = i_immediate;
        `S_FORMAT_INST: immediate = s_immediate;
        `B_FORMAT_INST: immediate = b_immediate;
        `U_FORMAT_INST: immediate = u_immediate;
        `J_FORMAT_INST: immediate = j_immediate;
        default         immediate = 32'h0000_0000;
    endcase
end

// read register file
assign read_rs1_idx = if_inst_in[19:15];
assign read_rs2_idx = if_inst_in[24:20];

// write register file
assign write_rd_idx  = (reg_write_in == 1'b1) ? write_rd_idx_in : 5'd0;
assign write_rd_data = write_rd_data_in;

// Update the ID/EX Register file
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        if_pc_o <= MACAN_START_PC;
        read_rs1_data_o <= 32'h0000_0000;
        read_rs2_data_o <= 32'h0000_0000;
        sign_immediate  <= 32'h0000_0000;
        inst_rd <= 5'b00000;
    end else begin
        if_pc_o <= if_pc_in;
        inst_rd <= if_inst_in[11:7];
        read_rs1_data_o <= read_rs1_data;
        read_rs2_data_o <= read_rs2_data;
        sign_immediate  <= immediate;
    end
end

endmodule

//--------------------------------------------------------------------------
