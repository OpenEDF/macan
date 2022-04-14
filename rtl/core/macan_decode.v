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
    output reg [4:0]   read_rs1_idx,
    output reg [4:0]   read_rs2_idx,
    output reg [4:0]   write_rd_idx,
    output reg [31:0]  write_rd_data,
    output reg         reg_write_o,
    output reg         reg_read_o,

    input wire [31:0]  read_rs1_data,
    input wire [31:0]  read_rs2_data,
    input wire         reg_read_done,

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

// immediate generation uniti, sign-Extend
assign i_immediate = {{21{if_inst_in[31]}}, if_inst_in[30:25], if_inst_in[24:21], if_inst_in[20]};
assign s_immediate = {{21{if_inst_in[31]}}, if_inst_in[30:25], if_inst_in[11:8], if_inst_in[7]};
assign b_immediate = {{20{if_inst_in[31]}}, if_inst_in[7], if_inst_in[30:25], if_inst_in[11:8], 1'b0};
assign u_immediate = {if_inst_in[31], if_inst_in[30:20], if_inst_in[19:12], {12{1'b0}}};
assign j_immediate = {{12{if_inst_in[31]}}, if_inst_in[19:12], if_inst_in[20], if_inst_in[30:25], if_inst_in[24:21], 1'b0};

// produce immedicates by base instruction format
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sign_immediate <= 32'h0000_0000;
    end else begin
        case (imm_src)
            `I_FORMAT_INST: sign_immediate <= i_immediate;
            `S_FORMAT_INST: sign_immediate <= s_immediate;
            `B_FORMAT_INST: sign_immediate <= b_immediate;
            `U_FORMAT_INST: sign_immediate <= u_immediate;
            `J_FORMAT_INST: sign_immediate <= j_immediate;
            default         sign_immediate <= 32'h0000_0000;
        endcase
    end
end

// read register file
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        read_rs1_idx <= 5'b00000;
        read_rs2_idx <= 5'b00000;
        reg_read_o   <= 1'b0; 
    end else begin
        reg_read_o   <= 1'b1; 
        read_rs1_idx <= if_inst_in[19:15];        
        read_rs2_idx <= if_inst_in[24:20];        
    end
end

// write register file
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        write_rd_idx  <= 5'b00000;
        write_rd_data <= 32'h0000_0000;
        reg_write_o   <= 1'b0;
    end else begin
        if (reg_write_in) begin
            reg_write_o   <= 1'b1;
            write_rd_idx  <= write_rd_idx_in; //from_wb;
            write_rd_data <= write_rd_data_in;
        end else begin
            reg_write_o   <= 1'b0;
            write_rd_idx  <= write_rd_idx_in; //from_wb;
            write_rd_data <= write_rd_data_in;
        end
    end
end

// output register file rs1 and rs2
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        read_rs1_data_o <= 32'h0000_0000;
        read_rs2_data_o <= 32'h0000_0000;
    end else begin
        if (reg_read_done) begin
            read_rs1_data_o <= read_rs1_data;
            read_rs2_data_o <= read_rs2_data;
        end else begin
            read_rs1_data_o <= read_rs1_data_o;
            read_rs2_data_o <= read_rs2_data_o;
        end
    end
end

// output pc
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        if_pc_o <= MACAN_START_PC;
    end else begin
        if_pc_o <= if_pc_in;
    end
end

// output register file rd
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        inst_rd <= 5'b00000;
    end else begin
        inst_rd <= if_inst_in[11:7];
    end
end

endmodule

//--------------------------------------------------------------------------
