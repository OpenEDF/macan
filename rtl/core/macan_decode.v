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
    input wire          clk,
    input wire          rst_n,

    // Input from IF/ID Register
    input wire  [31:0]  if_id_inst,
    input wire  [31:0]  if_id_pc,

    // Input from MEM/WB Register
    input wire          mem_wb_write,
    input wire  [4:0]   mem_wb_rd,
    input wire  [31:0]  mem_wb_data,

    // Interface for register file
    output wire [4:0]   read_rs1,
    output wire [4:0]   read_rs2,
    output wire [4:0]   write_rd,
    output wire [31:0]  write_data,

    input reg   [31:0]  rs1_data,
    input reg   [31:0]  rs2_data,

    // TO EX STAGE CONTROL SIGNAL
    input wire          cu_alu_imm_src,
    input wire          cu_branch_en,
    input wire          cu_jump_en,

    // TO MEM STAGE CONTROL SIGNAL
    input wire          cu_mem_write_en,
    input wire          cu_mem_read_en,

    // TO WB STAGE CONTROL SIGNAL
    input wire          cu_reg_write_en,
    input wire [1:0]    cu_result_src,

    // Outputs to ID/EX Register
    output reg  [31:0]  id_ex_pc,
    output reg  [31:0]  id_ex_rs1_data,
    output reg  [31:0]  id_ex_rs2_data,
    output reg  [31:0]  id_ex_sign_imm,
    output reg  [2:0]   id_ex_funct3,
    output reg  [6:0]   id_ex_funct7,
    output reg  [6:0]   id_ex_opcode,
    output reg  [4:0]   id_ex_rd,
    output reg  [4:0]   id_ex_shamt,

    // TO EX STAGE CONTROL SIGNAL
    output reg          id_ex_alu_imm_src,
    output reg          id_ex_branch_en,
    output reg          id_ex_jump_en,

    // TO MEM STAGE CONTROL SIGNAL
    output reg          id_ex_mem_write_en,
    output reg          id_ex_mem_read_en,

    // TO WB STAGE CONTROL SIGNAL
    output reg          id_ex_reg_write_en,
    output reg [1:0]    id_ex_result_src
);

/* register */
reg [4:0] reg_scr1;
reg [4:0] reg_scr2;
reg [4:0] reg_rd;
reg [2:0] reg_funct3;
reg [6:0] reg_funct7;
reg [6:0] reg_opcode;
reg       scr1_en;
reg       scr2_en;
reg signed [31:0] immediate;
reg [4:0] shamt;

/* wire */
wire [4:0] scr1;
wire [4:0] scr2;
wire [4:0] rd;
wire [2:0] funct3;
wire [6:0] funct7;
wire [6:0] opcode;

/* immediate */
wire signed [31:0] i_immediate;
wire signed [31:0] s_immediate;
wire signed [31:0] b_immediate;
wire signed [31:0] u_immediate;
wire signed [31:0] j_immediate;

/* prase instruction */
assign scr1 = if_id_inst[19:15];
assign scr2 = if_id_inst[24:20];
assign rd   = if_id_inst[11:7];
assign funct3 = if_id_inst[14:12];
assign funct7 = if_id_inst[31:25];
assign opcode = if_id_inst[6:0];

// immediate generation uniti, sign-Extend
assign i_immediate = {{21{if_id_inst[31]}}, if_id_inst[30:25], if_id_inst[24:21], if_id_inst[20]};
assign s_immediate = {{21{if_id_inst[31]}}, if_id_inst[30:25], if_id_inst[11:8], if_id_inst[7]};
assign b_immediate = {{20{if_id_inst[31]}}, if_id_inst[7], if_id_inst[30:25], if_id_inst[11:8], 1'b0};
assign u_immediate = {if_id_inst[31], if_id_inst[30:20], if_id_inst[19:12], {12{1'b0}}};
assign j_immediate = {{12{if_id_inst[31]}}, if_id_inst[19:12], if_id_inst[20], if_id_inst[30:25], if_id_inst[24:21], 1'b0};


// output instuction decode signal according the opcode/funct3/funct7
always @(*) begin
    if (!rst_n) begin
        reg_scr1   <= 5'b0;
        reg_scr2   <= 5'b0;
        reg_rd     <= 5'b0;
        reg_funct3 <= 3'b0;
        reg_funct7 <= 7'b0;
        reg_opcode <= 7'b0;
        scr1_en    <= 1'b0;
        scr2_en    <= 1'b0;
        immediate  <= 32'b0;
        shamt      <= 5'b0;
    end else begin
        case(opcode)
            `OPCODE_LUI: begin
                reg_scr1   <= 5'bz;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= 3'bz;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b0;
                scr2_en    <= 1'b0;
                immediate  <= u_immediate;
           end
           `OPCODE_AUIPC: begin
                reg_scr1   <= 5'bz;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= 3'bz;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b0;
                scr2_en    <= 1'b0;
                immediate  <= u_immediate;
           end
           `OPCODE_JAL: begin
                reg_scr1   <= 5'bz;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= 3'bz;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b0;
                scr2_en    <= 1'b0;
                immediate  <= j_immediate;
           end
           `OPCODE_JALR: begin
                reg_scr1   <= scr1;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= funct3;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b0;
                immediate  <= i_immediate;
           end
           `OPCODE_BRANCH: begin
                reg_scr1   <= scr1;
                reg_scr2   <= scr2;
                reg_rd     <= 5'bz;
                reg_funct3 <= funct3;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b1;
                immediate  <= b_immediate;
            end
           `OPCODE_LOAD: begin
                reg_scr1   <= scr1;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= funct3;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b0;
                immediate  <= i_immediate;
           end
           `OPCODE_STORE: begin
                reg_scr1   <= scr1;
                reg_scr2   <= scr2;
                reg_rd     <= 5'bz;
                reg_funct3 <= funct3;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b1;
                immediate  <= s_immediate;
           end
           `OPCODE_ALUI: begin
                reg_scr1   <= scr1;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= funct3;
                reg_funct7 <= 7'bz;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b0;
                immediate  <= i_immediate;
           end
           `OPCODE_ALU: begin
                reg_scr1   <= scr1;
                reg_scr2   <= scr2;
                reg_rd     <= rd;
                reg_funct3 <= funct3;
                reg_funct7 <= funct7;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b0;
                immediate  <= 32'bz;
                shamt      <= scr2;
           end
           `OPCODE_FENCE: begin
                reg_scr1   <= scr1;
                reg_scr2   <= 5'bz;
                reg_rd     <= rd;
                reg_funct3 <= 3'bz;
                reg_funct7 <= funct7;
                reg_opcode <= opcode;
                scr1_en    <= 1'b1;
                scr2_en    <= 1'b0;
                immediate  <= i_immediate;
           end
           `OPCODE_EXTEN: begin
                reg_scr1   <= 5'b0;
                reg_scr2   <= 5'b0;
                reg_rd     <= 5'b0;
                reg_funct3 <= 3'b0;
                reg_funct7 <= 7'b0;
                reg_opcode <= opcode;
                scr1_en    <= 1'b0;
                scr2_en    <= 1'b0;
                immediate  <= i_immediate;
           end
           default: begin
                reg_scr1   <= 5'b0;
                reg_scr2   <= 5'b0;
                reg_rd     <= 5'b0;
                reg_funct3 <= 3'b0;
                reg_funct7 <= 7'b0;
                reg_opcode <= 7'b0;
                scr1_en    <= 1'b0;
                scr2_en    <= 1'b0;
                immediate  <= 32'b0;
                shamt      <= 5'b0;
           end
        endcase
    end
end

// read register file
assign read_rs1 = (scr1_en == 1'b1) ? reg_scr1 : 5'b0;
assign read_rs2 = (scr2_en == 1'b1) ? reg_scr2 : 5'b0;

// write register file
assign write_rd   = (mem_wb_write == 1'b1) ? mem_wb_rd : 5'd0;
assign write_data = mem_wb_data;

// Update the ID/EX Register file
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        id_ex_pc        <= 32'b0;
        id_ex_rs1_data  <= 32'b0;
        id_ex_rs2_data  <= 32'b0;
        id_ex_sign_imm  <= 32'b0;
        id_ex_funct3    <= 3'b0;
        id_ex_funct7    <= 7'b0;
        id_ex_opcode    <= 7'b0;
        id_ex_rd        <= 5'b0;
        id_ex_shamt     <= 5'b0;
        id_ex_alu_imm_src  <= 1'b0;
        id_ex_branch_en    <= 1'b0;
        id_ex_jump_en      <= 1'b0;
        id_ex_mem_write_en <= 1'b0;
        id_ex_mem_read_en  <= 1'b0;
        id_ex_reg_write_en <= 1'b0;
        id_ex_result_src   <= 1'b0;
    end else begin
        id_ex_pc        <= if_id_pc;
        id_ex_rs1_data  <= rs1_data;
        id_ex_rs2_data  <= rs2_data;
        id_ex_sign_imm  <= immediate;
        id_ex_funct3    <= reg_funct3;
        id_ex_funct7    <= reg_funct7;
        id_ex_opcode    <= reg_opcode;
        id_ex_rd        <= reg_rd;
        id_ex_shamt     <= shamt;
        id_ex_alu_imm_src  <= cu_alu_imm_src; // 'cu' is control unit
        id_ex_branch_en    <= cu_branch_en;
        id_ex_jump_en      <= cu_jump_en;
        id_ex_mem_write_en <= cu_mem_write_en;
        id_ex_mem_read_en  <= cu_mem_read_en;
        id_ex_reg_write_en <= cu_reg_write_en;
        id_ex_result_src   <= cu_result_src;
    end
end

endmodule

//--------------------------------------------------------------------------
