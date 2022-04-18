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
// Brief: RISC-V Instruction Fetch file: read instruction from memory
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module macan_control_unit
//--------------------------------------------------------------------------
// Params
//--------------------------------------------------------------------------
#(

)

//--------------------------------------------------------------------------
// Ports
//-----------------------------:---------------------------------------------
(
    // Inputs
    input wire clk,
    input wire rst_n,

    // Input from IF/ID Register
    input wire [31:0] if_inst_in,

    // Outputs
    output wire [2:0]  imm_src,
    output reg  [5:0]  alu_op
);

// parameters for opcode
localparam OPCODE_LUI     = 7'b0110111;
localparam OPCODE_AUIPC   = 7'b0010111;
localparam OPCODE_JAL     = 7'b1101111;
localparam OPCODE_JALR    = 7'b1100111;
localparam OPCODE_BRANCH  = 7'b1100011;
localparam OPCODE_LOAD    = 7'b0000011;
localparam OPCODE_STORE   = 7'b0100011;
localparam OPCODE_ALUI    = 7'b0010011;
localparam OPCODE_ALU     = 7'b0110011;
localparam OPCODE_FENCE   = 7'b0001111;
localparam OPCODE_ECALL   = 7'b1110011;
localparam OPCODE_EBREAK  = 7'b1110011;

// parameters for alu
localparam ALU_ADD        = 11'b00000110011;
localparam ALU_SUB        = 11'b10000110011;

// RISC-V Instruction
wire [10:0] alu_inst;

// obtian opcode funct3 funct7
wire [6:0] opcode;
wire [2:0] funct3;
wire       funct7;      // funct7 [6:0] is inst[31:25] but only inst[30] is used

assign opcode = if_inst_in[6:0];
assign funct3 = if_inst_in[14:12];
assign funct7 = if_inst_in[30];
assign alu_inst = {funct7, funct3, opcode};

reg [2:0] reg_imm_src;
assign imm_src = reg_imm_src;
// output imm_src for select immediate by instruction format
always @(*) begin
    case (opcode)
        OPCODE_LUI, OPCODE_AUIPC:              reg_imm_src = `U_FORMAT_INST;
        OPCODE_JAL:                            reg_imm_src = `J_FORMAT_INST;
        OPCODE_JALR, OPCODE_LOAD, OPCODE_ALUI: reg_imm_src = `I_FORMAT_INST;
        OPCODE_BRANCH:                         reg_imm_src = `B_FORMAT_INST;
        OPCODE_STORE:                          reg_imm_src = `S_FORMAT_INST;
        default:                               reg_imm_src = `I_FORMAT_INST;
    endcase
end

// output instruction
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        alu_op <= 6'b000000;
    end else begin
        case (alu_inst)
            ALU_ADD : alu_op <= `EXE_ADD_OP;
            ALU_SUB : alu_op <= `EXE_SUB_OP;
            // TODO: Add other instruction
            default: alu_op <= `ILLEGAL_INST;
        endcase
    end
    $display("alu_inst = %11b", alu_inst);
end

endmodule
//--------------------------------------------------------------------------
