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
// Brief: RISC-V Instruction Execute file: execute or addrss calculation
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module macan_execute
//--------------------------------------------------------------------------
// Params
//--------------------------------------------------------------------------
#(

)
//--------------------------------------------------------------------------
// Ports
//--------------------------------------------------------------------------
(
    // Inputs
    input wire        clk,
    input wire        rst_n,

    // Input from the ID/EX
    input wire [31:0] id_pc_ex,
    input wire [31:0] id_rs1_data_ex,
    input wire [31:0] id_rs2_data_ex,
    input wire [31:0] id_sign_imm_ex,
    input wire [2:0]  id_funct3_ex,
    input wire [6:0]  id_funct7_ex,
    input wire [6:0]  id_opcode_ex,
    input wire [4:0]  id_ex_rd,
    input wire [4:0]  id_shamt_ex,

    input wire        id_aul_imm_src_ex,
    input wire        id_branch_en_ex,
    input wire        id_jump_en_ex,

    // Outputs to EX/MEM Register
    output reg [31:0] ex_pc_mem
);

reg [31:0] alu_result;

// RISCV EXECUTE
always @(*) begin
    case (id_opcode_ex)
        `OPCODE_LUI:

        `OPCODE_AUIPC:
        `OPCODE_JAL:
        `OPCODE_JALR:
        `OPCODE_BRANCH:
        `OPCODE_LOAD:
        `OPCODE_STORE:
        `OPCODE_ALUI:
        `OPCODE_ALU:
        `OPCODE_FENCE:
        `OPCODE_EXTEN:
        `default:
    endcase
end

// Update IF/EX Register
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ex_pc_mem <= 32'h0000_0000;
    end else begin
        ex_pc_mem <= id_pc_ex;
    end
end

endmodule
//--------------------------------------------------------------------------
