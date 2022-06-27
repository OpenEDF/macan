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
    input wire [4:0]  id_rd_ex,
    input wire [4:0]  id_shamt_ex,

    input wire        id_aul_imm_src_ex,
    input wire        id_branch_en_ex,
    input wire        id_jump_en_ex,

    // Outputs to EX/MEM Register
    output reg [31:0] ex_pc_mem,
    output reg [31:0] ex_alu_result_mem,
    output reg        ex_take_branch,
    output reg [31:0] ex_write_data_mem,
    output reg [4:0]  ex_rd_mem
);

reg [31:0] alu_result;
// ALU 12bit case data
wire [9:0] alu_determine;
assign alu_determine = {id_funct7_ex, id_funct3_ex}

// Fence 12bit data
wire [3:0] fm;
wire [3:0] pred;
wire [3:0] succ;
assign fm   = id_sign_imm_ex[11:8];
assign pred = id_sign_imm_ex[7:4];
assign succ = id_sign_imm_ex[3:0];

// ECALL and EBREAK
wire [11:0] exten_determine;
assign exten_determine = id_sign_imm_ex;

//TODO: optimize code size 
//wire exten_determine;
//assign exten_determine = id_sign_imm_ex[0];

// RISCV EXECUTE
always @(*) begin
    if (!rst_n) begin
        ex_alu_result   <= 32'h0;
        ex_take_branch  <= 1'b0;
        ex_rd_mem       <= 5'b0;
    end
    case (id_opcode_ex)
        `OPCODE_LUI:
            ex_rd_mem       <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex;
            ex_take_branch  <= 1'b0;
        `OPCODE_AUIPC:
            ex_rd_mem       <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex + id_pc_ex;
            ex_take_branch  <= 1'b0;
        `OPCODE_JAL:
            ex_rd_mem       <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex;
            ex_take_branch  <= 1'b0;
        `OPCODE_JALR:
            ex_rd_mem       <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex;
            ex_take_branch  <= 1'b0;
        `OPCODE_BRANCH:   //branch
            ex_rd_mem       <= 4'b0000;  // don't care
            case (id_funct3_ex)
                `RV32_BASE_INST_BEQ:
                    if (id_rs1_data_ex == id_rs2_data_ex) begin
                        ex_take_branch <= 1'b1;
                    else begin
                        ex_take_branch <= 1'b0;
                    end
                `RV32_BASE_INST_BNE:
                    if (id_rs1_data_ex != id_rs2_data_ex) begin
                        ex_take_branch <= 1'b1;
                    else begin
                        ex_take_branch <= 1'b0;
                    end
                `RV32_BASE_INST_BLT:
                    if ($signed(id_rs1_data_ex) < $signed(id_rs2_data_ex)) begin
                        ex_take_branch <= 1'b1;
                    else begin
                        ex_take_branch <= 1'b0;
                    end
                `RV21_BASE_INST_BGE:
                    if ($signed(id_rs1_data_ex) > $signed(id_rs2_data_ex)) begin
                        ex_take_branch <= 1'b1;
                    else begin
                        ex_take_branch <= 1'b0;
                    end
                `RV32_BASE_INST_BLTU:
                    if (id_rs1_data_ex <= id_rs2_data_ex) begin
                        ex_take_branch <= 1'b1;
                    else begin
                        ex_take_branch <= 1'b0;
                    end
                `RV32_BASE_INST_BGEU:
                    if (id_rs1_data_ex >= id_rs2_data_ex) begin
                        ex_take_branch <= 1'b1;
                    else begin
                        ex_take_branch <= 1'b0;
                    end
            endcase
        `OPCODE_LOAD:
            case (id_funct3_ex)
                RV32_BASE_INST_LB:
                RV32_BASE_INST_LH:
                RV32_BASE_INST_LW:
                RV32_BASE_INST_LBU:
                RV32_BASE_INST_LHU:
            endcase
        `OPCODE_STORE:
            case (id_funct3_ex)
                RV32_BASE_INST_SB:
                RV32_BASE_INST_SH:
                RV32_BASE_INST_SW:
            endcase
        `OPCODE_ALUI:
            ex_rd_mem <= id_rd_ex;
            case (id_funct3_ex)
                RV32_BASE_INST_ADDI:
                    ex_alu_result   <= id_rs1_data_ex + id_sign_imm_ex;
                RV32_BASE_INST_SLTI:
                    if ($signed(id_rs1_data_ex) < id_sign_imm_ex) begin
                        ex_alu_result <= 32'h1;
                    end else begin
                        ex_alu_result <= 32'h0;
                    end
                RV32_BASE_INST_SLTIU:
                    if (id_rs1_data_ex < id_sign_imm_ex) begin
                        ex_alu_result <= 32'h1;
                    end else begin
                        ex_alu_result <= 32'h0;
                    end
                RV32_BASE_INST_XORI:
                    ex_alu_result   <= id_rs1_data_ex ^ id_sign_imm_ex;
                RV32_BASE_INST_ORI:
                    ex_alu_result   <= id_rs1_data_ex | id_sign_imm_ex;
                RV32_BASE_INST_ANDI:
                    ex_alu_result   <= id_rs1_data_ex & id_sign_imm_ex;
                RV32_BASE_INST_SLLI:
                RV32_BASE_INST_SRLI:
                RV32_BASE_INST_SRAI:
            endcase
        `OPCODE_ALU:
            ex_rd_mem <= id_rd_ex;
            case (alu_determine)
                RV32_BASE_INST_ADD:
                    ex_alu_result   <= id_rs1_data_ex + id_rs2_data_ex;
                RV32_BASE_INST_SUB:
                    ex_alu_result   <= id_rs1_data_ex - id_rs2_data_ex; //TODO: x1 - x2; x2 - x1
                RV32_BASE_INST_SLL:
                    ex_alu_result   <= id_rs1_data_ex << id_rs2_data_ex[4:0];
                RV32_BASE_INST_SLT:
                RV32_BASE_INST_SLTU:
                RV32_BASE_INST_XOR:
                    ex_alu_result   <= id_rs1_data_ex ^ id_rs2_data_ex;
                RV32_BASE_INST_SRL:
                    ex_alu_result   <= id_rs1_data_ex >> id_rs2_data_ex[4:0];
                RV32_BASE_INST_SRA:
                RV32_BASE_INST_OR:
                    ex_alu_result   <= id_rs1_data_ex | id_rs2_data_ex;
                RV32_BASE_INST_AND:
                    ex_alu_result   <= id_rs1_data_ex & id_rs2_data_ex;
            endcase
        `OPCODE_FENCE:
        `OPCODE_EXTEN:
            case (exten_determine)
                RV32_BASE_INST_ECALL:
                    ex_rd_mem       <= 4'b0000;
                RV32_BASE_INST_EBREAK:
                    ex_rd_mem       <= 4'b0000;
            endcase
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
