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
    input wire [4:0]  id_rs1_ex,      // CSR Instructions uimm[4:0] need
    input wire [4:0]  id_shamt_ex,

    input wire        id_aul_imm_src_ex,
    input wire        id_branch_en_ex,
    input wire        id_jump_en_ex,

    // csr register interface
    input wire [31:0] csr_rd_data,
    output reg        csr_write_en,
    output reg        csr_read_en,
    output reg        csr_sel,
    output reg [11:0] csr_addr,
    output reg [4:0]  csr_bit_mask_or_uimm,

    // Outputs to EX/MEM Register
    output reg [31:0] ex_pc_mem,
    output reg [31:0] ex_alu_result_mem,
    output reg        ex_take_branch_mem,
    output reg [31:0] ex_write_data_mem,
    output reg        ex_write_mem_en_mem,
    output reg [31:0] ex_rd_wr_mem_addr_mem,
    output reg        ex_direct_wr_reg_mem,
    output reg [1:0]  ex_load_width_mem,
    output reg [1:0]  ex_store_width_mem,
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
wire [11:0] csr;
assign fm   = id_sign_imm_ex[31:28];
assign pred = id_sign_imm_ex[27:24];
assign succ = id_sign_imm_ex[23:20];
assign csr  = id_sign_imm_ex[31:20];

// ECALL and EBREAK
wire [11:0] exten_determine;
assign exten_determine = id_sign_imm_ex;

//TODO: optimize code size
//wire exten_determine;
//assign exten_determine = id_sign_imm_ex[0];

// temp register
reg [31:0] ex_alu_result;
reg        ex_take_branch;
reg [31:0] ex_write_data;
reg        ex_write_mem_en;
reg [31:0] ex_rd_wr_mem_addr;
reg        ex_direct_wr_reg;
reg [1:0]  ex_load_width;
reg [1:0]  ex_store_width;
reg [4:0]  ex_rd;

// RISCV EXECUTE
always @(*) begin
    if (!rst_n) begin
        ex_alu_result         <= 32'h0;
        ex_take_branch        <= 1'b0;
        ex_rd                 <= 5'b0;
        ex_rd_wr_mem_addr     <= 32'h0;
        ex_write_data         <= 32'h0;
        ex_write_mem_en       <= 1'b0;
        ex_direct_wr_reg      <= 1'b0;
        csr_write_en          <= 1'b0;
        csr_sel               <= 1'b0;
        csr_addr              <= 12'b0;
        csr_bit_mask_or_uimm  <= 5'b0;
    end
    case (id_opcode_ex)
        `OPCODE_LUI:
            ex_rd           <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex;
            ex_take_branch  <= 1'b0;
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b1;
        `OPCODE_AUIPC:
            ex_rd           <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex + id_pc_ex;
            ex_take_branch  <= 1'b0;
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b1;
        `OPCODE_JAL:
            ex_rd           <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex;
            ex_take_branch  <= 1'b0;
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b1;
        `OPCODE_JALR:
            ex_rd           <= id_rd_ex;
            ex_alu_result   <= id_sign_imm_ex;
            ex_take_branch  <= 1'b0;
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b1;
        `OPCODE_BRANCH:   //branch
            ex_rd           <= 4'b0000;  // don't care
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b0;
            case (id_funct3_ex)
                ex_alu_result <= 32'h0;
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
            ex_rd     <= id_rd_ex;
            ex_alu_result <= 32'h0;
            ex_rd_wr_mem_addr     <= id_rs1_data_ex + id_sign_imm_ex; //address = base + offset
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b0;
            case (id_funct3_ex)
                `RV32_BASE_INST_LB:
                    ex_load_width <= `LOAD_WIDTH_BYTE;
                `RV32_BASE_INST_LH:
                    ex_load_width <= `LOAD_WIDTH_HALF;
                `RV32_BASE_INST_LW:
                    ex_load_width <= `LOAD_WIDTH_WORD;
                `RV32_BASE_INST_LBU:
                    ex_load_width <= `LOAD_WIDTH_BYTE;
                `RV32_BASE_INST_LHU:
                    ex_load_width <= `LOAD_WIDTH_WORD;
            endcase
        `OPCODE_STORE:
            ex_write_data <= id_rs2_data_ex;
            ex_alu_result <= 32'h0;
            ex_rd_wr_mem_addr     <= id_rs1_data_ex + id_sign_imm_ex; // address = base + offset
            ex_write_mem_en       <= 1'b1;
            ex_direct_wr_reg      <= 1'b0;
            case (id_funct3_ex)
                `RV32_BASE_INST_SB:
                    ex_store_width <= 'STORE_WIDTH_BYTE;
                `RV32_BASE_INST_SH:
                    ex_store_width <= 'STORE_WIDTH_HALF;
                `RV32_BASE_INST_SW:
                    ex_store_width <= 'STORE_WIDTH_WORD;
            endcase
        `OPCODE_ALUI:
            ex_rd     <= id_rd_ex;
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b1;
            case (id_funct3_ex)
                `RV32_BASE_INST_ADDI:
                    ex_alu_result <= id_rs1_data_ex + id_sign_imm_ex;
                `RV32_BASE_INST_SLTI:
                    if ($signed(id_rs1_data_ex) < id_sign_imm_ex) begin
                        ex_alu_result <= 32'h1;
                    end else begin
                        ex_alu_result <= 32'h0;
                    end
                `RV32_BASE_INST_SLTIU:
                    if (id_rs1_data_ex < id_sign_imm_ex) begin
                        ex_alu_result <= 32'h1;
                    end else begin
                        ex_alu_result <= 32'h0;
                    end
                `RV32_BASE_INST_XORI:
                    ex_alu_result   <= id_rs1_data_ex ^ id_sign_imm_ex;
                `RV32_BASE_INST_ORI:
                    ex_alu_result   <= id_rs1_data_ex | id_sign_imm_ex;
                `RV32_BASE_INST_ANDI:
                    ex_alu_result   <= id_rs1_data_ex & id_sign_imm_ex;
                `RV32_BASE_INST_SLLI:
                    ex_alu_result   <= id_rs1_data_ex << id_sign_imm_ex;
                `RV32_BASE_INST_SRLI:
                    ex_alu_result   <= id_rs1_data_ex >> id_sign_imm_ex;
                `RV32_BASE_INST_SRAI:
                    ex_alu_result   <= id_rs1_data_ex >> id_sign_imm_ex;
            endcase
        `OPCODE_ALU:
            ex_rd     <= id_rd_ex;
            ex_rd_wr_mem_addr     <= 32'h0;
            ex_write_data         <= 32'h0;
            ex_write_mem_en       <= 1'b0;
            ex_direct_wr_reg      <= 1'b1;
            case (alu_determine)
                `RV32_BASE_INST_ADD:
                    ex_alu_result   <= id_rs1_data_ex + id_rs2_data_ex;
                `RV32_BASE_INST_SUB:
                    ex_alu_result   <= id_rs2_data_ex - id_rs1_data_ex;  // x2 - x1
                `RV32_BASE_INST_SLL:
                    ex_alu_result   <= id_rs1_data_ex << id_rs2_data_ex[4:0];
                `RV32_BASE_INST_SLT:
                    if ($signed(id_rs1_data_ex) < $signed(id_rs2_data_ex)) begin
                        ex_alu_result <= 32'h1;
                    else begin
                        ex_alu_result <= 32'h0;
                    end
                `RV32_BASE_INST_SLTU:
                    if (id_rs1_data_ex < id_rs2_data_ex) begin
                        ex_alu_result <= 32'h1;
                    else begin
                        ex_alu_result <= 32'h0;
                    end
                `RV32_BASE_INST_XOR:
                    ex_alu_result   <= id_rs1_data_ex ^ id_rs2_data_ex;
                `RV32_BASE_INST_SRL:
                    ex_alu_result   <= id_rs1_data_ex >> id_rs2_data_ex[4:0];
                `RV32_BASE_INST_SRA:
                    ex_alu_result   <= id_rs1_data_ex >> id_rs2_data_ex[4:0];
                `RV32_BASE_INST_OR:
                    ex_alu_result   <= id_rs1_data_ex | id_rs2_data_ex;
                `RV32_BASE_INST_AND:
                    ex_alu_result   <= id_rs1_data_ex & id_rs2_data_ex;
            endcase
        `OPCODE_FENCE:
            case (id_funct3_ex)
                csr_sel   <= 1'b1;
                ex_rd     <= id_rd_ex;
                RV32_ZICSR_INST_CSRRW:
                    if (id_rd_ex) begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b0;
                    end else begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b1;
                    end
                    ex_alu_result        <= csr_rd_data;
                    csr_addr             <= csr;
                    csr_bit_mask_or_uimm <= id_rs1_ex;
                RV32_ZICSR_INST_CSRRS:
                    if (id_rs1_ex) begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b0;
                    end else begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b1;
                    end
                    ex_alu_result        <= csr_rd_data;
                    csr_addr             <= csr;
                    csr_bit_mask_or_uimm <= id_rs1_ex;
                RV32_ZICSR_INST_CSRRC:
                    if (id_rs1_ex) begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b0;
                    end else begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b1;
                    end
                    ex_alu_result        <= csr_rd_data;
                    csr_addr             <= csr;
                    csr_bit_mask_or_uimm <= id_rs1_ex;
                RV32_ZICSR_INST_CSRRWI:
                    if (id_rd_ex) begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b1;
                    end else begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b0;
                    end
                    ex_alu_result  <= csr_rd_data;
                    csr_addr       <= csr;
                    csr_bit_mask_or_uimm <= id_rs1_ex;
                RV32_ZICSR_INST_CSRRSI:
                RV32_ZICSR_INST_CSRRCI:
                    if (id_rs1_ex) begin
                        csr_write_en     <= 1'b1;
                        csr_read_en      <= 1'b1;
                    end else begin
                        csr_write_en     <= 1'b0;
                        csr_read_en      <= 1'b1;
                    end
                    ex_alu_result        <= csr_rd_data;
                    csr_addr             <= csr;
                    csr_bit_mask_or_uimm <= id_rs1_ex;
            endcase
        `OPCODE_EXTEN:
            case (exten_determine)
                RV32_BASE_INST_ECALL:
                    ex_rd           <= 4'b0000;
                RV32_BASE_INST_EBREAK:
                    ex_rd           <= 4'b0000;
            endcase
        `default:
    endcase
end

// Update IF/EX Register
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        ex_pc_mem             <= 32'h0000_0000;
        ex_alu_result_mem     <= 32'h0000_0000;
        ex_take_branch        <= 1'b0;
        ex_write_data_mem     <= 32'h0000_0000;
        ex_write_mem_en_mem   <= 1'b0;
        ex_rd_wr_mem_addr_mem <= 32'h0000_0000;
        ex_direct_wr_reg_mem  <= 1'b0;
        ex_load_width_mem     <= 2'b0;
        ex_store_width_mem    <= 2'b0;
        ex_rd_mem             <= 5'b0;
    end else begin
        ex_pc_mem             <= id_pc_ex;
        ex_alu_result_mem     <= ex_alu_result;
        ex_take_branch_mem    <= ex_take_branch;
        ex_write_data_mem     <= ex_write_data;
        ex_write_mem_en_mem   <= ex_write_mem_en;
        ex_rd_wr_mem_addr_mem <= ex_rd_wr_mem_addr;
        ex_direct_wr_reg_mem  <= ex_direct_wr_reg;
        ex_load_width_mem     <= ex_load_width;
        ex_store_width_mem    <= ex_store_width;
        ex_rd_mem             <= ex_rd;
    end
end

endmodule
//--------------------------------------------------------------------------
