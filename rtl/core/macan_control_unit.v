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
//--------------------------------------------------------------------------
(
    // Inputs
    //input wire        clk,
    input wire        rst_n,

    // Input from IF/ID Register
    input wire [31:0] if_id_inst,

    // Outputs inter decode pipeline stage
    // TO EX STAGE CONTROL SIGNAL
    output reg        alu_imm_src,
    output reg        branch_en,
    output reg        jump_en,

    // TO MEM STAGE CONTROL SIGNAL
    output reg        mem_write_en,
    output reg        mem_read_en,

    // TO WB STAGE CONTROL SIGNAL
    output reg        reg_write_en,
    output reg [1:0]  result_src
);


// obtian opcode
wire [6:0] opcode;
assign opcode = if_id_inst[6:0];

// output control signal according the opcode
always @(*) begin
    if (!rst_n) begin
        alu_imm_src  <= 1'b0;
        branch_en    <= 1'b0;
        jump_en      <= 1'b0;
        mem_read_en  <= 1'b0;
        mem_write_en <= 1'b0;
        reg_write_en <= 1'b0;
        result_src   <= `WB_FROM_DONT_CARE;
    end else begin
        case(opcode)
            `OPCODE_LUI: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_AUIPC: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_JAL: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_JALR: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_BRANCH: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_DONT_CARE;
            end
            `OPCODE_LOAD: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_READ_MEM;
            end
            `OPCODE_STORE: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_DONT_CARE;
            end
            `OPCODE_ALUI: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_ALU: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_FENCE: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_ALU_RESULT;
            end
            `OPCODE_EXTEN: begin
                alu_imm_src  <= 1'b1;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b1;
                result_src   <= `WB_FROM_DONT_CARE;
            end
            default: begin
                alu_imm_src  <= 1'b0;
                branch_en    <= 1'b0;
                jump_en      <= 1'b0;
                mem_read_en  <= 1'b0;
                mem_write_en <= 1'b0;
                reg_write_en <= 1'b0;
                result_src   <= `WB_FROM_DONT_CARE;
            end
        endcase
    end
end
endmodule
//--------------------------------------------------------------------------
