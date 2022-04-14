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
`include "macan_execute.v"

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
    input wire clk,
    input wire rst_n,

    // Input from the ID/EX
    input wire [31:0] if_pc_in_e
    input wire [31:0] read_rs1_data;
    input wire [31:0] read_rs2_data;
    input wire [31:0] sign_imm;
    input wire [4:0]  inst_rd_e;
    input wire [5:0]  alu_op_e; 

    // Outputs
    output reg [31:0] alu_result;
    output reg [31:0] write_data;
    output reg  zero_flag;
);

// Update zero flag
wire ex_zer_flag;
assign ex_zero_flag = (ex_alu_result == 32'h0000_0000) ? 1'b1 : 1'b0;

reg [31:0] ex_alu_result;

// Execute ALU
always @(*) begin
    if (!rst_n) begin
        ex_alu_result <= 32'h0000_0000;
    end else begin
        case (alu_op_e)
            `EXE_ADD_OP: ex_alu_result <= read_rs1_data + read_rs2_data;
            `EXE_SUB_OP: ex_alu_result <= read_rs1_data - read_rs2_data;
            default:     ex_alu_result <= read_rs2_data + 32'h0000_0001;
        endcase      
    end 
end

// Update IF/EX Register
if (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        write_data <= 32'h0000_0000;
        alu_result <= 32'h0000_0000;
        zero_flag  <= 1'b0;
    end else begin
        alu_result <= ex_alu_result;
        write_dta  <= read_rs2_data;
        zero_flag  <= ex_zero_flag;
    end 
end

endmodule
//--------------------------------------------------------------------------
