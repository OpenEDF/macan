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
module macan_fetch
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
    input wire clk,
    input wire rst_n,
    input wire pc_br,
    input wire [31:0]  pc_br_imm,

    // interface to memory controller
    output reg         mem_cs,
    output reg [31:0]  mem_addr_o,
    input wire [31:0]  mem_data_in,

    // Outputs
    output reg [31:0]  if_inst_o,
    output reg [31:0]  if_pc_o
);

reg [31:0] if_pc = 32'h0000_0000;

// read memory fetch instruction
always @(*) begin
    if (!rst_n) begin
        mem_cs <= 1'b0;
        mem_addr_o <= MACAN_START_PC;
    end else begin
        mem_cs <= 1'b1;
        mem_addr_o <= if_pc;
    end
end

// update the pc
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        if_pc <= MACAN_START_PC;
    end else begin
        if (pc_br) begin   // branch
            if_pc <= pc_br_imm;
        end else begin     // normal
            if_pc <= if_pc + 4;
        end
    end
end

// update the IF/ID register must in one cycle
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        if_inst_o <= `RV32I_NOP;
        if_pc_o   <= MACAN_START_PC;
    end else begin
        if_inst_o <= mem_data_in;
        if_pc_o <= if_pc;
    end
end
endmodule
//--------------------------------------------------------------------------
