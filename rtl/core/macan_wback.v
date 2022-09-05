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
// Brief: RISC-V Pipeline for Write Back Stage, which places the result back
// into the register file in the middle of the datapath.
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module macan_wback
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

    // Input from MEM/WB Register
    input wire [31:0] mem_alu_result_wb,
    input wire [31:0] mem_write_data_wb,
    input wire        mem_wri,
    input wire [1:0]  mem_ctrl_src_wb;
    input wire [4:0]  mem_rd_wb,

    // Outputs data and destination register to middle datapath
    output wire [31:0] wb_data_write_reg;
    output wire [4:0]  wb_rd_reg;
);

// temp register vailable for temp data to register file
reg [31:0] temp_data_to_reg;

// Combination logic output to register file
always @(*) begin
    if (!rst_n) begin
        wb_data_write_reg <= 32'h0;
        wb_rd_reg <= 5'b0;
    end else begin
        case(mem_ctrl_src_wb) begin
            WB_TO_REG_CTRL_ALU: temp_data_to_reg <= mem_alu_result_wb;
            WB_TO_REG_RD_MEM:   temp_data_to_reg <= mem_write_data_wb;
            //TODO: pc + 4 to register WB_TO_REG_PC_PLUS4: temp_data_to_reg <=             ;
            default: temp_data_to_reg <= 32'h0;
        end
    end
end

assign wb_rd_reg = mem_rd_wb;
assign wb_data_write_reg = temp_data_to_reg;

endmodule
//--------------------------------------------------------------------------
