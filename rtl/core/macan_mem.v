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
// Brief: RISC-V Instruction Access memory file: load instruction reading the
// data memory using the address from the EXE/MEM pipeline regsiter and
// loading the data into the MEM/WB pipeline register.
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module macan_mem
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
    
    // Input from EX/MEM
    input wire [31:0] ex_alu_result_mem;
    input wire [31:0] ex_write_data_mem;
    input wire        ex_write_mem_en_mem;
    input wire        ex_wr_rd_mem_addr_mem;
    input wire        ex_direct_wr_reg_mem;
    input wire [1:0]  ex_load_width_mem;
    input wire [1:0]  ex_store_width_mem;
    input wire [4:0]  ex_rd_mem;

    // Interface for memory access or AMBA
    output reg [31:0] mem_addr;
    output reg        mem_read;
    output reg        mem_write;
    output reg [31:0] mem_write_data;

    input wire [31:0] mem_read_data;

    // Outputs to MEM/WB
    output reg [31:0] mem_rd_data_wb;
    output reg [31:0] mem_alu_result_wb;
    output reg [4:0]  mem_rd_wb;
);

reg [32:0] read_mem_data_mem;

// Read or write memory data
always @(*) begin
    if (!rst_n) begin
        mem_addr  <= 32'h0000_0000;
        mem_read  <= 1'b0;
        mem_write <= 1'b0;
        mem_write_data <= 32'h0000_0000;
    end else begin
        if (mem_read_in) begin
            memory_read <= 1'b1;
            memory_addr <= alu_result_ein;
        end else if (mem_write_in) begin
            memory_write <= 1'b1;
            memory_addr  <= alu_result_ein;
            memory_write_data <= read_rs2_write_data;
        end else begin
            memory_addr  <= 32'h0000_0000;
            memory_read  <= 1'b0;
            memory_write <= 1'b0;
            memory_write_data <= 32'h0000_0000;
        end
    end
end

// Update the MEM/WB Register stage
always (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mem_rd_data_wb    <= 32'h0;
        mem_alu_result_wb <= 32'h0;
        mem_rd_wb         <= 5'b0;
    end else begin
        mem_rd_data_wb    <= 32'h0;
        mem_alu_result_wb <= 32'h0;
        mem_rd_wb         <= 5'b0;
    end
end

endmodule
//--------------------------------------------------------------------------
