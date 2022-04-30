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
// Brief: RISC-V Instruction Access memory file: read or write memory
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
    input wire [31:0] alu_result_ein;
    input wire [31:0] read_rs2_write_data;
    input wire        mem_read_in;
    input wire        mem_write_in;

    // Interface for memory access
    output reg [31:0] memory_addr;
    output reg        memory_read;
    output reg        memory_write;
    output reg [31:0] memory_write_data;

    input reg  [31:0] memory_read_data;
    input reg         memory_read_done;

    // Outputs
    output reg [31:0] read_data_mem;
    output reg [31:0] alu_result_eo;
);

reg [32:0] read_mem_data_mem;

// Read or write memory data
always @(*) begin
    if (!rst_n) begin
        memory_addr  <= 32'h0000_0000;
        memory_read  <= 1'b0;
        memory_write <= 1'b0;
        memory_write_data <= 32'h0000_0000;
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

// Read data
always @(*) begin
    if(memory_read_done) begin
        read_mem_data_mem <= memory_read_data;
    end else begin
        read_mem_data_mem <= read_mem_data_mem;
    end
end

// Update the MEM/WB Register file
always (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        alu_result_eo <= 32'h0000_0000;
        read_data_mem <= 32'h0000_0000;
    end else begin
        alu_result_eo <= alu_result_ein;
        read_data_mem <= read_mem_data_mem; 
    end
end

endmodule
//--------------------------------------------------------------------------
