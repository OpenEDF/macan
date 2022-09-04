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
    input wire [31:0] ex_wr_rd_mem_addr_mem;
    input wire        ex_direct_wr_reg_mem;  //TODO: think about whether it's usefuli?
    input wire [1:0]  ex_load_width_mem;
    input wire [1:0]  ex_store_width_mem;
    input wire [4:0]  ex_rd_mem;

    // Interface for memory access or AMBA
    output reg [31:0] mem_addr;
    output reg        mem_we;
    output reg        mem_cs;
    output reg [31:0] mem_write_data;

    input wire [31:0] mem_read_data;

    // Outputs to MEM/WB
    output reg [31:0] mem_rd_data_wb;
    output reg [31:0] mem_alu_result_wb;
    output reg [4:0]  mem_rd_wb;
);

reg [31:0] read_mem_data_mem;

// Read or write memory data
always @(*) begin
    if (!rst_n) begin
        mem_addr  <= 32'h0;
        mem_we    <= 1'b0;
        mem_cs    <= 1'b0;
        mem_write_data <= 32'h0;
        read_mem_data_mem <= 32'h0;
    end else begin
        if (ex_write_mem_en_mem == 1'b0) begin // READ MEMORY
            mem_we    <= 1'b0;
            mem_cs    <= 1'b1;
            mem_addr  <= ex_wr_rd_mem_addr_mem;
            read_mem_data_mem <= mem_read_data;
        end else begin // WRITE MEMORY
            mem_we    <= 1'b1;
            mem_cs    <= 1'b1;
            mem_addr  <= ex_wr_rd_mem_addr_mem;
            mem_write_data <= ex_write_data_mem;
        end
    end
end

// TODO: load and store width

// Update the MEM/WB Register stage
always (posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mem_rd_data_wb    <= 32'h0;
        mem_alu_result_wb <= 32'h0;
        mem_rd_wb         <= 5'b0;
    end else begin
        mem_rd_data_wb    <= read_mem_data_mem;
        mem_alu_result_wb <= ex_alu_result_mem;
        mem_rd_wb         <= mem_rd_wb;
        // TODO: src is from control unit. mem_result_src_wb <= ;
        // TODO: control register file write enable ;
    end
end

endmodule
//--------------------------------------------------------------------------
