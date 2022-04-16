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
// Brief: testbench for 5-stages pipeline fetch stage
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module tb_top();

// clock and reset
reg clk;
reg rst_n;
reg pc_br;
reg  [31:0] pc_br_imm;
wire [31:0] if_pc_o;
wire [31:0] if_inst_o;

// instract memory interface
wire         mem_cs;
wire [31:0]  mem_addr_o;
wire [31:0]  mem_data_in;

// instance module
macan_fetch u0 (
    .clk(clk),
    .rst_n(rst_n),
    .pc_br(pc_br),
    .pc_br_imm(pc_br_imm),
    .mem_cs(mem_cs),
    .mem_addr_o(mem_addr_o),
    .mem_data_in(mem_data_in),
    .if_inst_o(if_inst_o),
    .if_pc_o(if_pc_o)
);

mem_rom u1 (
    .clk(clk),
    .addr_in(mem_addr_o),
    .cs(mem_cs),
    .data_o(mem_data_in)
);

// generate clock
initial begin
    clk = 1'b1;
    forever #5 clk = ~clk;
end

// init inst mem
initial begin
    integer k, j = 0;
    reg [31:0] temp_mem[0:255];
    $readmemh("inst.hex", temp_mem);
    for (k = 0; k <= 255; k = k + 1) begin
        u1.mem[j]     = temp_mem[k][7:0];
        u1.mem[j + 1] = temp_mem[k][15:8];
        u1.mem[j + 2] = temp_mem[k][23:16];
        u1.mem[j + 3] = temp_mem[k][31:24];
        j = j + 4;
    end
    $display("Program lode done.\n");
end

// init and setup
initial begin
    $display("[OK] Start test...");
    rst_n = 1'b0;
    #2
    rst_n = 1'b1;
    pc_br = 1'b0;
    pc_br_imm = 32'hx;
    #100
    rst_n = 1'b0;
    #2
    rst_n = 1'b1;
    #100
    $finish();
end

// monitor and dispaly
initial begin
    $monitor("[%4t] rst_n = %0b pc_br = %0b pc_br_imm = %H mem_cs = %0b mem_addr_o = %H mem_data_in = %H if_inst_o = %H if_pc_o = %H",
             $time, rst_n, pc_br, pc_br_imm, mem_cs, mem_addr_o, mem_data_in, if_inst_o, if_pc_o);
end

// dump macan.fsdb file
initial begin
    $fsdbDumpfile("macan.fsdb");
    $fsdbDumpvars(0, tb_top);
    $fsdbDumpMDA;
end

endmodule
//--------------------------------------------------------------------------
