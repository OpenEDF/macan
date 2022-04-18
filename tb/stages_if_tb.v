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

// fetch stage
reg pc_br;
reg  [31:0] pc_br_imm;
wire [31:0] if_pc_id;
wire [31:0] if_inst_id;

// instract memory interface
wire         mem_cs;
wire [31:0]  mem_addr_o;
wire [31:0]  mem_data_in;

// decode
wire [2:0]  imm_src_id;
wire [5:0]  alu_op_id;
wire [4:0]  rs1_idx;
wire [4:0]  rs2_idx;
wire [31:0] read_src1_data;
wire [31:0] read_src2_data;
wire [31:0] read_rs1_data_ex;
wire [31:0] read_rs2_data_ex;
wire [31:0] sign_immediate_ex;
wire [4:0]  inst_rd_ex;
wire [31:0] if_pc_ex;

// fetch
macan_fetch u0 (
    .clk(clk),
    .rst_n(rst_n),
    .pc_br(pc_br),
    .pc_br_imm(pc_br_imm),
    .mem_cs(mem_cs),
    .mem_addr_o(mem_addr_o),
    .mem_data_in(mem_data_in),
    .if_inst_o(if_inst_id),
    .if_pc_o(if_pc_id)
);

// instruction memory rom
mem_rom u1 (
    .clk(clk),
    .addr_in(mem_addr_o),
    .cs(mem_cs),
    .data_o(mem_data_in)
);

// decode
macan_decode u2 (
    .clk(clk),
    .rst_n(rst_n),

    .if_inst_in(if_inst_id),
    .if_pc_in(if_pc_id),
    .imm_src(imm_src_id),

    .reg_write_in(),
    .write_rd_idx_in(),
    .write_rd_data_in(),

    .read_rs1_idx(rs1_idx),
    .read_rs2_idx(rs2_idx),
    .write_rd_idx(),
    .write_rd_data(),

    .read_rs1_data(read_src1_data),
    .read_rs2_data(read_src2_data),

    .read_rs1_data_o(read_rs1_data_ex),
    .read_rs2_data_o(read_rs2_data_ex),
    .sign_immediate(sign_immediate_ex),
    .inst_rd(inst_rd_ex),
    .if_pc_o(if_pc_ex)
);

// control unit
macan_control_unit u3 (
    .clk(clk),
    .rst_n(rst_n),
    .if_inst_in(if_inst_id),
    .imm_src(imm_src_id),
    .alu_op(alu_op_id)
);

// register file
register_file u4 (
    .clk(clk),
    .rst_n(rst_n),
    .read_scr1_idx(rs1_idx),
    .read_scr2_idx(rs1_idx),
    .wbck_dest_idx(),
    .wbck_dest_dat(),

    .read_src1_data(read_src1_data),
    .read_src2_data(read_src2_data)
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
    /*
    $monitor("[%4t if] rst_n = %0b pc_br = %0b pc_br_imm = %H mem_cs = %0b mem_addr_o = %H mem_data_in = %H if_inst_id = %H if_pc_id = %H",
             $time, rst_n, pc_br, pc_br_imm, mem_cs, mem_addr_o, mem_data_in, if_inst_id, if_pc_id);
    */
    $monitor("[%4t id] pc = %H inst = %H rs1 = %2d rs2 = %2d rd = %2d src1 = %H src2 = %H imm = %H alu = %2d ",
             $time, if_pc_id, if_inst_id, rs1_idx, rs2_idx, inst_rd_ex, read_src1_data, read_src2_data, sign_immediate_ex, alu_op_id);
end

// dump macan.fsdb file
initial begin
    $fsdbDumpfile("macan.fsdb");
    $fsdbDumpvars(0, tb_top);
    $fsdbDumpMDA;
end

endmodule
//--------------------------------------------------------------------------
