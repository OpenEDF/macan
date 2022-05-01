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

// Assembly Pipeline
// clock and reset
reg  clk;
reg  rst_n;

// instruction fetch stage
reg  pc_br;
reg  [31:0]  branch_imm;

wire [31:0]  if_inst_id;     // IF/ID
wire [31:0]  if_pc_id;

// instructio memory interface
wire         mem_cs;
wire [31:0]  mem_addr_o;
wire [31:0]  mem_data_in;

// instruction decode stage
wire [4:0]   read_rs1;
wire [4:0]   read_rs2;
wire [31:0]  rs1_data;
wire [31:0]  rs2_data;

wire         cu_alu_imm_src;
wire         cu_branch_en;
wire         cu_jump_en;
wire         cu_mem_write_en;
wire         cu_mem_read_en;
wire         cu_reg_write_en;
wire         cu_result_src;

wire [31:0]  id_pc_ex;         // ID/EX
wire [31:0]  id_rs1_data_ex;
wire [31:0]  id_rs2_data_ex;
wire [31:0]  id_sign_imm_ex;
wire [2:0]   id_funct3_ex;
wire [6:0]   id_funct7_ex;
wire [6:0]   id_opcode_ex;
wire [4:0]   id_rd_ex;
wire [4:0]   id_rd_shamt;

wire         id_alu_imm_src_ex;
wire         id_branch_en_ex;
wire         id_jump_en_ex;
wire         id_mem_write_en_ex;
wire         id_mem_read_en_ex;
wire         id_reg_write_en_ex;
wire         id_result_src_ex;

wire [31:0]  ex_pc_mem;         // EX/MEM

// pipeline fetch stage
macan_fetch fetch_stage (
    .clk(clk),
    .rst_n(rst_n),
    .pc_br(pc_br),
    .cu_branch_imm(branch_imm),

    .mem_cs(mem_cs),
    .mem_addr_o(mem_addr_o),
    .mem_data_in(mem_data_in),

    .if_id_inst(if_inst_id),
    .if_id_pc(if_pc_id)
);

// instruction memory rom
mem_rom inst_mem (
    .clk(clk),
    .addr_in(mem_addr_o),
    .cs(mem_cs),
    .data_o(mem_data_in)
);

// pipeline decode stage
macan_decode decode_stage (
    .clk(clk),
    .rst_n(rst_n),

    .if_id_inst(if_inst_id),
    .if_id_pc(if_pc_id),

    .mem_wb_write(),
    .mem_wb_rd(),
    .mem_wb_data(),

    .read_rs1(read_rs1),
    .read_rs2(read_rs2),
    .write_rd(),
    .write_data(),

    .rs1_data(rs1_data),
    .rs2_data(rs2_data),

    .cu_alu_imm_src(cu_alu_imm_src),
    .cu_branch_en(cu_branch_en),
    .cu_jump_en(cu_jump_en),

    .cu_mem_write_en(cu_mem_write_en),
    .cu_mem_read_en(cu_mem_read_en),

    .cu_reg_write_en(cu_reg_write_en),
    .cu_result_src(cu_result_src),

    .id_ex_pc(id_pc_ex),
    .id_ex_rs1_data(id_rs1_data_ex),
    .id_ex_rs2_data(id_rs2_data_ex),
    .id_ex_sign_imm(id_sign_imm_ex),
    .id_ex_funct3(id_funct3_ex),
    .id_ex_funct7(id_funct7_ex),
    .id_ex_opcode(id_opcode_ex),
    .id_ex_rd(id_rd_ex),
    .id_ex_shamt(id_rd_shamt),

    .id_ex_alu_imm_src(id_alu_imm_src_ex),
    .id_ex_branch_en(id_branch_en_ex),
    .id_ex_jump_en(id_jump_en_ex),

    .id_ex_mem_write_en(id_mem_write_en_ex),
    .id_ex_mem_read_en(id_mem_read_en_ex),

    .id_ex_reg_write_en(id_reg_write_en_ex),
    .id_ex_result_src(id_result_src_ex)
);

// control unit
macan_control_unit control_unit (
    //.clk(clk),
    .rst_n(rst_n),

    .if_id_inst(if_pc_id),
    .alu_imm_src(cu_alu_imm_src),
    .branch_en(cu_branch_en),
    .jump_en(cu_jump_en),

    .mem_write_en(cu_mem_write_en),
    .mem_read_en(cu_mem_read_en),

    .reg_write_en(cu_reg_write_en),
    .result_src(cu_result_src)
);

// register file
register_file register (
    .clk(clk),
    .rst_n(rst_n),

    .read_scr1_idx(read_rs1),
    .read_scr2_idx(read_rs2),
    .wbck_dest_idx(),
    .wbck_dest_dat(),

    .read_src1_data(rs1_data),
    .read_src2_data(rs2_data)
);

// execute stage
macan_execute execute_stage (
    .clk(clk),
    .rst_n(rst_n),

    .id_pc_ex(id_pc_ex),
    .ex_pc_mem(ex_pc_mem)
);


// generate clock
initial begin
    clk = 1'b1;
    forever #5 clk = ~clk;
end

// init inst mem
integer  k = 0;
integer  j = 0;
initial begin
    reg [31:0] temp_mem[0:255];
    $readmemh("inst.hex", temp_mem);
    for (k = 0; k <= 255; k = k + 1) begin
        inst_mem.mem[j]     = temp_mem[k][7:0];
        inst_mem.mem[j + 1] = temp_mem[k][15:8];
        inst_mem.mem[j + 2] = temp_mem[k][23:16];
        inst_mem.mem[j + 3] = temp_mem[k][31:24];
        j = j + 4;
    end
    $display("Program loda done.\n");
end

// init and setup
initial begin
    $display("[OK] Start test...");
    rst_n = 1'b0;
    #2
    rst_n = 1'b1;
    pc_br = 1'b0;
    branch_imm = 32'hx;
    #1000
    $finish();
end

// dump macan.fsdb file
initial begin
    $fsdbDumpfile("macan.fsdb");
    $fsdbDumpvars(0, tb_top);
    $fsdbDumpMDA;
end

endmodule
//--------------------------------------------------------------------------
