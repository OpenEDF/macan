// Designer: Macro
// Brief: define and config
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// config define
//--------------------------------------------------------------------------
`define DEF_START_MACAN_PC 32'h0000_0000
`define RV32I_NOP          32'h0000_0013      // ADDI x0, x0, 0

// produce immediates by base instruction formats
`define I_FORMAT_INST      3'b000
`define S_FORMAT_INST      3'b001
`define B_FORMAT_INST      3'b010
`define U_FORMAT_INST      3'b011
`define J_FORMAT_INST      3'b100

// RISC-V instruction
`define EXE_ADD_OP          6'b011100
`define EXE_SUB_OP          6'b011101
`define ILLEGAL_INST        6'b111111

//--------------------------------------------------------------------------
