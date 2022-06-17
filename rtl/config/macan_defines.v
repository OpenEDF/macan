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
`define NOSUP_FORMAT_INST  3'b111

// RISC-V instruction opcode
`define OPCODE_LUI         7'b0110111
`define OPCODE_AUIPC       7'b0010111
`define OPCODE_JAL         7'b1101111
`define OPCODE_JALR        7'b1100111
`define OPCODE_BRANCH      7'b1100011
`define OPCODE_LOAD        7'b0000011
`define OPCODE_STORE       7'b0100011
`define OPCODE_ALUI        7'b0010011
`define OPCODE_ALU         7'b0110011
`define OPCODE_FENCE       7'b0001111
`define OPCODE_EXTEN       7'b1110011    // ECALL EBREAK CSRRXX

// RV32I Base Instruction Set  {funct7, funct3} or {funct3}
`define RV32_BASE_INST_LUI        3'0000 // NO NEED
`define RV32_BASE_INST_AUIPC      3'b000 // NO NEED
`define RV32_BASE_INST_JAL        3'b000 // NO NEED
`define RV32_BASE_INST_JALR       3'b000
`define RV32_BASE_INST_BEQ        3'b000
`define RV32_BASE_INST_BNE        3'b001
`define RV32_BASE_INST_BLT        3'b100
`define RV32_BASE_INST_BGE        3'b101
`define RV32_BASE_INST_BLTU       3'b110
`define RV32_BASE_INST_BGEU       3'b111
`define RV32_BASE_INST_LB         3'b000
`define RV32_BASE_INST_LH         3'b001
`define RV32_BASE_INST_LW         3'b010
`define RV32_BASE_INST_LBU        3'b100
`define RV32_BASE_INST_LHU        3'b101
`define RV32_BASE_INST_SB         3'b000
`define RV32_BASE_INST_SH         3'b001
`define RV32_BASE_INST_SW         3'b010
`define RV32_BASE_INST_ADDI       3'b000
`define RV32_BASE_INST_SLTI       3'b010
`define RV32_BASE_INST_SLTIU      3'b011
`define RV32_BASE_INST_XORI       3'b100
`define RV32_BASE_INST_ORI        3'b110
`define RV32_BASE_INST_ANDI       3'b111
`define RV32_BASE_INST_SLLI       10'b0000000001
`define RV32_BASE_INST_SRLI       10'b0000000101
`define RV32_BASE_INST_SRAI       10'b0100000101
`define RV32_BASE_INST_ADD        10'b0000000000
`define RV32_BASE_INST_SUB        10'b0100000000
`define RV32_BASE_INST_SLL        10'b0000000001
`define RV32_BASE_INST_SLT        10'b0000000010
`define RV32_BASE_INST_SLTU       10'b0000000011
`define RV32_BASE_INST_XOR        10'b0000000100
`define RV32_BASE_INST_SRL        10'b0000000101
`define RV32_BASE_INST_SRA        10'b0100000101
`define RV32_BASE_INST_OR         10'b0000000110
`define RV32_BASE_INST_AND        10'b0000000111
`define RV32_BASE_INST_FENCE      3'b000 // NO NEED
`define RV32_BASE_INST_ECALL      1'b0   // 12'b0000_0000_0000
`define RV32_BASE_INST_EBREAK     1'b1   // 12'b0000_0000_0001

//RV32/RV64 Zicsr Standard Extension
`define RV32_ZICSR_INST_CSRRW     3'b001
`define RV32_ZICSR_INST_CSRRS     3'b010
`define RV32_ZICSR_INST_CSRRC     3'b011
`define RV32_ZICSR_INST_CSRRWI    3'b101
`define RV32_ZICSR_INST_CSRRSI    3'b110
`define RV32_ZICSR_INST_CSRRCI    3'b111

// RISC-V write back to register from different source
`define WB_FROM_ALU_RESULT        2'b00
`define WB_FROM_READ_MEM          2'b01
`define WB_FROM_PLUS_PC4          2'b10
`define WB_FROM_DONT_CARE         2'b11
//--------------------------------------------------------------------------
