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
`define RV32_BASE_INST_ECALL      12'b0000_0000_0000   // 12'b0000_0000_0000
`define RV32_BASE_INST_EBREAK     12'b0000_0000_0001   // 12'b0000_0000_0001

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

// RISC-V load instruct width
`define LOAD_WIDTH_BYTE           2'b00
`define LOAD_WIDTH_HALF           2'b01
`define LOAD_WIDTH_WORD           2'b10

// RISC-V store instruct width
`define STORE_WIDTH_BYTE          2'b00
`define STORE_WIDTH_HALF          2'b01
`define STORE_WIDTH_WORD          2'b10

// Currently allocated RISC-V machine-level CSR address
// Machine Information Register
`define M_CSR_MVENDORID_ADDR      12'hF11
`define M_CSR_MARCHID_ADDR        12'hF12
`define M_CSR_MIMPID_ADDR         12'hF13
`define M_CSR_MHARTID_ADDR        12'hF14
`define M_CSR_MCONFIGPTR_ADDR     12'hF15

// Machine Trap Setup
`define M_CSR_MSTATUS_ADDR        12'h300
`define M_CSR_MISA_ADDR           12'h301
`define M_CSR_MEDELEG_ADDR        12'h302
`define M_CSR_MIDELEG_ADDR        12'h303
`define M_CSR_MIE_ADDR            12'h304
`define M_CSR_MTVEC_ADDR          12'h305
`define M_CSR_MCOUNTEREN_ADDR     12'h306
`define M_CSR_MSTATUSH_ADDR       12'h310

// Machine Trap Handing
`define M_CSR_MSCRATCH_ADDR       12'h340
`define M_CSR_MEPC_ADDR           12'h341
`define M_CSR_MCAUSE_ADDR         12'h342
`define M_CSR_MTVAL_ADDR          12'h343
`define M_CSR_MIP_ADDR            12'h344
`define M_CSR_MTINST_ADDR         12'h34A
`define M_CSR_MTVAL2_ADDR         12'h34B

// Machine Configuration
`define M_CSR_MENVCFG_ADDR        12'h30A
`define M_CSR_MENVCFGH_ADDR       12'h31A
`define M_CSR_MSECCFG_ADDR        12'h747
`define M_CSR_MSECCFGH_ADDR       12'h757

// Machine Memory Protection
`define M_CSR_PMPCFG0_ADDR        12'h3A0
`define M_CSR_PMPCFG1_ADDR        12'h3A1
`define M_CSR_PMPCFG2_ADDR        12'h3A2
`define M_CSR_PMPCFG3_ADDR        12'h3A3
`define M_CSR_PMPCFG4_ADDR        12'h3A4
`define M_CSR_PMPCFG5_ADDR        12'h3A5
`define M_CSR_PMPCFG6_ADDR        12'h3A6
`define M_CSR_PMPCFG7_ADDR        12'h3A7
`define M_CSR_PMPCFG8_ADDR        12'h3A8
`define M_CSR_PMPCFG9_ADDR        12'h3A9
`define M_CSR_PMPCFG10_ADDR       12'h3AA
`define M_CSR_PMPCFG11_ADDR       12'h3AB
`define M_CSR_PMPCFG12_ADDR       12'h3AC
`define M_CSR_PMPCFG13_ADDR       12'h3AD
`define M_CSR_PMPCFG14_ADDR       12'h3AE
`define M_CSR_PMPCFG15_ADDR       12'h3AF

`define M_CSR_PMPADDR0_ADDR       12'h3B0
`define M_CSR_PMPADDR1_ADDR       12'h3B1
`define M_CSR_PMPADDR2_ADDR       12'h3B2
`define M_CSR_PMPADDR3_ADDR       12'h3B3
`define M_CSR_PMPADDR4_ADDR       12'h3B4
`define M_CSR_PMPADDR5_ADDR       12'h3B5
`define M_CSR_PMPADDR6_ADDR       12'h3B6
`define M_CSR_PMPADDR7_ADDR       12'h3B7
`define M_CSR_PMPADDR8_ADDR       12'h3B8
`define M_CSR_PMPADDR9_ADDR       12'h3B9
`define M_CSR_PMPADDR10_ADDR      12'h3BA
`define M_CSR_PMPADDR11_ADDR      12'h3BB
`define M_CSR_PMPADDR12_ADDR      12'h3BC
`define M_CSR_PMPADDR13_ADDR      12'h3BD
`define M_CSR_PMPADDR14_ADDR      12'h3BE
`define M_CSR_PMPADDR15_ADDR      12'h3BF
// ...
`define M_CSR_PMPADDR63_ADDR      12'h3EF

// Machine Counter/Timer
`define M_CSR_MCYCLE_ADDR         12'hB00
`define M_CSR_MINSTRET_ADDR       12'hB02
`define M_CSR_MHPMCOUNTER3_ADDR   12'hB03
`define M_CSR_MHPMCOUNTER4_ADDR   12'hB04
// ...
`define M_CSR_MHPMCOUNTER31_ADDR  12'hB1F
`define M_CSR_MCYCCLEH_ADDR       12'hB80
`define M_CSR_MINSTRETH_ADDR      12'hB82
`define M_CSR_MHPMCOUNTER3H_ADDR  12'hB83
`define M_CSR_MHPMCOUNTER4H_ADDR  12'hB84
// ...
`define M_CSR_MHPMCOUNTER31H_ADDR 12'hB9F

// Machine Counter Setup
`define M_CSR_MCOUNTINHBIT_ADDR   12'h320
`define M_CSR_MHPMEVENT3_ADDR     12'h323
`define M_CSR_MHPMEVENT4_ADDR     12'h324
// ...
`define M_CSR_MHPMEVENT31_ADDR    12'h33F

// Debug/Trace Registers
`define M_CSR_TSELECT_ADDR        12'h7A0
`define M_CSR_TDARA1_ADDR         12'h7A1
`define M_CSR_TDATA2_ADDR         12'h7A2
`define M_CSR_TDATA3_ADDR         12'h7A3
`define M_CSR_MCONTEXT_ADDR       12'h7A8

// Debug Mode Registers
`define M_CSR_DCSR_ADDR           12'h7B0
`define M_CSR_DPC_ADDR            12'h7B1
`define M_CSR_DSCRATCH0_ADDR      12'h7B2
`define M_CSR_DSCRATCH1_ADDR      12'h7B3

// Write back to register file control source
`define WB_TO_REG_CTRL_ALU        2'b00
`define WB_TO_REG_RD_MEM          2'b01
`define WB_TO_REG_PC_PLUS4        2'b10
//--------------------------------------------------------------------------
