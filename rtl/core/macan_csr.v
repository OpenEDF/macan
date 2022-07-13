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
// Brief: RISC-V Instruction for Control and Status Instructions,the full
//        set of CSR instructions that operate on these CSRs.
// Change Log:
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------
// Include File
//--------------------------------------------------------------------------
`include "macan_defines.v"

//--------------------------------------------------------------------------
// Module
//--------------------------------------------------------------------------
module macan_csr
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

    // Inputs and Outputs Address
    input wire [11:0] csr_addr,
    input wire [31:0] write_data,
    input wire        write_en,
    input wire        csr_sel,

    // Outputs
    output reg  [31:0]  read_data,
    output wire [127:0] ctrl_bus
);

// Control and Status Register Memory Model: RISC-V defines a separate
// address space of 4096 Control and Status registers associated  with
// each hart.
// define the Currently allocated RISC-V machine-level CSR address
reg [31:0] mvendorid_ro;
reg [31:0] marchid_ro;
reg [31:0] mimpid_ro;
reg [31:0] mhartid_ro;
reg [31:0] mconfigptr_ro;
reg [31:0] mstatus_rw;
reg [31:0] misa_rw;
reg [31:0] medeleg_rw;
reg [31:0] mideleg_rw;
reg [31:0] mie_rw;
reg [31:0] mtvec_rw;
reg [31:0] mcounteren_rw;
reg [31:0] mstatush_rw;
reg [31:0] mscratch_rw;
reg [31:0] mepc_rw;
reg [31:0] mcause_rw;
reg [31:0] mtval_rw;
reg [31:0] mip_rw;
reg [31:0] mtinst_rw;
reg [31:0] mtval2_rw;
reg [31:0] menvcfg_rw;
reg [31:0] menvcfgh_rw;
reg [31:0] mseccfg_rw;
reg [31:0] mseccfgh_rw;
reg [31:0] mcycle_rw;
reg [31:0] minstret_rw;
reg [31:0] mcountinhibit_rw;
reg [31:0] tselect_rw;
reg [31:0] tdata1_rw;
reg [31:0] tdata2_rw;
reg [31:0] tdata3_rw;
reg [31:0] mcontext_rw;
reg [31:0] dcsr_rw;
reg [31:0] dpc_rw;
reg [31:0] dscratch0_rw;
reg [31:0] dscratch1_rw;

// read write data from csr memory
always @(*) begin
    if (!rst_n) begin
        mvendorid_ro     <= 32'h0;
        marchid_ro       <= 32'h0;
        mimpid_ro        <= 32'h0;
        mhartid_ro       <= 32'h0;
        mconfigptr_ro    <= 32'h0;
        mstatus_rw       <= 32'h0;
        misa_rw          <= 32'h0;
        medeleg_rw       <= 32'h0;
        mideleg_rw       <= 32'h0;
        mie_rw           <= 32'h0;
        mtvec_rw         <= 32'h0;
        mcounteren_rw    <= 32'h0;
        mstatush_rw      <= 32'h0;
        mscratch_rw      <= 32'h0;
        mepc_rw          <= 32'h0;
        mcause_rw        <= 32'h0;
        mtval_rw         <= 32'h0;
        mip_rw           <= 32'h0;
        mtinst_rw        <= 32'h0;
        mtval2_rw        <= 32'h0;
        menvcfg_rw       <= 32'h0;
        menvcfgh_rw      <= 32'h0;
        mseccfg_rw       <= 32'h0;
        mseccfgh_rw      <= 32'h0;
        mcycle_rw        <= 32'h0;
        minstret_rw      <= 32'h0;
        mcountinhibit_rw <= 32'h0;
        tselect_rw       <= 32'h0;
        tdata1_rw        <= 32'h0;
        tdata2_rw        <= 32'h0;
        tdata3_rw        <= 32'h0;
        mcontext_rw      <= 32'h0;
        dcsr_rw          <= 32'h0;
        dpc_rw           <= 32'h0;
        dscratch0_rw     <= 32'h0;
        dscratch1_rw     <= 32'h0;
        read_data        <= 32'h0;
    end else begin
        if (write_en & csr_sel) begin
            case (csr_addr)
                `M_CSR_MSTATUS_ADDR       mstatus_rw    <= write_data;
                `M_CSR_MISA_ADDR          misa_rw       <= write_data;
                `M_CSR_MEDELEG_ADDR       medeleg_rw    <= write_data;
                `M_CSR_MIDELEG_ADDR       mideleg_rw    <= write_data;
                `M_CSR_MIE_ADDR           mie           <= write_data;
                `M_CSR_MTVEC_ADDR         mtvec_rw      <= write_data;
                `M_CSR_MCOUNTEREN_ADDR    mcounteren_rw <= write_data;
                `M_CSR_MSTATUSH_ADDR      mstatush_rw   <= write_data;

                `M_CSR_MSCRATCH_ADDR      mscratch_rw   <= write_data;
                `M_CSR_MEPC_ADDR          mepc_rw       <= write_data;
                `M_CSR_MCAUSE_ADDR        mcause_rw     <= write_data;
                `M_CSR_MTVAL_ADDR         mtval_rw      <= write_data;
                `M_CSR_MIP_ADDR           mip_rw        <= write_data;
                `M_CSR_MTINST_ADDR        mtinst_rw     <= write_data;
                `M_CSR_MTVAL2_ADDR        mtval2_rw     <= write_data;

                `M_CSR_MENVCFG_ADDR       menvcfg_rw    <= write_data;
                `M_CSR_MENVCFGH_ADDR      menvcfgh_rw   <= write_data;
                `M_CSR_MSECCFG_ADDR       mseccfg_rw    <= write_data;
                `M_CSR_MSECCFGH_ADDR      mseccfg_rw    <= write_data;
            endcase
        end else begin
            case (csr_addr)
                `M_CSR_MVENDORID_ADDR:    read_data <= mvendorid_ro;
                `M_CSR_MARCHID_ADDR:      read_data <= marchid_ro;
                `M_CSR_MIMPID_ADDR:       read_data <= mimpid_ro;
                `M_CSR_MHARTID_ADDR:      read_data <= mhartid_ro;
                `M_CSR_MCONFIGPTR_ADDR:   read_data <= mconfigptr_ro;

                default read_data <= 32'h0;
                // This register must be readable in any implementation,
                // but a value of 0 can be returned to indicate the field is not implemented
            endcase
        end
    end

// output the control bus signal
assign ctrl_bus = {mie[25], misa[16]};

endmodule

//--------------------------------------------------------------------------
