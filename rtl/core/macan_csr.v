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
    output reg  [31:0] read_data,
    output wire [127:0] ctrl_bus
);

// Control and Status Register Memory Model: RISC-V defines a separate
// address space of 4096 Control and Status registers associated  with
// each hart.
// define the Currently allocated RISC-V machine-level CSR address
reg [31:0] mvendorid;
reg [31:0] marchid;
reg [31:0] mimpid;
reg [31:0] mhartid;
reg [31:0] mconfigptr;
reg [31:0] mstatus;
reg [31:0] misa;
reg [31:0] medeleg;
reg [31:0] medeleg;
reg [31:0] mie;
reg [31:0] mtvec;
reg [31:0] mcounteren;
reg [31:0] mstatush;
reg [31:0] mscratch;
reg [31:0] mepc;
reg [31:0] mcause;
reg [31:0] mtval;
reg [31:0] mip;
reg [31:0] mtinst;
reg [31:0] mtval2;
reg [31:0] menvcfg;
reg [31:0] menvcfgh;
reg [31:0] mseccfg;
reg [31:0] mseccfgh;
reg [31:0] mcycle;
reg [31:0] minstret;
reg [31:0] mcountinhibit;
reg [31:0] tselect;
reg [31:0] tdata1;
reg [31:0] tdata2;
reg [31:0] tdata3;
reg [31:0] mcontext;
reg [31:0] dcsr;
reg [31:0] dpc;
reg [31:0] dscratch0;
reg [31:0] dscratch1;

// read write data from csr memory
always @(*) begin
    if (!rst_n) begin
        mvendorid     <= 32'h0;
        marchid       <= 32'h0;
        mimpid        <= 32'h0;
        mhartid       <= 32'h0;
        mconfigptr    <= 32'h0;
        mstatus       <= 32'h0;
        misa          <= 32'h0;
        medeleg       <= 32'h0;
        medeleg       <= 32'h0;
        mie           <= 32'h0;
        mtvec         <= 32'h0;
        mcounteren    <= 32'h0;
        mstatush      <= 32'h0;
        mscratch      <= 32'h0;
        mepc          <= 32'h0;
        mcause        <= 32'h0;
        mtval         <= 32'h0;
        mip           <= 32'h0;
        mtinst        <= 32'h0;
        mtval2        <= 32'h0;
        menvcfg       <= 32'h0;
        menvcfgh      <= 32'h0;
        mseccfg       <= 32'h0;
        mseccfgh      <= 32'h0;
        mcycle        <= 32'h0;
        minstret      <= 32'h0;
        mcountinhibit <= 32'h0;
        tselect       <= 32'h0;
        tdata1        <= 32'h0;
        tdata2        <= 32'h0;
        tdata3        <= 32'h0;
        mcontext      <= 32'h0;
        dcsr          <= 32'h0;
        dpc           <= 32'h0;
        dscratch0     <= 32'h0;
        dscratch1     <= 32'h0;
        read_data     <= 32'h0;
    end else begin
        if (write_en & csr_sel) begin
            case (csr_addr)
                `M_CSR_MVENDORID_ADDR:    mvendorid <= write_data;
                `M_CSR_MARCHID_ADDR:
                `M_CSR_AMIMPID_ADDR:
                `M_CSR_AMHARTID_ADDR:
                `M_CSR_MCONFIGPTR_ADDR:

            endcase
        end else begin
            case (csr_addr)
                `M_CSR_MVENDORID_ADDR:    read__data <= mvendorid;
                `M_CSR_MARCHID_ADDR:
                `M_CSR_AMIMPID_ADDR:
                `M_CSR_AMHARTID_ADDR:
                `M_CSR_MCONFIGPTR_ADDR:
            endcase
        end
    end

// output the control bus signal
assign ctrl_bus = {mie[25], misa[16]};

endmodule

//--------------------------------------------------------------------------
