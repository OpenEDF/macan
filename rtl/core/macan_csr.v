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

    // Outputs
    output reg  [31:0] read_data,
    output wire [31:0] ctrl_bus
);

// Control and Status Register Memory Model: RISC-V defines a separate
// address space of 4096 Control and Status registers associated  with
// each hart. width = 32, depth = 4096
reg [31:0] csr_mem_reg[0:4095];

// initialize memory and register
initial begin
    csr_mem_reg[`M_CSR_MVENDORID_ADDR] = 32'h12345678;
end

// read write data from csr memory
always @(*) begin
    if (!rst_n) begin
        read_data <= 32'h0;
        ctrl_bus <= 32'h0;
    end else begin
        if (write_en) begin
            csr_mem_reg[csr_addr] <= write_data;
        end else begin
            read_data <= csr_mem_reg[csr_addr];
        end
    end
end

// output the control bus signal
assign ctrl_bus = {csr_mem_reg[`M_CSR_MVENDORID_ADDR][3:0]};

endmodule

//--------------------------------------------------------------------------
