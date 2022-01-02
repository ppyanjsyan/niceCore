/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Create Date: 2021.12.22
 *  Description: instruction memory module of rv32Core
 */
`include "configure.h"

module instMEM(
    input                        i_clk,
    input                        i_resetn,
    input                        valid,
    input      [`IMEM_ADDRW-1:0] raddr,
    output                       ready,
    output reg [32*4-1:0]        rinst
);
   
    reg [32*4-1:0] instmem [0:`IMEM_DEPTH-1];
    
    initial begin
        if (INST_MEM_INIT_FILE != "") begin
            $display("Preloading instruction memory data from %s", INST_MEM_INIT_FILE);
            $readmemh(INST_MEM_INIT_FILE, instmem);
        end
    end
    
    reg rd_ready;   
    assign ready = valid && rd_ready;
    
    always @(posedge i_clk or negedge i_resetn) begin
        if(~i_resetn) begin
            rd_ready <= 1'b0;
            rinst <= (32*4)'b0;
        end else begin
            if(valid) begin
                rd_ready <= 1'b1;
                rinst <= instmem[raddr];
            end else begin
                rd_ready <= 1'b0;
            end
        end
    end
endmodule //instMEM
