/*
 *  rv32Core -- an out-of-order superscalar processor based on the RV32 instruction set.
 *
 *  Author     : Hongbin Wu
 *  Create Date: 2021.12.22
 *  Description: data memory module of rv32Core
 */
`include "cofigure.h"

module dataMEM
(
    input                              i_clk,
    input                              i_resetn,
    input                              valid,
    input                              we,
    input      [`DMEM_ADDRW-1:0] addr,
    input      [31:0]             wdata,
    output                             ready,
    output reg [31:0]             rdata
);

    reg [31:0] datamem [0:`DMEM_DEPTH-1];
    
    initial begin
        if (DATA_MEM_INIT_FILE != "") begin
            $display("Preloading instruction memory data from %s", DATA_MEM_INIT_FILE);
            $readmemh(DATA_MEM_INIT_FILE, datamem);
        end
    end    
    
    reg wr_ready;
    assign ready = valid && wr_ready;
    
    always @(posedge i_clk or negedge i_resetn) begin
        if(~i_resetn) begin
            ready <= 1'b0;
            rdata <= 32'b0;
        end else begin
            if(valid) begin
                if(we) begin
                    wr_ready      <= 1'b1;
                    rdata         <= 32'b0;
                    datamem[addr] <= wdata;                    
                end else begin
                    wr_ready      <= 1'b1;
                    rdata         <= datamem[addr];
                end
            end else begin
                wr_ready  <= 1'b0;
            end
        end
   end
endmodule //dataMEM
