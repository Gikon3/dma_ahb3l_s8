module dma_buf_reg #(parameter wbus = 32) (
    input  logic            i_clk,
    input  logic            i_nreset,

    input  logic [wbus-1:0] i_wdata,
    input  logic            i_put,
    input  logic            i_pull,
    output logic [wbus-1:0] o_rdata,
    output logic            o_empty,
    output logic            o_full
);

logic [wbus-1:0]    buffer;

always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) buffer <= 'd0;
    else if (i_put) buffer <= i_wdata;

always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) o_full <= 1'b0;
    else if (i_put) o_full <= 1'b1;
    else if (i_pull) o_full <= 1'b0;

assign o_rdata = buffer;
assign o_empty = ~o_full;

endmodule
