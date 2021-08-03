module dma_save_haddr #(parameter wbus = 32) (
    input  logic                i_clk,
    input  logic                i_nreset,

    input  logic                i_init,
    input  logic [wbus-1:0]     i_init_val,
    input  logic                i_refresh,
    input  logic [wbus-1:0]     i_nxt_val,
    output logic [wbus-1:0]     o_val
);

always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) o_val <= {wbus{1'b0}};
    else if (i_init) o_val <= i_init_val;
    else if (i_refresh) o_val <= i_nxt_val;

endmodule
