module dma_swap #(parameter wbus = 1) (
    input  logic            i_en0,
    input  logic            i_en1,
    input  logic [wbus-1:0] i_val0,
    input  logic [wbus-1:0] i_val1,
    output logic [wbus-1:0] o_val
    );

assign o_val = i_en0 ? i_val0:
               i_en1 ? i_val1: 'd0;

endmodule
