module dma_reg_sel #(parameter numb_ch = 1) (
    input  logic [numb_ch-1:0]          i_x,
    output logic [$clog2(numb_ch):0]    i_q
);

always_comb begin
    i_q = 0;
    for (int i = 0; i < numb_ch; ++i) begin
        if (i_x[i]) i_q = i;
    end
end

endmodule
