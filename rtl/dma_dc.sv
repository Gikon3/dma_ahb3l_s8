module dma_dc #(parameter size_addr = 1, size_en = 1, numb_out = 1) (
    input  logic [size_addr-1:0]    i_addr,
    input  logic [size_en-1:0]      i_en,
    output logic [size_en-1:0]      o_out[numb_out-1:0]
);

always_comb begin: decoder
    for (int i = 0; i < numb_out; ++i) begin
        if (i == i_addr) begin
            o_out[i] = i_en;
        end
        else begin
            o_out[i] = 'd0;
        end
    end
end

endmodule
