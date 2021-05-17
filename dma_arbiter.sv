module dma_arbiter #(parameter numb_ch = 1, size_exp = 5) (
    input  logic                            i_clk,
    input  logic                            i_nreset,

    input  logic                            i_en_stream[numb_ch-1:0],
    input  logic [1:0]                      i_pl[numb_ch-1:0],
    input  logic                            i_relevance_req,
    input  logic                            i_requests[numb_ch-1:0],
    input  logic [size_exp:0]               i_left_bytes[numb_ch-1:0],
    output logic [dma_log2(numb_ch)-1:0]    o_stream_sel
);

logic [numb_ch-1:0] fifo_not_empty;

always_comb begin: arbiter_fifo_not_empty
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        fifo_not_empty[ch_cnt] = |i_left_bytes[ch_cnt];
    end
end

always_comb begin: arbiter_decoder
    o_stream_sel = 0;
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        if (i_en_stream[ch_cnt] && fifo_not_empty[ch_cnt] && (i_requests[ch_cnt] | i_relevance_req)) begin
            o_stream_sel = ch_cnt;
            break;
        end
    end
end

endmodule
