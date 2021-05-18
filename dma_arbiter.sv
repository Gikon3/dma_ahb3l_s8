module dma_arbiter #(parameter numb_ch = 1, fifo_size = 5) (
    input  logic                            i_clk,
    input  logic                            i_nreset,

    input  logic                            i_en_stream[numb_ch-1:0],
    input  logic [1:0]                      i_pl[numb_ch-1:0],
    input  logic                            i_relevance_req,
    input  logic                            i_requests[numb_ch-1:0],
    input  logic [fifo_size:0]              i_left_bytes[numb_ch-1:0],
    input  logic                            i_master_ready,
    output logic [dma_log2(numb_ch)-1:0]    o_stream_sel,
    output logic                            o_master_en
);

genvar ch;

enum logic [1:0] {st_swap, st_work} state_list;

logic               change_state;
logic [1:0]         nxt_state;
logic [1:0]         cur_state;
logic               state_swap;
logic               state_work;

logic               sw_swap_to_work;
logic               sw_work_to_swap;

logic                           en_stream_or;
logic                           fifo_not_empty_full[numb_ch-1:0];
logic [dma_log2(numb_ch)-1:0]   nxt_stream;
logic                           change_stream;

assign change_state = 1;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) cur_state <= 'd0;
    else if (change_state) cur_state <= nxt_state;

assign state_swap = cur_state == st_swap;
assign state_work = cur_state == st_work;

assign sw_swap_to_work = state_swap & change_stream & en_stream_or;
assign sw_work_to_swap = state_work & (o_stream_sel != nxt_stream);

always_comb begin: fsm_arbiter
    case (cur_state)
        st_swap: begin
            if (sw_swap_to_work) begin
                nxt_state = st_work;
            end
            else begin
                nxt_state = st_swap;
            end
        end
        st_work: begin
            if (sw_work_to_swap) begin
                nxt_state = st_swap;
            end
            else begin
                nxt_state = st_work;
            end
        end
        default: begin
            nxt_state = st_swap;
        end
    endcase
end

always_comb begin: arbiter_sw_idle_to_work_gen
    en_stream_or = 0;
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        en_stream_or |= i_en_stream[ch_cnt];
    end
end

always_comb begin: arbiter_fifo_not_empty
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        fifo_not_empty_full[ch_cnt] = |i_left_bytes[ch_cnt];
    end
end

always_comb begin: arbiter_nxt_stream
    nxt_stream = 0;
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        if (i_en_stream[ch_cnt] && fifo_not_empty_full[ch_cnt] && (i_requests[ch_cnt] || !i_relevance_req)) begin
            nxt_stream = ch_cnt;
            break;
        end
    end
end

assign change_stream = state_swap & i_master_ready;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) o_stream_sel <= 'd0;
    else if (change_stream) o_stream_sel <= nxt_stream;

assign o_master_en = state_work;

endmodule
