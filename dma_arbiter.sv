module dma_arbiter #(parameter numb_ch = 1, fifo_size = 5) (
    input  logic                            i_clk,
    input  logic                            i_nreset,

    input  logic                            i_en_stream[numb_ch-1:0],
    input  logic [1:0]                      i_size[numb_ch-1:0],
    input  logic [1:0]                      i_burst[numb_ch-1:0],
    input  logic [17:0]                     i_ndt[numb_ch-1:0],
    input  logic [1:0]                      i_pl[numb_ch-1:0],
    input  logic                            i_relevance_req[numb_ch-1:0],
    input  logic                            i_requests[numb_ch-1:0],
    input  logic [fifo_size:0]              i_left_bytes[numb_ch-1:0],
    input  logic                            i_master_ready,
    output logic [dma_log2(numb_ch)-1:0]    o_stream_sel,
    output logic                            o_master_en
);

genvar ch;

enum logic [1:0] {st_idle, st_swap, st_work} state_list;
enum logic [1:0] {bytew, hword, word} size_list;
enum logic [1:0] {single, inc4, inc8, inc16} burst_list;

logic               change_state;
logic [1:0]         nxt_state;
logic [1:0]         cur_state;
logic               state_swap;
logic               state_work;

logic               sw_swap_to_work;
logic               sw_work_to_swap;

logic                           en_stream_or;
logic [11:0]                    fifo_left_beats[numb_ch-1:0];
logic                           fifo_residual_data[numb_ch-1:0];
logic                           fifo_left_data[numb_ch-1:0];
logic [1:0]                     pl[numb_ch-1:0];
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
            if (sw_swap_to_work) nxt_state = st_work;
            else nxt_state = st_swap;
        end
        st_work: begin
            if (sw_work_to_swap) nxt_state = st_swap;
            else nxt_state = st_work;
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

always_comb begin: arbiter_fifo_left_beats
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        case (i_size[ch_cnt])
            bytew: fifo_left_beats[ch_cnt] = i_left_bytes[ch_cnt];
            hword: fifo_left_beats[ch_cnt] = i_left_bytes[ch_cnt] >> 1;
            default: fifo_left_beats[ch_cnt] = i_left_bytes[ch_cnt] >> 2;
        endcase
        case (i_burst[ch_cnt])
            single: fifo_left_beats[ch_cnt] = fifo_left_beats[ch_cnt];
            inc4: fifo_left_beats[ch_cnt] = fifo_left_beats[ch_cnt] << 2;
            inc8: fifo_left_beats[ch_cnt] = fifo_left_beats[ch_cnt] << 4;
            default: fifo_left_beats[ch_cnt] = fifo_left_beats[ch_cnt] << 8;
        endcase
    end
end

always_comb begin: arbiter_fifo_residual_data
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        case (i_burst[ch_cnt])
            single: fifo_residual_data[ch_cnt] = i_ndt[ch_cnt] < i_size[ch_cnt];
            inc4: fifo_residual_data[ch_cnt] = i_ndt[ch_cnt] < (i_size[ch_cnt] << 2);
            inc8: fifo_residual_data[ch_cnt] = i_ndt[ch_cnt] < (i_size[ch_cnt] << 4);
            default: fifo_residual_data[ch_cnt] = i_ndt[ch_cnt] < (i_size[ch_cnt] << 8);
        endcase
    end
end

always_comb begin: arbiter_fifo_left_data
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        fifo_left_data[ch_cnt] = |fifo_left_beats[ch_cnt] | fifo_residual_data[ch_cnt];
    end
end

always_comb begin: arbiter_pl
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        if (i_en_stream[ch_cnt]) begin
            pl[ch_cnt] = i_pl[ch_cnt];
        end
        else begin
            pl[ch_cnt] = 0;
        end
    end
end

always_comb begin: arbiter_nxt_stream
    nxt_stream = o_stream_sel;
    for (int ch_cnt = numb_ch - 1; ch_cnt >= 0; --ch_cnt) begin
        if (i_en_stream[ch_cnt] && pl[ch_cnt] >= pl[nxt_stream] && fifo_left_data[ch_cnt] && (i_requests[ch_cnt] || !i_relevance_req[ch_cnt])) begin
            nxt_stream = ch_cnt;
        end
    end
end

assign change_stream = state_swap & i_master_ready;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) o_stream_sel <= 'd0;
    else if (change_stream) o_stream_sel <= nxt_stream;

assign o_master_en = state_work;

endmodule
