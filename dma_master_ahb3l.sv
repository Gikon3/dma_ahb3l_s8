module dma_master_ahb3l #(parameter fifo_size_exp = 5) (
    input  logic                    i_hclk,
    input  logic                    i_hnreset,

    // AHB connect to slave
    output logic [31:0]             o_haddr,
    output logic                    o_hwrite,
    output logic [2:0]              o_hsize,
    output logic [2:0]              o_hburst,
    output logic [3:0]              o_hprot,
    output logic [1:0]              o_htrans,
    output logic [31:0]             o_hwdata,
    output logic                    o_hmastlock,

    input  logic                    i_hready,
    input  logic                    i_hresp,
    input  logic [31:0]             i_hrdata,

    // connect to control module
    input  logic                    i_enable,
    input  logic                    i_dis_stream,
    input  logic                    i_init_haddr,
    input  logic                    i_request,
    input  logic                    i_relevance_req,
    input  logic [31:0]             i_addr,
    input  logic                    i_write,
    input  logic                    i_inc_en,
    input  logic                    i_fix_inc,
    input  logic [1:0]              i_burst,
    input  logic [1:0]              i_size,
    input  logic [31:0]             i_wdata,
    output logic [31:0]             o_rdata,
    input  logic [17:0]             i_ndtr,
    input  logic [fifo_size_exp:0]  i_fifo_left_bytes,
    output logic                    o_setget_data,
    output logic                    o_last_trnsct,
    output logic                    o_smaller_size,

    // flags
    output logic                    o_idle,
    output logic                    o_fail
);

localparam wbus = 32;
localparam wbyte = 8;

enum logic [1:0] {bytew, hword, word} size_list;
enum logic [2:0] {h_bytew, h_hword, h_word} hsize_list;
enum logic [2:0] {single, inc, wrap4, inc4, wrap8, inc8, wrap16, inc16} hburst_list;
enum logic [1:0] {idle, busy, nonseq, seq} htrans_list;
enum logic [2:0] {st_idle, st_addr, st_addata, st_data, st_error} state_list;

logic               burst_single;
logic               burst_inc4;
logic               burst_inc8;
logic               burst_inc16;

logic               hresp_error;
logic               ndtr_left_0_beats;
logic               ndtr_left_1_beats;
logic               ndtr_left_2_beats;
logic               ndtr_left_1_and_more_beats;
logic               ndtr_left_2_and_more_beats;
logic               ndtr_left_1p1_and_less_trans;
logic               fifo_left_1_beats;
logic               fifo_left_2_beats;
logic               fifo_more_left_2_beats;
logic               fifo_left_2_and_more_beats;
logic               fifo_left_1_and_more_trans;
logic               fifo_left_1p1_and_more_trans;
//     logic               fifo_left_more_2_trans;

logic               fifo_flush;
logic               dis_stream;
logic               small_data;

logic               sw_idle_to_addr;
logic               sw_addr_to_addata;
logic               sw_addata_to_data;
logic               sw_addr_to_data;
logic               sw_data_to_addr;
logic               sw_to_idle;
logic               sw_data_to_idle;
logic               sw_to_error;
logic               sw_addr_to_error;
logic               sw_addata_to_error;
logic               sw_data_to_error;
logic               sw_error_to_idle;

logic [2:0]         nxt_state;
logic [2:0]         cur_state;
logic               state_idle;
logic               state_addr;
logic               state_addata;
logic               state_data;
logic               state_error;

logic               count_beat_rst;
logic               count_beat_en;
logic [3:0]         count_beat;
logic               end_beat;

logic               residual_data_clr;
logic               residual_data_set;
logic               residual_data;

logic               haddr_reload;
logic               haddr_inc;
logic [wbus-1:0]    nxt_haddr;
logic               start_work;
logic               haddr_delay_set;
logic [wbus-1:0]    haddr_delay;
logic               haddr_refresh;

assign burst_single = i_burst == 2'h0 || residual_data;
assign burst_inc4 = i_burst == 2'h1 && !residual_data;
assign burst_inc8 = i_burst == 2'h2 && !residual_data;
assign burst_inc16 = i_burst == 2'h3 && !residual_data;

assign hresp_error = i_hresp;

assign ndtr_left_0_beats = i_ndtr == 'd0;
assign ndtr_left_1_beats = i_ndtr == 'd1;
assign ndtr_left_2_beats = i_ndtr == 'd2;
assign ndtr_left_1_and_more_beats = ~ndtr_left_0_beats;
assign ndtr_left_2_and_more_beats = ~ndtr_left_0_beats & ~ndtr_left_1_beats;

always_comb begin: decode_ndtr_left_1p1_and_less_trans
    if (burst_single) begin
        ndtr_left_1p1_and_less_trans = i_ndtr <= 'd2;
    end
    else if (burst_inc4) begin
        ndtr_left_1p1_and_less_trans = i_ndtr <= 'd5;
    end
    else if (burst_inc8) begin
        ndtr_left_1p1_and_less_trans = i_ndtr <= 'd9;
    end
    else begin  // burst_inc16
        ndtr_left_1p1_and_less_trans = i_ndtr <= 'd17;
    end
end

assign fifo_left_1_beats = i_size == bytew ? i_fifo_left_bytes == 'd1:
                           i_size == hword ? i_fifo_left_bytes[fifo_size_exp:1] == 'd1:
                           i_fifo_left_bytes[fifo_size_exp:2] == 'd1;
assign fifo_left_2_beats = i_size == bytew ? i_fifo_left_bytes == 'd2:
                           i_size == hword ? i_fifo_left_bytes[fifo_size_exp:1] == 'd2:
                           i_fifo_left_bytes[fifo_size_exp:2] == 'd2;
assign fifo_more_left_2_beats = i_size == bytew ? i_fifo_left_bytes > 'd2:
                                i_size == hword ? i_fifo_left_bytes[fifo_size_exp:1] > 'd2:
                                i_fifo_left_bytes[fifo_size_exp:2] > 'd2;
assign fifo_left_2_and_more_beats = fifo_left_2_beats | fifo_more_left_2_beats;

always_comb begin: decode_fifo_left_1_and_more_trans
    case (i_size)
        bytew: begin
            if (burst_single) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd1;
            end
            else if (burst_inc4) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd4;
            end
            else if (burst_inc8) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd8;
            end
            else begin  // burst_inc16
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd16;
            end
        end
        hword: begin
            if (burst_single) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd2;
            end
            else if (burst_inc4) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd8;
            end
            else if (burst_inc8) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd16;
            end
            else begin  // burst_inc16
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd32;
            end
        end
        default: begin
            if (burst_single) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd4;
            end
            else if (burst_inc4) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd16;
            end
            else if (burst_inc8) begin
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd32;
            end
            else begin  // burst_inc16
                fifo_left_1_and_more_trans = i_fifo_left_bytes >= 'd64;
            end
        end
    endcase
end

always_comb begin: decode_fifo_left_1p1_and_more_trans
    case (i_size)
        bytew: begin
            if (burst_single) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd2;
            end
            else if (burst_inc4) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd5;
            end
            else if (burst_inc8) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd9;
            end
            else begin  // burst_inc16
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd17;
            end
        end
        hword: begin
            if (burst_single) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd4;
            end
            else if (burst_inc4) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd10;
            end
            else if (burst_inc8) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd18;
            end
            else begin  // burst_inc16
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd34;
            end
        end
        default: begin
            if (burst_single) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd8;
            end
            else if (burst_inc4) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd20;
            end
            else if (burst_inc8) begin
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd36;
            end
            else begin  // burst_inc16
                fifo_left_1p1_and_more_trans = i_fifo_left_bytes >= 'd68;
            end
        end
    endcase
end

//     always_comb begin: decode_fifo_left_more_2_trans
//         case (i_size)
//             bytew: begin
//                 if (burst_single) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd2;
//                 end
//                 else if (burst_inc4) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd8;
//                 end
//                 else if (burst_inc8) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd16;
//                 end
//                 else begin  // burst_inc16
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd32;
//                 end
//             end
//             hword: begin
//                 if (burst_single) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd4;
//                 end
//                 else if (burst_inc4) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd16;
//                 end
//                 else if (burst_inc8) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd32;
//                 end
//                 else begin  // burst_inc16
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd64;
//                 end
//             end
//             default: begin
//                 if (burst_single) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd8;
//                 end
//                 else if (burst_inc4) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd32;
//                 end
//                 else if (burst_inc8) begin
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd64;
//                 end
//                 else begin  // burst_inc16
//                     fifo_left_more_2_trans = i_fifo_left_bytes > 'd128;
//                 end
//             end
//         endcase
//     end

assign fifo_flush = (i_dis_stream & i_write & ~i_relevance_req) | ~i_dis_stream;
assign dis_stream = i_dis_stream & ~i_write & end_beat;
assign small_data = (fifo_left_2_beats | ndtr_left_2_beats) & (end_beat | i_relevance_req);

// fsm
assign sw_idle_to_addr = state_idle & i_enable & fifo_left_1_and_more_trans & ndtr_left_1_and_more_beats &
    (i_request | ~i_relevance_req) & fifo_flush;
assign sw_addr_to_addata = state_addr & i_hready &
    fifo_left_2_and_more_beats & ndtr_left_2_and_more_beats & (~end_beat | ~i_relevance_req);
assign sw_addata_to_data = state_addata & i_hready & (small_data | (end_beat & i_relevance_req) |
    (end_beat & ~fifo_left_1_and_more_trans) | (end_beat & ndtr_left_1p1_and_less_trans) | dis_stream);
assign sw_addr_to_data = state_addr & i_hready &
    (fifo_left_1_beats | ndtr_left_1_beats | (end_beat & i_relevance_req) | dis_stream);
assign sw_data_to_addr = state_data & i_enable & i_hready & /*fifo_left_more_2_trans*/fifo_left_1p1_and_more_trans &
    ndtr_left_2_and_more_beats & (i_request | ~i_relevance_req) & fifo_flush;
assign sw_to_idle = ~i_enable;
assign sw_data_to_idle = i_hready;
assign sw_to_error = hresp_error;
assign sw_addr_to_error = state_addr & sw_to_error;
assign sw_addata_to_error = state_addata & sw_to_error;
assign sw_data_to_error = state_data & sw_to_error;
assign sw_error_to_idle = sw_to_idle | i_hready;

always_comb begin: fsm_master
    case (cur_state)
        st_idle: begin
            if (sw_idle_to_addr) begin
                nxt_state = st_addr;
            end
            else begin
                nxt_state = st_idle;
            end
        end
        st_addr: begin
            if (sw_addr_to_error) begin
                nxt_state = st_error;
            end
            else if (sw_addr_to_addata) begin
                nxt_state = st_addata;
            end
            else if (sw_addr_to_data) begin
                nxt_state = st_data;
            end
            else begin
                nxt_state = st_addr;
            end
        end
        st_addata: begin
            if (sw_addata_to_error) begin
                nxt_state = st_error;
            end
            else if (sw_addata_to_data) begin
                nxt_state = st_data;
            end
            else begin
                nxt_state = st_addata;
            end
        end
        st_data: begin
            if (sw_data_to_error) begin
                nxt_state = st_error;
            end
            else if (sw_data_to_addr) begin
                nxt_state = st_addr;
            end
            else if (sw_data_to_idle) begin
                nxt_state = st_idle;
            end
            else begin
                nxt_state = st_data;
            end
        end
        st_error: begin
            if (sw_error_to_idle) begin
                nxt_state = st_idle;
            end
            else begin
                nxt_state = st_error;
            end
        end
        default: begin
            nxt_state = st_idle;
        end
    endcase
end

always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) cur_state <= st_idle;
    else cur_state <= nxt_state;

assign state_idle = (cur_state == st_idle);
assign state_addr = (cur_state == st_addr);
assign state_addata = (cur_state == st_addata);
assign state_data = (cur_state == st_data);
assign state_error = (cur_state == st_error);

// calculate beats
assign count_beat_rst = state_idle | ((state_data | end_beat) & i_hready);
assign count_beat_en = (state_addr | state_addata) & i_hready;
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) count_beat <= 4'd0;
    else if (count_beat_rst) count_beat <= 4'd0;
    else if (count_beat_en) count_beat <= count_beat + 4'd1;

assign end_beat = burst_single ? count_beat == 4'd0:
                  burst_inc4 ? count_beat == 4'd3:
                  burst_inc8 ? count_beat == 4'd7:
                  burst_inc16 ? count_beat == 4'd15: 1'b0;

// sending residual data
assign residual_data_clr = ~i_enable | ndtr_left_0_beats;
assign residual_data_set = i_hready & (((state_idle | (state_addata & end_beat)) & ndtr_left_1p1_and_less_trans) |
    ((sw_addata_to_data | sw_addata_to_data | state_idle) & i_dis_stream));
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) residual_data <= 1'b0;
    else if (residual_data_clr) residual_data <= 1'b0;
    else if (residual_data_set) residual_data <= 1'b1;

// address increment
assign haddr_reload = (state_idle & start_work) |
    (sw_addr_to_data & ndtr_left_1_beats) | (sw_addata_to_data & ndtr_left_2_beats);
assign haddr_inc = (state_addr | state_addata) & i_inc_en & i_hready;
always_comb begin: next_haddress
    if (haddr_reload) begin
        nxt_haddr = i_addr;
    end
    else if (haddr_inc) begin
        if (i_fix_inc) begin
            nxt_haddr = o_haddr + 32'h4;
        end
        else begin
            case (i_size)
                bytew: begin
                    nxt_haddr = o_haddr + 32'h1;
                end
                hword: begin
                    nxt_haddr = o_haddr + 32'h2;
                end
                word: begin
                    nxt_haddr = o_haddr + 32'h4;
                end
                default: nxt_haddr = o_haddr;
            endcase
        end
    end
    else begin
        nxt_haddr = o_haddr;
    end
end

assign start_work = i_init_haddr;

// FIFO output
assign haddr_delay_set = (state_addr | state_addata) & i_hready;
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) haddr_delay <= 32'd0;
    else if (haddr_delay_set) haddr_delay <= o_haddr;

always_comb begin: rdata_decode
    case (i_size)
        bytew: begin
            if (haddr_delay[1:0] == 2'h0) begin
                o_rdata = {{(wbus-wbyte){1'b0}}, i_hrdata[7:0]};
            end
            else if (haddr_delay[1:0] == 2'h1) begin
                o_rdata = {{(wbus-wbyte){1'b0}}, i_hrdata[15:8]};
            end
            else if (haddr_delay[1:0] == 2'h2) begin
                o_rdata = {{(wbus-wbyte){1'b0}}, i_hrdata[23:16]};
            end
            else begin
                o_rdata = {{(wbus-wbyte){1'b0}}, i_hrdata[31:24]};
            end
        end
        hword: begin
            if (haddr_delay[1:0] == 2'h0) begin
                o_rdata = {{(wbus-wbyte*2){1'b0}}, i_hrdata[15:0]};
            end
            else begin
                o_rdata = {{(wbus-wbyte*2){1'b0}}, i_hrdata[31:16]};
            end
        end
        default: begin
            o_rdata = i_hrdata;
        end
    endcase
end

assign o_setget_data = (state_addata | state_data) & i_hready;

// AHB output
assign o_hmastlock = state_addr | state_addata;

assign haddr_refresh = haddr_reload | state_addr | state_addata;
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) o_haddr <= {wbus{1'b0}};
    else if (haddr_refresh) o_haddr <= nxt_haddr;

assign o_hwrite = (state_addr | state_addata) & i_write;
assign o_hsize = (state_addr | state_addata) ? {1'b0, i_size}: 3'd0;

always_comb begin: hburst_decode
    if ((state_addr || state_addata) && !residual_data) begin
        casex ({burst_inc16, burst_inc8, burst_inc4, burst_single})
            4'bxxx1: begin
                o_hburst = single;
            end
            4'bxx10: begin
                o_hburst = inc4;
            end
            4'bx100: begin
                o_hburst = inc8;
            end
            4'b1000: begin
                o_hburst = inc16;
            end
            default: o_hburst = single;
        endcase
    end
    else begin
        o_hburst = single;
    end
end

assign o_hprot = 4'h0;
assign o_htrans = (state_addr || state_addata) && (count_beat == 4'd0 || residual_data) ? nonseq:
                  (state_addr || state_addata) ? seq: idle;
assign o_hwdata = i_size == bytew ? {4{i_wdata[wbyte-1:0]}}:
                  i_size == hword ? {2{i_wdata[wbyte*2-1:0]}}: i_wdata;

// swap address output
assign o_last_trnsct = ~i_init_haddr & haddr_reload;

// low data remainder flag
always_comb begin: decode_smaller_size
    if (state_idle) begin
        case (i_size)
            bytew: begin
                o_smaller_size = (i_fifo_left_bytes < 'd1);
            end
            hword: begin
                o_smaller_size = (i_fifo_left_bytes < 'd2);
            end
            default: begin
                o_smaller_size = (i_fifo_left_bytes < 'd4);
            end
        endcase
    end
    else begin
        o_smaller_size = 1'b0;
    end
end

// status output
assign o_idle = state_idle;
assign o_fail = hresp_error | state_error;

endmodule
