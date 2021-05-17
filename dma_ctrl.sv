module dma_ctrl #(parameter fifo_size_exp = 5) (
    input  logic        i_clk,
    input  logic        i_nreset,

    // coonnect to config registers
    input  logic                    i_enable,
    input  logic [15:0]             i_ndtr,
    input  logic [17:0]             i_ndtr_src,
    input  logic [1:0]              i_dir,
    input  logic                    i_dmdis,    // direct mode disable
    input  logic [1:0]              i_msize,
    input  logic [1:0]              i_psize,
    output logic                    o_ndtr_decr,
    output logic                    o_ndtr_src_decr,
//     output logic                    o_fail,
    output logic                    o_dmbuf_empty,
    output logic                    o_smaller_size,

    // connect to memory master
    output logic                    o_enable_m,
    output logic                    o_write_m,
    output logic [31:0]             o_wdata_m,
    input  logic [31:0]             i_rdata_m,
    output logic [17:0]             o_ndtr_m,
    output logic [fifo_size_exp:0]  o_fifo_left_bytes_m,
    input  logic                    i_setget_data_m,
    input  logic                    i_smaller_size_m,
//     input  logic                    i_fail_m,

    // connect to peripheral master
    output logic                    o_enable_p,
    output logic                    o_relevance_req,
    output logic                    o_write_p,
    output logic [31:0]             o_wdata_p,
    input  logic [31:0]             i_rdata_p,
    output logic [17:0]             o_ndtr_p,
    output logic [fifo_size_exp:0]  o_fifo_left_bytes_p,
    input  logic                    i_setget_data_p,
    input  logic                    i_smaller_size_p,
//     input  logic                    i_fail_p,

    // connect to FIFO
    output logic [1:0]              o_numb_bytes_put,
    output logic [1:0]              o_numb_bytes_pull,
    output logic                    o_fifo_put,
    output logic                    o_fifo_pull,
    output logic [31:0]             o_fifo_wdata,
    input  logic [31:0]             i_fifo_rdata,
    input  logic [fifo_size_exp:0]  i_fifo_left_put,
    input  logic [fifo_size_exp:0]  i_fifo_left_pull
);

enum logic [1:0] {periph_to_mem, mem_to_periph, mem_to_mem} dir_list;
enum logic [1:0] {bytew, hword, word} size_list;

localparam wbus = 32;

logic dir_per_to_mem;
logic dir_mem_to_per;
logic dir_mem_to_mem;
logic pbus_to_mbus;
logic mbus_to_pbus;

logic [2:0] numb_bytes_msize;
logic [2:0] numb_bytes_psize;
logic       put_data;
logic       pull_data;

logic               wren_buffer;
logic [wbus-1:0]    buffer;
logic               rden_buffer;
logic               buffer_full;
logic               buffer_empty;

assign dir_per_to_mem = i_dir == periph_to_mem;
assign dir_mem_to_per = i_dir == mem_to_periph;
assign dir_mem_to_mem = i_dir == mem_to_mem;
assign pbus_to_mbus = dir_per_to_mem | dir_mem_to_mem;
assign mbus_to_pbus = dir_mem_to_per;

always_comb begin: decoder_msize
    case (i_msize)
        bytew: begin
            numb_bytes_msize = 'd1;
        end
        hword: begin
            numb_bytes_msize = 'd2;
        end
        default: begin
            numb_bytes_msize = 'd4;
        end
    endcase
end

always_comb begin: decoder_psize
    case (i_psize)
        bytew: begin
            numb_bytes_psize = 'd1;
        end
        hword: begin
            numb_bytes_psize = 'd2;
        end
        default: begin
            numb_bytes_psize = 'd4;
        end
    endcase
end

assign put_data = pbus_to_mbus ? i_setget_data_p:
                  mbus_to_pbus ? i_setget_data_m: 1'b0;
assign pull_data = pbus_to_mbus ? i_setget_data_m:
                   mbus_to_pbus ? i_setget_data_p: 1'b0;

assign o_enable_m = i_enable;
assign o_write_m = pbus_to_mbus;
assign o_wdata_m = pbus_to_mbus && i_dmdis ? i_fifo_rdata:
                   pbus_to_mbus && !i_dmdis ? buffer: 'd0;
assign o_ndtr_m = pbus_to_mbus ? {2'd0, i_ndtr}:
                    mbus_to_pbus ? i_ndtr_src: 18'd0;
always_comb begin: decode_left_bytes_memory
    if (i_dmdis) begin
        if (pbus_to_mbus) begin
            o_fifo_left_bytes_m = i_fifo_left_pull;
        end
        else if (mbus_to_pbus) begin
            o_fifo_left_bytes_m = i_fifo_left_put;
        end
        else begin
            o_fifo_left_bytes_m = 'd0;
        end
    end
    else begin
        if ((pbus_to_mbus && buffer_full) || (mbus_to_pbus && buffer_empty)) begin
            o_fifo_left_bytes_m = {{(fifo_size_exp-2){1'b0}}, numb_bytes_msize};
        end
        else begin
            o_fifo_left_bytes_m = 'd0;
        end
    end
end

assign o_enable_p = i_enable;
assign o_relevance_req = dir_per_to_mem | dir_mem_to_per;
assign o_write_p = mbus_to_pbus;
assign o_wdata_p = mbus_to_pbus && i_dmdis ? i_fifo_rdata:
                   mbus_to_pbus && !i_dmdis ? buffer: 'd0;
assign o_ndtr_p = pbus_to_mbus ? i_ndtr_src:
                  mbus_to_pbus ? {2'd0, i_ndtr}: 18'd0;
always_comb begin: decode_left_bytes_periph
    if (i_dmdis) begin
        if (o_write_m) begin
            o_fifo_left_bytes_p = i_fifo_left_put;
        end
        else if (mbus_to_pbus) begin
            o_fifo_left_bytes_p = i_fifo_left_pull;
        end
        else begin
            o_fifo_left_bytes_p = 'd0;
        end
    end
    else begin
        if ((o_write_m && buffer_empty) || (mbus_to_pbus && buffer_full)) begin
            o_fifo_left_bytes_p = {{(fifo_size_exp-2){1'b0}}, numb_bytes_psize};
        end
        else begin
            o_fifo_left_bytes_p = 'd0;
        end
    end
end

assign o_numb_bytes_put = pbus_to_mbus ? i_psize:
                          mbus_to_pbus ? i_msize: 2'h0;
assign o_numb_bytes_pull = pbus_to_mbus ? i_msize:
                           mbus_to_pbus ? i_psize: 2'h0;
assign o_fifo_put = put_data & i_dmdis;
assign o_fifo_pull = pull_data & i_dmdis;
assign o_fifo_wdata = pbus_to_mbus ? i_rdata_p:
                      mbus_to_pbus ? i_rdata_m: 'd0;

assign o_ndtr_decr = pull_data;
assign o_ndtr_src_decr = put_data;

// assign o_fail = i_fail_m | i_fail_p;

// buffer for direct mode
assign wren_buffer = put_data & !i_dmdis;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) buffer <= 'd0;
    else if (wren_buffer) buffer <= o_fifo_wdata;

assign rden_buffer = pull_data & !i_dmdis;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) buffer_full <= 1'b0;
    else if (wren_buffer) buffer_full <= 1'b1;
    else if (rden_buffer) buffer_full <= 1'b0;

assign buffer_empty = !buffer_full;
assign o_dmbuf_empty = buffer_empty;
assign o_smaller_size = pbus_to_mbus ? i_smaller_size_m:
                        mbus_to_pbus ? i_smaller_size_p: 1'b0;

endmodule
