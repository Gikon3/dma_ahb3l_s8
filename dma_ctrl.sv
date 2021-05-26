module dma_ctrl #(parameter fifo_size_exp = 5) (
    input  logic        i_clk,
    input  logic        i_nreset,

    // coonnect to config registers
    input  logic [1:0]              i_dir,
    input  logic                    i_dir_mbus_to_pbus,
    input  logic                    i_dir_pbus_to_mbus,
    input  logic                    i_dmdis,    // direct mode disable
    input  logic [1:0]              i_msize,
    input  logic [1:0]              i_psize,
    output logic                    o_ndtr_decr,
    output logic                    o_ndtr_src_decr,
    output logic                    o_smaller_size,

    // connect to memory master
    output logic                    o_write_m,
    output logic [31:0]             o_wdata_m,
    input  logic [31:0]             i_rdata_m,
    input  logic                    i_setget_data_m,
    input  logic                    i_smaller_size_m,

    // connect to peripheral master
    output logic                    o_relevance_req,
    output logic                    o_write_p,
    output logic [31:0]             o_wdata_p,
    input  logic [31:0]             i_rdata_p,
    input  logic                    i_setget_data_p,
    input  logic                    i_smaller_size_p,

    // connect to FIFO
    output logic [1:0]              o_numb_bytes_put,
    output logic [1:0]              o_numb_bytes_pull,
    output logic                    o_fifo_put,
    output logic                    o_fifo_pull,
    output logic [31:0]             o_fifo_wdata,
    input  logic [31:0]             i_fifo_rdata,
    input  logic [fifo_size_exp:0]  i_fifo_left_put,
    input  logic [fifo_size_exp:0]  i_fifo_left_pull,

    // connect to buffer reg
    output logic                    o_buf_put,
    output logic                    o_buf_pull,
    output logic [31:0]             o_buf_wdata,
    input  logic [31:0]             i_buf_rdata
);

enum logic [1:0] {periph_to_mem, mem_to_periph, mem_to_mem} dir_list;

localparam wbus = 32;

logic   dir_per_to_mem;
logic   dir_mem_to_per;

logic   put_data;
logic   pull_data;

assign dir_per_to_mem = i_dir == periph_to_mem;
assign dir_mem_to_per = i_dir == mem_to_periph;

assign put_data = i_dir_pbus_to_mbus ? i_setget_data_p:
                  i_dir_mbus_to_pbus ? i_setget_data_m: 1'b0;
assign pull_data = i_dir_pbus_to_mbus ? i_setget_data_m:
                   i_dir_mbus_to_pbus ? i_setget_data_p: 1'b0;

assign o_write_m = i_dir_pbus_to_mbus;
assign o_wdata_m = i_dir_pbus_to_mbus && i_dmdis ? i_fifo_rdata:
                   i_dir_pbus_to_mbus && !i_dmdis ? i_buf_rdata: 'd0;

assign o_relevance_req = dir_per_to_mem | dir_mem_to_per;
assign o_write_p = i_dir_mbus_to_pbus;
assign o_wdata_p = i_dir_mbus_to_pbus && i_dmdis ? i_fifo_rdata:
                   i_dir_mbus_to_pbus && !i_dmdis ? i_buf_rdata: 'd0;

assign o_numb_bytes_put = i_dir_pbus_to_mbus ? i_psize:
                          i_dir_mbus_to_pbus ? i_msize: 2'h0;
assign o_numb_bytes_pull = i_dir_pbus_to_mbus ? i_msize:
                           i_dir_mbus_to_pbus ? i_psize: 2'h0;
assign o_fifo_put = put_data & i_dmdis;
assign o_fifo_pull = pull_data & i_dmdis;
assign o_fifo_wdata = i_dir_pbus_to_mbus ? i_rdata_p:
                      i_dir_mbus_to_pbus ? i_rdata_m: 'd0;

assign o_ndtr_decr = pull_data;
assign o_ndtr_src_decr = put_data;

// buffer for direct mode
assign o_buf_put = put_data & !i_dmdis;
assign o_buf_pull = pull_data & !i_dmdis;
assign o_buf_wdata = o_fifo_wdata;

assign o_smaller_size = i_dir_pbus_to_mbus ? i_smaller_size_m:
                        i_dir_mbus_to_pbus ? i_smaller_size_p: 1'b0;

endmodule
