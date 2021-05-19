module dma_ahb3l_s8ch8 (
    input  logic        i_hclk,
    input  logic        i_hnreset,

    // AHB connection to master
    input  logic        i_hsel_s,
    input  logic [31:0] i_haddr_s,
    input  logic        i_hwrite_s,
    input  logic [2:0]  i_hsize_s,
    input  logic [1:0]  i_htrans_s,
    input  logic        i_hready_s,
    input  logic [31:0] i_hwdata_s,

    output logic        o_hreadyout_s,
    output logic        o_hresp_s,
    output logic [31:0] o_hrdata_s,

    // AHB connect to memory
    output logic [31:0] o_haddr_m,
    output logic        o_hwrite_m,
    output logic [2:0]  o_hsize_m,
    output logic [2:0]  o_hburst_m,
    output logic [3:0]  o_hprot_m,
    output logic [1:0]  o_htrans_m,
    output logic [31:0] o_hwdata_m,
    output logic        o_hmastlock_m,

    input  logic        i_hready_m,
    input  logic        i_hresp_m,
    input  logic [31:0] i_hrdata_m,

    // AHB connect to periph
    output logic [31:0] o_haddr_p,
    output logic        o_hwrite_p,
    output logic [2:0]  o_hsize_p,
    output logic [2:0]  o_hburst_p,
    output logic [3:0]  o_hprot_p,
    output logic [1:0]  o_htrans_p,
    output logic [31:0] o_hwdata_p,
    output logic        o_hmastlock_p,

    input  logic        i_hready_p,
    input  logic        i_hresp_p,
    input  logic [31:0] i_hrdata_p,

    // peripheral request
    input  logic [7:0]  i_req_s0,
    input  logic [7:0]  i_req_s1,
    input  logic [7:0]  i_req_s2,
    input  logic [7:0]  i_req_s3,
    input  logic [7:0]  i_req_s4,
    input  logic [7:0]  i_req_s5,
    input  logic [7:0]  i_req_s6,
    input  logic [7:0]  i_req_s7,
    input  logic [7:0]  i_ltr_s0,
    input  logic [7:0]  i_ltr_s1,
    input  logic [7:0]  i_ltr_s2,
    input  logic [7:0]  i_ltr_s3,
    input  logic [7:0]  i_ltr_s4,
    input  logic [7:0]  i_ltr_s5,
    input  logic [7:0]  i_ltr_s6,
    input  logic [7:0]  i_ltr_s7,

    // interrupt
    output logic        o_interrupt[7:0]
);

localparam numb_ch = 8;
localparam fifo_size = 4;
localparam wbus = 32;
localparam wbyte = 8;

genvar ch;

logic [7:0]  requests[7:0];
logic [7:0]  ltrnscts[7:0];

logic [wbus-1:0]        regs_addr;
logic                   regs_read_en;
logic                   regs_write_en;
logic [wbus/wbyte-1:0]  regs_byte_strobe;
logic [wbus-1:0]        regs_wdata;
logic [wbus-1:0]        regs_rdata;

logic [dma_log2(numb_ch)-1:0] stream_sel_mp;
logic [dma_log2(numb_ch)-1:0] stream_sel_pp;

logic               request[numb_ch-1:0];
logic               ltrnsct[numb_ch-1:0];
logic               enable_m;
logic               write_m;
logic [wbus-1:0]    wdata_m;
logic [wbus-1:0]    rdata_m;
// logic [17:0]        ndtr_m;
logic               setget_data_m;
logic               last_trnsct_m;
logic               smaller_size_m;
logic               idle_m;
logic               fail_m;

logic               enable_p;
logic               relevance_request;
logic               write_p;
logic [wbus-1:0]    wdata_p;
logic [wbus-1:0]    rdata_p;
// logic [17:0]        ndtr_p;
logic               setget_data_p;
logic               smaller_size_p;
logic               idle_p;
logic               fail_p;

logic               fifo_put_dc_mp[numb_ch-1:0];
logic [1:0]         fifo_numb_bytes_put_dc_mp[numb_ch-1:0];
logic               fifo_pull_dc_mp[numb_ch-1:0];
logic [1:0]         fifo_numb_bytes_pull_dc_mp[numb_ch-1:0];
logic [wbus-1:0]    fifo_wdata_dc_mp[numb_ch-1:0];

logic               fifo_put_dc_pp[numb_ch-1:0];
logic [1:0]         fifo_numb_bytes_put_dc_pp[numb_ch-1:0];
logic               fifo_pull_dc_pp[numb_ch-1:0];
logic [1:0]         fifo_numb_bytes_pull_dc_pp[numb_ch-1:0];
logic [wbus-1:0]    fifo_wdata_dc_pp[numb_ch-1:0];

logic               fifo_put_dc_final[numb_ch-1:0];
logic [1:0]         fifo_numb_bytes_put_dc_final[numb_ch-1:0];
logic               fifo_pull_dc_final[numb_ch-1:0];
logic [1:0]         fifo_numb_bytes_pull_dc_final[numb_ch-1:0];
logic [wbus-1:0]    fifo_wdata_dc_final[numb_ch-1:0];

logic               buf_put_dc_mp[numb_ch-1:0];
logic               buf_pull_dc_mp[numb_ch-1:0];
logic [wbus-1:0]    buf_wdata_dc_mp[numb_ch-1:0];

logic               buf_put_dc_pp[numb_ch-1:0];
logic               buf_pull_dc_pp[numb_ch-1:0];
logic [wbus-1:0]    buf_wdata_dc_pp[numb_ch-1:0];

logic               buf_put_dc_final[numb_ch-1:0];
logic               buf_pull_dc_final[numb_ch-1:0];
logic [wbus-1:0]    buf_wdata_dc_final[numb_ch-1:0];

logic               fifo_put_mp;
logic [1:0]         fifo_numb_bytes_put_mp;
logic               fifo_pull_mp;
logic [1:0]         fifo_numb_bytes_pull_mp;
logic [wbus-1:0]    fifo_wdata_mp;

logic               fifo_put_pp;
logic [1:0]         fifo_numb_bytes_put_pp;
logic               fifo_pul_ppl;
logic [1:0]         fifo_numb_bytes_pull_pp;
logic [wbus-1:0]    fifo_wdata_pp;

logic [1:0]         fifo_fullness[numb_ch-1:0];
logic               fifo_empty[numb_ch-1:0];
logic               fifo_full[numb_ch-1:0];
logic               fifo_overrun[numb_ch-1:0];
logic               fifo_underrun[numb_ch-1:0];
logic [wbus-1:0]    fifo_rdata[numb_ch-1:0];
logic [fifo_size:0] fifo_left_put[numb_ch-1:0];
logic [fifo_size:0] fifo_left_pull[numb_ch-1:0];

logic [fifo_size:0] fifo_left_bytes_mp[numb_ch-1:0];
logic [fifo_size:0] fifo_left_bytes_pp[numb_ch-1:0];

logic               buf_put_mp;
logic               buf_pull_mp;
logic [wbus-1:0]    buf_wdata_mp;

logic               buf_put_pp;
logic               buf_pull_pp;
logic [wbus-1:0]    buf_wdata_pp;

logic [wbus-1:0]    buf_rdata[numb_ch-1:0];
logic               buf_full[numb_ch-1:0];
logic               buf_empty[numb_ch-1:0];

logic               isr_teif_set[numb_ch-1:0];
logic               isr_dmeif_set[numb_ch-1:0];
logic               isr_feif_set[numb_ch-1:0];
logic               isr_tcif[numb_ch-1:0];
logic               isr_htif[numb_ch-1:0];
logic               isr_teif[numb_ch-1:0];
logic               isr_dmeif[numb_ch-1:0];
logic               isr_feif[numb_ch-1:0];

logic [2:0]         cr_chsel[numb_ch-1:0];
logic [1:0]         cr_mburst[numb_ch-1:0];
logic [1:0]         cr_pburst[numb_ch-1:0];
logic [1:0]         cr_pl[numb_ch-1:0];
logic               cr_pincos[numb_ch-1:0];
logic [1:0]         cr_msize[numb_ch-1:0];
logic [1:0]         cr_psize[numb_ch-1:0];
logic               cr_minc[numb_ch-1:0];
logic               cr_pinc[numb_ch-1:0];
logic [1:0]         cr_dir[numb_ch-1:0];
logic               dir_mbus_to_pbus[numb_ch-1:0];
logic               dir_pbus_to_mbus[numb_ch-1:0];
logic               cr_tcie[numb_ch-1:0];
logic               cr_htie[numb_ch-1:0];
logic               cr_teie[numb_ch-1:0];
logic               cr_dmeie[numb_ch-1:0];
logic               cr_en[numb_ch-1:0];
logic [15:0]        ndtr[numb_ch-1:0];
logic [17:0]        ndtr_src[numb_ch-1:0];
logic [wbus-1:0]    par[numb_ch-1:0];
logic [wbus-1:0]    mar[numb_ch-1:0];
logic               fcr_feie[numb_ch-1:0];
logic               fcr_dmdis[numb_ch-1:0];
logic [1:0]         fcr_fth[numb_ch-1:0];
logic               dis_stream[numb_ch-1:0];
logic               init_haddr[numb_ch-1:0];
logic               fifo_flush[numb_ch-1:0];

logic [17:0]        ndt_mp[numb_ch-1:0];
logic [17:0]        ndt_pp[numb_ch-1:0];

logic               cr_ct_swap_mp;
logic               ndtr_decr_mp;
logic               ndtr_src_decr_mp;
logic               smaller_size_mp;
logic               cr_ct_swap_pp;
logic               ndtr_decr_pp;
logic               ndtr_src_decr_pp;
logic               smaller_size_pp;
logic               cr_ct_swap_dc_mp[numb_ch-1:0];
logic               ndtr_decr_dc_mp[numb_ch-1:0];
logic               ndtr_src_decr_dc_mp[numb_ch-1:0];
logic               ahb_fail_dc_mp[numb_ch-1:0];
logic               idle_dc_mp[numb_ch-1:0];
logic               smaller_size_dc_mp[numb_ch-1:0];

logic               cr_ct_swap_dc_pp[numb_ch-1:0];
logic               ndtr_decr_dc_pp[numb_ch-1:0];
logic               ndtr_src_decr_dc_pp[numb_ch-1:0];
logic               ahb_fail_dc_pp[numb_ch-1:0];
logic               idle_dc_pp[numb_ch-1:0];
logic               smaller_size_dc_pp[numb_ch-1:0];

logic               cr_ct_swap_dc_final[numb_ch-1:0];
logic               ndtr_decr_dc_final[numb_ch-1:0];
logic               ndtr_src_decr_dc_final[numb_ch-1:0];
logic               ahb_fail_dc_final[numb_ch-1:0];
logic               smaller_size_dc_final[numb_ch-1:0];

logic [wbus-1:0]    haddr_save_mp[numb_ch-1:0];
logic [wbus-1:0]    haddr_save_pp[numb_ch-1:0];

logic               haddr_save_refresh_mp;
logic [wbus-1:0]    haddr_save_nxt_mp;

logic               haddr_save_refresh_pp;
logic [wbus-1:0]    haddr_save_nxt_pp;

logic               haddr_save_refresh_dc_mp[numb_ch-1:0];
logic               haddr_save_refresh_dc_pp[numb_ch-1:0];

assign requests[0] = i_req_s0;
assign requests[1] = i_req_s1;
assign requests[2] = i_req_s2;
assign requests[3] = i_req_s3;
assign requests[4] = i_req_s4;
assign requests[5] = i_req_s5;
assign requests[6] = i_req_s6;
assign requests[7] = i_req_s7;
assign ltrnscts[0] = i_ltr_s0;
assign ltrnscts[1] = i_ltr_s1;
assign ltrnscts[2] = i_ltr_s2;
assign ltrnscts[3] = i_ltr_s3;
assign ltrnscts[4] = i_ltr_s4;
assign ltrnscts[5] = i_ltr_s5;
assign ltrnscts[6] = i_ltr_s6;
assign ltrnscts[7] = i_ltr_s7;

dma_slave_ahb3l dma_slave (
    .i_hclk(i_hclk),
    .i_hnreset(i_hnreset),

    // AHB connection to master
    .i_hsel(i_hsel_s),
    .i_haddr(i_haddr_s),
    .i_htrans(i_htrans_s),
    .i_hsize(i_hsize_s),
    .i_hwrite(i_hwrite_s),
    .i_hready(i_hready_s),
    .i_hwdata(i_hwdata_s),

    .o_hreadyout(o_hreadyout_s),
    .o_hresp(o_hresp_s),
    .o_hrdata(o_hrdata_s),

    // register interface
    .o_addr(regs_addr),
    .o_read_en(regs_read_en),
    .o_write_en(regs_write_en),
    .o_byte_strobe(regs_byte_strobe),
    .o_wdata(regs_wdata),
    .i_rdata(regs_rdata)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_fifo_left_bytes_array
        dma_left_bytes #(fifo_size) dma_fifo_left_bytes(
            .i_left_put(fifo_left_put[ch]),
            .i_left_pull(fifo_left_pull[ch]),
            .i_buf_full(buf_full[ch]),
            .i_buf_empty(buf_empty[ch]),
            .i_dir_pbus_to_mbus(dir_pbus_to_mbus[ch]),
            .i_dir_mbus_to_pbus(dir_mbus_to_pbus[ch]),
            .i_direct_dis(fcr_dmdis[ch]),
            .i_msize(cr_msize[ch]),
            .i_psize(cr_psize[ch]),
            .o_left_bytes_mp(fifo_left_bytes_mp[ch]),
            .o_left_bytes_pp(fifo_left_bytes_pp[ch])
        );
    end
endgenerate

dma_arbiter #(numb_ch, fifo_size) dma_arbiter_mp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    .i_en_stream(cr_en),
    .i_size(cr_msize),
    .i_burst(cr_mburst),
    .i_ndt(ndt_mp),
    .i_pl(cr_pl),
    .i_relevance_req(1'b0),
    .i_requests(request),
    .i_left_bytes(fifo_left_bytes_mp),
    .i_master_ready(idle_m),
    .o_stream_sel(stream_sel_mp),
    .o_master_en(enable_m)
);

dma_arbiter #(numb_ch, fifo_size) dma_arbiter_pp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    .i_en_stream(cr_en),
    .i_size(cr_psize),
    .i_burst(cr_pburst),
    .i_ndt(ndt_pp),
    .i_pl(cr_pl),
    .i_relevance_req(1'b1),
    .i_requests(request),
    .i_left_bytes(fifo_left_bytes_pp),
    .i_master_ready(idle_p),
    .o_stream_sel(stream_sel_pp),
    .o_master_en(enable_p)
);

dma_ctrl #(fifo_size) dma_ctrl_mp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    // coonnect to config registers
    // .i_ndtr(ndtr[stream_sel_mp]),
    // .i_ndtr_src(ndtr_src[stream_sel_mp]),
    .i_dir(cr_dir[stream_sel_mp]),
    .i_dir_mbus_to_pbus(dir_mbus_to_pbus[stream_sel_mp]),
    .i_dir_pbus_to_mbus(dir_pbus_to_mbus[stream_sel_mp]),
    .i_dmdis(fcr_dmdis[stream_sel_mp]),
    .i_msize(cr_msize[stream_sel_mp]),
    .i_psize(cr_psize[stream_sel_mp]),
    .o_ndtr_decr(ndtr_decr_mp),
    .o_ndtr_src_decr(ndtr_src_decr_mp),
    .o_smaller_size(smaller_size_mp),

    // connect to memory master
    .o_write_m(write_m),
    .o_wdata_m(wdata_m),
    .i_rdata_m(rdata_m),
    // .o_ndtr_m(ndtr_m),
    .i_setget_data_m(setget_data_m),
    .i_smaller_size_m(smaller_size_m),

    // connect to peripheral master
    .o_relevance_req(),
    .o_write_p(),
    .o_wdata_p(),
    .i_rdata_p(/*rdata_p*/32'd0),
    // .o_ndtr_p(),
    .i_setget_data_p(/*setget_data_p*/1'b0),
    .i_smaller_size_p(/*smaller_size_p*/1'b0),

    // connect to FIFO
    .o_numb_bytes_put(fifo_numb_bytes_put_mp),
    .o_numb_bytes_pull(fifo_numb_bytes_pull_mp),
    .o_fifo_put(fifo_put_mp),
    .o_fifo_pull(fifo_pull_mp),
    .o_fifo_wdata(fifo_wdata_mp),
    .i_fifo_rdata(fifo_rdata[stream_sel_mp]),
    .i_fifo_left_put(fifo_left_put[stream_sel_mp]),
    .i_fifo_left_pull(fifo_left_pull[stream_sel_mp]),

    // connect to buffer reg
    .o_buf_put(buf_put_mp),
    .o_buf_pull(buf_pull_mp),
    .o_buf_wdata(buf_wdata_mp),
    .i_buf_rdata(buf_rdata[stream_sel_mp])
);

dma_ctrl #(fifo_size) dma_ctrl_pp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    // coonnect to config registers
    // .i_ndtr(ndtr[stream_sel_pp]),
    // .i_ndtr_src(ndtr_src[stream_sel_pp]),
    .i_dir(cr_dir[stream_sel_pp]),
    .i_dir_mbus_to_pbus(dir_mbus_to_pbus[stream_sel_pp]),
    .i_dir_pbus_to_mbus(dir_pbus_to_mbus[stream_sel_pp]),
    .i_dmdis(fcr_dmdis[stream_sel_pp]),
    .i_msize(cr_msize[stream_sel_pp]),
    .i_psize(cr_psize[stream_sel_pp]),
    .o_ndtr_decr(ndtr_decr_pp),
    .o_ndtr_src_decr(ndtr_src_decr_pp),
    .o_smaller_size(smaller_size_pp),

    // connect to memory master
    .o_write_m(),
    .o_wdata_m(),
    .i_rdata_m(/*rdata_m*/32'd0),
    // .o_ndtr_m(),
    .i_setget_data_m(/*setget_data_m*/1'b0),
    .i_smaller_size_m(/*smaller_size_m*/1'b0),

    // connect to peripheral master
    .o_relevance_req(relevance_request),
    .o_write_p(write_p),
    .o_wdata_p(wdata_p),
    .i_rdata_p(rdata_p),
    // .o_ndtr_p(ndtr_p),
    .i_setget_data_p(setget_data_p),
    .i_smaller_size_p(smaller_size_p),

    // connect to FIFO
    .o_numb_bytes_put(fifo_numb_bytes_put_pp),
    .o_numb_bytes_pull(fifo_numb_bytes_pull_pp),
    .o_fifo_put(fifo_put_pp),
    .o_fifo_pull(fifo_pull_pp),
    .o_fifo_wdata(fifo_wdata_pp),
    .i_fifo_rdata(fifo_rdata[stream_sel_pp]),
    .i_fifo_left_put(fifo_left_put[stream_sel_pp]),
    .i_fifo_left_pull(fifo_left_pull[stream_sel_pp]),

    // connect to buffer reg
    .o_buf_put(buf_put_pp),
    .o_buf_pull(buf_pull_pp),
    .o_buf_wdata(buf_wdata_pp),
    .i_buf_rdata(buf_rdata[stream_sel_pp])
);

always_comb begin: g_ndt_mp
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        if (dir_mbus_to_pbus[ch_cnt]) begin
            ndt_mp[ch_cnt] = ndtr_src[ch_cnt];
        end
        else if (dir_pbus_to_mbus[ch_cnt]) begin
            ndt_mp[ch_cnt] = {2'h0, ndtr[ch_cnt]};
        end
        else begin
            ndt_mp[ch_cnt] = 'd0;
        end
    end
end

always_comb begin: g_ndt_pp
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        if (dir_mbus_to_pbus[ch_cnt]) begin
            ndt_pp[ch_cnt] = {2'h0, ndtr[ch_cnt]};
        end
        else if (dir_pbus_to_mbus[ch_cnt]) begin
            ndt_pp[ch_cnt] = ndtr_src[ch_cnt];
        end
        else begin
            ndt_pp[ch_cnt] = 'd0;
        end
    end
end

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_haddr_save_mp_array
        dma_save_haddr #(wbus) dma_save_haddr_mp (
            .i_clk(i_hclk),
            .i_nreset(i_hnreset),

            .i_init(init_haddr[ch]),
            .i_init_val(mar[ch]),
            .i_refresh(haddr_save_refresh_dc_mp[ch]),
            .i_nxt_val(haddr_save_nxt_mp),
            .o_val(haddr_save_mp[ch])
        );
    end
endgenerate

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_haddr_save_pp_array
        dma_save_haddr #(wbus) dma_save_haddr_pp (
            .i_clk(i_hclk),
            .i_nreset(i_hnreset),

            .i_init(init_haddr[ch]),
            .i_init_val(par[ch]),
            .i_refresh(haddr_save_refresh_dc_pp[ch]),
            .i_nxt_val(haddr_save_nxt_pp),
            .o_val(haddr_save_pp[ch])
        );
    end
endgenerate

dma_master_ahb3l #(fifo_size) dma_master_mp (
    .i_hclk(i_hclk),
    .i_hnreset(i_hnreset),

    .o_haddr(o_haddr_m),
    .o_hwrite(o_hwrite_m),
    .o_hsize(o_hsize_m),
    .o_hburst(o_hburst_m),
    .o_hprot(o_hprot_m),
    .o_htrans(o_htrans_m),
    .o_hwdata(o_hwdata_m),
    .o_hmastlock(o_hmastlock_m),

    .i_hready(i_hready_m),
    .i_hresp(i_hresp_m),
    .i_hrdata(i_hrdata_m),

    .i_enable(enable_m),
    .i_dis_stream(dis_stream[stream_sel_mp]),
    .i_init_haddr(/*init_haddr[stream_sel_mp]*/1'b0),
    .i_request(1'b1),
    .i_relevance_req(1'b0),
    .i_addr(/*mar[stream_sel_mp]*/'d0),
    .i_write(write_m),
    .i_inc_en(cr_minc[stream_sel_mp]),
    .i_fix_inc(1'b0),
    .i_burst(cr_mburst[stream_sel_mp]),
    .i_size(cr_msize[stream_sel_mp]),
    .i_wdata(wdata_m),
    .o_rdata(rdata_m),
    .i_ndtr(/*ndtr_m*/ndt_mp[stream_sel_mp]),
    .i_fifo_left_bytes(fifo_left_bytes_mp[stream_sel_mp]),
    .o_setget_data(setget_data_m),
    .o_last_trnsct(last_trnsct_m),
    .o_smaller_size(smaller_size_m),

    .i_haddr_save(haddr_save_mp[stream_sel_mp]),
    .o_haddr_save_refresh(haddr_save_refresh_mp),
    .o_haddr_save_nxt(haddr_save_nxt_mp),

    .o_idle(idle_m),
    .o_fail(fail_m)
);

dma_master_ahb3l #(fifo_size) dma_master_pp (
    .i_hclk(i_hclk),
    .i_hnreset(i_hnreset),

    .o_haddr(o_haddr_p),
    .o_hwrite(o_hwrite_p),
    .o_hsize(o_hsize_p),
    .o_hburst(o_hburst_p),
    .o_hprot(o_hprot_p),
    .o_htrans(o_htrans_p),
    .o_hwdata(o_hwdata_p),
    .o_hmastlock(o_hmastlock_p),

    .i_hready(i_hready_p),
    .i_hresp(i_hresp_p),
    .i_hrdata(i_hrdata_p),

    .i_enable(enable_p),
    .i_dis_stream(dis_stream[stream_sel_pp]),
    .i_init_haddr(/*init_haddr[stream_sel_pp]*/1'b0),
    .i_request(request[stream_sel_pp]),
    .i_relevance_req(relevance_request),
    .i_addr(/*par[stream_sel_pp]*/'d0),
    .i_write(write_p),
    .i_inc_en(cr_pinc[stream_sel_pp]),
    .i_fix_inc(cr_pincos[stream_sel_pp]),
    .i_burst(cr_pburst[stream_sel_pp]),
    .i_size(cr_psize[stream_sel_pp]),
    .i_wdata(wdata_p),
    .o_rdata(rdata_p),
    .i_ndtr(/*ndtr_p*/ndt_pp[stream_sel_pp]),
    .i_fifo_left_bytes(fifo_left_bytes_pp[stream_sel_pp]),
    .o_setget_data(setget_data_p),
    .o_last_trnsct(),
    .o_smaller_size(smaller_size_p),

    .i_haddr_save(haddr_save_pp[stream_sel_pp]),
    .o_haddr_save_refresh(haddr_save_refresh_pp),
    .o_haddr_save_nxt(haddr_save_nxt_pp),

    .o_idle(idle_p),
    .o_fail(fail_p)
);

dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_haddr_save_refresh_mp (
    .i_addr(stream_sel_mp),
    .i_en(haddr_save_refresh_mp),
    .o_out(haddr_save_refresh_dc_mp)
);

dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_haddr_save_refresh_pp (
    .i_addr(stream_sel_pp),
    .i_en(haddr_save_refresh_pp),
    .o_out(haddr_save_refresh_dc_pp)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_req_ch_mux_array
        assign request[ch] = requests[ch][cr_chsel[ch]];
    end
endgenerate

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_ltrnsct_ch_mux_array
        assign ltrnsct[ch] = ltrnscts[ch][cr_chsel[ch]];
    end
endgenerate

dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_put_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_put_mp),
    .o_out(fifo_put_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) z_dma_dc_numb_bytes_put_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_numb_bytes_put_mp),
    .o_out(fifo_numb_bytes_put_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_pull_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_pull_mp),
    .o_out(fifo_pull_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) z_dma_dc_numb_bytes_pull_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_numb_bytes_pull_mp),
    .o_out(fifo_numb_bytes_pull_dc_mp)
);
dma_dc #(dma_log2(numb_ch), wbus, numb_ch) z_dma_dc_fifo_wdata_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_wdata_mp),
    .o_out(fifo_wdata_dc_mp)
);

dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_put_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_put_pp),
    .o_out(fifo_put_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) z_dma_dc_numb_bytes_put_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_numb_bytes_put_pp),
    .o_out(fifo_numb_bytes_put_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_pull_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_pull_pp),
    .o_out(fifo_pull_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) z_dma_dc_numb_bytes_pull_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_numb_bytes_pull_pp),
    .o_out(fifo_numb_bytes_pull_dc_pp)
);
dma_dc #(dma_log2(numb_ch), wbus, numb_ch) z_dma_dc_fifo_wdata_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_wdata_pp),
    .o_out(fifo_wdata_dc_pp)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_fifo_array
        assign fifo_put_dc_final[ch] = fifo_put_dc_mp[ch] | fifo_put_dc_pp[ch];
        assign fifo_numb_bytes_put_dc_final[ch] = fifo_numb_bytes_put_dc_mp[ch] | fifo_numb_bytes_put_dc_pp[ch];
        assign fifo_pull_dc_final[ch] = fifo_pull_dc_mp[ch] | fifo_pull_dc_pp[ch];
        assign fifo_numb_bytes_pull_dc_final[ch] = fifo_numb_bytes_pull_dc_mp[ch] | fifo_numb_bytes_pull_dc_pp[ch];
        assign fifo_wdata_dc_final[ch] = fifo_wdata_dc_mp[ch] | fifo_wdata_dc_pp[ch];

        dma_fifo #(fifo_size) dma_fifo (
            .i_clk(i_hclk),
            .i_nreset(i_hnreset),

            .i_put(fifo_put_dc_final[ch]),
            .i_numb_bytes_put(fifo_numb_bytes_put_dc_final[ch]),
            .i_pull(fifo_pull_dc_final[ch]),
            .i_numb_bytes_pull(fifo_numb_bytes_pull_dc_final[ch]),
            .i_wdata(fifo_wdata_dc_final[ch]),
            .i_filling_thresh(fcr_fth[ch]),
            .i_clear(fifo_flush[ch]),

            .o_fullness(fifo_fullness[ch]),
            .o_empty(fifo_empty[ch]),
            .o_full(fifo_full[ch]),
            .o_overrun(fifo_overrun[ch]),
            .o_underrun(fifo_underrun[ch]),
            .o_rdata(fifo_rdata[ch]),

            .o_left_put(fifo_left_put[ch]),
            .o_left_pull(fifo_left_pull[ch])
        );
    end
endgenerate

dma_dc #(dma_log2(numb_ch), wbus, numb_ch) z_dma_dc_buf_wdata_mp (
    .i_addr(stream_sel_mp),
    .i_en(buf_wdata_mp),
    .o_out(buf_wdata_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_buf_put_mp (
    .i_addr(stream_sel_mp),
    .i_en(buf_put_mp),
    .o_out(buf_put_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_buf_pull_mp (
    .i_addr(stream_sel_mp),
    .i_en(buf_pull_mp),
    .o_out(buf_pull_dc_mp)
);

dma_dc #(dma_log2(numb_ch), wbus, numb_ch) z_dma_dc_buf_wdata_pp (
    .i_addr(stream_sel_pp),
    .i_en(buf_wdata_pp),
    .o_out(buf_wdata_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_buf_put_pp (
    .i_addr(stream_sel_pp),
    .i_en(buf_put_pp),
    .o_out(buf_put_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_buf_pull_pp (
    .i_addr(stream_sel_pp),
    .i_en(buf_pull_pp),
    .o_out(buf_pull_dc_pp)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_buffer_reg_array
        assign buf_wdata_dc_final[ch] = buf_wdata_dc_mp[ch] | buf_wdata_dc_pp[ch];
        assign buf_put_dc_final[ch] = buf_put_dc_mp[ch] | buf_put_dc_pp[ch];
        assign buf_pull_dc_final[ch] = buf_pull_dc_mp[ch] | buf_pull_dc_pp[ch];

        dma_buf_reg #(wbus) dma_buffer_reg (
            .i_clk(i_hclk),
            .i_nreset(i_hnreset),

            .i_wdata(buf_wdata_dc_final[ch]),
            .i_put(buf_put_dc_final[ch]),
            .i_pull(buf_pull_dc_final[ch]),
            .o_rdata(buf_rdata[ch]),
            .o_empty(buf_empty[ch]),
            .o_full(buf_full[ch])
        );
    end
endgenerate

assign cr_ct_swap_mp = last_trnsct_m;
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_cr_ct_swap_mp (
    .i_addr(stream_sel_mp),
    .i_en(cr_ct_swap_mp),
    .o_out(cr_ct_swap_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_ndtr_decr_mp (
    .i_addr(stream_sel_mp),
    .i_en(ndtr_decr_mp),
    .o_out(ndtr_decr_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_ndtr_src_decr_mp (
    .i_addr(stream_sel_mp),
    .i_en(ndtr_src_decr_mp),
    .o_out(ndtr_src_decr_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_smaller_size_mp (
    .i_addr(stream_sel_mp),
    .i_en(smaller_size_mp),
    .o_out(smaller_size_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_ahb_fail_mp (
    .i_addr(stream_sel_mp),
    .i_en(fail_m),
    .o_out(ahb_fail_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_idle_mp (
    .i_addr(stream_sel_mp),
    .i_en(idle_m),
    .o_out(idle_dc_mp)
);

assign cr_ct_swap_pp = 1'b0;
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_cr_ct_swap_pp (
    .i_addr(stream_sel_pp),
    .i_en(cr_ct_swap_pp),
    .o_out(cr_ct_swap_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_ndtr_decr_pp (
    .i_addr(stream_sel_pp),
    .i_en(ndtr_decr_pp),
    .o_out(ndtr_decr_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_ndtr_src_decr_pp (
    .i_addr(stream_sel_pp),
    .i_en(ndtr_src_decr_pp),
    .o_out(ndtr_src_decr_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_smaller_size_pp (
    .i_addr(stream_sel_pp),
    .i_en(smaller_size_pp),
    .o_out(smaller_size_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_ahb_fail_pp (
    .i_addr(stream_sel_pp),
    .i_en(fail_p),
    .o_out(ahb_fail_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) z_dma_dc_idle_pp (
    .i_addr(stream_sel_pp),
    .i_en(idle_p),
    .o_out(idle_dc_pp)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_cr_ct_swap_dc_merge
        assign cr_ct_swap_dc_final[ch] = cr_ct_swap_dc_mp[ch] | cr_ct_swap_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: g_ndtr_decr_dc_merge
        assign ndtr_decr_dc_final[ch] = ndtr_decr_dc_mp[ch] | ndtr_decr_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: g_ndtr_src_decr_dc_merge
        assign ndtr_src_decr_dc_final[ch] = ndtr_src_decr_dc_mp[ch] | ndtr_src_decr_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: g_smaller_size_dc_merge
        assign smaller_size_dc_final[ch] = smaller_size_dc_mp[ch] | smaller_size_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: g_ahb_fail_dc_merge
        assign ahb_fail_dc_final[ch] = ahb_fail_dc_mp[ch] | ahb_fail_dc_pp[ch];
    end
endgenerate

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_init_isr
        assign isr_teif_set[ch] = ahb_fail_dc_final[ch];
        assign isr_dmeif_set[ch] = 1'b0;
        assign isr_feif_set[ch] = fifo_overrun[ch] | fifo_underrun[ch];
    end
endgenerate

dma_regs #(numb_ch, fifo_size) dma_regs (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    .i_addr(regs_addr),
    .i_read_en(regs_read_en),
    .i_write_en(regs_write_en),
    .i_byte_strobe(regs_byte_strobe),
    .i_wdata(regs_wdata),
    .o_rdata(regs_rdata),

    // interrupt status
    .i_isr_teif_set(isr_teif_set),
    .i_isr_dmeif_set(isr_dmeif_set),
    .i_isr_feif_set(isr_feif_set),
    .o_isr_tcif(isr_tcif),
    .o_isr_htif(isr_htif),
    .o_isr_teif(isr_teif),
    .o_isr_dmeif(isr_dmeif),
    .o_isr_feif(isr_feif),

    // config signals
    .i_cr_ct_swap(cr_ct_swap_dc_final),
    .i_ndtr_decr(ndtr_decr_dc_final),
    .i_ndtr_src_decr(ndtr_src_decr_dc_final),
    .i_fifo_fullness(fifo_fullness),
    .i_fifo_empty(fifo_empty),
    .i_fifo_full(fifo_full),
    .i_idle_mem(idle_dc_mp),
    .i_idle_periph(idle_dc_pp),
    .i_dmbuf_empty(buf_empty),
    .i_smaller_size(smaller_size_dc_final),
    .i_ltrnsct(ltrnsct),

    .o_cr_chsel(cr_chsel),
    .o_cr_mburst(cr_mburst),
    .o_cr_pburst(cr_pburst),
    .o_cr_pl(cr_pl),
    .o_cr_pincos(cr_pincos),
    .o_cr_msize(cr_msize),
    .o_cr_psize(cr_psize),
    .o_cr_minc(cr_minc),
    .o_cr_pinc(cr_pinc),
    .o_cr_dir(cr_dir),
    .o_dir_mbus_to_pbus(dir_mbus_to_pbus),
    .o_dir_pbus_to_mbus(dir_pbus_to_mbus),
    .o_cr_tcie(cr_tcie),
    .o_cr_htie(cr_htie),
    .o_cr_teie(cr_teie),
    .o_cr_dmeie(cr_dmeie),
    .o_cr_en(cr_en),
    .o_ndtr(ndtr),
    .o_ndtr_src(ndtr_src),
    .o_par(par),
    .o_mar(mar),
    .o_fcr_feie(fcr_feie),
    .o_fcr_dmdis(fcr_dmdis),
    .o_fcr_fth(fcr_fth),

    .o_dis_stream(dis_stream),
    .o_init_haddr(init_haddr),
    .o_fifo_flush(fifo_flush)
);

// interrupt generation
generate
    for (ch = 0; ch < numb_ch; ++ch) begin: g_int_generation
        assign o_interrupt[ch] = (isr_tcif[ch] & cr_tcie[ch]) | (isr_htif[ch] & cr_htie[ch]) |
            (isr_teif[ch] & cr_teie[ch]) | (isr_dmeif[ch] & cr_dmeie[ch]) | (isr_feif[ch] & fcr_feie[ch]);
    end
endgenerate

endmodule
