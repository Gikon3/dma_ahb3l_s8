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
localparam fifo_size_exp = 4;
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

logic                   request[numb_ch-1:0];
logic                   request_mux_pp;
logic                   ltrnsct[numb_ch-1:0];
logic                   enable_m;
logic                   write_m;
logic [wbus-1:0]        wdata_m;
logic [wbus-1:0]        rdata_m;
logic [17:0]            ndtr_m;
logic [fifo_size_exp:0] fifo_left_bytes_m;
logic                   setget_data_m;
logic                   last_trnsct_m;
logic                   smaller_size_m;
logic                   idle_m;
logic                   fail_m;

logic                   enable_p;
logic                   relevance_request;
logic                   write_p;
logic [wbus-1:0]        wdata_p;
logic [wbus-1:0]        rdata_p;
logic [17:0]            ndtr_p;
logic [fifo_size_exp:0] fifo_left_bytes_p;
logic                   setget_data_p;
logic                   smaller_size_p;
logic                   idle_p;
logic                   fail_p;

logic                   fifo_put_dc_mp[numb_ch-1:0];
logic [1:0]             fifo_numb_bytes_put_dc_mp[numb_ch-1:0];
logic                   fifo_pull_dc_mp[numb_ch-1:0];
logic [1:0]             fifo_numb_bytes_pull_dc_mp[numb_ch-1:0];
logic [wbus-1:0]        fifo_wdata_dc_mp[numb_ch-1:0];

logic                   fifo_put_dc_pp[numb_ch-1:0];
logic [1:0]             fifo_numb_bytes_put_dc_pp[numb_ch-1:0];
logic                   fifo_pull_dc_pp[numb_ch-1:0];
logic [1:0]             fifo_numb_bytes_pull_dc_pp[numb_ch-1:0];
logic [wbus-1:0]        fifo_wdata_dc_pp[numb_ch-1:0];

logic                   fifo_put_dc_final[numb_ch-1:0];
logic [1:0]             fifo_numb_bytes_put_dc_final[numb_ch-1:0];
logic                   fifo_pull_dc_final[numb_ch-1:0];
logic [1:0]             fifo_numb_bytes_pull_dc_final[numb_ch-1:0];
logic [wbus-1:0]        fifo_wdata_dc_final[numb_ch-1:0];

logic [wbus-1:0]        fifo_rdata_mux_mp;
logic [fifo_size_exp:0] fifo_left_put_mux_mp;
logic [fifo_size_exp:0] fifo_left_pull_mux_mp;

logic [wbus-1:0]        fifo_rdata_mux_pp;
logic [fifo_size_exp:0] fifo_left_put_mux_pp;
logic [fifo_size_exp:0] fifo_left_pull_mux_pp;

logic                   fifo_put_mp;
logic [1:0]             fifo_numb_bytes_put_mp;
logic                   fifo_pull_mp;
logic [1:0]             fifo_numb_bytes_pull_mp;
logic [wbus-1:0]        fifo_wdata_mp;
logic                   fifo_put_pp;
logic [1:0]             fifo_numb_bytes_put_pp;
logic                   fifo_pul_ppl;
logic [1:0]             fifo_numb_bytes_pull_pp;
logic [wbus-1:0]        fifo_wdata_pp;
logic [1:0]             fifo_fullness[numb_ch-1:0];
logic                   fifo_empty[numb_ch-1:0];
logic                   fifo_full[numb_ch-1:0];
logic                   fifo_overrun[numb_ch-1:0];
logic                   fifo_underrun[numb_ch-1:0];
logic [wbus-1:0]        fifo_rdata[numb_ch-1:0];
logic [fifo_size_exp:0] fifo_left_put[numb_ch-1:0];
logic [fifo_size_exp:0] fifo_left_pull[numb_ch-1:0];

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
logic [1:0]         cr_mburst_mux_mp;
logic [1:0]         cr_pburst_mux_mp;
logic               cr_pincos_mux_mp;
logic [1:0]         cr_msize_mux_mp;
logic [1:0]         cr_psize_mux_mp;
logic               cr_minc_mux_mp;
logic               cr_pinc_mux_mp;
logic [1:0]         cr_dir_mux_mp;
logic               cr_en_mux_mp;
logic [15:0]        ndtr_mux_mp;
logic [17:0]        ndtr_src_mux_mp;
logic [wbus-1:0]    par_mux_mp;
logic [wbus-1:0]    mar_mux_mp;
logic               fcr_dmdis_mux_mp;
logic               dis_stream_mux_mp;
logic               init_haddr_mux_mp;
logic [1:0]         cr_mburst_mux_pp;
logic [1:0]         cr_pburst_mux_pp;
logic               cr_pincos_mux_pp;
logic [1:0]         cr_msize_mux_pp;
logic [1:0]         cr_psize_mux_pp;
logic               cr_minc_mux_pp;
logic               cr_pinc_mux_pp;
logic [1:0]         cr_dir_mux_pp;
logic               cr_en_mux_pp;
logic [15:0]        ndtr_mux_pp;
logic [17:0]        ndtr_src_mux_pp;
logic [wbus-1:0]    par_mux_pp;
logic [wbus-1:0]    mar_mux_pp;
logic               fcr_dmdis_mux_pp;
logic               dis_stream_mux_pp;
logic               init_haddr_mux_pp;

logic               cr_ct_swap_mp;
logic               ndtr_decr_mp;
logic               ndtr_src_decr_mp;
// logic               ahb_fail_mp;
logic               dmbuf_empty_mp;
logic               smaller_size_mp;
logic               cr_ct_swap_pp;
logic               ndtr_decr_pp;
logic               ndtr_src_decr_pp;
// logic               ahb_fail_pp;
logic               dmbuf_empty_pp;
logic               smaller_size_pp;
logic               cr_ct_swap_dc_mp[numb_ch-1:0];
logic               ndtr_decr_dc_mp[numb_ch-1:0];
logic               ndtr_src_decr_dc_mp[numb_ch-1:0];
logic               ahb_fail_dc_mp[numb_ch-1:0];
logic               idle_dc_mp[numb_ch-1:0];
// logic               idle_m_dc_mp[numb_ch-1:0];
// logic               idle_p_dc_mp[numb_ch-1:0];
logic               dmbuf_empty_dc_mp[numb_ch-1:0];
logic               smaller_size_dc_mp[numb_ch-1:0];

logic               cr_ct_swap_dc_pp[numb_ch-1:0];
logic               ndtr_decr_dc_pp[numb_ch-1:0];
logic               ndtr_src_decr_dc_pp[numb_ch-1:0];
logic               ahb_fail_dc_pp[numb_ch-1:0];
logic               idle_dc_pp[numb_ch-1:0];
// logic               idle_m_dc_pp[numb_ch-1:0];
// logic               idle_p_dc_pp[numb_ch-1:0];
logic               dmbuf_empty_dc_pp[numb_ch-1:0];
logic               smaller_size_dc_pp[numb_ch-1:0];

logic               cr_ct_swap_dc_final[numb_ch-1:0];
logic               ndtr_decr_dc_final[numb_ch-1:0];
logic               ndtr_src_decr_dc_final[numb_ch-1:0];
logic               ahb_fail_dc_final[numb_ch-1:0];
// logic               idle_dc_final[numb_ch-1:0];
logic               dmbuf_empty_dc_final[numb_ch-1:0];
logic               smaller_size_dc_final[numb_ch-1:0];

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

logic [fifo_size_exp:0]     fifo_left_bytes_mp[numb_ch-1:0];
logic [fifo_size_exp:0]     fifo_left_bytes_pp[numb_ch-1:0];

always_comb begin: decode_left_bytes_mp
    int pbus_to_mbus;
    int mbus_to_pbus;
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        pbus_to_mbus = cr_dir[ch_cnt] == 0 | cr_dir[ch_cnt] == 1;
        mbus_to_pbus = cr_dir[ch_cnt] == 2;

        if (fcr_dmdis[ch_cnt]) begin
            if (pbus_to_mbus) begin
                fifo_left_bytes_pp[ch_cnt] = fifo_left_put_mux_mp;
            end
            else if (mbus_to_pbus) begin
                fifo_left_bytes_pp[ch_cnt] = fifo_left_pull_mux_mp;
            end
            else begin
                fifo_left_bytes_pp[ch_cnt] = 'd0;
            end
        end
        else begin
            if ((pbus_to_mbus) || (mbus_to_pbus)) begin
                fifo_left_bytes_mp[ch_cnt] = {{(fifo_size_exp-2){1'b0}}, 3'd4};
            end
            else begin
                fifo_left_bytes_mp[ch_cnt] = 'd0;
            end
        end
    end
end

always_comb begin: decode_left_bytes_pp
    int pbus_to_mbus;
    int mbus_to_pbus;
    for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
        pbus_to_mbus = cr_dir[ch_cnt] == 0 | cr_dir[ch_cnt] == 1;
        mbus_to_pbus = cr_dir[ch_cnt] == 2;

        if (fcr_dmdis[ch_cnt]) begin
            if (pbus_to_mbus) begin
                fifo_left_bytes_pp[ch_cnt] = fifo_left_put_mux_pp;
            end
            else if (mbus_to_pbus) begin
                fifo_left_bytes_pp[ch_cnt] = fifo_left_pull_mux_pp;
            end
            else begin
                fifo_left_bytes_pp[ch_cnt] = 'd0;
            end
        end
        else begin
            if ((pbus_to_mbus) || (mbus_to_pbus)) begin
                fifo_left_bytes_pp[ch_cnt] = {{(fifo_size_exp-2){1'b0}}, 3'd4};
            end
            else begin
                fifo_left_bytes_pp[ch_cnt] = 'd0;
            end
        end
    end
end

dma_arbiter #(numb_ch, fifo_size_exp) dma_arbiter_mp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    .i_en_stream(cr_en),
    .i_pl(cr_pl),
    .i_relevance_req(1'b0),
    .i_requests(request),
    .i_left_bytes(/*fifo_left_bytes_m*/fifo_left_bytes_mp),
    .o_stream_sel(stream_sel_mp)
);

dma_arbiter #(numb_ch, fifo_size_exp) dma_arbiter_pp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    .i_en_stream(cr_en),
    .i_pl(cr_pl),
    .i_relevance_req(1'b1),
    .i_requests(request),
    .i_left_bytes(fifo_left_bytes_pp),
    .o_stream_sel(stream_sel_pp)
);

dma_ctrl #(fifo_size_exp) dma_ctrl_mp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    // coonnect to config registers
    .i_enable(cr_en_mux_mp),
    .i_ndtr(ndtr_mux_mp),
    .i_ndtr_src(ndtr_src_mux_mp),
    .i_dir(cr_dir_mux_mp),
    .i_dmdis(fcr_dmdis_mux_mp),
    .i_msize(cr_msize_mux_mp),
    .i_psize(cr_psize_mux_mp),
    .o_ndtr_decr(ndtr_decr_mp),
    .o_ndtr_src_decr(ndtr_src_decr_mp),
//     .o_fail(ahb_fail),
    .o_dmbuf_empty(dmbuf_empty_mp),
    .o_smaller_size(smaller_size_mp),

    // connect to memory master
    .o_enable_m(enable_m),
    .o_write_m(write_m),
    .o_wdata_m(wdata_m),
    .i_rdata_m(rdata_m),
    .o_ndtr_m(ndtr_m),
    .o_fifo_left_bytes_m(fifo_left_bytes_m),
    .i_setget_data_m(setget_data_m),
    .i_smaller_size_m(smaller_size_m),
//     .i_fail_m(fail_m),

    // connect to peripheral master
    .o_enable_p(),
    .o_relevance_req(),
    .o_write_p(),
    .o_wdata_p(),
    .i_rdata_p(rdata_p),
    .o_ndtr_p(),
    .o_fifo_left_bytes_p(),
    .i_setget_data_p(setget_data_p),
    .i_smaller_size_p(smaller_size_p),
//     .i_fail_p(fail_p),

    // connect to FIFO
    .o_numb_bytes_put(/*fifo_numb_bytes_put*/fifo_numb_bytes_put_mp),
    .o_numb_bytes_pull(/*fifo_numb_bytes_pull*/fifo_numb_bytes_pull_mp),
    .o_fifo_put(/*fifo_put*/fifo_put_mp),
    .o_fifo_pull(/*fifo_pull*/fifo_pull_mp),
    .o_fifo_wdata(/*fifo_wdata*/fifo_wdata_mp),
    .i_fifo_rdata(/*fifo_rdata_mux*/fifo_rdata_mux_mp),
    .i_fifo_left_put(/*fifo_left_put_mux*/fifo_left_put_mux_mp),
    .i_fifo_left_pull(/*fifo_left_pull_mux*/fifo_left_pull_mux_mp)
);

dma_ctrl #(fifo_size_exp) dma_ctrl_pp (
    .i_clk(i_hclk),
    .i_nreset(i_hnreset),

    // coonnect to config registers
    .i_enable(cr_en_mux_pp),
    .i_ndtr(ndtr_mux_pp),
    .i_ndtr_src(ndtr_src_mux_pp),
    .i_dir(cr_dir_mux_pp),
    .i_dmdis(fcr_dmdis_mux_pp),
    .i_msize(cr_msize_mux_pp),
    .i_psize(cr_psize_mux_pp),
    .o_ndtr_decr(ndtr_decr_pp),
    .o_ndtr_src_decr(ndtr_src_decr_pp),
//     .o_fail(ahb_fail),
    .o_dmbuf_empty(dmbuf_empty_pp),
    .o_smaller_size(smaller_size_pp),

    // connect to memory master
    .o_enable_m(),
    .o_write_m(),
    .o_wdata_m(),
    .i_rdata_m(rdata_m),
    .o_ndtr_m(),
    .o_fifo_left_bytes_m(),
    .i_setget_data_m(setget_data_m),
    .i_smaller_size_m(smaller_size_m),
//     .i_fail_m(fail_m),

    // connect to peripheral master
    .o_enable_p(enable_p),
    .o_relevance_req(relevance_request),
    .o_write_p(write_p),
    .o_wdata_p(wdata_p),
    .i_rdata_p(rdata_p),
    .o_ndtr_p(ndtr_p),
    .o_fifo_left_bytes_p(fifo_left_bytes_p),
    .i_setget_data_p(setget_data_p),
    .i_smaller_size_p(smaller_size_p),
//     .i_fail_p(fail_p),

    // connect to FIFO
    .o_numb_bytes_put(fifo_numb_bytes_put_pp),
    .o_numb_bytes_pull(fifo_numb_bytes_pull_pp),
    .o_fifo_put(fifo_put_pp),
    .o_fifo_pull(fifo_pull_pp),
    .o_fifo_wdata(fifo_wdata_pp),
    .i_fifo_rdata(fifo_rdata_mux_pp),
    .i_fifo_left_put(fifo_left_put_mux_pp),
    .i_fifo_left_pull(fifo_left_pull_mux_pp)
);

dma_master_ahb3l #(fifo_size_exp) dma_master_mp (
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
    .i_dis_stream(dis_stream_mux_mp),
    .i_init_haddr(init_haddr_mux_mp),
    .i_request(1'b1),
    .i_relevance_req(1'b0),
    .i_addr(mar_mux_mp),
    .i_write(write_m),
    .i_inc_en(cr_minc_mux_mp),
    .i_fix_inc(1'b0),
    .i_burst(cr_mburst_mux_mp),
    .i_size(cr_msize_mux_mp),
    .i_wdata(wdata_m),
    .o_rdata(rdata_m),
    .i_ndtr(ndtr_m),
    .i_fifo_left_bytes(fifo_left_bytes_m),
    .o_setget_data(setget_data_m),
    .o_last_trnsct(last_trnsct_m),
    .o_smaller_size(smaller_size_m),

    .o_idle(idle_m),
    .o_fail(fail_m)
);

dma_master_ahb3l #(fifo_size_exp) dma_master_pp (
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
    .i_dis_stream(dis_stream_mux_pp),
    .i_init_haddr(init_haddr_mux_pp),
    .i_request(request_mux_pp),
    .i_relevance_req(relevance_request),
    .i_addr(par_mux_mp),
    .i_write(write_p),
    .i_inc_en(cr_pinc_mux_pp),
    .i_fix_inc(cr_pincos_mux_pp),
    .i_burst(cr_pburst_mux_pp),
    .i_size(cr_psize_mux_pp),
    .i_wdata(wdata_p),
    .o_rdata(rdata_p),
    .i_ndtr(ndtr_p),
    .i_fifo_left_bytes(fifo_left_bytes_p),
    .o_setget_data(setget_data_p),
    .o_last_trnsct(),
    .o_smaller_size(smaller_size_p),

    .o_idle(idle_p),
    .o_fail(fail_p)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: dma_req_ch_mux_array
        assign request[ch] = requests[ch][cr_chsel[ch]];
    end
endgenerate

assign request_mux_pp = request[stream_sel_pp];

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: dma_ltrnsct_ch_mux_array
        assign ltrnsct[ch] = ltrnscts[ch][cr_chsel[ch]];
    end
endgenerate

assign fifo_rdata_mux_mp = fifo_rdata[stream_sel_mp];
assign fifo_left_put_mux_mp = fifo_left_put[stream_sel_mp];
assign fifo_left_pull_mux_mp = fifo_left_pull[stream_sel_mp];

assign fifo_rdata_mux_pp = fifo_rdata[stream_sel_pp];
assign fifo_left_put_mux_pp = fifo_left_put[stream_sel_pp];
assign fifo_left_pull_mux_pp = fifo_left_pull[stream_sel_pp];

dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_put_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_put_mp),
    .o_out(fifo_put_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) dma_dc_numb_bytes_put_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_numb_bytes_put_mp),
    .o_out(fifo_numb_bytes_put_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_pull_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_pull_mp),
    .o_out(fifo_pull_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) dma_dc_numb_bytes_pull_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_numb_bytes_pull_mp),
    .o_out(fifo_numb_bytes_pull_dc_mp)
);
dma_dc #(dma_log2(numb_ch), wbus, numb_ch) dma_dc_fifo_wdata_mp (
    .i_addr(stream_sel_mp),
    .i_en(fifo_wdata_mp),
    .o_out(fifo_wdata_dc_mp)
);

dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_put_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_put_pp),
    .o_out(fifo_put_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) dma_dc_numb_bytes_put_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_numb_bytes_put_pp),
    .o_out(fifo_numb_bytes_put_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_pull_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_pull_pp),
    .o_out(fifo_pull_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 2, numb_ch) dma_dc_numb_bytes_pull_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_numb_bytes_pull_pp),
    .o_out(fifo_numb_bytes_pull_dc_pp)
);
dma_dc #(dma_log2(numb_ch), wbus, numb_ch) dma_dc_fifo_wdata_pp (
    .i_addr(stream_sel_pp),
    .i_en(fifo_wdata_pp),
    .o_out(fifo_wdata_dc_pp)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: fifo_put_dc_merge
        assign fifo_put_dc_final[ch] = fifo_put_dc_mp[ch] | fifo_put_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: fifo_numb_bytes_put_dc_merge
        assign fifo_numb_bytes_put_dc_final[ch] = fifo_numb_bytes_put_dc_mp[ch] | fifo_numb_bytes_put_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: fifo_pull_dc_merge
        assign fifo_pull_dc_final[ch] = fifo_pull_dc_mp[ch] | fifo_pull_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: fifo_numb_bytes_pull_dc_merge
        assign fifo_numb_bytes_pull_dc_final[ch] = fifo_numb_bytes_pull_dc_mp[ch] | fifo_numb_bytes_pull_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: fifo_wdata_dc_merge
        assign fifo_wdata_dc_final[ch] = fifo_wdata_dc_mp[ch] | fifo_wdata_dc_pp[ch];
    end
endgenerate

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: dma_fifo_array
        dma_fifo #(fifo_size_exp) dma_fifo (
            .i_clk(i_hclk),
            .i_nreset(i_hnreset),

            .i_put(/*fifo_put_dc*/fifo_put_dc_final[ch]),
            .i_numb_bytes_put(/*fifo_numb_bytes_put_dc*/fifo_numb_bytes_put_dc_final[ch]),
            .i_pull(/*fifo_pull_dc*/fifo_pull_dc_final[ch]),
            .i_numb_bytes_pull(/*fifo_numb_bytes_pull_dc*/fifo_numb_bytes_pull_dc_final[ch]),
            .i_wdata(/*fifo_wdata*/fifo_wdata_dc_final[ch]),
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

assign cr_mburst_mux_mp = cr_mburst[stream_sel_mp];
assign cr_pburst_mux_mp = cr_pburst[stream_sel_mp];
assign cr_pincos_mux_mp = cr_pincos[stream_sel_mp];
assign cr_msize_mux_mp = cr_msize[stream_sel_mp];
assign cr_psize_mux_mp = cr_psize[stream_sel_mp];
assign cr_minc_mux_mp = cr_minc[stream_sel_mp];
assign cr_pinc_mux_mp = cr_pinc[stream_sel_mp];
assign cr_dir_mux_mp = cr_dir[stream_sel_mp];
assign cr_en_mux_mp = cr_en[stream_sel_mp];
assign ndtr_mux_mp = ndtr[stream_sel_mp];
assign ndtr_src_mux_mp = ndtr_src[stream_sel_mp];
assign par_mux_mp = par[stream_sel_mp];
assign mar_mux_mp = mar[stream_sel_mp];
assign fcr_dmdis_mux_mp = fcr_dmdis[stream_sel_mp];
assign dis_stream_mux_mp = dis_stream[stream_sel_mp];
assign init_haddr_mux_mp = init_haddr[stream_sel_mp];

assign cr_mburst_mux_pp = cr_mburst[stream_sel_pp];
assign cr_pburst_mux_pp = cr_pburst[stream_sel_pp];
assign cr_pincos_mux_pp = cr_pincos[stream_sel_pp];
assign cr_msize_mux_pp = cr_msize[stream_sel_pp];
assign cr_psize_mux_pp = cr_psize[stream_sel_pp];
assign cr_minc_mux_pp = cr_minc[stream_sel_pp];
assign cr_pinc_mux_pp = cr_pinc[stream_sel_pp];
assign cr_dir_mux_pp = cr_dir[stream_sel_pp];
assign cr_en_mux_pp = cr_en[stream_sel_pp];
assign ndtr_mux_pp = ndtr[stream_sel_pp];
assign ndtr_src_mux_pp = ndtr_src[stream_sel_pp];
assign par_mux_pp = par[stream_sel_pp];
assign mar_mux_pp = mar[stream_sel_pp];
assign fcr_dmdis_mux_pp = fcr_dmdis[stream_sel_pp];
assign dis_stream_mux_pp = dis_stream[stream_sel_pp];
assign init_haddr_mux_pp = init_haddr[stream_sel_pp];

assign cr_ct_swap_mp = last_trnsct_m;
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_cr_ct_swap_mp (
    .i_addr(stream_sel_mp),
    .i_en(cr_ct_swap_mp),
    .o_out(cr_ct_swap_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_ndtr_decr_mp (
    .i_addr(stream_sel_mp),
    .i_en(ndtr_decr_mp),
    .o_out(ndtr_decr_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_ndtr_src_decr_mp (
    .i_addr(stream_sel_mp),
    .i_en(ndtr_src_decr_mp),
    .o_out(ndtr_src_decr_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_dmbuf_empty_mp (
    .i_addr(stream_sel_mp),
    .i_en(dmbuf_empty_mp),
    .o_out(dmbuf_empty_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_smaller_size_mp (
    .i_addr(stream_sel_mp),
    .i_en(smaller_size_mp),
    .o_out(smaller_size_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_ahb_fail_mp (
    .i_addr(stream_sel_mp),
    .i_en(fail_m),
    .o_out(ahb_fail_dc_mp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_idle_mp (
    .i_addr(stream_sel_mp),
    .i_en(idle_m),
    .o_out(idle_dc_mp)
);

assign cr_ct_swap_pp = 1'b0;
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_cr_ct_swap_pp (
    .i_addr(stream_sel_pp),
    .i_en(cr_ct_swap_pp),
    .o_out(cr_ct_swap_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_ndtr_decr_pp (
    .i_addr(stream_sel_pp),
    .i_en(ndtr_decr_pp),
    .o_out(ndtr_decr_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_ndtr_src_decr_pp (
    .i_addr(stream_sel_pp),
    .i_en(ndtr_src_decr_pp),
    .o_out(ndtr_src_decr_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_dmbuf_empty_pp (
    .i_addr(stream_sel_pp),
    .i_en(dmbuf_empty_pp),
    .o_out(dmbuf_empty_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_smaller_size_pp (
    .i_addr(stream_sel_pp),
    .i_en(smaller_size_pp),
    .o_out(smaller_size_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_ahb_fail_pp (
    .i_addr(stream_sel_pp),
    .i_en(fail_p),
    .o_out(ahb_fail_dc_pp)
);
dma_dc #(dma_log2(numb_ch), 1, numb_ch) dma_dc_idle_pp (
    .i_addr(stream_sel_pp),
    .i_en(idle_p),
    .o_out(idle_dc_pp)
);

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: cr_ct_swap_dc_merge
        assign cr_ct_swap_dc_final[ch] = cr_ct_swap_dc_mp[ch] | cr_ct_swap_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: ndtr_decr_dc_merge
        assign ndtr_decr_dc_final[ch] = ndtr_decr_dc_mp[ch] | ndtr_decr_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: ndtr_src_decr_dc_merge
        assign ndtr_src_decr_dc_final[ch] = ndtr_src_decr_dc_mp[ch] | ndtr_src_decr_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: dmbuf_empty_dc_merge
        assign dmbuf_empty_dc_final[ch] = dmbuf_empty_dc_mp[ch] | dmbuf_empty_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: smaller_size_dc_merge
        assign smaller_size_dc_final[ch] = smaller_size_dc_mp[ch] | smaller_size_dc_pp[ch];
    end
    for (ch = 0; ch < numb_ch; ++ch) begin: ahb_fail_dc_merge
        assign ahb_fail_dc_final[ch] = ahb_fail_dc_mp[ch] | ahb_fail_dc_pp[ch];
    end
endgenerate

// assign cr_ct_swap_dc_final = cr_ct_swap_dc_mp | cr_ct_swap_dc_pp;
// assign ndtr_decr_dc_final = ndtr_decr_dc_mp | ndtr_decr_dc_pp;
// assign ndtr_src_decr_dc_final = ndtr_src_decr_dc_mp | ndtr_src_decr_dc_pp;
// assign dmbuf_empty_dc_final = dmbuf_empty_dc_mp | dmbuf_empty_dc_pp;
// assign smaller_size_dc_final = smaller_size_dc_mp | smaller_size_dc_pp;
// assign ahb_fail_dc_final = ahb_fail_dc_mp | ahb_fail_dc_pp;
// // assign idle_dc_final = idle_dc_mp | idle_dc_pp;

generate
    for (ch = 0; ch < numb_ch; ++ch) begin: init_isr
        assign isr_teif_set[ch] = /*ahb_fail_dc*/ahb_fail_dc_final[ch];
        assign isr_dmeif_set[ch] = 1'b0;
        assign isr_feif_set[ch] = fifo_overrun[ch] | fifo_underrun[ch];
    end
endgenerate

dma_regs #(numb_ch, fifo_size_exp) dma_regs (
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
    .i_cr_ct_swap(/*cr_ct_swap_dc*/cr_ct_swap_dc_final),
    .i_ndtr_decr(/*ndtr_decr_dc*/ndtr_decr_dc_final),
    .i_ndtr_src_decr(/*ndtr_src_decr_dc*/ndtr_src_decr_dc_final),
    .i_fifo_fullness(fifo_fullness),
    .i_fifo_empty(fifo_empty),
    .i_fifo_full(fifo_full),
    .i_idle_mem(/*idle_m_dc*/idle_dc_mp),
    .i_idle_periph(/*idle_p_dc*/idle_dc_pp),
    .i_dmbuf_empty(/*dmbuf_empty_dc*/dmbuf_empty_dc_final),
    .i_smaller_size(/*smaller_size_dc*/smaller_size_dc_final),
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
    for (ch = 0; ch < numb_ch; ++ch) begin: interrupt_generation
        assign o_interrupt[ch] = (isr_tcif[ch] & cr_tcie[ch]) | (isr_htif[ch] & cr_htie[ch]) |
            (isr_teif[ch] & cr_teie[ch]) | (isr_dmeif[ch] & cr_dmeie[ch]) | (isr_feif[ch] & fcr_feie[ch]);
    end
endgenerate

endmodule
