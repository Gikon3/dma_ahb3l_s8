module dma_regs #(parameter numb_ch = 1, fifo_size_exp = 5) (
    input  logic        i_clk,
    input  logic        i_nreset,

    input  logic [31:0] i_addr,
    input  logic        i_read_en,
    input  logic        i_write_en,
    input  logic [3:0]  i_byte_strobe,
    input  logic [31:0] i_wdata,
    output logic [31:0] o_rdata,

    // flag status
    input  logic        i_isr_teif_set[numb_ch-1:0],
    input  logic        i_isr_dmeif_set[numb_ch-1:0],
    input  logic        i_isr_feif_set[numb_ch-1:0],
    output logic        o_isr_tcif[numb_ch-1:0],
    output logic        o_isr_htif[numb_ch-1:0],
    output logic        o_isr_teif[numb_ch-1:0],
    output logic        o_isr_dmeif[numb_ch-1:0],
    output logic        o_isr_feif[numb_ch-1:0],

    // config signals
    input  logic        i_cr_ct_swap[numb_ch-1:0],
    input  logic        i_ndtr_decr[numb_ch-1:0],
    input  logic        i_ndtr_src_decr[numb_ch-1:0],
    input  logic [1:0]  i_fifo_fullness[numb_ch-1:0],
    input  logic        i_fifo_empty[numb_ch-1:0],
    input  logic        i_fifo_full[numb_ch-1:0],
    input  logic        i_idle_mem[numb_ch-1:0],
    input  logic        i_idle_periph[numb_ch-1:0],
    input  logic        i_dmbuf_empty[numb_ch-1:0],
    input  logic        i_smaller_size[numb_ch-1:0],
    input  logic        i_ltrnsct[numb_ch-1:0],

    output logic [2:0]  o_cr_chsel[numb_ch-1:0],
    output logic [1:0]  o_cr_mburst[numb_ch-1:0],
    output logic [1:0]  o_cr_pburst[numb_ch-1:0],
    output logic [1:0]  o_cr_pl[numb_ch-1:0],
    output logic        o_cr_pincos[numb_ch-1:0],
    output logic [1:0]  o_cr_msize[numb_ch-1:0],
    output logic [1:0]  o_cr_psize[numb_ch-1:0],
    output logic        o_cr_minc[numb_ch-1:0],
    output logic        o_cr_pinc[numb_ch-1:0],
    output logic [1:0]  o_cr_dir[numb_ch-1:0],
    output logic        o_cr_tcie[numb_ch-1:0],
    output logic        o_cr_htie[numb_ch-1:0],
    output logic        o_cr_teie[numb_ch-1:0],
    output logic        o_cr_dmeie[numb_ch-1:0],
    output logic        o_cr_en[numb_ch-1:0],
    output logic [15:0] o_ndtr[numb_ch-1:0],
    output logic [17:0] o_ndtr_src[numb_ch-1:0],
    output logic [31:0] o_par[numb_ch-1:0],
    output logic [31:0] o_mar[numb_ch-1:0],
    output logic        o_fcr_feie[numb_ch-1:0],
    output logic        o_fcr_dmdis[numb_ch-1:0],
    output logic [1:0]  o_fcr_fth[numb_ch-1:0],

    output logic        o_dis_stream[numb_ch-1:0],
    output logic        o_init_haddr[numb_ch-1:0],
    output logic        o_fifo_flush[numb_ch-1:0]
    );

    enum logic [1:0] {periph_to_mem, mem_to_periph, mem_to_mem} dir_list;
    enum logic [1:0] {bytew, hword, word} size_list;
    enum logic [1:0] {single, inc4, inc8, inc16} burst_list;
    enum logic [1:0] {quarter, half, three_quarter, full} filling_fifo_list;

    localparam fifo_size = 2 ** fifo_size_exp;

    localparam wbus = 32;
    localparam wbyte = 8;

    localparam numb_conf_regs = 6;
    localparam width_addr = /*$clog2*/dma_log2(numb_conf_regs * numb_ch + 4 - 1);

    localparam lisr_addr   = 4'h0;  // DMA low interrupt status register
    localparam hisr_addr   = 4'h1;  // DMA high interrupt status register
    localparam lifcr_addr  = 4'h2;  // DMA low interrupt flag clear register
    localparam hifcr_addr  = 4'h3;  // DMA high interrupt flag clear register
    localparam s0cr_addr   = 4'h4;  // DMA stream 0 configuration register
    localparam s0ndtr_addr = 4'h5;  // DMA stream 0 number of data register
    localparam s0par_addr  = 4'h6;  // DMA stream 0 peripheral address register
    localparam s0m0ar_addr = 4'h7;  // DMA stream 0 memory 0 address register
    localparam s0m1ar_addr = 4'h8;  // DMA stream 0 memory 1 address register
    localparam s0fcr_addr  = 4'h9;  // DMA stream 0 FIFO control register

    localparam numb_ch_max_lisr = wbus / 16 * 2 + wbus % 16 / numb_conf_regs;
    localparam numb_ch_max_hisr = numb_ch_max_lisr;

    localparam strobe_b0 = 4'b0001;
    localparam strobe_b1 = 4'b0010;
    localparam strobe_b2 = 4'b0100;
    localparam strobe_b3 = 4'b1000;

    genvar ch;

    logic [2:0]             size_mem_bytes[numb_ch-1:0];
    logic [2:0]             size_per_bytes[numb_ch-1:0];
    logic [4:0]             burst_mem_beats[numb_ch-1:0];
    logic [4:0]             burst_per_beats[numb_ch-1:0];
    logic [fifo_size_exp:0] fifo_thr_bytes[numb_ch-1:0];
    logic [7:0]             mem_transm_bytes[numb_ch-1:0];
    logic [7:0]             per_transm_bytes[numb_ch-1:0];
    logic                   fifo_fth_valid[numb_ch-1:0];
    logic [2:0]             fcr_fs[numb_ch-1:0];

    logic [width_addr-1:0]  addr;
    logic                   byte0_sel;
    logic                   byte1_sel;
    logic                   byte2_sel;
    logic                   byte3_sel;

    logic               reg_lisr_sel;
    logic               reg_hisr_sel;
    logic               reg_lifcr_sel;
    logic               reg_hifcr_sel;
    logic               reg_cr_sel[numb_ch-1:0];
    logic               reg_ndtr_sel[numb_ch-1:0];
    logic               reg_par_sel[numb_ch-1:0];
    logic               reg_m0ar_sel[numb_ch-1:0];
    logic               reg_m1ar_sel[numb_ch-1:0];
    logic               reg_fcr_sel[numb_ch-1:0];

    logic [5:0]         isr[numb_ch-1:0];
    logic               isr_tcif_clr[numb_ch-1:0];
    logic               isr_tcif_set[numb_ch-1:0];
    logic               isr_htif_clr[numb_ch-1:0];
    logic               isr_htif_set[numb_ch-1:0];
    logic               isr_teif_clr[numb_ch-1:0];
    logic               isr_dmeif_clr[numb_ch-1:0];
    logic               isr_feif_clr[numb_ch-1:0];
    logic               isr_tcif[numb_ch-1:0];
    logic               isr_htif[numb_ch-1:0];
    logic               isr_teif[numb_ch-1:0];
    logic               isr_dmeif[numb_ch-1:0];
    logic               isr_feif_set[numb_ch-1:0];
    logic               isr_feif[numb_ch-1:0];
    logic               cr_write_en[numb_ch-1:0];
    logic               cr_write_en_prot[numb_ch-1:0];
    logic [2:0]         cr_chsel[numb_ch-1:0];
    logic               cr_mburst_correct[numb_ch-1:0];
    logic [1:0]         cr_mburst[numb_ch-1:0];
    logic               cr_pburst_correct[numb_ch-1:0];
    logic [1:0]         cr_pburst[numb_ch-1:0];
    logic               cr_ct_swap[numb_ch-1:0];
    logic               cr_ct[numb_ch-1:0];
    logic               cr_dbm[numb_ch-1:0];
    logic [1:0]         cr_pl[numb_ch-1:0];
    logic               cr_pincos_correct[numb_ch-1:0];
    logic               cr_pincos[numb_ch-1:0];
    logic               cr_msize_correct[numb_ch-1:0];
    logic [1:0]         cr_msize[numb_ch-1:0];
    logic [1:0]         cr_psize[numb_ch-1:0];
    logic               cr_minc[numb_ch-1:0];
    logic               cr_pinc[numb_ch-1:0];
    logic               cr_circ_correct0[numb_ch-1:0];
    logic               cr_circ_correct1[numb_ch-1:0];
    logic               cr_circ[numb_ch-1:0];
    logic [1:0]         cr_dir[numb_ch-1:0];
    logic               dir_pbus_to_mbus[numb_ch-1:0];
    logic               cr_pfctrl_correct[numb_ch-1:0];
    logic               cr_pfctrl[numb_ch-1:0];
    logic               cr_tcie[numb_ch-1:0];
    logic               cr_htie[numb_ch-1:0];
    logic               cr_teie[numb_ch-1:0];
    logic               cr_dmeie[numb_ch-1:0];
    logic               cr_en_write_en[numb_ch-1:0];
    logic               cr_en_set[numb_ch-1:0];
    logic               dmdis_cmplt[numb_ch-1:0];
    logic               dmen_cmplt[numb_ch-1:0];
    logic               to_per_cmplt[numb_ch-1:0];
    logic               cr_en_clr_dis[numb_ch-1:0];
    logic               cr_en_clr_cmplt[numb_ch-1:0];
    logic               cr_en_clr[numb_ch-1:0];
    logic               cr_en[numb_ch-1:0];
    logic               pend_cr_en[numb_ch-1:0];
    logic               pend_dis_stream_set[numb_ch-1:0];
    logic               pend_dis_stream_clr[numb_ch-1:0];
    logic               pend_dis_stream[numb_ch-1:0];
    logic               pause_set[numb_ch-1:0];
    logic               pause_clr[numb_ch-1:0];
    logic               pause[numb_ch-1:0];
    logic [17:0]        nxt_ndtr_src[numb_ch-1:0];
    logic               ndtr_write_en[numb_ch-1:0];
    logic               ndtr_reload_no_circ[numb_ch-1:0];
    logic               ndtr_reload_circ[numb_ch-1:0];
    logic               ndtr_load[numb_ch-1:0];
    logic               ndtr_correct[numb_ch-1:0];
    logic [15:0]        ndtr[numb_ch-1:0];
    logic               ndtr_shadow_correct[numb_ch-1:0];
    logic [15:0]        ndtr_shadow[numb_ch-1:0];
    logic               ndtr_empty[numb_ch-1:0];
    logic               end_trans[numb_ch-1:0];
    logic               ndtr_src_reload_no_circ[numb_ch-1:0];
    logic               ndtr_src_reload_circ[numb_ch-1:0];
    logic               ndtr_src_load[numb_ch-1:0];
    logic               ndtr_src_correct[numb_ch-1:0];
    logic [17:0]        ndtr_src[numb_ch-1:0];
    logic               ndtr_src_empty[numb_ch-1:0];
    logic               par_write_en[numb_ch-1:0];
    logic [wbus-1:0]    par[numb_ch-1:0];
    logic               m0ar_write_en[numb_ch-1:0];
    logic [wbus-1:0]    m0ar[numb_ch-1:0];
    logic               m1ar_write_en[numb_ch-1:0];
    logic [wbus-1:0]    m1ar[numb_ch-1:0];
    logic               fcr_write_en[numb_ch-1:0];
    logic               fcr_feie[numb_ch-1:0];
    logic               fcr_dmdis[numb_ch-1:0];
    logic [1:0]         fcr_fth[numb_ch-1:0];

    logic [wbus-1:0]    lisr;
    logic [wbus-1:0]    hisr;
    logic [wbus-1:0]    lisr_out;
    logic [wbus-1:0]    hisr_out;
    logic [wbus-1:0]    lifcr_out;
    logic [wbus-1:0]    hifcr_out;
    logic [wbus-1:0]    cr_out[numb_ch-1:0];
    logic [wbus-1:0]    ndtr_out[numb_ch-1:0];
    logic [wbus-1:0]    par_out[numb_ch-1:0];
    logic [wbus-1:0]    m0ar_out[numb_ch-1:0];
    logic [wbus-1:0]    m1ar_out[numb_ch-1:0];
    logic [wbus-1:0]    fcr_out[numb_ch-1:0];

    logic               sel_mar[numb_ch-1:0];

    // fifo threshold check
    always_comb begin: size_mem_bytes_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            case (cr_msize[ch_cnt])
                bytew: begin
                    size_mem_bytes[ch_cnt] = 3'd1;
                end
                hword: begin
                    size_mem_bytes[ch_cnt] = 3'd2;
                end
                default: begin
                    size_mem_bytes[ch_cnt] = 3'd4;
                end
            endcase
        end
    end

    always_comb begin: size_periph_bytes_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            case (cr_psize[ch_cnt])
                bytew: begin
                    size_per_bytes[ch_cnt] = 3'd1;
                end
                hword: begin
                    size_per_bytes[ch_cnt] = 3'd2;
                end
                default: begin
                    size_per_bytes[ch_cnt] = 3'd4;
                end
            endcase
        end
    end

    always_comb begin: burst_mem_beats_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            case (cr_mburst[ch_cnt])
                single: begin
                    burst_mem_beats[ch_cnt] = 5'd1;
                end
                inc4: begin
                    burst_mem_beats[ch_cnt] = 5'd4;
                end
                inc8: begin
                    burst_mem_beats[ch_cnt] = 5'd8;
                end
                default: begin
                    burst_mem_beats[ch_cnt] = 5'd16;
                end
            endcase
        end
    end

    always_comb begin: burst_periph_beats_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            case (cr_pburst[ch_cnt])
                single: begin
                    burst_per_beats[ch_cnt] = 5'd1;
                end
                inc4: begin
                    burst_per_beats[ch_cnt] = 5'd4;
                end
                inc8: begin
                    burst_per_beats[ch_cnt] = 5'd8;
                end
                default: begin
                    burst_per_beats[ch_cnt] = 5'd16;
                end
            endcase
        end
    end

    always_comb begin: fifo_threshold_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            case (fcr_fth[ch_cnt])
                quarter: begin
                    fifo_thr_bytes[ch_cnt] = fifo_size / 4;
                end
                half: begin
                    fifo_thr_bytes[ch_cnt] = fifo_size / 2;
                end
                three_quarter: begin
                    fifo_thr_bytes[ch_cnt] = fifo_size / 4 * 3;
                end
                default: begin
                    fifo_thr_bytes[ch_cnt] = fifo_size;
                end
            endcase
        end
    end

    generate
        for(ch = 0; ch < numb_ch; ++ch) begin: fifo_fth_valid_check
            assign mem_transm_bytes[ch] = size_mem_bytes[ch] * burst_mem_beats[ch];
            assign per_transm_bytes[ch] = size_per_bytes[ch] * burst_per_beats[ch];
            assign fifo_fth_valid[ch] = (fifo_thr_bytes[ch] >= mem_transm_bytes[ch]) &
                (fifo_thr_bytes[ch] >= per_transm_bytes[ch]);
        end
    endgenerate

    // fifo flags
    always_comb begin: fifo_flags_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            if (i_fifo_empty[ch_cnt]) begin
                fcr_fs[ch_cnt] = 3'h4;
            end
            else if (i_fifo_full[ch_cnt]) begin
                fcr_fs[ch_cnt] = 3'h5;
            end
            else begin
                fcr_fs[ch_cnt] = {1'b0, i_fifo_fullness[ch_cnt]};
            end
        end
    end

    // address decode
    assign addr = i_addr[width_addr+2-1:2];

    assign byte0_sel = i_byte_strobe[0];
    assign byte1_sel = i_byte_strobe[1];
    assign byte2_sel = i_byte_strobe[2];
    assign byte3_sel = i_byte_strobe[3];

    always_comb begin: reg_select
        logic [width_addr-1:0] addr_shift;
        reg_lisr_sel = (addr == lisr_addr);
        reg_hisr_sel = (addr == hisr_addr);
        reg_lifcr_sel = (addr == lifcr_addr);
        reg_hifcr_sel = (addr == hifcr_addr);
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            addr_shift = (numb_conf_regs * ch_cnt);
            reg_cr_sel[ch_cnt] = (addr == (s0cr_addr + addr_shift));
            reg_ndtr_sel[ch_cnt] = (addr == (s0ndtr_addr + addr_shift));
            reg_par_sel[ch_cnt] = (addr == (s0par_addr + addr_shift));
            reg_m0ar_sel[ch_cnt] = (addr == (s0m0ar_addr + addr_shift));
            reg_m1ar_sel[ch_cnt] = (addr == (s0m1ar_addr + addr_shift));
            reg_fcr_sel[ch_cnt] = (addr == (s0fcr_addr + addr_shift));
        end
    end

    // isr combine bits
    generate
        for(ch = 0; ch < numb_ch; ++ch) begin: isr_flags_combine
            assign isr[ch] = {isr_tcif[ch], isr_htif[ch], isr_teif[ch], isr_dmeif[ch], 1'b0, isr_feif[ch]};
        end
    endgenerate

    // write regs
    always_comb begin: isr_clr_flags
        logic isr_byte_sel;
        logic write_en;
        logic reg_sel;
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            isr_byte_sel = ch_cnt < numb_ch_max_lisr ? ch_cnt: ch_cnt - numb_ch_max_lisr;
            write_en = i_write_en & i_byte_strobe[isr_byte_sel];
            reg_sel = (reg_lifcr_sel && ch_cnt < numb_ch_max_lisr) || (reg_hifcr_sel && ch_cnt >= numb_ch_max_lisr);
            isr_tcif_clr[ch_cnt] = (write_en & reg_sel & i_wdata[isr_byte_sel * wbyte + 0]) |
                (cr_circ[ch_cnt] & isr_tcif[ch_cnt] & !pause[ch_cnt]);
            isr_htif_clr[ch_cnt] = (write_en & reg_sel & i_wdata[isr_byte_sel * wbyte + 2]) |
                (cr_circ[ch_cnt] & isr_tcif[ch_cnt] & !pause[ch_cnt]);
            isr_teif_clr[ch_cnt] = write_en & reg_sel & i_wdata[isr_byte_sel * wbyte + 3];
            isr_dmeif_clr[ch_cnt] = write_en & reg_sel & i_wdata[isr_byte_sel * wbyte + 4];
            isr_feif_clr[ch_cnt] = write_en & reg_sel & i_wdata[isr_byte_sel * wbyte + 5];
        end
    end

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: interrupt_flags_reg
            assign isr_tcif_set[ch] = (cr_en[ch] & end_trans[ch]) | pause_set[ch] | (cr_circ[ch] & ndtr_empty[ch]);
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) isr_tcif[ch] <= 1'b0;
                else if (isr_tcif_set[ch]) isr_tcif[ch] <= 1'b1;
                else if (isr_tcif_clr[ch]) isr_tcif[ch] <= 1'b0;

            assign isr_htif_set[ch] = cr_en[ch] & (ndtr[ch] == (ndtr_shadow[ch] / 2));
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) isr_htif[ch] <= 1'b0;
                else if (isr_htif_set[ch]) isr_htif[ch] <= 1'b1;
                else if (isr_htif_clr[ch]) isr_htif[ch] <= 1'b0;

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) isr_teif[ch] <= 1'b0;
                else if (i_isr_teif_set[ch]) isr_teif[ch] <= 1'b1;
                else if (isr_teif_clr[ch]) isr_teif[ch] <= 1'b0;

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) isr_dmeif[ch] <= 1'b0;
                else if (i_isr_dmeif_set[ch]) isr_dmeif[ch] <= 1'b1;
                else if (isr_dmeif_clr[ch]) isr_dmeif[ch] <= 1'b0;

            assign isr_feif_set[ch] = i_isr_feif_set[ch] | (pend_cr_en[ch] & ~fifo_fth_valid[ch]);
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) isr_feif[ch] <= 1'b0;
                else if (isr_feif_set[ch]) isr_feif[ch] <= 1'b1;
                else if (isr_feif_clr[ch]) isr_feif[ch] <= 1'b0;
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: ctrl_reg
            assign cr_write_en[ch] = i_write_en & reg_cr_sel[ch];
            assign cr_write_en_prot[ch] = cr_write_en[ch] & ~cr_en[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_chsel[ch] <= 3'd0;
                else if (cr_write_en_prot[ch] && byte3_sel) cr_chsel[ch] <= i_wdata[27:25];

            assign cr_mburst_correct[ch] = pend_cr_en[ch] & ~fcr_dmdis[ch] & |cr_mburst[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_mburst[ch] <= 2'd0;
                else if (cr_write_en_prot[ch]) begin
                    if (byte3_sel) cr_mburst[ch][1] <= i_wdata[24];
                    if (byte2_sel) cr_mburst[ch][0] <= i_wdata[23];
                end
                else if (cr_mburst_correct[ch]) cr_mburst[ch] <= 2'h0;

            assign cr_pburst_correct[ch] = pend_cr_en[ch] & ~fcr_dmdis[ch] & |cr_pburst[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_pburst[ch] <= 2'd0;
                else if (cr_write_en_prot[ch] && byte2_sel) cr_pburst[ch] <= i_wdata[22:21];
                else if (cr_pburst_correct[ch]) cr_pburst[ch] <= 2'h0;

            assign cr_ct_swap[ch] = cr_en[ch] & cr_circ[ch] & i_cr_ct_swap[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_ct[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte2_sel) cr_ct[ch] <= i_wdata[19];
                else if (cr_ct_swap[ch]) cr_ct[ch] <= ~cr_ct[ch];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_dbm[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte2_sel) cr_dbm[ch] <= i_wdata[18];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_pl[ch] <= 2'd0;
                else if (cr_write_en_prot[ch] && byte2_sel) cr_pl[ch] <= i_wdata[17:16];

            assign cr_pincos_correct[ch] = pend_cr_en[ch] & (~fcr_dmdis[ch] | (|cr_pburst[ch])) & cr_pincos[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_pincos[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte1_sel) cr_pincos[ch] <= i_wdata[15];
                else if (cr_pincos_correct[ch]) cr_pincos[ch] <= 1'b0;

            assign cr_msize_correct[ch] = pend_cr_en[ch] && ~fcr_dmdis[ch] && cr_msize[ch] != cr_psize[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_msize[ch] <= 2'd0;
                else if (cr_write_en_prot[ch] && byte1_sel) cr_msize[ch] <= i_wdata[14:13];
                else if (cr_msize_correct[ch]) cr_msize[ch] <= cr_psize[ch];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_psize[ch] <= 2'd0;
                else if (cr_write_en_prot[ch] && byte1_sel) cr_psize[ch] <= i_wdata[12:11];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_minc[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte1_sel) cr_minc[ch] <= i_wdata[10];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_pinc[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte1_sel) cr_pinc[ch] <= i_wdata[9];

            assign cr_circ_correct0[ch] = pend_cr_en[ch] & cr_pfctrl[ch] & ~cr_pfctrl_correct[ch] & cr_circ[ch];
            assign cr_circ_correct1[ch] = pend_cr_en[ch] & cr_dbm[ch] & ~cr_circ[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_circ[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte1_sel) cr_circ[ch] <= i_wdata[8];
                else if (cr_circ_correct0[ch]) cr_circ[ch] <= 1'b0;
                else if (cr_circ_correct1[ch]) cr_circ[ch] <= 1'b1;

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_dir[ch] <= 2'd0;
                else if (cr_write_en_prot[ch] && byte0_sel) cr_dir[ch] <= i_wdata[7:6];

            assign dir_pbus_to_mbus[ch] = (cr_dir[ch] == periph_to_mem) || (cr_dir[ch] == mem_to_mem);

            assign cr_pfctrl_correct[ch] = pend_cr_en[ch] & (cr_dir[ch] == mem_to_mem) & cr_pfctrl[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_pfctrl[ch] <= 1'd0;
                else if (cr_write_en_prot[ch] && byte0_sel) cr_pfctrl[ch] <= i_wdata[5];
                else if (cr_pfctrl_correct[ch]) cr_pfctrl[ch] <= 1'b0;

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_tcie[ch] <= 1'd0;
                else if (cr_write_en[ch] && byte0_sel) cr_tcie[ch] <= i_wdata[4];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_htie[ch] <= 1'd0;
                else if (cr_write_en[ch] && byte0_sel) cr_htie[ch] <= i_wdata[3];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_teie[ch] <= 1'd0;
                else if (cr_write_en[ch] && byte0_sel) cr_teie[ch] <= i_wdata[2];

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_dmeie[ch] <= 1'd0;
                else if (cr_write_en[ch] && byte0_sel) cr_dmeie[ch] <= i_wdata[1];

            assign cr_en_write_en[ch] = i_write_en & reg_cr_sel[ch] & byte0_sel;
            assign cr_en_set[ch] = pend_cr_en[ch] & fifo_fth_valid[ch];
            assign dmdis_cmplt[ch] = fcr_dmdis[ch] & i_smaller_size[ch];
            assign dmen_cmplt[ch] = ~fcr_dmdis[ch] & i_dmbuf_empty[ch];
            assign to_per_cmplt[ch] = (cr_dir[ch] == mem_to_periph) & i_idle_mem[ch] & i_idle_periph[ch];
            assign cr_en_clr_dis[ch] = pend_dis_stream[ch] & (dmdis_cmplt[ch] | dmen_cmplt[ch] | to_per_cmplt[ch]);
            assign cr_en_clr_cmplt[ch] = ndtr_empty[ch] & ~cr_circ[ch];
            assign cr_en_clr[ch] = cr_en_clr_dis[ch] | cr_en_clr_cmplt[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) cr_en[ch] <= 1'd0;
                else if (cr_en_set[ch]) cr_en[ch] <= 1'b1;
                else if (cr_en_clr[ch]) cr_en[ch] <= 1'b0;

            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) pend_cr_en[ch] <= 1'd0;
                else pend_cr_en[ch] <= cr_en_write_en[ch] & i_wdata[0];

            assign pend_dis_stream_set[ch] = (cr_en[ch] & cr_en_write_en[ch] & ~i_wdata[0]) |
                (cr_pfctrl[ch] & i_ltrnsct[ch]);
            assign pend_dis_stream_clr[ch] = (~pend_cr_en[ch] & ~cr_en[ch]) | cr_en_clr[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) pend_dis_stream[ch] <= 1'd0;
                else if (pend_dis_stream_set[ch]) pend_dis_stream[ch] <= 1'b1;
                else if (pend_dis_stream_clr[ch]) pend_dis_stream[ch] <= 1'b0;

            assign pause_set[ch] = cr_en_clr_dis[ch];
            assign pause_clr[ch] = cr_en[ch] | (cr_write_en_prot[ch] & ~(cr_en_write_en[ch] & i_wdata[0])) |
                ndtr_write_en[ch] | par_write_en[ch] | m0ar_write_en[ch] | m1ar_write_en[ch] | fcr_write_en[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) pause[ch] <= 1'b0;
                else if (pause_set[ch]) pause[ch] <= 1'b1;
                else if (pause_clr[ch]) pause[ch] <= 1'b0;
        end
    endgenerate

    always_comb begin: next_ndtr_src_decode
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            if (dir_pbus_to_mbus[ch_cnt]) begin
                nxt_ndtr_src[ch_cnt] = (ndtr_shadow[ch_cnt] << cr_msize[ch_cnt]) >> cr_psize[ch_cnt];
            end
            else begin
                nxt_ndtr_src[ch_cnt] = (ndtr_shadow[ch_cnt] << cr_psize[ch_cnt]) >> cr_msize[ch_cnt];
            end
        end
    end

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: ndt_reg
            assign ndtr_write_en[ch] = i_write_en & reg_ndtr_sel[ch] & ~cr_en[ch];
            assign ndtr_reload_no_circ[ch] = ~cr_en[ch] & cr_en_write_en[ch] & ~pause[ch];
            assign ndtr_reload_circ[ch] = cr_en[ch] & cr_circ[ch] & ndtr_empty[ch] & ~pause[ch];
            assign ndtr_load[ch] = ndtr_reload_no_circ[ch] | ndtr_reload_circ[ch];
            assign ndtr_correct[ch] = pend_cr_en[ch] & cr_pfctrl[ch] & ~cr_pfctrl_correct[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) ndtr[ch] <= 16'd0;
                else if (ndtr_correct[ch]) ndtr[ch] <= 16'hFFFF;
                else if (ndtr_write_en[ch]) begin
                    if (byte0_sel) ndtr[ch][7:0] <= i_wdata[7:0];
                    if (byte1_sel) ndtr[ch][15:8] <= i_wdata[15:8];
                end
                else if (ndtr_load[ch]) ndtr[ch] <= ndtr_shadow[ch];
                else if (i_ndtr_decr[ch]) ndtr[ch] <= ndtr[ch] - 16'd1;

            assign ndtr_shadow_correct[ch] = ndtr_correct[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) ndtr_shadow[ch] <= 16'd0;
                else if (ndtr_shadow_correct[ch]) ndtr_shadow[ch] <= 16'hFFFF;
                else if (ndtr_write_en[ch]) begin
                    if (byte0_sel) ndtr_shadow[ch][7:0] <= i_wdata[7:0];
                    if (byte1_sel) ndtr_shadow[ch][15:8] <= i_wdata[15:8];
                end

            assign ndtr_empty[ch] = ~|ndtr[ch];
            assign end_trans[ch] = ndtr_empty[ch] & i_fifo_empty[ch];

            assign ndtr_src_reload_no_circ[ch] = ndtr_reload_no_circ[ch];
            assign ndtr_src_reload_circ[ch] = cr_en[ch] & cr_circ[ch] & ndtr_src_empty[ch] & ~pause[ch];
            assign ndtr_src_load[ch] = ndtr_src_reload_no_circ[ch] | ndtr_src_reload_circ[ch];
            assign ndtr_src_correct[ch] = ndtr_correct[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) ndtr_src[ch] <= 18'd0;
                else if (ndtr_src_correct[ch]) ndtr_src[ch] <= 18'h3FFFF;
                else if (cr_msize_correct[ch]) ndtr_src[ch] <= {2'h0, ndtr_shadow[ch]};
                else if (ndtr_src_load[ch]) ndtr_src[ch] <= nxt_ndtr_src[ch];
                else if (i_ndtr_src_decr[ch]) ndtr_src[ch] <= ndtr_src[ch] - 16'd1;

             assign ndtr_src_empty[ch] = ~|ndtr_src[ch];
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: pa_reg
            assign par_write_en[ch] = i_write_en & reg_par_sel[ch] & ~cr_en[ch];
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) par[ch] <= {wbus{1'b0}};
                else if (par_write_en[ch]) begin
                    if (byte0_sel) par[ch][7:0] <= i_wdata[7:0];
                    if (byte1_sel) par[ch][15:8] <= i_wdata[15:8];
                    if (byte2_sel) par[ch][23:16] <= i_wdata[23:16];
                    if (byte3_sel) par[ch][31:24] <= i_wdata[31:24];
                end
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: m0a_reg
            assign m0ar_write_en[ch] = i_write_en & reg_m0ar_sel[ch] & (~cr_en[ch] | cr_dbm[ch] & cr_ct[ch]);
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) m0ar[ch] <= {wbus{1'b0}};
                else if (m0ar_write_en[ch]) begin
                    if (byte0_sel) m0ar[ch][7:0] <= i_wdata[7:0];
                    if (byte1_sel) m0ar[ch][15:8] <= i_wdata[15:8];
                    if (byte2_sel) m0ar[ch][23:16] <= i_wdata[23:16];
                    if (byte3_sel) m0ar[ch][31:24] <= i_wdata[31:24];
                end
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: m1a_reg
            assign m1ar_write_en[ch] = i_write_en & reg_m1ar_sel[ch] & (~cr_en[ch] | cr_dbm[ch] & ~cr_ct[ch]);
            always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) m1ar[ch] <= {wbus{1'b0}};
                else if (m1ar_write_en[ch]) begin
                    if (byte0_sel) m1ar[ch][7:0] <= i_wdata[7:0];
                    if (byte1_sel) m1ar[ch][15:8] <= i_wdata[15:8];
                    if (byte2_sel) m1ar[ch][23:16] <= i_wdata[23:16];
                    if (byte3_sel) m1ar[ch][31:24] <= i_wdata[31:24];
                end
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: fifo_ctrl_reg
            assign fcr_write_en[ch] = i_write_en & reg_fcr_sel[ch] & ~cr_en[ch];
             always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) fcr_feie[ch] <= 1'b0;
                else if (fcr_write_en[ch] && byte0_sel) fcr_feie[ch] <= i_wdata[7];

             always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) fcr_dmdis[ch] <= 1'b0;
                else if (fcr_write_en[ch] && byte0_sel) fcr_dmdis[ch] <= i_wdata[2];

             always_ff @(posedge i_clk, negedge i_nreset)
                if (!i_nreset) fcr_fth[ch] <= 2'd0;
                else if (fcr_write_en[ch] && byte0_sel) fcr_fth[ch] <= i_wdata[1:0];
       end
    endgenerate

    // read regs
    always_comb begin: combine_lisr_all
        lisr = 0;
        for (int ch_cnt = 0; ch_cnt < (numb_ch < numb_ch_max_lisr ? numb_ch: numb_ch_max_lisr); ++ch_cnt) begin
            lisr |= isr[ch_cnt] << (wbyte * ch_cnt);
        end
    end

    always_comb begin: combine_hisr_all
        hisr = 0;
        for (int ch_cnt = 0; ch_cnt < numb_ch - numb_ch_max_lisr; ++ch_cnt) begin
            hisr |= isr[ch_cnt + numb_ch_max_lisr] << (wbyte * ch_cnt);
        end
    end

    always_comb begin: combine_regs_output
        lisr_out = lisr;
        hisr_out = hisr;
        lifcr_out = 0;
        hifcr_out = 0;
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            cr_out[ch_cnt] = {{(wbus-28){1'b0}}, cr_chsel[ch_cnt], cr_mburst[ch_cnt], cr_pburst[ch_cnt],
                                1'b0, cr_ct[ch_cnt], cr_dbm[ch_cnt], cr_pl[ch_cnt],
                                cr_pincos[ch_cnt], cr_msize[ch_cnt], cr_psize[ch_cnt], cr_minc[ch_cnt],
                                cr_pinc[ch_cnt], cr_circ[ch_cnt], cr_dir[ch_cnt], cr_pfctrl[ch_cnt],
                                cr_tcie[ch_cnt], cr_htie[ch_cnt], cr_teie[ch_cnt], cr_dmeie[ch_cnt],
                                cr_en[ch_cnt]};
            ndtr_out[ch_cnt] = {{(wbus-16){1'b0}}, ndtr[ch_cnt]};
            par_out[ch_cnt] = par[ch_cnt];
            m0ar_out[ch_cnt] = m0ar[ch_cnt];
            m1ar_out[ch_cnt] = m1ar[ch_cnt];
            fcr_out[ch_cnt] = {{(wbus-8){1'b0}}, fcr_feie[ch_cnt], 1'b0, fcr_fs[ch_cnt],
                                 fcr_dmdis[ch_cnt], fcr_fth[ch_cnt]};
        end
    end

    always_comb begin: read_regs_mux
        logic [numb_ch-1:0] cr_sel;
        logic [numb_ch-1:0] ndtr_sel;
        logic [numb_ch-1:0] par_sel;
        logic [numb_ch-1:0] m0ar_sel;
        logic [numb_ch-1:0] m1ar_sel;
        logic [numb_ch-1:0] fcr_sel;
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            cr_sel[ch_cnt] = reg_cr_sel[ch_cnt];
        end
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            ndtr_sel[ch_cnt] = reg_ndtr_sel[ch_cnt];
        end
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            par_sel[ch_cnt] = reg_par_sel[ch_cnt];
        end
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            m0ar_sel[ch_cnt] = reg_m0ar_sel[ch_cnt];
        end
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            m1ar_sel[ch_cnt] = reg_m1ar_sel[ch_cnt];
        end
        for (int ch_cnt = 0; ch_cnt < numb_ch; ++ch_cnt) begin
            fcr_sel[ch_cnt] = reg_fcr_sel[ch_cnt];
        end

        if (i_read_en) begin
            if (reg_lisr_sel) begin
                o_rdata = lisr_out;
            end
            else if (reg_hisr_sel) begin
                o_rdata = hisr_out;
            end
            else if (reg_lifcr_sel) begin
                o_rdata = lifcr_out;
            end
            else if (reg_hifcr_sel) begin
                o_rdata = hifcr_out;
            end
            else if (|cr_sel) begin
                o_rdata = cr_out[dma_log2(cr_sel)];
            end
            else if (|ndtr_sel) begin
                o_rdata = ndtr_out[dma_log2(ndtr_sel)];
            end
            else if (|par_sel) begin
                o_rdata = par_out[dma_log2(par_sel)];
            end
            else if (|m0ar_sel) begin
                o_rdata = m0ar_out[dma_log2(m0ar_sel)];
            end
            else if (|m1ar_sel) begin
                o_rdata = m1ar_out[dma_log2(m1ar_sel)];
            end
            else if (|fcr_sel) begin
                o_rdata = fcr_out[dma_log2(fcr_sel)];
            end
            else begin
                o_rdata = {wbus{1'b0}};
            end
        end
        else begin
            o_rdata = {wbus{1'b0}};
        end
    end

    // output
    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: output_isr_reg
            assign o_isr_tcif[ch] = isr_tcif[ch];
            assign o_isr_htif[ch] = isr_htif[ch];
            assign o_isr_teif[ch] = isr_teif[ch];
            assign o_isr_dmeif[ch] = isr_dmeif[ch];
            assign o_isr_feif[ch] = isr_feif[ch];
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: output_ctrl_reg
            assign sel_mar[ch] = cr_ct[ch] ^ cr_ct_swap[ch];
            assign o_cr_chsel[ch] = cr_chsel[ch];
            assign o_cr_mburst[ch] = cr_mburst[ch];
            assign o_cr_pburst[ch] = cr_pburst[ch];
            assign o_cr_pl[ch] = cr_pl[ch];
            assign o_cr_pincos[ch] = cr_pincos[ch];
            assign o_cr_msize[ch] = cr_msize[ch];
            assign o_cr_psize[ch] = cr_psize[ch];
            assign o_cr_minc[ch] = cr_minc[ch];
            assign o_cr_pinc[ch] = cr_pinc[ch];
            assign o_cr_dir[ch] = cr_dir[ch];
            assign o_cr_tcie[ch] = cr_tcie[ch];
            assign o_cr_htie[ch] = cr_htie[ch];
            assign o_cr_teie[ch] = cr_teie[ch];
            assign o_cr_dmeie[ch] = cr_dmeie[ch];
            assign o_cr_en[ch] = cr_en[ch];
            assign o_ndtr[ch] = ndtr[ch];
            assign o_ndtr_src[ch] = ndtr_src[ch];
            assign o_par[ch] = par[ch];
            assign o_mar[ch] = !sel_mar[ch] ? m0ar[ch]: m1ar[ch];
            assign o_fcr_feie[ch] = fcr_feie[ch];
            assign o_fcr_dmdis[ch] = fcr_dmdis[ch];
            assign o_fcr_fth[ch] = fcr_fth[ch];
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: output_pend_dis_stream
            assign o_dis_stream[ch] = pend_dis_stream[ch];
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: output_init_haddr
            assign o_init_haddr[ch] = cr_en_set[ch] & ~pause[ch];
        end
    endgenerate

    generate
        for (ch = 0; ch < numb_ch; ++ch) begin: output_fifo_flush
            assign o_fifo_flush[ch] = ~cr_en[ch] & ~pause[ch];
        end
    endgenerate

endmodule
