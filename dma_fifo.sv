module dma_fifo #(parameter size_exp = 5) (
    input  logic                i_clk,
    input  logic                i_nreset,

    input  logic                i_put,
    input  logic [1:0]          i_numb_bytes_put,   // x, 4, 2, 1
    input  logic                i_pull,
    input  logic [1:0]          i_numb_bytes_pull,  // x, 4, 2, 1
    input  logic [31:0]         i_wdata,
    input  logic [1:0]          i_filling_thresh,   // 1, 3/4, 1/2, 1/4
    input  logic                i_clear,

    output logic [1:0]          o_fullness,         // 1...3/4, 3/4...1/2, 1/2...1/4, 1/4...0
    output logic                o_empty,
    output logic                o_full,
    output logic                o_overrun,
    output logic                o_underrun,
    output logic [31:0]         o_rdata,

    output logic [size_exp:0]   o_left_put,
    output logic [size_exp:0]   o_left_pull
);

enum logic [1:0] {quarter, half, three_quarter, full} filling_list;

localparam wbus = 32;
localparam wbyte = 8;

localparam size = 2 ** size_exp;    // in bytes
localparam numb_bytes_1_code = 2'h0;
localparam numb_bytes_2_code = 2'h1;
localparam numb_bytes_4_code = 2'h2;
localparam numb_bytes_1 = 3'h1;
localparam numb_bytes_2 = 3'h2;
localparam numb_bytes_4 = 3'h4;
localparam numb_bytes_err = 3'h0;
localparam strobe_1bytes = 4'b0001;
localparam strobe_2bytes = 4'b0011;
localparam strobe_4bytes = 4'b1111;
localparam strobe_err = 4'b0000;

logic [size_exp-1:0]    p_write_nxt;
logic                   p_write_inc;
logic [size_exp-1:0]    p_write;            // write pointer
logic [size_exp-1:0]    p_read_nxt;
logic                   p_read_inc;
logic [size_exp-1:0]    p_read;             // read pointer
logic [2:0]             inc_numb_bytes_write;
logic [3:0]             byte_strobe_write;
logic [2:0]             inc_numb_bytes_read;
logic                   mem_wr_en;
logic [wbyte-1:0]       mem[size-1:0];
logic [size_exp:0]      fullness_count_perm;
logic                   fullness_count_inc;
logic                   fullness_count_dec;
logic [size_exp:0]      fullness_count_nxt;
logic                   fullness_count_refresh;
logic [size_exp:0]      fullness_count;
logic [size_exp:0]      fullness_border;
logic                   fullness_for_write;
logic                   fullness_for_read;

// increment pointers
assign p_write_nxt = p_write + inc_numb_bytes_write;
assign p_write_inc = i_put & fullness_for_write;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) p_write <= 'd0;
    else if (i_clear) p_write <= 'd0;
    else if (p_write_inc) p_write <= p_write_nxt;

assign p_read_nxt = p_read + inc_numb_bytes_read;
assign p_read_inc = i_pull & fullness_for_read;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) p_read <= 'd0;
    else if (i_clear) p_read <= 'd0;
    else if (p_read_inc) p_read <= p_read_nxt;

// decode signals
always_comb begin
    case (i_numb_bytes_put)
        numb_bytes_1_code: inc_numb_bytes_write = numb_bytes_1;
        numb_bytes_2_code: inc_numb_bytes_write = numb_bytes_2;
        numb_bytes_4_code: inc_numb_bytes_write = numb_bytes_4;
        default: inc_numb_bytes_write = numb_bytes_err;
    endcase
end

always_comb begin
    case (i_numb_bytes_put)
        numb_bytes_1_code: byte_strobe_write = strobe_1bytes;
        numb_bytes_2_code: byte_strobe_write = strobe_2bytes;
        numb_bytes_4_code: byte_strobe_write = strobe_4bytes;
        default: byte_strobe_write = strobe_err;
    endcase
end

always_comb begin
    case (i_numb_bytes_pull)
        numb_bytes_1_code: inc_numb_bytes_read = numb_bytes_1;
        numb_bytes_2_code: inc_numb_bytes_read = numb_bytes_2;
        numb_bytes_4_code: inc_numb_bytes_read = numb_bytes_4;
        default: inc_numb_bytes_read = numb_bytes_err;
    endcase
end

// memory operations
assign mem_wr_en = p_write_inc;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) for(int i = 0; i < size; ++i) mem[i] <= {wbyte{1'b0}};
    else if (mem_wr_en) begin
        case (i_numb_bytes_put)
            numb_bytes_1_code: begin
                mem[p_write] <= i_wdata[7:0];
            end
            numb_bytes_2_code: begin
                mem[(p_write+1)%size] <= i_wdata[15:8];
                mem[(p_write+0)%size] <= i_wdata[7:0];
            end
            default: begin
                mem[(p_write+3)%size] <= i_wdata[31:24];
                mem[(p_write+2)%size] <= i_wdata[23:16];
                mem[(p_write+1)%size] <= i_wdata[15:8];
                mem[(p_write+0)%size] <= i_wdata[7:0];
            end
        endcase
    end

always_comb begin
    case (i_numb_bytes_pull)
        numb_bytes_1_code: o_rdata = {{(wbus-wbyte){1'b0}}, {mem[p_read]}};
        numb_bytes_2_code: o_rdata = {{(wbus-wbyte*2){1'b0}}, mem[(p_read+1)%size], mem[p_read]};
        default: o_rdata = {mem[(p_read+3)%size], mem[(p_read+2)%size], mem[(p_read+1)%size], mem[p_read]};
    endcase
end

// fullness logic
always_comb begin
    if (i_put && !i_pull) begin
        fullness_count_perm = fullness_count + inc_numb_bytes_write;
    end
    else if (!i_put && i_pull) begin
        fullness_count_perm = fullness_count - inc_numb_bytes_read;
    end
    else if (i_put && i_pull) begin
        fullness_count_perm = fullness_count + inc_numb_bytes_write - inc_numb_bytes_read;
    end
    else begin
        fullness_count_perm = fullness_count;
    end
end

assign fullness_count_inc = p_write_inc;
assign fullness_count_dec = p_read_inc;
always_comb begin
    if (fullness_count_inc && !fullness_count_dec) begin
        fullness_count_nxt = fullness_count + inc_numb_bytes_write;
    end
    else if (!fullness_count_inc && fullness_count_dec) begin
        fullness_count_nxt = fullness_count - inc_numb_bytes_read;
    end
    else if (fullness_count_inc && fullness_count_dec) begin
        fullness_count_nxt = fullness_count + inc_numb_bytes_write - inc_numb_bytes_read;
    end
    else begin
        fullness_count_nxt = fullness_count;
    end
end

assign fullness_count_refresh = fullness_count_inc | fullness_count_dec;
always_ff @(posedge i_clk, negedge i_nreset)
    if (!i_nreset) fullness_count <= 'd0;
    else if (i_clear) fullness_count <= 'd0;
    else if (fullness_count_refresh) fullness_count <= fullness_count_nxt;

always_comb begin
    if (fullness_count < (size/4)) begin
        o_fullness = 2'd0;
    end
    else if (fullness_count >= (size/4) && fullness_count < (size/2)) begin
        o_fullness = 2'd1;
    end
    else if (fullness_count >= (size/2) && fullness_count < (size/4*3)) begin
        o_fullness = 2'd2;
    end
    else begin
        o_fullness = 2'd3;
    end
end

always_comb begin
    case (i_filling_thresh)
        quarter: begin
            fullness_border = size / 4;
        end
        half: begin
            fullness_border = size / 2;
        end
        three_quarter: begin
            fullness_border = size / 4 * 3;
        end
        full: begin
            fullness_border = size;
        end
        default: fullness_border = 'h0;
    endcase
end

assign fullness_for_write = fullness_count_perm <= fullness_border;
assign fullness_for_read = fullness_count >= inc_numb_bytes_read && !o_empty;

assign o_empty = ~|fullness_count;
assign o_full = fullness_count == size;

assign o_overrun = !fullness_for_write & i_put;
assign o_underrun = o_empty & i_pull;

// left bytes
assign o_left_put = fullness_border - fullness_count;
assign o_left_pull = fullness_count;

endmodule
