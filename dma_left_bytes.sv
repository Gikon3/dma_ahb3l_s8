module dma_left_bytes #(parameter fifo_size = 5) (
    input  logic [fifo_size:0]  i_left_put,
    input  logic [fifo_size:0]  i_left_pull,
    input  logic                i_buf_full,
    input  logic                i_buf_empty,
    input  logic                i_dir_pbus_to_mbus,
    input  logic                i_dir_mbus_to_pbus,
    input  logic                i_direct_dis,
    input  logic [1:0]          i_msize,
    input  logic [1:0]          i_psize,
    output logic [fifo_size:0]  o_left_bytes_mp,
    output logic [fifo_size:0]  o_left_bytes_pp
);

enum logic [1:0] {bytew, hword, word} size_list;

logic [2:0]                 numb_bytes_msize;
logic [2:0]                 numb_bytes_psize;

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

always_comb begin: decode_left_bytes_mp
    if (i_direct_dis) begin
        if (i_dir_pbus_to_mbus) begin
            o_left_bytes_mp = i_left_pull;
        end
        else if (i_dir_mbus_to_pbus) begin
            o_left_bytes_mp = i_left_put;
        end
        else begin
            o_left_bytes_mp = 'd0;
        end
    end
    else begin
        if ((i_dir_pbus_to_mbus && i_buf_full) || (i_dir_mbus_to_pbus && i_buf_empty)) begin
            o_left_bytes_mp = {{(fifo_size-2){1'b0}}, numb_bytes_msize};
        end
        else begin
            o_left_bytes_mp = 'd0;
        end
    end
end

always_comb begin: decode_left_bytes_pp
    if (i_direct_dis) begin
        if (i_dir_pbus_to_mbus) begin
            o_left_bytes_pp = i_left_put;
        end
        else if (i_dir_mbus_to_pbus) begin
            o_left_bytes_pp = i_left_pull;
        end
        else begin
            o_left_bytes_pp = 'd0;
        end
    end
    else begin
        if ((i_dir_pbus_to_mbus && i_buf_empty) || (i_dir_mbus_to_pbus && i_buf_full)) begin
            o_left_bytes_pp = {{(fifo_size-2){1'b0}}, numb_bytes_psize};
        end
        else begin
            o_left_bytes_pp = 'd0;
        end
    end
end

endmodule
