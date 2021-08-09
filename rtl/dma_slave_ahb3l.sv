module dma_slave_ahb3l (
    input  logic        i_hclk,
    input  logic        i_hnreset,

    // AHB connection to master
    input  logic        i_hsel,
    input  logic [31:0] i_haddr,
    input  logic [1:0]  i_htrans,
    input  logic [2:0]  i_hsize,
    input  logic        i_hwrite,
    input  logic        i_hready,
    input  logic [31:0] i_hwdata,

    output logic        o_hreadyout,
    output logic        o_hresp,
    output logic [31:0] o_hrdata,

    // register interface
    output logic [31:0] o_addr,
    output logic        o_read_en,
    output logic        o_write_en,
    output logic [3:0]  o_byte_strobe,
    output logic [31:0] o_wdata,
    input  logic [31:0] i_rdata
);

localparam wbus = 32;
localparam size_byte = 3'b000;
localparam size_hword = 3'b001;
localparam size_word = 3'b010;

localparam strobe_err = 4'b0000;
localparam strobe_b0 = 4'b0001;
localparam strobe_b1 = 4'b0010;
localparam strobe_b2 = 4'b0100;
localparam strobe_b3 = 4'b1000;
localparam strobe_hw0 = 4'b0011;
localparam strobe_hw1 = 4'b1100;
localparam strobe_w = 4'b1111;

logic               trans_req;
logic               read_req;
logic               write_req;
logic [wbus-1:0]    addr;
logic               update_read_en;
logic               read_en;
logic               update_write_en;
logic               write_en;
logic [3:0]         byte_strobe_nxt;
logic               update_byte_strobe;
logic [3:0]         byte_strobe;

assign trans_req = i_hready & i_hsel & i_htrans[1];
assign read_req = trans_req & !i_hwrite;
assign write_req = trans_req & i_hwrite;

always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) addr <= {wbus{1'b0}};
    else if (trans_req) addr <= i_haddr;
    else if (i_hready) addr <= {wbus{1'b0}};

assign update_read_en = read_req | (read_en & i_hready);
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) read_en <= 1'b0;
    else if (update_read_en) read_en <= read_req;

assign update_write_en = write_req | (write_en & i_hready);
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) write_en <= 1'b0;
    else if (update_write_en) write_en <= write_req;

always_comb
    if (i_hsize == size_byte)
        case (i_haddr[1:0])
            2'b00: byte_strobe_nxt = strobe_b0;
            2'b01: byte_strobe_nxt = strobe_b1;
            2'b10: byte_strobe_nxt = strobe_b2;
            2'b11: byte_strobe_nxt = strobe_b3;
            default: byte_strobe_nxt = strobe_err;
        endcase
    else if (i_hsize == size_hword)
        if (i_haddr[1] == 1'b0)
            byte_strobe_nxt = strobe_hw0;
        else
            byte_strobe_nxt = strobe_hw1;
    else
        byte_strobe_nxt = strobe_w;

assign update_byte_strobe = update_read_en | update_write_en;
always_ff @(posedge i_hclk, negedge i_hnreset)
    if (!i_hnreset) byte_strobe <= 4'd0;
    else if (update_byte_strobe) byte_strobe <= byte_strobe_nxt;

// AHB if out
assign o_hreadyout = 1'b1;
assign o_hresp = 1'b0;
assign o_hrdata = i_rdata;

// register if out
assign o_addr = addr;
assign o_read_en = read_en;
assign o_write_en = write_en;
assign o_byte_strobe = byte_strobe;
assign o_wdata = i_hwdata;

endmodule
