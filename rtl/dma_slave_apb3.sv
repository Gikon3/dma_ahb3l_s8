module dma_slave_apb3 (
    input  logic        i_clk,
    input  logic        i_pclk,
    input  logic        i_pnreset,

    // APB connection to master
    input  logic        i_psel,
    input  logic        i_penable,
    input  logic [31:0] i_paddr,
    input  logic        i_pwrite,
    input  logic [31:0] i_pwdata,

    output logic        o_pready,
    output logic        o_pslverr,
    output logic [31:0] o_prdata,

    // register interface
    output logic [31:0] o_addr,
    output logic        o_read_en,
    output logic        o_write_en,
    output logic [3:0]  o_byte_strobe,
    output logic [31:0] o_wdata,
    input  logic [31:0] i_rdata
);

enum logic [1:0] {st_idle, st_setup, st_access} state_list;

logic [2:0]     sync_accum;
logic           sync;
logic           ready;
logic [31:0]    data_rd;

logic       req_rd;
logic       req_wr;

logic [1:0] state;
logic [1:0] nxt_state;
logic       state_idle;
logic       state_setup;
logic       state_access;
logic       sw_idle_to_setup;
logic       sw_setup_to_access;
logic       sw_access_to_idle;
logic       sw_access_to_setup;

// sync
always_ff @(posedge i_clk, negedge i_pnreset)
    if (!i_pnreset) sync_accum <= 'd0;
    else if (sync) sync_accum <= 'd0;
    else if (!ready)sync_accum <= {sync_accum[1:0], i_psel};

assign sync = sync_accum[2] & sync_accum[1] & ~ready;

always_ff @(posedge i_clk, negedge i_pnreset)
    if (!i_pnreset) ready <= 1'b0;
    else if (state_idle || state_setup) ready <= 1'b0;
    else if (sync) ready <= 1'b1;

always_ff @(posedge i_clk, negedge i_pnreset)
    if (!i_pnreset) data_rd <= 'd0;
    else if (sync) data_rd <= i_rdata;

// transfer
assign req_rd = i_psel & ~i_pwrite;
assign req_wr = i_psel & i_pwrite;

// fsm
assign state_idle = state == st_idle;
assign state_setup = state == st_setup;
assign state_access = state == st_access;

assign sw_idle_to_setup = req_rd | req_wr;
assign sw_setup_to_access = /*1'b1*/req_rd | req_wr;
assign sw_access_to_idle = /*ready*/1'b0;
assign sw_access_to_setup = /*1'b0*/ready;

always_comb begin: fsm
    case (state)
        // st_idle: begin
        //     if (sw_idle_to_setup) nxt_state = st_setup;
        //     else nxt_state = st_idle;
        // end
        st_setup: begin
            if (sw_setup_to_access) nxt_state = st_access;
            else nxt_state = st_setup;
        end
        st_access: begin
            if (sw_access_to_idle) nxt_state = st_idle;
            else if (sw_access_to_setup) nxt_state = st_setup;
            else nxt_state = st_access;
        end
        default: begin
            nxt_state = st_idle;
        end
    endcase
end

always_ff @(posedge i_pclk, negedge i_pnreset)
    if (!i_pnreset) state <= st_setup;
    else state <= nxt_state;

// APB if out
assign o_pready  = state_setup | (state_access & ready);
assign o_pslverr = 1'b0;
assign o_prdata = data_rd;

// register if out
assign o_addr = {24'd0, i_paddr[7:0]};
assign o_read_en = req_rd & sync;
assign o_write_en = req_wr & sync;
assign o_byte_strobe = 4'b1111;
assign o_wdata = i_pwdata;

endmodule
