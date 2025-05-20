/**********************************************************************
 * DO WHAT THE FUCK YOU WANT TO AND DON'T BLAME US PUBLIC LICENSE     *
 *                    Version 3, April 2008                           *
 *                                                                    *
 * Copyright (C) 2018 Luke Wren                                       *
 *                                                                    *
 * Everyone is permitted to copy and distribute verbatim or modified  *
 * copies of this license document and accompanying software, and     *
 * changing either is allowed.                                        *
 *                                                                    *
 *   TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION  *
 *                                                                    *
 * 0. You just DO WHAT THE FUCK YOU WANT TO.                          *
 * 1. We're NOT RESPONSIBLE WHEN IT DOESN'T FUCKING WORK.             *
 *                                                                    *
 *********************************************************************/
/*
 * Strict-priority N:1 AHBL arbiter
 *
 * Lower port numbers are higher priority.
 * Use concat lists to wire masters in:
 *   .N_PORTS(2)
 *   ...
 *   .src_haddr({mast1_haddr, mast0_haddr})
 *   ...
 * Recommend that wiring up either be scripted, or done by unpaid intern.
 */

// TODO: no burst support!


module ahbl_arbiter #(
	parameter SLAVE_ID = 0,
	parameter N_PORTS = 2,
	//parameter W_ADDR = 32,
	//parameter W_DATA = 32,
	parameter CONN_MASK = {N_PORTS{1'b1}},
	`include "hazard3_config.vh"
) (
	// Global signals
	input wire                       clk,
	input wire                       rst_n,

	// From masters; function as slave ports
	input  wire [N_PORTS*W_ADDR-1:0] src_d_pc,
	input  wire [N_PORTS*W_DATA-1:0] src_hartid,
	input  wire [N_PORTS-1:0]        src_hready,
	output wire [N_PORTS-1:0]        src_hready_resp,
	output wire [N_PORTS-1:0]        src_hresp,
	input  wire [N_PORTS*W_ADDR-1:0] src_haddr,
	input  wire [N_PORTS-1:0]        src_hwrite,
	input  wire [N_PORTS*2-1:0]      src_htrans,
	input  wire [N_PORTS*3-1:0]      src_hsize,
	input  wire [N_PORTS*3-1:0]      src_hburst,
	input  wire [N_PORTS*4-1:0]      src_hprot,
	input  wire [N_PORTS-1:0]        src_hmastlock,
	input  wire [N_PORTS*W_DATA-1:0] src_hwdata,
	output wire [N_PORTS*W_DATA-1:0] src_hrdata,
        // exclusive access signaling
        input  wire [N_PORTS-1:0]       src_hexcl,
        input  wire [N_PORTS*8-1:0]     src_hmaster,
        output wire [N_PORTS-1:0]       src_hexokay,

	// To slave; functions as master port
	output wire [W_ADDR-1:0] 	 dst_d_pc,
	output wire [W_DATA-1:0]         dst_hartid,
	output wire                      dst_hready,
	input  wire                      dst_hready_resp,
	input  wire                      dst_hresp,
	output wire [W_ADDR-1:0]         dst_haddr,
	output wire                      dst_hwrite,
	output wire [1:0]                dst_htrans,
	output wire [2:0]                dst_hsize,
	output wire [2:0]                dst_hburst,
	output wire [3:0]                dst_hprot,
	output wire                      dst_hmastlock,
	output wire [W_DATA-1:0]         dst_hwdata,
	input  wire [W_DATA-1:0]         dst_hrdata,
        // exclusive access signaling   
        output wire        		 dst_hexcl,
        output wire [7:0]     		 dst_hmaster,
        input wire        		 dst_hexokay
);

integer i;

reg  [N_PORTS-1:0] mast_req_a;
wire [N_PORTS-1:0] mast_gnt_a;

always @ (*) begin
	for (i = 0; i < N_PORTS; i = i + 1) begin
		// HTRANS == 2'b10, 2'b11 when active
		mast_req_a[i] = src_htrans[i * 2 + 1] && CONN_MASK[i];
	end
end

onehot_priority #(
	.W_INPUT(N_PORTS)
) arb_priority (
	.clk(clk),
	.rst_n(rst_n),
	.in(mast_req_a),
	.out(mast_gnt_a)
);

`ifdef laur0
//always @(mast_gnt_a) begin
reg [N_PORTS-1:0] o_mast_gnt_a=0;
integer tcnt=0;
always @(posedge clk) begin
	tcnt = tcnt + 1;
	if(o_mast_gnt_a != mast_gnt_a) begin
		o_mast_gnt_a = mast_gnt_a;
		$display("s%1d gnt_a=%x pc0=%x shaddr=%x pc1=%x shaddr=%x dhaddr=%x shwr=%2x dhwr=%1x shready_resp=%x dhready_resp=%x dh=%1x dpc=%x", 
			SLAVE_ID, mast_gnt_a, src_d_pc[31:0], src_haddr[31:0], src_d_pc[63:32], src_haddr[63:32], dst_haddr, src_hwrite, dst_hwrite,
			src_hready_resp, dst_hready_resp, dst_hartid, dst_d_pc);
	end
end
always @(dst_haddr or dst_hwrite or dst_hartid or dst_htrans or dst_hready_resp or dst_hready) begin
	if((tcnt > 442862 && tcnt < 442900) || (src_d_pc[63:32] >= pc_trace_start && src_d_pc[63:32] <= pc_trace_stop)) begin
		$display("  s%1d pc=%x dpc=%x dhaddr=%x dhwr=%x dhrd=%x dh=%1x dhtrans=%x dhrdy_resp=%x dhrdy=%x shrdy=%x shrdy_resp=%x req_a=%x gnt_d=%x shtr=%x ahtr=%x t=%8d",
			SLAVE_ID, src_d_pc[63:32], dst_d_pc, dst_haddr, dst_hwrite, dst_hrdata, dst_hartid, dst_htrans, dst_hready_resp, dst_hready, src_hready_resp, src_hready, mast_req_a, mast_gnt_d, src_htrans, src_htrans, tcnt);
	end
	if(dst_hresp) begin
		$display("error in arbiter");
		$finish;
	end
end
`endif

// AHB State Machine

reg [N_PORTS-1:0] mast_gnt_d;
assign dst_hready = |src_hready; 

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		mast_gnt_d <= {N_PORTS{1'b0}};
	end else begin
		if (dst_hready) begin
			mast_gnt_d <= mast_gnt_a;
		end
	end
end

// Data-phase signal passthrough

// There are two reasons to report ready:
// - the master is currently not in data phase with the arbiter (IDLE)
// - the master is in data phase with both arbiter and slave, and slave is ready
reg [N_PORTS-1:0] r_src_hready_resp;
assign src_hready_resp = r_src_hready_resp;
always @ (*) begin
        for (i = 0; i < N_PORTS; i = i + 1) begin 
                r_src_hready_resp[i] = !mast_gnt_d ? dst_hready_resp : (mast_gnt_d[i] & dst_hready_resp);
        end
end

assign src_hresp = mast_gnt_d & {N_PORTS{dst_hresp}};
assign src_hrdata = {N_PORTS{dst_hrdata}};
assign src_hexokay = mast_gnt_d & {N_PORTS{dst_hexokay}};

onehot_mux #(
	.W_INPUT(W_DATA),
	.N_INPUTS(N_PORTS)
) hwdata_mux (
	.in(src_hwdata),
	.sel(mast_gnt_d),
	.out(dst_hwdata)
);

// Pass through address-phase signals based on grant

onehot_mux #(
        .W_INPUT(W_ADDR),
        .N_INPUTS(N_PORTS)
) mux_dst_d_pc (
        .in(src_d_pc),
        .sel(mast_gnt_a),
        .out(dst_d_pc)
);

onehot_mux #(
        .W_INPUT(W_DATA),
        .N_INPUTS(N_PORTS)
) mux_dst_hartid (
        .in(src_hartid),
        .sel(mast_gnt_a),
        .out(dst_hartid)
);

onehot_mux #(
        .W_INPUT(1),
        .N_INPUTS(N_PORTS)
) mux_hexcl (
        .in(src_hexcl),
        .sel(mast_gnt_a),
        .out(dst_hexcl)
);

onehot_mux #(
        .W_INPUT(8),
        .N_INPUTS(N_PORTS)
) mux_hmaster (
        .in(src_hmaster),
        .sel(mast_gnt_a),
        .out(dst_hmaster)
);

onehot_mux #(
	.W_INPUT(W_ADDR),
	.N_INPUTS(N_PORTS)
) mux_haddr (
	.in(src_haddr),
	.sel(mast_gnt_a),
	.out(dst_haddr)
);

onehot_mux #(
	.W_INPUT(1),
	.N_INPUTS(N_PORTS)
) mux_hwrite (
	.in(src_hwrite),
	.sel(mast_gnt_a),
	.out(dst_hwrite)
);

onehot_mux #(
	.W_INPUT(2),
	.N_INPUTS(N_PORTS)
) mux_hwtrans (
	.in(src_htrans),
	.sel(mast_gnt_a),
	.out(dst_htrans)
);

onehot_mux #(
	.W_INPUT(3),
	.N_INPUTS(N_PORTS)
) mux_hsize (
	.in(src_hsize),
	.sel(mast_gnt_a),
	.out(dst_hsize)
);

onehot_mux #(
	.W_INPUT(3),
	.N_INPUTS(N_PORTS)
) mux_hburst (
	.in(src_hburst),
	.sel(mast_gnt_a),
	.out(dst_hburst)
);

onehot_mux #(
	.W_INPUT(4),
	.N_INPUTS(N_PORTS)
) mux_hprot (
	.in(src_hprot),
	.sel(mast_gnt_a),
	.out(dst_hprot)
);

onehot_mux #(
	.W_INPUT(1),
	.N_INPUTS(N_PORTS)
) mux_hmastlock (
	.in(src_hmastlock),
	.sel(mast_gnt_a),
	.out(dst_hmastlock)
);

endmodule
