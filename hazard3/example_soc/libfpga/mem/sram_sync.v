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

// Generate a (hopefully inference-compatible) memory with synchronous
// read/write, and optional per-byte write enable

module sram_sync #(
	parameter WIDTH = 32,
	parameter DEPTH = 1 << 11,
	parameter BYTE_ENABLE = 0,
	parameter PRELOAD_FILE = "",
	parameter ADDR_WIDTH = $clog2(DEPTH), // Let this default
	`include "hazard3_config.vh"
) (
	input wire                                     clk,
	input wire [31:0]			       d_pc,
	input wire [(BYTE_ENABLE ? WIDTH / 8 : 1)-1:0] wen,
	input wire                                     ren,
	input wire [ADDR_WIDTH-1:0]                    addr,
	input wire [WIDTH-1:0]                         wdata,
	output reg [WIDTH-1:0]                         rdata
);

integer f;
reg opened=0, closed=0;
reg [31:0] timecnt=0;
always @(posedge clk) begin
    if(!opened) begin
            opened = 1;
            f = $fopen("ch", "w");
            if (f == 0) begin
                $display ("ERROR: fh not opened");
                $finish;
            end
    end
    timecnt <= timecnt + 1;
    if(timecnt > 500000) begin
                        closed <= 1;
                        $fclose(f);
                        //$finish();
    end
end

`ifdef FPGA_ICE40
localparam FPGA_ICE40_DEFINED = 1;
`else
localparam FPGA_ICE40_DEFINED = 0;
`endif

`ifdef SIM
localparam SIM_DEFINED = 1;
`else
localparam SIM_DEFINED = 0;
`endif

generate
if (FPGA_ICE40_DEFINED && WIDTH == 32 && DEPTH == 1 << 15) begin: up5k_spram
// Special case: use all SPRAMs on UP5k

wire [31:0] rdata0;
wire [31:0] rdata1;

SB_SPRAM256KA ram00 (
	.ADDRESS    (addr[13:0]),
	.DATAIN     (wdata[15:0]),
	.MASKWREN   ({wen[1], wen[1], wen[0], wen[0]}),
	.WREN       (wen[1] || wen[0]),
	.CHIPSELECT ((wen[1] || wen[0] || ren) && !addr[14]),
	.CLOCK      (clk),
	.STANDBY    (1'b0),
	.SLEEP      (1'b0),
	.POWEROFF   (1'b1),
	.DATAOUT    (rdata0[15:0])
);

SB_SPRAM256KA ram01 (
	.ADDRESS    (addr[13:0]),
	.DATAIN     (wdata[31:16]),
	.MASKWREN   ({wen[3], wen[3], wen[2], wen[2]}),
	.WREN       (wen[3] || wen[2]),
	.CHIPSELECT ((wen[3] || wen[2] || ren) && !addr[14]),
	.CLOCK      (clk),
	.STANDBY    (1'b0),
	.SLEEP      (1'b0),
	.POWEROFF   (1'b1),
	.DATAOUT    (rdata0[31:16])
);

SB_SPRAM256KA ram10 (
	.ADDRESS    (addr[13:0]),
	.DATAIN     (wdata[15:0]),
	.MASKWREN   ({wen[1], wen[1], wen[0], wen[0]}),
	.WREN       (wen[1] || wen[0]),
	.CHIPSELECT ((wen[1] || wen[0] || ren) && addr[14]),
	.CLOCK      (clk),
	.STANDBY    (1'b0),
	.SLEEP      (1'b0),
	.POWEROFF   (1'b1),
	.DATAOUT    (rdata1[15:0])
);

SB_SPRAM256KA ram11 (
	.ADDRESS    (addr[13:0]),
	.DATAIN     (wdata[31:16]),
	.MASKWREN   ({wen[3], wen[3], wen[2], wen[2]}),
	.WREN       (wen[3] || wen[2]),
	.CHIPSELECT ((wen[3] || wen[2] || ren) && addr[14]),
	.CLOCK      (clk),
	.STANDBY    (1'b0),
	.SLEEP      (1'b0),
	.POWEROFF   (1'b1),
	.DATAOUT    (rdata1[31:16])
);
reg chipselect_prev;
always @ (posedge clk)
	if (|wen || ren)
		chipselect_prev <= addr[14];

always @ (*) rdata = chipselect_prev ? rdata1 : rdata0;

end else begin: behav_mem
// Behavioural model, but Yosys does a great job of this on ECP5 and iCE40.

genvar i;

reg [WIDTH-1:0] mem [0:DEPTH-1];
reg  [7:0] mem_bbl [0:DEPTH*4-1];

        initial begin: preload
                integer n;
                for (n = 0; n < DEPTH; n = n + 1)
                        mem[n] = {WIDTH{1'b0}};
                if (PRELOAD_FILE != "")
                        $readmemh(PRELOAD_FILE, mem_bbl);
                for (n = 0; n < DEPTH; n = n + 1) begin
                        mem[n] = {mem_bbl[n*4+3],mem_bbl[n*4+2],mem_bbl[n*4+1],mem_bbl[n*4]};
			if(n <= 10)
				$display("mem[%d]=%x", n, mem[n]);
                end
        end

if (BYTE_ENABLE) begin: has_byte_enable
	for (i = 0; i < WIDTH / 8; i = i + 1) begin: byte_mem
		always @ (posedge clk) begin
			if (wen[i])
				mem[addr][8 * i +: 8] <= wdata[8 * i +: 8];
			if (ren)
				rdata[8 * i +: 8] <= mem[addr][8 * i +: 8];
		end
	end
	reg [31:0] j=0, lj=0;
	always @ (posedge clk) begin
		if(ren) begin
                                if(j < 20 || (d_pc >= pc_trace_start && d_pc <= pc_trace_stop && lj < 10)) begin
                                        j <= j+1;
                                        if(d_pc >= pc_trace_start && d_pc <= pc_trace_stop && lj < 10)
                                                lj <= lj+1;
                                        $display("mem read addr %x data %x d_pc %x time %8d", {addr,2'b00},
                                                mem[addr], d_pc, $time);
                                end
				$fwrite(f, "mem read addr %8x data %8x\n", {addr,2'b00}, mem[addr]);
		end 
		if(wen) begin
                                if(j < 20 || (d_pc >= pc_trace_start && d_pc <= pc_trace_stop && lj < 20)) begin
                                        j <= j+1;
                                        if(d_pc >= pc_trace_start && d_pc <= pc_trace_stop && lj < 20)
                                                lj <= lj+1;
                                        $display("mem write addr %x data %x mask %x d_pc %x time %8d", {addr,2'b00},
                                                wdata, wen, d_pc, $time);
                                end
				$fwrite(f, "mem write addr %8x data %8x mask %1x\n", {addr,2'b00}, wdata, wen);
		end
		`ifdef laur0
		if((ren || wen) && addr[1:0]) begin
			$display("sram sync addr=%x ren=%x wen=%x", addr, ren, wen);
			$finish();
		end
		`endif

	end

end else begin: no_byte_enable
	always @ (posedge clk) begin
		if (wen)
			mem[addr] <= wdata;
		if (ren)
			rdata <= mem[addr];
	end
end

end
endgenerate

endmodule
