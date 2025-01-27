`include "define.vh"

module m_topsim (
`ifndef ICARUS
	input wire clk, 
`endif
	    // tang nano 20k SDRAM
    output wire O_sdram_clk,
    output wire O_sdram_cke,
    output wire O_sdram_cs_n,            // chip select
    output wire O_sdram_cas_n,           // columns addrefoc select
    output wire O_sdram_ras_n,           // row address select
    output wire O_sdram_wen_n,           // write enable
    inout wire [31:0] IO_sdram_dq,       // 32 bit bidirectional data bus
    output wire [10:0] O_sdram_addr,     // 11 bit multiplexed address bus
    output wire [1:0] O_sdram_ba,        // two banks
    output wire [3:0] O_sdram_dqm       // 32/4
);

wire pll_clk, clk_sdram;
`ifdef SIM_MODE
    assign pll_clk = clk;
    assign clk_sdram = clk;
`else
    Gowin_rPLL_nes pll_nes(
    .clkin(clk),
    .clkout(pll_clk),          // FREQ main clock
    .clkoutp(clk_sdram)    // FREQ main clock phase shifted
    );
`endif

reg RST_X=0;
example_soc es (
	// System clock + reset
	.clk(pll_clk),
	.rst_n(RST_X),
	.clk_sdram(clk_sdram),

	// JTAG port to RISC-V JTAG-DTM
	.tck(1'b0),
	.trst_n(1'b0),
	.tms(1'b0),
	.tdi(1'b1),
	.tdo(),

	// IO
	.uart_tx(),
	.uart_rx(1'b1),

    // tang nano 20k SDRAM
    .O_sdram_clk(O_sdram_clk),
    .O_sdram_cke(O_sdram_cke),
    .O_sdram_cs_n(O_sdram_cs_n),            // chip select
    .O_sdram_cas_n(O_sdram_cas_n),           // columns addrefoc select
    .O_sdram_ras_n(O_sdram_ras_n),           // row address select
    .O_sdram_wen_n(O_sdram_wen_n),           // write enable
    .IO_sdram_dq(IO_sdram_dq),       // 32 bit bidirectional data bus
    .O_sdram_addr(O_sdram_addr),     // 11 bit multiplexed address bus
    .O_sdram_ba(O_sdram_ba),        // two banks
    .O_sdram_dqm(O_sdram_dqm)       // 32/4
);

`ifdef ICARUS
reg clk=0;
always begin
	clk = 0;
	#5;
	clk = 1;
	#5;
end
`endif

reg [31:0] cnt=0;
always @(posedge clk) begin
	if(cnt < 100)
		cnt <= cnt + 1;
	else
		RST_X <= 1;
end

endmodule
