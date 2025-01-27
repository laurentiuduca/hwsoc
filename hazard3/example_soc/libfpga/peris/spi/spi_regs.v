/*******************************************************************************
*                          AUTOGENERATED BY REGBLOCK                           *
*                            Do not edit manually.                             *
*          Edit the source file (or regblock utility) and regenerate.          *
*******************************************************************************/

// Block name           : spi
// Bus type             : apb
// Bus data width       : 32
// Bus address width    : 16

module spi_regs (
	input wire clk,
	input wire rst_n,
	
	// APB Port
	input wire apbs_psel,
	input wire apbs_penable,
	input wire apbs_pwrite,
	input wire [15:0] apbs_paddr,
	input wire [31:0] apbs_pwdata,
	output wire [31:0] apbs_prdata,
	output wire apbs_pready,
	output wire apbs_pslverr,
	
	// Register interfaces
	output reg  csr_csauto_o,
	output reg  csr_cs_o,
	output reg  csr_loopback_o,
	output reg  csr_read_en_o,
	output reg  csr_cpol_o,
	output reg  csr_cpha_o,
	input wire  csr_busy_i,
	output reg [5:0] div_o,
	input wire [7:0] fstat_txlevel_i,
	input wire  fstat_txfull_i,
	input wire  fstat_txempty_i,
	input wire  fstat_txover_i,
	input wire [7:0] fstat_rxlevel_i,
	input wire  fstat_rxfull_i,
	input wire  fstat_rxempty_i,
	input wire  fstat_rxover_i,
	input wire  fstat_rxunder_i,
	output reg [7:0] tx_o,
	output reg tx_wen,
	input wire [7:0] rx_i,
	output reg rx_ren
);

// APB adapter
wire [31:0] wdata = apbs_pwdata;
reg [31:0] rdata;
wire wen = apbs_psel && apbs_penable && apbs_pwrite;
wire ren = apbs_psel && apbs_penable && !apbs_pwrite;
wire [15:0] addr = apbs_paddr & 16'h1c;
assign apbs_prdata = rdata;
assign apbs_pready = 1'b1;
assign apbs_pslverr = 1'b0;

localparam ADDR_CSR = 0;
localparam ADDR_DIV = 4;
localparam ADDR_FSTAT = 8;
localparam ADDR_TX = 12;
localparam ADDR_RX = 16;

wire __csr_wen = wen && addr == ADDR_CSR;
wire __csr_ren = ren && addr == ADDR_CSR;
wire __div_wen = wen && addr == ADDR_DIV;
wire __div_ren = ren && addr == ADDR_DIV;
wire __fstat_wen = wen && addr == ADDR_FSTAT;
wire __fstat_ren = ren && addr == ADDR_FSTAT;
wire __tx_wen = wen && addr == ADDR_TX;
wire __tx_ren = ren && addr == ADDR_TX;
wire __rx_wen = wen && addr == ADDR_RX;
wire __rx_ren = ren && addr == ADDR_RX;

wire  csr_csauto_wdata = wdata[9];
wire  csr_csauto_rdata;
wire  csr_cs_wdata = wdata[8];
wire  csr_cs_rdata;
wire  csr_loopback_wdata = wdata[5];
wire  csr_loopback_rdata;
wire  csr_read_en_wdata = wdata[4];
wire  csr_read_en_rdata;
wire  csr_cpol_wdata = wdata[3];
wire  csr_cpol_rdata;
wire  csr_cpha_wdata = wdata[2];
wire  csr_cpha_rdata;
wire  csr_busy_wdata = wdata[0];
wire  csr_busy_rdata;
wire [31:0] __csr_rdata = {22'h0, csr_csauto_rdata, csr_cs_rdata, 2'h0, csr_loopback_rdata, csr_read_en_rdata, csr_cpol_rdata, csr_cpha_rdata, 1'h0, csr_busy_rdata};
assign csr_csauto_rdata = csr_csauto_o;
assign csr_cs_rdata = csr_cs_o;
assign csr_loopback_rdata = csr_loopback_o;
assign csr_read_en_rdata = csr_read_en_o;
assign csr_cpol_rdata = csr_cpol_o;
assign csr_cpha_rdata = csr_cpha_o;
assign csr_busy_rdata = csr_busy_i;

wire [5:0] div_wdata = wdata[5:0];
wire [5:0] div_rdata;
wire [31:0] __div_rdata = {26'h0, div_rdata};
assign div_rdata = div_o;

wire [7:0] fstat_txlevel_wdata = wdata[7:0];
wire [7:0] fstat_txlevel_rdata;
wire  fstat_txfull_wdata = wdata[8];
wire  fstat_txfull_rdata;
wire  fstat_txempty_wdata = wdata[9];
wire  fstat_txempty_rdata;
wire  fstat_txover_wdata = wdata[10];
wire  fstat_txover_rdata;
wire [7:0] fstat_rxlevel_wdata = wdata[23:16];
wire [7:0] fstat_rxlevel_rdata;
wire  fstat_rxfull_wdata = wdata[24];
wire  fstat_rxfull_rdata;
wire  fstat_rxempty_wdata = wdata[25];
wire  fstat_rxempty_rdata;
wire  fstat_rxover_wdata = wdata[26];
wire  fstat_rxover_rdata;
wire  fstat_rxunder_wdata = wdata[27];
wire  fstat_rxunder_rdata;
wire [31:0] __fstat_rdata = {4'h0, fstat_rxunder_rdata, fstat_rxover_rdata, fstat_rxempty_rdata, fstat_rxfull_rdata, fstat_rxlevel_rdata, 5'h0, fstat_txover_rdata, fstat_txempty_rdata, fstat_txfull_rdata, fstat_txlevel_rdata};
assign fstat_txlevel_rdata = fstat_txlevel_i;
assign fstat_txfull_rdata = fstat_txfull_i;
assign fstat_txempty_rdata = fstat_txempty_i;
reg  fstat_txover;
assign fstat_txover_rdata = fstat_txover;
assign fstat_rxlevel_rdata = fstat_rxlevel_i;
assign fstat_rxfull_rdata = fstat_rxfull_i;
assign fstat_rxempty_rdata = fstat_rxempty_i;
reg  fstat_rxover;
assign fstat_rxover_rdata = fstat_rxover;
reg  fstat_rxunder;
assign fstat_rxunder_rdata = fstat_rxunder;

wire [7:0] tx_wdata = wdata[7:0];
wire [7:0] tx_rdata;
wire [31:0] __tx_rdata = {24'h0, tx_rdata};
assign tx_rdata = 8'h0;

wire [7:0] rx_wdata = wdata[7:0];
wire [7:0] rx_rdata;
wire [31:0] __rx_rdata = {24'h0, rx_rdata};
assign rx_rdata = rx_i;

always @ (*) begin
	case (addr)
		ADDR_CSR: rdata = __csr_rdata;
		ADDR_DIV: rdata = __div_rdata;
		ADDR_FSTAT: rdata = __fstat_rdata;
		ADDR_TX: rdata = __tx_rdata;
		ADDR_RX: rdata = __rx_rdata;
		default: rdata = 32'h0;
	endcase
	tx_wen = __tx_wen;
	tx_o = tx_wdata;
	rx_ren = __rx_ren;
end

always @ (posedge clk or negedge rst_n) begin
	if (!rst_n) begin
		csr_csauto_o <= 1'h1;
		csr_cs_o <= 1'h0;
		csr_loopback_o <= 1'h0;
		csr_read_en_o <= 1'h1;
		csr_cpol_o <= 1'h0;
		csr_cpha_o <= 1'h0;
		div_o <= 6'h1;
		fstat_txover <= 1'h0;
		fstat_rxover <= 1'h0;
		fstat_rxunder <= 1'h0;
	end else begin
		if (__csr_wen)
			csr_csauto_o <= csr_csauto_wdata;
		if (__csr_wen)
			csr_cs_o <= csr_cs_wdata;
		if (__csr_wen)
			csr_loopback_o <= csr_loopback_wdata;
		if (__csr_wen)
			csr_read_en_o <= csr_read_en_wdata;
		if (__csr_wen)
			csr_cpol_o <= csr_cpol_wdata;
		if (__csr_wen)
			csr_cpha_o <= csr_cpha_wdata;
		if (__div_wen)
			div_o <= div_wdata;
		fstat_txover <= (fstat_txover && !(__fstat_wen && fstat_txover_wdata)) || fstat_txover_i;
		fstat_rxover <= (fstat_rxover && !(__fstat_wen && fstat_rxover_wdata)) || fstat_rxover_i;
		fstat_rxunder <= (fstat_rxunder && !(__fstat_wen && fstat_rxunder_wdata)) || fstat_rxunder_i;
	end
end

endmodule
