
`include "sd_defines.h"

module hazard3_sd #(
        parameter W_ADDR = 32,
        parameter W_DATA = 32,
	parameter ramdisk="ramdisk2.hex",
	parameter sd_model_log_file={"sd_model.log"}
  	//parameter wb_memory_file={`BIN_DIR, "/wb_memory.txt"})
) (
        input wire                clk,
        input wire                rst_n,

        // AHB5 Master port
        output reg  [W_ADDR-1:0]  haddr,
        output reg                hwrite,
        output reg  [1:0]         htrans,
        output reg  [2:0]         hsize,
        output wire [2:0]         hburst,
        output reg  [3:0]         hprot,
        output wire               hmastlock,
        //output reg  [7:0]         hmaster,
        output reg                hexcl,
        input  wire               hready,
        input  wire               hresp,
        //input  wire               hexokay,
        output reg [W_DATA-1:0]  hwdata,
        input  wire [W_DATA-1:0]  hrdata,

        // APB Port
        input wire psel,
        input wire penable,
        input wire pwrite,
        input wire [15:0] paddr,
        input wire [31:0] pwdata,
        output reg [31:0] prdata,
        output wire pready,
        output wire pslverr
);

reg wb_clk;
reg wb_rst;
wire [31:0] wbs_sds_dat_i;
wire [31:0] wbs_sds_dat_o;
wire [31:0] wbs_sds_adr_i;
wire [3:0] wbs_sds_sel_i;
wire wbs_sds_we_i;
wire wbs_sds_cyc_i;
wire wbs_sds_stb_i;
wire wbs_sds_ack_o;
wire [31:0] wbm_sdm_adr_o;
wire [3:0] wbm_sdm_sel_o;
wire wbm_sdm_we_o;
wire [31:0] wbm_sdm_dat_i;
wire [31:0] wbm_sdm_dat_o;
wire wbm_sdm_cyc_o;
wire wbm_sdm_stb_o;
wire wbm_sdm_ack_i;
wire [2:0] wbm_sdm_cti_o;
wire [1:0] wbm_sdm_bte_o;

wire sd_cmd_oe;
wire sd_dat_oe;
wire cmdIn;
wire [3:0] datIn;
tri sd_cmd;
tri [3:0] sd_dat;

assign sd_cmd = sd_cmd_oe ? cmdIn: 1'bz;
assign sd_dat =  sd_dat_oe  ? datIn : 4'bz;

wire sd_clk_pad_o;
wire int_cmd, int_data;

sdModel #(.ramdisk (ramdisk),
    .log_file (sd_model_log_file)) sdModelTB0(
    .sdClk(sd_clk_pad_o),
    .cmd(sd_cmd),
    .dat(sd_dat)
    ); 

sdc_controller sd_controller_top_dut(
    .wb_clk_i(wb_clk),
    .wb_rst_i(wb_rst),
    .wb_dat_i(wbs_sds_dat_i),
    .wb_dat_o(wbs_sds_dat_o),
    .wb_adr_i(wbs_sds_adr_i[7:0]),
    .wb_sel_i(wbs_sds_sel_i),
    .wb_we_i(wbs_sds_we_i),
    .wb_stb_i(wbs_sds_stb_i),
    .wb_cyc_i(wbs_sds_cyc_i),
    .wb_ack_o(wbs_sds_ack_o),
    .m_wb_adr_o(wbm_sdm_adr_o),
    .m_wb_sel_o(wbm_sdm_sel_o),
    .m_wb_we_o(wbm_sdm_we_o),
    .m_wb_dat_o(wbm_sdm_dat_o),
    .m_wb_dat_i(wbm_sdm_dat_i),
    .m_wb_cyc_o(wbm_sdm_cyc_o),
    .m_wb_stb_o(wbm_sdm_stb_o),
    .m_wb_ack_i(wbm_sdm_ack_i),
    .m_wb_cti_o(wbm_sdm_cti_o),
    .m_wb_bte_o(wbm_sdm_bte_o),
    .sd_cmd_dat_i(sd_cmd),
    .sd_cmd_out_o(cmdIn),
    .sd_cmd_oe_o(sd_cmd_oe),
    .sd_dat_dat_i(sd_dat),
    .sd_dat_out_o(datIn),
    .sd_dat_oe_o( sd_dat_oe),
    .sd_clk_o_pad(sd_clk_pad_o),
    .sd_clk_i_pad(wb_clk),
    .int_cmd (int_cmd),
    .int_data (int_data)
    );

`ifdef laur0
WB_MASTER_BEHAVIORAL wb_master0(
    .CLK_I(wb_clk),
    .RST_I(wb_rst),
    .TAG_I(5'h0), //Not in use
    .TAG_O(), //Not in use
    .ACK_I(wbs_sds_ack_o),
    .ADR_O(wbs_sds_adr_i),
    .CYC_O(wbs_sds_cyc_i),
    .DAT_I(wbs_sds_dat_o),
    .DAT_O(wbs_sds_dat_i),
    .ERR_I(1'b0), //Not in use
    .RTY_I(1'b0), //inactive (1'b0)
    .SEL_O(wbs_sds_sel_i),
    .STB_O(wbs_sds_stb_i),
    .WE_O (wbs_sds_we_i),
    .CAB_O() //Not in use
    );

WB_SLAVE_BEHAVIORAL
  #(.wb_memory_file (wb_memory_file))
   wb_slave0(
    .CLK_I(wb_clk),
    .RST_I(wb_rst),
    .ACK_O(wbm_sdm_ack_i),
    .ADR_I(wbm_sdm_adr_o),
    .CYC_I(wbm_sdm_cyc_o),
    .DAT_O(wbm_sdm_dat_i),
    .DAT_I(wbm_sdm_dat_o),
    .ERR_O(),
    .RTY_O(), //Not in use
    .SEL_I(wbm_sdm_sel_o),
    .STB_I(wbm_sdm_stb_o),
    .WE_I (wbm_sdm_we_o),
    .CAB_I(1'b0)
    );
`endif

endmodule
