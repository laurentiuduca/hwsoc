// modified by Laurentiu-Cristian Duca, 2023-09-12
// used to copy linux from microsd to ram
// adapted from https://github.com/WangXuan95/FPGA-SDcard-Reader
//--------------------------------------------------------------------------------------------------------

`include "define.vh"

module sdspi_reader (
    input  wire         clk,
    // rstn active-low, You can re-read SDcard by pushing the reset button.
    input  wire         rstn,
    //input  wire [2:0]   w_main_init_state,
    //input  wire [7:0]   w_ctrl_state,
    output wire [31:0]	w_reader_status,
    input  wire [31:0]  sdspi_status,
        // signals connect to SD controller
        output reg         psel,
        output reg         penable,
        output reg         pwrite,
        output reg  [15:0] paddr,
        output reg  [31:0] pwdata,
        input wire  [31:0] prdata,
        input wire         pready,
        input wire         pslverr,
	input  wire        sdsbusy,
    //output reg  [31:0]  DATA,
    //output reg          WE,
    //output reg          DONE,
    // user read sector command interface (sync with clk)
    input  wire         rstart,
    input  wire [31:0]  rsector,
    output reg         rbusy,
    output reg         rdone,
    // sector data output interface (sync with clk)
    output reg          outen,             // when outen=1, a byte of sector content is read out from outbyte
    output reg  [ 8:0]  outaddr,           // outaddr from 0 to 511, because the sector size is 512
    output reg  [ 7:0]  outbyte            // a byte of sector content

    );

reg [31:0] waddr=0, tc=0;
reg [7:0] state=0;
assign w_reader_status = {rsector[15:0], tc[11:0], state[3:0]};
wire [7:0] sdctrlstate = sdspi_status[15:8];
wire [7:0] sdstate = sdspi_status[7:0];
reg [7:0] firstbyte=0, first=0;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin
        	psel <= 0;
	        penable <= 0;
	        pwrite <= 0;
	        paddr <= 0;
	        pwdata <= 0;
            state <= 0;
	    outaddr <= 0;
	    outbyte <= 0;
	    waddr <= 0;
	    tc <= 0;
        end else begin
            if(state == 0) begin
		rdone <= 0;
	        if (rstart && !sdsbusy && sdstate == 0 && sdctrlstate == 0 && !pready && !rdone) begin
                        //trigger start reading new sector
			pwrite <= 1;
			psel <= 1;
			penable <= 1;
			pwdata <= rsector;
			paddr <= `SDSPI_DEVADDR; // < `SDSPI_BLOCKADDR;
			state <= 1;
			waddr <= 0;
			rbusy <= 1;
			tc <= tc+1;
		end
	    end else if(state == 1) begin
		    if(pready) begin
			// command was taken
			pwrite <= 0;
			psel <= 0;
			penable <= 0;
			state <= 2;
		    end
	    end else if(state == 2) begin
		if(sdsbusy && sdstate)
			state <= 3;
	    end else if(state == 3) begin
		    if(!sdsbusy && sdstate == 0 && sdctrlstate == 0) begin
			// sector was read
			state <= 10;
		    end
	    end else if(state == 10) begin
		    if(!pready && sdctrlstate == 0) begin
			// read byte from sd ctrl mem
			pwrite <= 0;
                        psel <= 1;
                        penable <= 1;
                        paddr <= `SDSPI_BLOCKADDR + waddr;
			state <= 11;
		    end
	    end else if(state == 11) begin
		    if(pready) begin
			psel <= 0;
                        penable <= 0;
			outbyte <= prdata[7:0];
			outen <= 1;
			outaddr <= waddr;
			waddr <= waddr + 1;
			rbusy <= 0;
			if(waddr == 0 && first == 0) begin
				first <= 1;
				firstbyte <= prdata[7:0];
			end
			state <= 12;
		    end
            end else if(state == 12) begin
		    outen <= 0;
		    state <= 13;
	    end else if(state == 13) begin
		    if(waddr < `SDSPI_BLOCKSIZE)
		    	state <= 10;
		    else begin
			    rdone <= 1;
			    state <= 0;
		    end
            end
        end
    end

endmodule
