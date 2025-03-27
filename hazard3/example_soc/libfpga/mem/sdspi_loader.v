// modified by Laurentiu-Cristian Duca, 2023-09-12
// used to copy linux from microsd to ram
// adapted from https://github.com/WangXuan95/FPGA-SDcard-Reader
//--------------------------------------------------------------------------------------------------------

`include "define.vh"

module sdspi_loader (
    input  wire         clk27mhz,
    // rstn active-low, You can re-read SDcard by pushing the reset button.
    input  wire         resetn,
    input  wire [2:0]   w_main_init_state,
    input  wire [7:0]   w_ctrl_state,
    output wire [7:0]	w_loader_state,
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
    output reg  [31:0]  DATA,
    output reg          WE,
    output reg          DONE
    );

reg [31:0] rsector=0, i=0, waddr=0;
reg [7:0] state=0, rcnt=0;
assign w_loader_state = state;

    always @(posedge clk27mhz) begin
        if(!resetn) begin
        	psel <= 0;
	        penable <= 0;
	        pwrite <= 0;
	        paddr <= 0;
	        pwdata <= 0;
            rsector <= 0;
            state <= 0;
            DATA <= 0;
            WE <= 0;
            DONE <= 0;
            i <= 0;
	    rcnt <= 0;
	    waddr <= 0;
        end else begin
            if(state == 0) begin
                if (DONE==0)
                    if(w_main_init_state == 3 && !sdsbusy) begin
                        //trigger start reading new sector
			pwrite <= 1;
			psel <= 1;
			penable <= 1;
			pwdata <= rsector;
			paddr <= `SDSPI_DEVADDR; // < `SDSPI_BLOCKADDR;
			state <= 1;
                    end
	    end else if(state == 1) begin
		    if(pready) begin
			psel <= 0;
			penable <= 0;
			state <= 2;
		    end
	    end else if(state == 2) begin
		if(sdsbusy)
			state <= 3;
	    end else if(state == 3) begin
		    if(!sdsbusy) begin
			// sector was read
			state <= 10;
			DATA <= 0;
			rcnt <= 0;
		    end
	    end else if(state == 10) begin
			// read 4 bytes from sd ctrl mem
			pwrite <= 0;
                        psel <= 1;
                        penable <= 1;
                        paddr <= `SDSPI_BLOCKADDR + waddr;
			state <= 11;
	    end else if(state == 11) begin
		    if(pready) begin
			DATA <= {prdata[7:0], DATA[31:8]};
			rcnt <= rcnt + 1;
			waddr <= waddr + 1;
			if(rcnt >= 3) begin
				rcnt <= 0;
				psel <= 0;
                        	penable <= 0;
				state <= 20;
			end else
				state <= 10;
		    end

            end else if(state == 20) begin
		// send sector to main
                if(w_ctrl_state == 0)
                    if((i < `SDSPI_BLOCKSIZE) && ((rsector << $clog2(`SDSPI_BLOCKSIZE))+i) < `BIN_SIZE) begin
                        //DATA <= {mem[i+3], mem[i+2], mem[i+1], mem[i]};
                        WE <= 1;
                        i <= i + 4;
                        state <= 21;
                    end else begin
                        i <= 0;
                        state <= 0;
                        rsector <= rsector + 1;
                        if(waddr>=`BIN_SIZE)
                            DONE <= 1;
                    end
            end else if(state == 21) begin
                if(w_ctrl_state != 0) begin
                    WE <= 0;
                    state <= 10;
                end
            end
        end
    end

endmodule
