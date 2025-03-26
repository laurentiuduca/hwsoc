
`include "define.vh"

module hazard3_sd #(
	parameter DEVADDR=16'h8000,
        parameter W_ADDR = 32,
        parameter W_DATA = 32
) (
        input wire                clk,
        input wire                rst_n,

        // APB Port
        input wire psel,
        input wire penable,
        input wire pwrite,
        input wire [15:0] paddr,
        input wire [31:0] pwdata,
        output reg [31:0] prdata,
        output reg pready,
        output wire pslverr,

	// sd signals
	output wire spi_mosi, spi_clk, spi_cs,
	input wire spi_miso
);

reg [7:0] ctrlstate;

wire bus_write = pwrite && psel && penable;
wire bus_read = !pwrite && psel && penable;

`define BLOCKSIZE 512
`define BLOCK_ADDR (DEVADDR + `BLOCKSIZE)
`define ADDRUH 16'h4000

reg       outen;
reg [7:0] outbyte;
reg [31:0] auxdata=0;

reg [7:0] m[0:`BLOCKSIZE-1];
integer i;
initial for(i=0; i < `BLOCKSIZE; i=i+1) m[i] <= 0;
reg [3:0] mcnt=0;
reg mr1=0, mr2=0, mw1=0, mw2=0;
reg [7:0] midata1=0, midata2=0;
wire [7:0] midata;
reg [7:0] mout=0;
reg [31:0] maddr1, maddr2;
wire [31:0] maddr=(mr1 | mw1) ? maddr1 : maddr2;
wire mw = mw1 | mw2;
assign midata = mw1 ? midata1 : midata2;
always @ (posedge clk) begin
        if(mw)
                m[maddr] <= midata;
        mout <= m[maddr];
end
//assign mout = m[maddr];

//sd state machine
reg [7:0] state=0, errstate=0;
reg [31:0] sdsbaddr=0, oecnt=0;
reg sdsrd=0, sdswr=0;
wire sdserror;
wire sdsbusy=0;
reg noerror=1;
reg [2:0] errorcode=0;
wire [2:0] sdserror_code;
reg [7:0] sdsdin=0, firstdin=0;
reg sdsdin_valid=0;
wire sdsdin_taken;
wire [7:0] sdsdout;
wire sdsdout_avail;
reg sdsdout_taken=0;
wire [1:0] state_o;
wire [7:0] sdsfsm_o;

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		ctrlstate <= 0;
		pready <= 0;
		midata1 <= 0;
		maddr1 <= 0;
		mw1 <= 0;
		mr1 <= 0;
		mcnt <= 0;
	end else if(ctrlstate == 0) begin
		pready <= 0;
		if(bus_write && pready == 0) begin
			$display("bus w paddr=%x pwdata=%x pready=%x", paddr, pwdata, pready);
			if(paddr == 16'h8100) begin
				$display("finish");
				$finish;
			end
			if(paddr < `BLOCK_ADDR) begin
				if(pwdata == 1) begin
					// read block
					ctrlstate <= 2;
				end else begin
					// write block;
					ctrlstate <= 12;
				end
			end else begin
				// write to our block mem
				ctrlstate <= 5;
				auxdata <= pwdata;
				midata1 <= pwdata[7:0];
				maddr1 <= paddr - DEVADDR;
				mw1 <= 1;
				mcnt <= 0;
			end
		end else if(bus_read && pready == 0) begin
			$display("bus r paddr=%x pready=%x", paddr, pready);
                        if(paddr < `BLOCK_ADDR) begin
                               prdata <= {24'd0, sdserror_code, sdserror, 3'd0, sdsbusy};
			       ctrlstate <= 1;
			end else begin
				// read from our block mem
                                ctrlstate <= 15;
                                maddr1 <= paddr - DEVADDR;
				mr1 <= 1;
				mcnt <= 0;
				prdata <= 0;
			end
		end
	end else if(ctrlstate == 1) begin
			pready <= 1;
			prdata <= {24'd0, sdserror_code, sdserror, 3'd0, sdsbusy};
			ctrlstate <= 0;
	end else if(ctrlstate == 2) begin
		// read block command 
	end else if(ctrlstate == 12) begin
		// write block command
        end else if(ctrlstate == 5) begin
			// write to mem
			mcnt <= mcnt + 1;
			auxdata <= {8'h0, auxdata[31:8]};
			midata1 <= auxdata[15:8];
			maddr1 <= maddr1 + 1;
			if(mcnt == 0) begin
				$display("\tbus w addr=%x data=%x", maddr1, midata1);
        	               	ctrlstate <= 6;
                               	mw1 <= 0;
			end
	end else if(ctrlstate == 6) begin
			pready <= 1;
			ctrlstate <= 0;
	end else if(ctrlstate == 15) begin
			// read from mem
			ctrlstate <= 16;
			mr1 <= 0;
	end else if(ctrlstate == 16) begin
                        mcnt <= mcnt + 1;
                        maddr1 <= maddr1 + 1;
			//prdata <= {prdata[23:0], mout};
			prdata <= {mout, mout, mout, mout}; // single char
                        if(mcnt == 0) begin
				$display("\tbus r paddr=%x data=%x", maddr1, mout);
                               	pready <= 1;
                               	ctrlstate <= 0;
                               	mr1 <= 0;
                        end else 
				ctrlstate <= 15;
	end
end



endmodule
