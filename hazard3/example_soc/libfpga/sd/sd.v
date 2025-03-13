
module hazard3_sd #(
        parameter W_ADDR = 32,
        parameter W_DATA = 32	
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

assign hburst = 0;

wire writecmd = pwrite && psel && penable;
wire readcmd = !pwrite && psel && penable;
reg [7:0] state;
reg rbusy, havesetprdata;
reg [31:0] rwdata;
integer i;
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		state <= 0;
		rbusy <= 0;
		havesetprdata <= 0;
		hwdata <= 0;
		hsize <= 0;
		htrans <= 0;
		prdata <= 0;
		rwdata <= 0;
		hprot <= 0;
		i <= 0;
	end else begin
		if(state == 0) begin
			if(writecmd) begin
				$display("sd state 0 write %x", pwdata);
				rbusy <= 0;
				state <= 1;
				rwdata <= pwdata;
			end else if (readcmd) begin
				rbusy <= 0;
				state <= 10;
				$display("sd state 0 read");
			end
		end else if(state == 1) begin
			rbusy <= 0;
			if(!writecmd) begin
				hwrite <= 1;
                                htrans <= 2'b10; // nonseq transfer
                                haddr <= rwdata;
				hsize <= 2; // 0=1byte, 1=2bytes, 2=4bytes
                                hwdata <= 32'h76543210;
				state <= 2;
			end
		end else if(state == 2) begin
			if(i < 10) begin
				$display("sd state 2 writecmd=%x hready=%x htrans=%x\n", writecmd, hready, htrans);
				i <= i + 1;
			end
			if(!hready) begin
				htrans <= 0; // idle
				hwrite <= 0;
				rbusy <= 0;
				state <= 3;
			end
		end else if(state == 3) begin
                        if(i < 20) begin
                                $display("sd state 3 writecmd=%x hready=%x htrans=%x\n", writecmd, hready, htrans);
                                i <= i + 1;
                        end
			if(!writecmd && hready)
				state <= 0;
		end else if(state == 10) begin
                                htrans <= 2'b10; // nonseq transfer
                                haddr <= rwdata;
                                hsize <= 2; // 0=1byte, 1=2bytes, 2=4bytes
                                state <= 12;
		end else if(state == 12) begin
			if(!hready) begin
				htrans <= 0; // idle
                                rbusy <= 0;
				havesetprdata <= 0;
                                state <= 13;
			end
                        if(i < 30) begin
                                $display("sd state 12 readcmd=%x hready=%x htrans=%x hrdata=%x\n", readcmd, hready, htrans, hrdata);
                                i <= i + 1;
                        end
		end else if(state == 13) begin
			if(hready && !havesetprdata) begin
				prdata <= hrdata;
				havesetprdata <= 1;
				$display("sd state 13 readcmd=%x hready=%x htrans=%x prdata <= hrdata=%x\n", readcmd, hready, htrans, hrdata);
			end
                        if(i < 40) begin
                                $display("sd state 13 readcmd=%x hready=%x htrans=%x hrdata=%x\n", readcmd, hready, htrans, hrdata);
                                i <= i + 1;
                        end

			if(!readcmd && hready) begin
				state <= 0;
				rbusy <= 0;
			end
		end
	end
end

assign pready=!rbusy;

endmodule
