
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
        output wire [W_DATA-1:0]  hwdata,
        input  wire [W_DATA-1:0]  hrdata,

        // APB Port
        input wire psel,
        input wire penable,
        input wire pwrite,
        input wire [15:0] paddr,
        input wire [31:0] pwdata,
        output wire [31:0] prdata,
        output wire pready,
        output wire pslverr
);


reg [7:0] state;
always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		state <= 0;
	end else begin
		if(state == 0) begin
			if(pwrite)
				$display("sd write %x", pwdata);
		end
	end
end

assign pready=1;

endmodule
