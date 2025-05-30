// Simple UART for RISCBoy

// - APB slave interface
// - 8 data bits, 1 stop bit, 1 start bit ONLY

`include "define.vh"

module uart_mini (
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

	input wire rx,
	output wire tx,
	input wire cts,
	output reg rts,

	output wire irq,
	output wire dreq
);

assign apbs_prdata=32'h0;
assign apbs_pslverr=0;

reg [7:0] state;
reg r_tx_ready, r_was_write;

wire w_tx_ready;
reg         r_uart_we;
reg   [7:0] r_uart_data;

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		r_uart_we <= 0;
		r_uart_data <= 0;
		state <= 0;
		r_tx_ready <= 0;
		r_was_write <= 0;
	end else if(state == 0) begin	
		if(apbs_psel && apbs_penable && apbs_pwrite) begin
			$write("---uart-write %x r_was_write=%x %d\n", apbs_pwdata, r_was_write, $time);
		        if(r_was_write == 0) begin
				if(w_tx_ready) begin
					r_uart_data <= apbs_pwdata[7:0];
	                       		r_uart_we <= 1;
					state <= 1;
					r_was_write <= 1;
				end
			end
		end else
			r_was_write <= 0;
		r_tx_ready <= 0;
	end else if(state == 1) begin
		if(!w_tx_ready) begin
			r_uart_we <= 0;
			state <= 2;
		end
	end else if(state == 2) begin
		if(w_tx_ready) begin
			r_tx_ready <= 1;
			state <= 0;
		end
	end
end

assign irq=0;
assign apbs_pready = r_tx_ready;

UartTx UartTx0(clk, rst_n, r_uart_data, r_uart_we, tx, w_tx_ready);


endmodule

module UartTx(CLK, RST_X, DATA, WE, TXD, READY);
    input  wire        CLK, RST_X, WE;
    input  wire [7:0]  DATA;
    output reg         TXD, READY;
    reg [8:0]   cmd;
    reg [31:0]  waitnum;
    reg [3:0]   cnt;

    always @(posedge CLK) begin
        if(!RST_X) begin
            TXD       <= 1'b1;
            READY     <= 1'b1;
            cmd       <= 9'h1ff;
            waitnum   <= 0;
            cnt       <= 0;
        end else if( READY ) begin
            TXD       <= 1'b1;
            waitnum   <= 0;
            if( WE )begin
		`ifdef SIM_MODE
                $write("%c", DATA);
                `ifndef VERILATOR
                $fflush();
                `endif
		`endif
                READY <= 1'b0;
                cmd   <= {DATA, 1'b0};
                cnt   <= 10;
            end
        end else if( waitnum >= `SERIAL_WCNT ) begin
            TXD       <= cmd[0];
            READY     <= (cnt == 1);
            cmd       <= {1'b1, cmd[8:1]};
            waitnum   <= 1;
            cnt       <= cnt - 1;
        end else begin
            waitnum   <= waitnum + 1;
        end
    end
endmodule

