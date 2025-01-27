/* Created by Laurentiu-Cristian Duca,
 * spdx license identifier: Apache 2.0
 * write-back cache
 */
`default_nettype none
/**************************************************************************************************/
`include "define.vh"
//`include "hazard3_config.vh"
/**************************************************************************************************/

/**** Memory Controller with Cache                                                             ****/

/**************************************************************************************************/

module cache_ctrl#(parameter PRELOAD_FILE = "",
	`include "hazard3_config.vh"
)
     (
     input wire                          clk,
     input wire 			 rst_x,
     input wire 			 clk_sdram,
     input wire [W_ADDR-1:0]          	 d_pc,
     // user interface ports
     input  wire                         i_rd_en,
     input  wire                         i_wr_en,
     input  wire [31:0]                  i_addr,
     input  wire [31:0]                  i_data,
     output wire [31:0]                  o_data,
     output wire                         o_busy,
     input  wire [3:0]                   i_mask,

     output reg [6:0] 			 state,
     output wire 			 c_oe,
    // tang nano 20k SDRAM
    output wire O_sdram_clk,
    output wire O_sdram_cke,
    output wire O_sdram_cs_n,            // chip select
    output wire O_sdram_cas_n,           // columns address select
    output wire O_sdram_ras_n,           // row address select
    output wire O_sdram_wen_n,           // write enable
    inout wire [31:0] IO_sdram_dq,       // 32 bit bidirectional data bus
    output wire [10:0] O_sdram_addr,     // 11 bit multiplexed address bus
    output wire [1:0] O_sdram_ba,        // two banks
    output wire [3:0] O_sdram_dqm        // 32/4
    );

    m_dram_cache#(32,32,`CACHE_SIZE/4)
			cache (.CLK(clk), .RST_X(1'b1), .w_flush(1'b0), .w_clr(1'b0), 
				.w_we(r_c_we), .w_dirtyi(r_c_dirtyi), .w_dirtyo(c_dirtyo), 
				.w_addr(i_addr), .w_idata(r_c_idata), .i_mask(r_mask), .w_odata(c_odata), .w_oe(c_oe), 
				.w_cache_addr(w_cache_addr));

    wire [31:0] w_cache_addr;
    reg [31:0] r_odata=0;
    reg [3:0] r_mask=0;
    wire c_dirtyo;
    wire [31:0] c_odata;
    reg [31:0] r_c_idata=0;
    reg r_c_dirtyi=0, r_c_we=0, r_busy=0;
    reg [6:0] state_next=0, ostate=0;
    assign o_busy = r_busy;
    assign o_data = state == 0 && i_rd_en && c_oe ? c_odata : r_odata;
    reg rw=0;
    integer i=0, j=0, k=0, l=0, m=0, n=0, nr=0, nh=0, nw=0, cnt=0;

    always @(posedge clk or negedge rst_x) begin
	    if(!rst_x) begin
		    state <= 0;
		    state_next <= 0;
		    r_c_idata <= 0;
		    r_c_dirtyi <= 0;
		    r_c_we <= 0;
		    r_dram_idata <= 0;
		    r_dram_addr <= 0;
		    r_rd_en <= 0;
		    r_wr_en <= 0;
		    r_odata <= 0;
		    r_busy <= 0;
		    r_mask <= 0;
	    end else begin
		    if(state == 0) begin
			    if(i_rd_en) begin
				if(c_oe)
				    nh <= nh + 1;
			    	nr <= nr + 1;
			    end
			    if(i_rd_en) begin
				rw <= 0;
				if(c_oe) begin
					$fwrite(f, "mem read addr %8x data %8x\n", i_addr, o_data);
				end else begin
					r_busy <= 1;
					if(c_dirtyo) begin
						// save old data to ram
						// and load data from ram
                        	                // and write ram data in cache
						r_dram_idata <= c_odata;
						r_dram_addr <= w_cache_addr;
						r_wr_en <= 1;
						state <= 1;
						state_next <= 3;
						if(j == 0) begin
							$display("\tfirst i_rd_en && !c_oe and dirty time=%0d", $time);
						end
						j <= j + 1;
					end else begin // !c_dirtyo and !c_oe
						// load data from ram
                        	                // write ram data in cache
						r_c_dirtyi <= 0;
						state <= 3;
						if(k == 0) begin
                                                	$display("\tfirst i_rd_en && !c_oe and not dirty time=%0d", $time);
	                                        end
						k <= k + 1;
					end
				end
			    end else if(i_wr_en) begin
				r_busy <= 1;
				rw <= 1;
				$fwrite(f, "mem write addr %8x data %8x mask %1x\n", i_addr, i_data, i_mask);
				//if(i_addr == 32'h64) 
                        	//	$display("\tcache we w_addr = 64, data=%x mask=%b c_oe=%x odata=%x", i_data, i_mask, c_oe, c_odata);
				nw <= nw + 1;
				if(c_oe) begin
					// write new data in cache and set dirty
                                        r_c_idata <= i_data;
                                        r_mask <= i_mask;
                                        r_c_dirtyi <= 1;
                                        r_c_we <= 1;
                                        state <= 7;
                                        if(m == 0) begin
                                            $display("\tfirst i_wr_en m simple time=%0d", $time);
                                        end
                                        m <= m + 1;
				end else begin
					if(c_dirtyo) begin
						// save old data to ram
	                                        // write new data in cache
        	                                r_dram_idata <= c_odata;
                	                        r_dram_addr <= w_cache_addr;
                        	                r_wr_en <= 1;
                                	        state <= 1;
                                        	r_mask <= i_mask;
	                                        state_next <= 8;
        	                                r_odata <= i_data;
                	                        r_c_dirtyi <= 1;
                        	                if(l == 0) begin
                                	                $display("\tfirst i_wr_en c_dirtyo && !c_oe time=%0d", $time);
                                        	end
	                                        l <= l+1;
					end else begin
						// write from ram to cache
                                                // and then write new data to cache
						if(i_mask == 4'b1111) begin
							// directly write to cache
							r_c_idata <= i_data;
				                        r_c_dirtyi <= 1;
				                        r_c_we <= 1;
				                        r_mask <= i_mask;
				                        state <= 18;
						end else begin
                                                	r_rd_en <= 1;
                                                	r_dram_addr <= i_addr;
                                                	state <= 14;
						end
						if(n == 0) begin
                                                        $display("\tfirst i_wr_en n simple time=%0d", $time);
                                                end
                                                n <= n + 1;
					end
				end
			end
		    end else if(state == 1) begin
			// wait old data to be written in ram
		    	if(w_dram_busy) begin
				r_wr_en <= 0;
				state <= 2;
			end
		    end else if(state == 2) begin
			if(!w_dram_busy)
				state <= state_next;
		    end else if(state == 3) begin
			// load data from ram
			r_rd_en <= 1;
			r_dram_addr <= i_addr;
			state <= 4;
		    end else if(state == 4) begin
			if(w_dram_busy) begin
				r_rd_en <= 0;
				state <= 5;
			end
		    end else if(state == 5) begin
			if(!w_dram_busy) begin
				state <= 6;
				r_odata <= w_dram_odata;
				r_mask <= 4'b1111;
			end
		    end else if(state == 6) begin
			if(rw == 0)
			    $fwrite(f, "mem read addr %8x data %8x\n", i_addr, r_odata);
			// write data to cache
			if(i_addr >= pc_trace_start && i_addr <= pc_trace_stop)
				$display("\t write to cache i_addr=%x data=%x time=%0d", i_addr, r_odata, $time);
			r_c_idata <= r_odata;
			r_c_dirtyi <= 0;
			r_c_we <= 1;
			state <= 7;
		    end else if(state == 7) begin
			r_c_we <= 0;
			r_c_dirtyi <= 0;
			state <= 0;
			r_busy <= 0;
		    end else if(state == 8) begin
                        // write data to cache
                        if(i_addr >= pc_trace_start && i_addr <= pc_trace_stop)
                                $display("\t write to cache i_addr=%x data=%x", i_addr, r_odata);
                        r_c_idata <= r_odata;
                        r_c_dirtyi <= 1;
                        r_c_we <= 1;
                        state <= 7;
                    end else if(state == 14) begin
                        if(w_dram_busy) begin
                                r_rd_en <= 0;
                                state <= 15;
                        end
                    end else if(state == 15) begin
                        if(!w_dram_busy) begin
                                state <= 16;
                                r_odata <= w_dram_odata;
                                r_mask <= 4'b1111;
                        end
                    end else if(state == 16) begin
                        // write data to cache
                        if(i_addr >= pc_trace_start && i_addr <= pc_trace_stop)
                                $display("\t write to cache i_addr=%x data=%x time=%0d", i_addr, r_odata, $time);
                        r_c_idata <= r_odata;
                        r_c_dirtyi <= 0;
                        r_c_we <= 1;
                        state <= 17;
                    end else if(state == 17) begin
			//$display("state 17 r c dirtyi");
			r_c_idata <= i_data;
			r_c_dirtyi <= 1;
			r_c_we <= 1;
			r_mask <= i_mask;
			state <= 18;
		    end else if(state == 18) begin
			r_c_dirtyi <= 0;
			r_c_we <= 0;
			r_busy <= 0;
			state <= 0;
		    end
	    end

	    if(state != ostate && i < 20) begin
		ostate <= state;
		i <= i+1;
	    	$display("\t state=%x i_addr=%x i_rd_en=%x i_wr_en=%x c_oe=%x o_busy=%x o_data=%x", 
		    state, i_addr, i_rd_en, i_wr_en, c_oe, o_busy, o_data);
	    end

	    cnt <= cnt + 1;
	    if(cnt > 3000000000) begin
		    //$display("%0d: nr=%0d nh=%0d nw=%0d j=%0d k=%0d l=%0d m=%0d n=%0d", $time, nr, nh, nw, j, k, l, m, n);
		    //$finish;
	    end
    end

    wire w_dram_busy;
    wire sdram_fail;
    wire w_late_refresh;
    wire [7:0] w_mem_state;
    wire calib_done;
    reg [31:0] r_dram_addr=0;
    reg [31:0] r_dram_idata=0;
    reg r_rd_en=0, r_wr_en=0;
    wire [31:0] w_dram_odata;
    DRAM_conRV #(.PRELOAD_FILE(PRELOAD_FILE))
    dram_con (
                               // user interface ports
                               .i_rd_en(r_rd_en),
                               .i_wr_en(r_wr_en),
                               .i_addr(r_dram_addr),
                               .i_data(r_dram_idata),
                               .o_data(w_dram_odata),
                               .o_busy(w_dram_busy),
                               .i_ctrl(4'b1111/*i_mask*/),
                               .sys_state(0), // not used
                               .w_bus_cpustate(0), // not used
                               .mem_state(w_mem_state), // not used

                               .clk(clk),
                               .rst_x(rst_x),
                               .clk_sdram(clk_sdram),
                               .o_init_calib_complete(calib_done),
                               .sdram_fail(sdram_fail),
                               `ifdef TN_DRAM_REFRESH
                               .r_late_refresh(w_late_refresh),
                               `endif

                                `ifdef SIM_MODE
                                .w_mtime(0) // not used
                                `else
                               .O_sdram_clk(O_sdram_clk),
                               .O_sdram_cke(O_sdram_cke),
                               .O_sdram_cs_n(O_sdram_cs_n),            // chip select
                               .O_sdram_cas_n(O_sdram_cas_n),           // columns address select
                               .O_sdram_ras_n(O_sdram_ras_n),           // row address select
                               .O_sdram_wen_n(O_sdram_wen_n),           // write enable
                               .IO_sdram_dq(IO_sdram_dq),       // 32 bit bidirectional data bus
                               .O_sdram_addr(O_sdram_addr),     // 11 bit multiplexed address bus
                               .O_sdram_ba(O_sdram_ba),        // two banks
                               .O_sdram_dqm(O_sdram_dqm)       // 32/4
                               `endif
                               );

integer f;
reg opened=0, closed=0;
reg [31:0] timecnt=0;
always @(posedge clk) begin
    if(!opened) begin
            opened = 1;
            f = $fopen("cl", "w");
            if (f == 0) begin
                $display ("ERROR: fl not opened");
                $finish;
            end
    end
    timecnt <= timecnt + 1;
    if(timecnt > 50000000) begin
                        closed <= 1;
                        $fclose(f);
                        //$finish();
    end
end

endmodule

/**************************************************************************************************/

/*** Single-port RAM with asynchronous read                                                      ***/
module m_bram#(parameter WIDTH=32, ENTRY=256)(CLK, w_we, w_addr, w_idata, i_mask, w_odata, w_dirtyi, w_dirtyo);
  input  wire                     CLK, w_we;
  input  wire [$clog2(ENTRY)-1:0] w_addr;
  input  wire         [WIDTH-1:0] w_idata;
  input wire [3:0]                i_mask;
  input wire w_dirtyi;
  output wire w_dirtyo;
  output wire          [WIDTH-1:0] w_odata;

  // add dirty flag
  reg          [WIDTH-1:0]  mem [0:ENTRY-1];
  //reg r_dirty[0:ENTRY-1];

  integer i;
  initial for (i=0;i<ENTRY;i=i+1) begin 
	  mem[i]=0;
	  //r_dirty[i]=0;
  end

  always  @(posedge  CLK)  begin
	if (w_we) begin
		//r_dirty[w_addr] <= w_dirtyi;
		`ifdef laur0
		if(w_dirtyi) begin
			$display("w_addr=%x w_dirtyi=1", w_addr);
			$finish;
		end
		`endif
		mem[w_addr] <= w_idata;
	end
    //r_odata <= mem[w_addr];
  end
  assign w_odata = mem[w_addr];
  //assign w_dirtyo=r_dirty[w_addr];
endmodule

/**************************************************************************************************/
/*** Simple Direct Mapped Cache Sync CLK for DRAM                                               ***/
/**************************************************************************************************/
module m_dram_cache#(parameter ADDR_WIDTH = 30, D_WIDTH = 32, ENTRY = 1024)
    (CLK, RST_X, w_flush, w_clr, w_we, w_dirtyi, w_dirtyo, w_addr, w_idata, i_mask, w_odata, w_oe, w_cache_addr);
    input  wire                     CLK, RST_X;
    input  wire                     w_flush, w_we, w_clr;
    input wire w_dirtyi;
    output wire w_dirtyo;
    input  wire [ADDR_WIDTH-1:0]    w_addr;
    output wire [ADDR_WIDTH-1:0]    w_cache_addr;
    input  wire    [D_WIDTH-1:0]    w_idata;
    input wire [3:0] i_mask;
    output wire    [D_WIDTH-1:0]    w_odata;
    output wire                     w_oe;             //output enable

    // index and tag
    wire                [$clog2(ENTRY)-1:0] w_idx;
    wire [(ADDR_WIDTH - $clog2(ENTRY))-1:0] w_tag;
    assign {w_tag, w_idx} = w_addr;

    wire                                            w_mwe = w_clr | w_we | !RST_X | w_flush;
    wire                      [$clog2(ENTRY)-1:0]   w_maddr = w_idx;
    wire [ADDR_WIDTH - $clog2(ENTRY) + D_WIDTH + 1:0]   w_mwdata = w_we ? {1'b1, w_dirtyi, w_tag, w_laurdata/*w_idata*/} : 0;
    wire [ADDR_WIDTH - $clog2(ENTRY) + D_WIDTH + 1:0]   w_modata;

    wire                                            w_mvalid;
    wire                     [$clog2(ENTRY)-1:0]    w_midx;
    wire      [(ADDR_WIDTH - $clog2(ENTRY))-1:0]    w_mtag;
    wire                           [D_WIDTH-1:0]    w_mdata;
    assign {w_mvalid, w_dirtyo, w_mtag, w_mdata} = w_modata;
    assign w_cache_addr = {w_mtag, w_idx};


    m_bram#((ADDR_WIDTH - $clog2(ENTRY) + D_WIDTH)+1+1, ENTRY)
        mem(.CLK(CLK), .w_we(w_mwe), .w_addr(w_maddr), .w_idata(w_mwdata), 
		.i_mask(i_mask), .w_odata(w_modata), .w_dirtyi(w_dirtyi), .w_dirtyo(/*w_dirtyo*/));

    assign w_odata  = w_mdata;
    assign w_oe     = (w_mvalid && w_mtag == w_tag);

    wire [D_WIDTH-1:0] w_laurdata;
    assign w_laurdata =
	// 1 unit
        (i_mask == 4'b0001) ? {w_mdata[31:8], w_idata[7:0]} :
        (i_mask == 4'b0010) ? {w_mdata[31:16], w_idata[15:8], w_mdata[7:0]} :
        (i_mask == 4'b0100) ? {w_mdata[31:24], w_idata[23:16], w_mdata[15:0]} :
        (i_mask == 4'b1000) ? {w_idata[31:24], w_mdata[23:0]} :
        // 2 units
        (i_mask == 4'b0011) ? {w_mdata[31:16], w_idata[15:0]} :
        (i_mask == 4'b0101) ? {w_mdata[31:24], w_idata[23:16], w_mdata[15:8], w_idata[7:0]} :
        (i_mask == 4'b1001) ? {w_idata[31:24], w_mdata[23:8], w_idata[7:0]} :
        (i_mask == 4'b0110) ? {w_mdata[31:24], w_idata[23:8], w_mdata[7:0]} :
        (i_mask == 4'b1010) ? {w_idata[31:24], w_mdata[23:16], w_idata[15:8], w_mdata[7:0]} :
        (i_mask == 4'b1100) ? {w_idata[31:16], w_mdata[15:0]} :
        // 3 units
        (i_mask == 4'b0111) ? {w_mdata[31:24], w_idata[23:0]} :
        (i_mask == 4'b1011) ? {w_idata[31:24], w_mdata[23:16], w_idata[15:0]} :
        (i_mask == 4'b1101) ? {w_idata[31:16], w_mdata[15:8], w_idata[7:0]} :
        (i_mask == 4'b1110) ? {w_idata[31:8], w_mdata[7:0]} :
        // 4 units
        (i_mask == 4'b1111) ? w_idata : w_mdata;

	always @(posedge CLK) begin
		if((i_mask != 4'b0000) &&
	        // 1 unit
        	(i_mask != 4'b0001) &&
	        (i_mask != 4'b0010) &&
        	(i_mask != 4'b0100) &&
	        (i_mask != 4'b1000) &&
        	// 2 units
	        (i_mask != 4'b0011) &&
        	(i_mask != 4'b0101) &&
	        (i_mask != 4'b1001) && 
        	(i_mask != 4'b0110) && 
	        (i_mask != 4'b1010) && 
        	(i_mask != 4'b1100) && 
	        // 3 units
        	(i_mask != 4'b0111) && 
	        (i_mask != 4'b1011) && 
        	(i_mask != 4'b1101) && 
	        (i_mask != 4'b1110) &&
		(i_mask != 4'b1111)) begin
			$display("i_mask=%b", i_mask);
			$finish;
		end
	end

endmodule 
/**************************************************************************************************/

