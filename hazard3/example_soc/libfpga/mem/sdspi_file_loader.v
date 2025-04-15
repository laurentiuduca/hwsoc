//--------------------------------------------------------------------------------------------------------
// Module  : fpga_top
// Type    : synthesizable, FPGA's top, IP's example design
// Standard: Verilog 2001 (IEEE1364-2001)
// Function: an example of sd_file_reader, read a file from SDcard and send file content to UART
//           this example runs on Digilent Nexys4-DDR board (Xilinx Artix-7),
//           see http://www.digilent.com.cn/products/product-nexys-4-ddr-artix-7-fpga-trainer-board.html
//--------------------------------------------------------------------------------------------------------

`include "define.vh"

module sdspi_file_loader #(parameter SD_CLK_DIV = 3'd2) (
    input  wire         clk27mhz,
    input wire resetn,
    input  wire [2:0]   w_main_init_state,
    input  wire [7:0]   w_ctrl_state,
    output wire [5:0]  tangled,
    output reg  [31:0]  DATA,
    output reg          WE,
    output reg          DONE,
        output wire [31:0]  w_reader_status,
        // signals connect to SD controller
        output wire        m_psel,
        output wire        m_penable,
        output wire        m_pwrite,
        output wire [15:0] m_paddr,
        output wire [31:0] m_pwdata,
        input  wire [31:0] m_prdata,
        input  wire        m_pready,
        input  wire        m_pslverr,
        input  wire        m_sdsbusy,
        input  wire [31:0] m_sdspi_status
);


wire clk = clk27mhz;
wire [8:0] led;

//----------------------------------------------------------------------------------------------------
// sd_file_reader
//----------------------------------------------------------------------------------------------------
wire       outen;     // when outen=1, a byte of file content is read out from outbyte
wire [7:0] outbyte;   // a byte of file content

sdspi_file_reader #(
    .FILE_NAME_LEN    ( 11      ),         // the length of "example.txt" (in bytes)
    .FILE_NAME        ( "initmem.bin"  ),  // file name to read
    .CLK_DIV          ( SD_CLK_DIV     )
) u_sd_file_reader (
    .rstn             ( resetn         ),
    .clk              ( clk            ),
    .card_stat        ( led[3:0]       ),  // show the sdcard initialize status
    .card_type        ( led[5:4]       ),  // 0=UNKNOWN    , 1=SDv1    , 2=SDv2  , 3=SDHCv2
    .filesystem_type  ( led[7:6]       ),  // 0=UNASSIGNED , 1=UNKNOWN , 2=FAT16 , 3=FAT32 
    .file_found       ( led[  8]       ),  // 0=file not found, 1=file found
    .outen            ( outen          ),
    .outbyte          ( outbyte        ),
    				.w_reader_status(w_reader_status_orig),
                                // signals connect to SD controller
                                .m_psel(m_psel),
                                .m_penable(m_penable),
                                .m_pwrite(m_pwrite),
                                .m_paddr(m_paddr),
                                .m_pwdata(m_pwdata),
                                .m_prdata(m_prdata),
                                .m_pready(m_pready),
                                .m_pslverr(m_pslverr),
                                .m_sdsbusy(m_sdsbusy),
                                .m_sdspi_status(m_sdspi_status));

    assign tangled = ~{DONE, led[8:4]};

    reg [7:0] state=0;
    reg [1:0] waddr=0;
    reg [7:0] mem[0:3];
    reg       rdone;
    reg [31:0] rsector = 0, i=0;
    
    always @(posedge clk27mhz) begin
        if(!resetn) begin
            rsector <= 0;
            state <= 0;
            WE <= 0;
            DONE <= 0;
            i <= 0;
        end else begin
            if(state == 0) begin
                if (DONE==0) begin
                    if(rdone) begin
                        state <= 20;
                    end
                end
            end else if(state == 20) begin
                if(w_ctrl_state == 0)
                    if(i < `BIN_SIZE) begin
                        WE <= 1;
                        i <= i + 4;
                        state <= 21;
                    end
            end else if(state == 21) begin
                if(w_ctrl_state != 0) begin
                    WE <= 0;
                    state <= 0;
                    if(i>=`BIN_SIZE)
                        DONE <= 1;
                end
            end
        end
    end

    reg rstatechanged=0;
    always @(posedge clk27mhz) begin
        if(!resetn) begin
            waddr <= 0;
            DATA <= 0;
        end else begin
            if(DONE==0 && outen && w_main_init_state == 3) begin
                mem[waddr] <= outbyte;
                waddr <= (waddr + 1) & 2'b11;
                if(waddr == 2'b11) begin
                    DATA <= {outbyte, mem[2], mem[1], mem[0]};
                    rdone <= 1;
                end
            end else if (rdone)
                rdone <= 0;
        end
    end

// laur
wire [31:0] w_reader_status_orig;
assign w_reader_status = w_reader_status_orig; //{i[23:0], w_reader_status_orig[7:0]};
endmodule
