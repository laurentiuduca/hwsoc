// laur
`define SIM_MODE
`define TN_DRAM_REFRESH // for tang nano
`define SIM_TNSRAM // tang nano not only sim ram

`define FREQ 27_000_000
`define MEM_SIZE (8*1024*1024)
`define BBL_SIZE (8*1024*1024)

`define CACHE_SIZE (32*1024)

`ifdef SIM_MODE
`define SERIAL_WCNT 2
`else
`define SERIAL_WCNT (`FREQ / 115200)
`endif

`define XLEN    32
`define LATENCY 0

