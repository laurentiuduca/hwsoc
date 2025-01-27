/*******************************************************************************
*                          AUTOGENERATED BY REGBLOCK                           *
*                            Do not edit manually.                             *
*          Edit the source file (or regblock utility) and regenerate.          *
*******************************************************************************/

#ifndef _XIP_REGS_H_
#define _XIP_REGS_H_

// Block name           : xip
// Bus type             : apb
// Bus data width       : 32
// Bus address width    : 16

#define XIP_CSR_OFFS 0
#define XIP_TXDATA_OFFS 4
#define XIP_RXDATA_OFFS 8

/*******************************************************************************
*                                     CSR                                      *
*******************************************************************************/

// Control and status register

// Field: CSR_DIRECT  Access: RW
// If 1, enable direct mode. Chip select is held asserted until direct mode is
// disabled, and individual bytes can be written to TXDATA and then read from
// RXDATA. If you enable direct mode whilst executing from XIP, you're gonna
// have a bad time.
#define XIP_CSR_DIRECT_LSB  0
#define XIP_CSR_DIRECT_BITS 1
#define XIP_CSR_DIRECT_MASK 0x1
// Field: CSR_BUSY  Access: ROV
// Reads as 1 when a direct mode transfer is in progress. Note a direct mode
// transfer takes precisely 16 cycles (in between the APB write and readback, so
// at least 19 cycles total), so you could insert a precise delay in your code
// instead of polling this bit.
#define XIP_CSR_BUSY_LSB  1
#define XIP_CSR_BUSY_BITS 1
#define XIP_CSR_BUSY_MASK 0x2

/*******************************************************************************
*                                    TXDATA                                    *
*******************************************************************************/

// Direct mode transmit data

// Field: TXDATA  Access: WF
#define XIP_TXDATA_LSB  0
#define XIP_TXDATA_BITS 8
#define XIP_TXDATA_MASK 0xff

/*******************************************************************************
*                                    RXDATA                                    *
*******************************************************************************/

// Direct mode receive data

// Field: RXDATA  Access: RF
#define XIP_RXDATA_LSB  0
#define XIP_RXDATA_BITS 8
#define XIP_RXDATA_MASK 0xff

#endif // _XIP_REGS_H_
