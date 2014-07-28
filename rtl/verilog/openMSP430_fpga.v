//----------------------------------------------------------------------------
// Copyright (C) 2001 Authors
//
// This source file may be used and distributed without restriction provided
// that this copyright statement is not removed from the file and that any
// derivative work contains the original copyright notice and the associated
// disclaimer.
//
// This source file is free software; you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation; either version 2.1 of the License, or
// (at your option) any later version.
//
// This source is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public
// License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this source; if not, write to the Free Software Foundation,
// Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//----------------------------------------------------------------------------
// 
// *File Name: openMSP430_fpga.v
// 
// *Module Description:
//                      openMSP430 FPGA Top-level for the Diligent
//                     Spartan-3 starter kit.
//
// *Author(s):
//              - Olivier Girard,    olgirard@gmail.com
//
//----------------------------------------------------------------------------
// $Rev: 155 $
// $LastChangedBy: olivier.girard $
// $LastChangedDate: 2012-10-16 03:05:05 +0530 (Tue, 16 Oct 2012) $
//----------------------------------------------------------------------------
`include "openMSP430_defines.v"
`define min_hw 
module openMSP430_fpga (

    reset_ip ,	    
// Slide Switches
`ifndef min_hw 
    SW7,
    SW6,
    SW5,
    SW4,
    SW3,
    SW2,
    SW1,
    SW0,
`endif
// Push Button Switches
/*
    BTN3,
    BTN2,
    BTN1,
    BTN0,
*/
// LEDs
    LED7,
    LED6,
    LED5,
    LED4,
    LED3,
    LED2,
    LED1,
    LED0,

// Four-Sigit, Seven-Segment LED Display
`ifndef min_hw 
    SEG_A,
    SEG_B,
    SEG_C,
    SEG_D,
    SEG_E,
    SEG_F,
    SEG_G,
    SEG_DP,
    SEG_AN0,
    SEG_AN1,
    SEG_AN2,
    SEG_AN3,

// RS-232 Port
    UART_RXD_A,
    UART_TXD_A,
`endif
    UART_RXD,
    UART_TXD
);

input     reset_ip;

// Slide Switches
`ifndef min_hw 
input     SW7;
input     SW6;
input     SW5;
input     SW4;
input     SW3;
input     SW2;
input     SW1;
input     SW0;
`endif
// Push Button Switches
/*
input     BTN3;
input     BTN2;
input     BTN1;
input     BTN0;
*/
// LEDs
output    LED7;
output    LED6;
output    LED5;
output    LED4;
output    LED3;
output    LED2;
output    LED1;
output    LED0;

// Four-Sigit, Seven-Segment LED Display
`ifndef min_hw 
output    SEG_A;
output    SEG_B;
output    SEG_C;
output    SEG_D;
output    SEG_E;
output    SEG_F;
output    SEG_G;
output    SEG_DP;
output    SEG_AN0;
output    SEG_AN1;
output    SEG_AN2;
output    SEG_AN3;

// RS-232 Port
input     UART_RXD_A;
output    UART_TXD_A;
`endif
input     UART_RXD;
output    UART_TXD;

//=============================================================================
// 1)  INTERNAL WIRES/REGISTERS/PARAMETERS DECLARATION
//=============================================================================

// openMSP430 output buses
wire        [13:0] per_addr;
wire        [15:0] per_din;
wire         [1:0] per_we;
wire [`DMEM_MSB:0] dmem_addr;
wire        [15:0] dmem_din;
wire         [1:0] dmem_wen;
wire [`PMEM_MSB:0] pmem_addr;
wire        [15:0] pmem_din;
wire         [1:0] pmem_wen;
wire        [13:0] irq_acc;

// openMSP430 input buses
wire   	    [13:0] irq_bus;
wire        [15:0] per_dout;
wire        [15:0] dmem_dout;
wire        [15:0] pmem_dout;

// GPIO
wire         [7:0] p1_din;
wire         [7:0] p1_dout;
wire         [7:0] p1_dout_en;
wire         [7:0] p1_sel;
wire         [7:0] p2_din;
wire         [7:0] p2_dout;
wire         [7:0] p2_dout_en;
wire         [7:0] p2_sel;
wire         [7:0] p3_din;
wire         [7:0] p3_dout;
wire         [7:0] p3_dout_en;
wire         [7:0] p3_sel;
wire        [15:0] per_dout_dio;

// Timer A
wire        [15:0] per_dout_tA;

// 7 segment driver
wire        [15:0] per_dout_7seg;

// Simple UART
wire               irq_uart_rx;
wire               irq_uart_tx;
wire        [15:0] per_dout_uart;
wire               hw_uart_txd;
wire               hw_uart_rxd;

      
// Others
wire               reset_pin;


//=============================================================================
// 2)  CLOCK GENERATION
//=============================================================================
defparam OSCH_inst.NOM_FREQ = "20.46"; 

OSCH OSCH_inst (
.STDBY (1'b0) ,
.OSC (clk_sys) ,
.SEDSTDBY ()
);

// 2)  RESET GENERATION
//=============================================================================

assign  reset_n = reset_ip;

//=============================================================================
// 4)  OPENMSP430
//=============================================================================

openMSP430 openMSP430_0 (

// OUTPUTs
    .aclk              (),             // ASIC ONLY: ACLK
    .aclk_en           (aclk_en),      // FPGA ONLY: ACLK enable
    .dbg_freeze        (dbg_freeze),   // Freeze peripherals
    .dbg_i2c_sda_out   (),             // Debug interface: I2C SDA OUT
    .dbg_uart_txd      (dbg_uart_txd), // Debug interface: UART TXD
    .dco_enable        (),             // ASIC ONLY: Fast oscillator enable
    .dco_wkup          (),             // ASIC ONLY: Fast oscillator wake-up (asynchronous)
    .dmem_addr         (dmem_addr),    // Data Memory address
    .dmem_cen          (dmem_cen),     // Data Memory chip enable (low active)
    .dmem_din          (dmem_din),     // Data Memory data input
    .dmem_wen          (dmem_wen),     // Data Memory write enable (low active)
    .irq_acc           (irq_acc),      // Interrupt request accepted (one-hot signal)
    .lfxt_enable       (),             // ASIC ONLY: Low frequency oscillator enable
    .lfxt_wkup         (),             // ASIC ONLY: Low frequency oscillator wake-up (asynchronous)
    .mclk              (mclk),         // Main system clock
    .per_addr          (per_addr),     // Peripheral address
    .per_din           (per_din),      // Peripheral data input
    .per_we            (per_we),       // Peripheral write enable (high active)
    .per_en            (per_en),       // Peripheral enable (high active)
    .pmem_addr         (pmem_addr),    // Program Memory address
    .pmem_cen          (pmem_cen),     // Program Memory chip enable (low active)
    .pmem_din          (pmem_din),     // Program Memory data input (optional)
    .pmem_wen          (pmem_wen),     // Program Memory write enable (low active) (optional)
    .puc_rst           (puc_rst),      // Main system reset
    .smclk             (),             // ASIC ONLY: SMCLK
    .smclk_en          (smclk_en),     // FPGA ONLY: SMCLK enable

// INPUTs
    .cpu_en            (1'b1),         // Enable CPU code execution (asynchronous and non-glitchy)
    .dbg_en            (1'b1),         // Debug interface enable (asynchronous and non-glitchy)
    .dbg_i2c_addr      (7'h00),        // Debug interface: I2C Address
    .dbg_i2c_broadcast (7'h00),        // Debug interface: I2C Broadcast Address (for multicore systems)
    .dbg_i2c_scl       (1'b1),         // Debug interface: I2C SCL
    .dbg_i2c_sda_in    (1'b1),         // Debug interface: I2C SDA IN
    .dbg_uart_rxd      (dbg_uart_rxd), // Debug interface: UART RXD (asynchronous)
    .dco_clk           (clk_sys),      // Fast oscillator (fast clock)
    .dmem_dout         (dmem_dout),    // Data Memory data output
    .irq               (irq_bus),      // Maskable interrupts
    .lfxt_clk          (1'b0),         // Low frequency oscillator (typ 32kHz)
    .nmi               (nmi),          // Non-maskable interrupt (asynchronous)
    .per_dout          (per_dout),     // Peripheral data output
    .pmem_dout         (pmem_dout),    // Program Memory data output
    .reset_n           (reset_n),      // Reset Pin (low active, asynchronous and non-glitchy)
    .scan_enable       (1'b0),         // ASIC ONLY: Scan enable (active during scan shifting)
    .scan_mode         (1'b0),         // ASIC ONLY: Scan mode
    .wkup              (1'b0)          // ASIC ONLY: System Wake-up (asynchronous and non-glitchy)
);


//=============================================================================
// 5)  OPENMSP430 PERIPHERALS
//=============================================================================

//
// Digital I/O
//-------------------------------

omsp_gpio #(.P1_EN(1),
            .P2_EN(1),
            .P3_EN(1),
            .P4_EN(0),
            .P5_EN(0),
            .P6_EN(0)) gpio_0 (

// OUTPUTs
    .irq_port1    (irq_port1),     // Port 1 interrupt
    .irq_port2    (irq_port2),     // Port 2 interrupt
    .p1_dout      (p1_dout),       // Port 1 data output
    .p1_dout_en   (p1_dout_en),    // Port 1 data output enable
    .p1_sel       (p1_sel),        // Port 1 function select
    .p2_dout      (p2_dout),       // Port 2 data output
    .p2_dout_en   (p2_dout_en),    // Port 2 data output enable
    .p2_sel       (p2_sel),        // Port 2 function select
    .p3_dout      (p3_dout),       // Port 3 data output
    .p3_dout_en   (p3_dout_en),    // Port 3 data output enable
    .p3_sel       (p3_sel),        // Port 3 function select
    .p4_dout      (),              // Port 4 data output
    .p4_dout_en   (),              // Port 4 data output enable
    .p4_sel       (),              // Port 4 function select
    .p5_dout      (),              // Port 5 data output
    .p5_dout_en   (),              // Port 5 data output enable
    .p5_sel       (),              // Port 5 function select
    .p6_dout      (),              // Port 6 data output
    .p6_dout_en   (),              // Port 6 data output enable
    .p6_sel       (),              // Port 6 function select
    .per_dout     (per_dout_dio),  // Peripheral data output
			     
// INPUTs
    .mclk         (mclk),          // Main system clock
    .p1_din       (p1_din),        // Port 1 data input
    .p2_din       (p2_din),        // Port 2 data input
    .p3_din       (p3_din),        // Port 3 data input
    .p4_din       (8'h00),         // Port 4 data input
    .p5_din       (8'h00),         // Port 5 data input
    .p6_din       (8'h00),         // Port 6 data input
    .per_addr     (per_addr),      // Peripheral address
    .per_din      (per_din),       // Peripheral data input
    .per_en       (per_en),        // Peripheral enable (high active)
    .per_we       (per_we),        // Peripheral write enable (high active)
    .puc_rst      (puc_rst)        // Main system reset
);

//
// Timer A
//----------------------------------------------

omsp_timerA timerA_0 (

// OUTPUTs
    .irq_ta0      (irq_ta0),       // Timer A interrupt: TACCR0
    .irq_ta1      (irq_ta1),       // Timer A interrupt: TAIV, TACCR1, TACCR2
    .per_dout     (per_dout_tA),   // Peripheral data output
    .ta_out0      (ta_out0),       // Timer A output 0
    .ta_out0_en   (ta_out0_en),    // Timer A output 0 enable
    .ta_out1      (ta_out1),       // Timer A output 1
    .ta_out1_en   (ta_out1_en),    // Timer A output 1 enable
    .ta_out2      (ta_out2),       // Timer A output 2
    .ta_out2_en   (ta_out2_en),    // Timer A output 2 enable

// INPUTs
    .aclk_en      (aclk_en),       // ACLK enable (from CPU)
    .dbg_freeze   (dbg_freeze),    // Freeze Timer A counter
    .inclk        (inclk),         // INCLK external timer clock (SLOW)
    .irq_ta0_acc  (irq_acc[9]),    // Interrupt request TACCR0 accepted
    .mclk         (mclk),          // Main system clock
    .per_addr     (per_addr),      // Peripheral address
    .per_din      (per_din),       // Peripheral data input
    .per_en       (per_en),        // Peripheral enable (high active)
    .per_we       (per_we),        // Peripheral write enable (high active)
    .puc_rst      (puc_rst),       // Main system reset
    .smclk_en     (smclk_en),      // SMCLK enable (from CPU)
    .ta_cci0a     (ta_cci0a),      // Timer A capture 0 input A
    .ta_cci0b     (ta_cci0b),      // Timer A capture 0 input B
    .ta_cci1a     (ta_cci1a),      // Timer A capture 1 input A
    .ta_cci1b     (1'b0),          // Timer A capture 1 input B
    .ta_cci2a     (ta_cci2a),      // Timer A capture 2 input A
    .ta_cci2b     (1'b0),          // Timer A capture 2 input B
    .taclk        (taclk)          // TACLK external timer clock (SLOW)
);

   
//
// Four-Digit, Seven-Segment LED Display driver
//----------------------------------------------
`ifndef min_hw
driver_7segment driver_7segment_0 (

// OUTPUTs
    .per_dout     (per_dout_7seg), // Peripheral data output
    .seg_a        (seg_a_),        // Segment A control
    .seg_b        (seg_b_),        // Segment B control
    .seg_c        (seg_c_),        // Segment C control
    .seg_d        (seg_d_),        // Segment D control
    .seg_e        (seg_e_),        // Segment E control
    .seg_f        (seg_f_),        // Segment F control
    .seg_g        (seg_g_),        // Segment G control
    .seg_dp       (seg_dp_),       // Segment DP control
    .seg_an0      (seg_an0_),      // Anode 0 control
    .seg_an1      (seg_an1_),      // Anode 1 control
    .seg_an2      (seg_an2_),      // Anode 2 control
    .seg_an3      (seg_an3_),      // Anode 3 control

// INPUTs
    .mclk         (mclk),          // Main system clock
    .per_addr     (per_addr),      // Peripheral address
    .per_din      (per_din),       // Peripheral data input
    .per_en       (per_en),        // Peripheral enable (high active)
    .per_we       (per_we),        // Peripheral write enable (high active)
    .puc_rst      (puc_rst)        // Main system reset
);
`endif

//
// Simple full duplex UART (8N1 protocol)
//----------------------------------------
`ifndef min_hw
omsp_uart #(.BASE_ADDR(15'h0080)) uart_0 (

// OUTPUTs
    .irq_uart_rx  (irq_uart_rx),   // UART receive interrupt
    .irq_uart_tx  (irq_uart_tx),   // UART transmit interrupt
    .per_dout     (per_dout_uart), // Peripheral data output
    .uart_txd     (hw_uart_txd),   // UART Data Transmit (TXD)

// INPUTs
    .mclk         (mclk),          // Main system clock
    .per_addr     (per_addr),      // Peripheral address
    .per_din      (per_din),       // Peripheral data input
    .per_en       (per_en),        // Peripheral enable (high active)
    .per_we       (per_we),        // Peripheral write enable (high active)
    .puc_rst      (puc_rst),       // Main system reset
    .smclk_en     (smclk_en),      // SMCLK enable (from CPU)
    .uart_rxd     (hw_uart_rxd)    // UART Data Receive (RXD)
);
`endif
//
// Combine peripheral data buses
//-------------------------------

assign per_dout = per_dout_dio  |
                  per_dout_tA   |
                  per_dout_7seg |
                  per_dout_uart;
   
//
// Assign interrupts
//-------------------------------

assign nmi        =  1'b0;
assign irq_bus    = {1'b0,         // Vector 13  (0xFFFA)
                     1'b0,         // Vector 12  (0xFFF8)
                     1'b0,         // Vector 11  (0xFFF6)
                     1'b0,         // Vector 10  (0xFFF4) - Watchdog -
                     irq_ta0,      // Vector  9  (0xFFF2)
                     irq_ta1,      // Vector  8  (0xFFF0)
                     irq_uart_rx,  // Vector  7  (0xFFEE)
                     irq_uart_tx,  // Vector  6  (0xFFEC)
                     1'b0,         // Vector  5  (0xFFEA)
                     1'b0,         // Vector  4  (0xFFE8)
                     irq_port2,    // Vector  3  (0xFFE6)
                     irq_port1,    // Vector  2  (0xFFE4)
                     1'b0,         // Vector  1  (0xFFE2)
                     1'b0};        // Vector  0  (0xFFE0)

//
// GPIO Function selection
//--------------------------

// P1.0/TACLK      I/O pin / Timer_A, clock signal TACLK input
// P1.1/TA0        I/O pin / Timer_A, capture: CCI0A input, compare: Out0 output
// P1.2/TA1        I/O pin / Timer_A, capture: CCI1A input, compare: Out1 output
// P1.3/TA2        I/O pin / Timer_A, capture: CCI2A input, compare: Out2 output
// P1.4/SMCLK      I/O pin / SMCLK signal output
// P1.5/TA0        I/O pin / Timer_A, compare: Out0 output
// P1.6/TA1        I/O pin / Timer_A, compare: Out1 output
// P1.7/TA2        I/O pin / Timer_A, compare: Out2 output
wire [7:0] p1_io_mux_b_unconnected;
wire [7:0] p1_io_dout;
wire [7:0] p1_io_dout_en;
wire [7:0] p1_io_din;

io_mux #8 io_mux_p1 (
		     .a_din      (p1_din),
		     .a_dout     (p1_dout),
		     .a_dout_en  (p1_dout_en),

		     .b_din      ({p1_io_mux_b_unconnected[7],
                                   p1_io_mux_b_unconnected[6],
                                   p1_io_mux_b_unconnected[5],
                                   p1_io_mux_b_unconnected[4],
                                   ta_cci2a,
                                   ta_cci1a,
                                   ta_cci0a,
                                   taclk
                                  }),
		     .b_dout     ({ta_out2,
                                   ta_out1,
                                   ta_out0,
                                   (smclk_en & mclk),
                                   ta_out2,
                                   ta_out1,
                                   ta_out0,
                                   1'b0
                                  }),
		     .b_dout_en  ({ta_out2_en,
                                   ta_out1_en,
                                   ta_out0_en,
                                   1'b1,
                                   ta_out2_en,
                                   ta_out1_en,
                                   ta_out0_en,
                                   1'b0
                                  }),

   	 	     .io_din     (p1_io_din),
		     .io_dout    (p1_io_dout),
		     .io_dout_en (p1_io_dout_en),

		     .sel        (p1_sel)
);



// P2.0/ACLK       I/O pin / ACLK output
// P2.1/INCLK      I/O pin / Timer_A, clock signal at INCLK
// P2.2/TA0        I/O pin / Timer_A, capture: CCI0B input
// P2.3/TA1        I/O pin / Timer_A, compare: Out1 output
// P2.4/TA2        I/O pin / Timer_A, compare: Out2 output
wire [7:0] p2_io_mux_b_unconnected;
wire [7:0] p2_io_dout;
wire [7:0] p2_io_dout_en;
wire [7:0] p2_io_din;

io_mux #8 io_mux_p2 (
		     .a_din      (p2_din),
		     .a_dout     (p2_dout),
		     .a_dout_en  (p2_dout_en),

		     .b_din      ({p2_io_mux_b_unconnected[7],
                                   p2_io_mux_b_unconnected[6],
                                   p2_io_mux_b_unconnected[5],
                                   p2_io_mux_b_unconnected[4],
                                   p2_io_mux_b_unconnected[3],
                                   ta_cci0b,
                                   inclk,
                                   p2_io_mux_b_unconnected[0]
                                  }),
		     .b_dout     ({1'b0,
                                   1'b0,
                                   1'b0,
                                   ta_out2,
                                   ta_out1,
                                   1'b0,
                                   1'b0,
                                   (aclk_en & mclk)
                                  }),
		     .b_dout_en  ({1'b0,
                                   1'b0,
                                   1'b0,
                                   ta_out2_en,
                                   ta_out1_en,
                                   1'b0,
                                   1'b0,
                                   1'b1
                                  }),

   	 	     .io_din     (p2_io_din),
		     .io_dout    (p2_io_dout),
		     .io_dout_en (p2_io_dout_en),

		     .sel        (p2_sel)
);


//=============================================================================
// 6)  PROGRAM AND DATA MEMORIES
//=============================================================================

// Data Memory
ram_8x512 ram_8x512_hi_0 (
    .Address         (dmem_addr),
    .Clock           (clk_sys),
    .Data            (dmem_din[15:8]),
    .Q               (dmem_dout[15:8]),
    .ClockEn         (~dmem_cen),
    .Reset           (~reset_n),
    .WE              (~dmem_wen[1])
);
ram_8x512 ram_8x512_lo_0 (
    .Address         (dmem_addr),
    .Clock           (clk_sys),
    .Data            (dmem_din[7:0]),
    .Q               (dmem_dout[7:0]),
    .ClockEn         (~dmem_cen),
    .Reset           (~reset_n),
    .WE              (~dmem_wen[0])
);


// Program Memory
ram_8x2k rom_8x2k_hi_0 (
    .Address         (pmem_addr),
    .Clock           (clk_sys),
    .Data            (pmem_din[15:8]),
    .Q               (pmem_dout[15:8]),
    .ClockEn         (~pmem_cen),
    .Reset           (~reset_n),
    .WE              (~pmem_wen[1])
);

ram_8x2k rom_8x2k_lo_0 (
    .Address         (pmem_addr),
    .Clock           (clk_sys),
    .Data            (pmem_din[7:0]),
    .Q               (pmem_dout[7:0]),
    .ClockEn         (~pmem_cen),
    .Reset           (~reset_n),
    .WE              (~pmem_wen[0])
);



//=============================================================================
// 7)  I/O CELLS
//=============================================================================


// Slide Switches (Port 1 inputs)
//--------------------------------
`ifndef min_hw 
assign p3_din[0] = SW0 ;
assign p3_din[1] = SW1 ;
assign p3_din[2] = SW2 ;
assign p3_din[3] = SW3 ;
assign p3_din[4] = SW4 ;
assign p3_din[5] = SW5 ;
assign p3_din[6] = SW6 ;
assign p3_din[7] = SW7 ;
`endif
// LEDs (Port 1 outputs)
//-----------------------
assign LED0 = p3_dout[0] & p3_dout_en[0] ;
assign LED1 = p3_dout[1] & p3_dout_en[1] ;
assign LED2 = p3_dout[2] & p3_dout_en[2] ;
assign LED3 = p3_dout[3] & p3_dout_en[3] ;
assign LED4 = p3_dout[4] & p3_dout_en[4] ;
assign LED5 = p3_dout[5] & p3_dout_en[5] ;
assign LED6 = p3_dout[6] & p3_dout_en[6] ;
assign LED7 = p3_dout[7] & p3_dout_en[7] ;

// Four-Sigit, Seven-Segment LED Display
//---------------------------------------
`ifndef min_hw 
assign SEG_A = seg_a_;
assign SEG_B = seg_b_;
assign SEG_C = seg_c_;
assign SEG_D = seg_d_;
assign SEG_E = seg_e_;
assign SEG_F = seg_f_;
assign SEG_G = seg_g_;
assign SEG_DP= seg_dp_;
assign SEG_AN0 = seg_an0_;
assign SEG_AN1 = seg_an1_;
assign SEG_AN2 = seg_an2_;
assign SEG_AN3 = seg_an3_;
assign hw_uart_rxd = UART_RXD_A ;
assign UART_TXD_A = hw_uart_txd ;
`endif
// RS-232 Port
//----------------------
// P1.1 (TX) and P2.2 (RX)
assign p1_io_din      = 8'h00;
assign p2_io_din[7:3] = 5'h00;
assign p2_io_din[1:0] = 2'h0;

assign dbg_uart_rxd = UART_RXD;
assign UART_TXD =  dbg_uart_txd ;
   
endmodule // openMSP430_fpga
