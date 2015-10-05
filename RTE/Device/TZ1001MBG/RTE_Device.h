/**
 * @file RTE_Device.h
 * @brief RTE Device Configuration for Toshiba TZ10xx
 * @date $Date:: 2015-06-17 16:12:02 +0900 #$
 * @note
 */

/*
 * COPYRIGHT (C) 2014
 * TOSHIBA CORPORATION SEMICONDUCTOR & STORAGE PRODUCTS COMPANY
 * ALL RIGHTS RESERVED
 *
 * THE SOURCE CODE AND ITS RELATED DOCUMENTATION IS PROVIDED "AS IS". TOSHIBA
 * CORPORATION MAKES NO OTHER WARRANTY OF ANY KIND, WHETHER EXPRESS, IMPLIED OR,
 * STATUTORY AND DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * SATISFACTORY QUALITY, NON INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * THE SOURCE CODE AND DOCUMENTATION MAY INCLUDE ERRORS. TOSHIBA CORPORATION
 * RESERVES THE RIGHT TO INCORPORATE MODIFICATIONS TO THE SOURCE CODE IN LATER
 * REVISIONS OF IT, AND TO MAKE IMPROVEMENTS OR CHANGES IN THE DOCUMENTATION OR
 * THE PRODUCTS OR TECHNOLOGIES DESCRIBED THEREIN AT ANY TIME.
 * 
 * TOSHIBA CORPORATION SHALL NOT BE LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGE OR LIABILITY ARISING FROM YOUR USE OF THE SOURCE CODE OR
 * ANY DOCUMENTATION, INCLUDING BUT NOT LIMITED TO, LOST REVENUES, DATA OR
 * PROFITS, DAMAGES OF ANY SPECIAL, INCIDENTAL OR CONSEQUENTIAL NATURE, PUNITIVE
 * DAMAGES, LOSS OF PROPERTY OR LOSS OF PROFITS ARISING OUT OF OR IN CONNECTION
 * WITH THIS AGREEMENT, OR BEING UNUSABLE, EVEN IF ADVISED OF THE POSSIBILITY OR
 * PROBABILITY OF SUCH DAMAGES AND WHETHER A CLAIM FOR SUCH DAMAGE IS BASED UPON
 * WARRANTY, CONTRACT, TORT, NEGLIGENCE OR OTHERWISE.
 */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

// <o> Enable #51 erratum workaround. <0=>Disabled <1=>Enabled
// <i> When BOOTMODE3 is set to 0(Internal LDO supply the power), this workaround should be enabled.
#define RTE_WORKAROUND_51 0

// <e> GPIO
// <i> Configuration settings for Driver_GPIO
#define RTE_GPIO 1
//  <o0>  GPIO_0 Pin  <0=>(disabled) <1=>MCU_GPIO_0
//  <o1>  GPIO_1 Pin  <0=>(disabled) <1=>MCU_GPIO_1
//  <o2>  GPIO_2 Pin  <0=>(disabled) <1=>MCU_GPIO_2
//  <o3>  GPIO_3 Pin  <0=>(disabled) <1=>MCU_GPIO_3
//  <o4>  GPIO_4 Pin  <0=>(disabled) <1=>MCU_GPIO_4
//  <o5>  GPIO_5 Pin  <0=>(disabled) <1=>MCU_GPIO_5
//  <o6>  GPIO_6 Pin  <0=>(disabled) <1=>MCU_GPIO_6
//  <o7>  GPIO_7 Pin  <0=>(disabled) <1=>MCU_GPIO_7
//  <o8>  GPIO_8 Pin  <0=>(disabled) <1=>MCU_GPIO_8    <2=>MCU_ADC24_SYNC
//  <o9>  GPIO_9 Pin  <0=>(disabled) <1=>MCU_GPIO_9
//  <o10> GPIO_10 Pin <0=>(disabled) <1=>MCU_GPIO_10   <2=>MCU_UA2_RTS_N
//  <o11> GPIO_11 Pin <0=>(disabled) <1=>MCU_GPIO_11   <2=>MCU_UA2_CTS_N
//  <o12> GPIO_12 Pin <0=>(disabled) <1=>MCU_GPIO_12   <2=>MCU_UA1_RXD
//  <o13> GPIO_13 Pin <0=>(disabled) <1=>MCU_GPIO_13   <2=>MCU_UA1_TXD
//  <o14> GPIO_14 Pin <0=>(disabled) <1=>MCU_GPIO_14   <2=>MCU_I2C1_DATA
//  <o15> GPIO_15 Pin <0=>(disabled) <1=>MCU_GPIO_15   <2=>MCU_I2C1_CLK
//  <o16> GPIO_16 Pin <0=>(disabled) <1=>MCU_I2C0_DATA <2=>MCU_SPIM0_CS_N
//  <o17> GPIO_17 Pin <0=>(disabled) <1=>MCU_I2C0_CLK  <2=>MCU_SPIM0_CLK
//  <o18> GPIO_18 Pin <0=>(disabled) <1=>MCU_UA2_RXD   <2=>MCU_SPIM0_MOSI
//  <o19> GPIO_19 Pin <0=>(disabled) <1=>MCU_UA2_TXD   <2=>MCU_SPIM0_MISO
//  <o20> GPIO_20 Pin <0=>(disabled) <1=>MCU_UA0_RXD   <2=>MCU_SPIM1_CS_N
//  <o21> GPIO_21 Pin <0=>(disabled) <1=>MCU_UA0_TXD   <2=>MCU_SPIM1_CLK
//  <o22> GPIO_22 Pin <0=>(disabled) <1=>MCU_UA1_RTS_N <2=>MCU_SPIM1_MOSI
//  <o23> GPIO_23 Pin <0=>(disabled) <1=>MCU_UA1_CTS_N <2=>MCU_SPIM1_MISO
//  <o24> GPIO_24 Pin <0=>(disabled) <1=>MCU_GPIO_24
//  <o25> GPIO_25 Pin <0=>(disabled) <1=>MCU_GPIO_25
//  <o26> GPIO_26 Pin <0=>(disabled) <1=>MCU_GPIO_26
//  <o27> GPIO_27 Pin <0=>(disabled) <1=>MCU_GPIO_27
//  <o28> GPIO_28 Pin <0=>(disabled) <1=>MCU_GPIO_28
//  <o29> GPIO_29 Pin <0=>(disabled) <1=>MCU_GPIO_29
//  <o30> GPIO_30 Pin <0=>(disabled) <1=>MCU_GPIO_30
//  <o31> GPIO_31 Pin <0=>(disabled) <1=>MCU_GPIO_31
#if RTE_GPIO
#define RTE_GPIO_0_ID 1
#define RTE_GPIO_1_ID 1
#define RTE_GPIO_2_ID 1
#define RTE_GPIO_3_ID 1
#define RTE_GPIO_4_ID 1
#define RTE_GPIO_5_ID 1
#define RTE_GPIO_6_ID 1
#define RTE_GPIO_7_ID 1
#define RTE_GPIO_8_ID 0
#define RTE_GPIO_9_ID 0
#define RTE_GPIO_10_ID 1
#define RTE_GPIO_11_ID 1
#define RTE_GPIO_12_ID 0
#define RTE_GPIO_13_ID 0
#define RTE_GPIO_14_ID 0
#define RTE_GPIO_15_ID 0
#define RTE_GPIO_16_ID 2
#define RTE_GPIO_17_ID 2
#define RTE_GPIO_18_ID 2
#define RTE_GPIO_19_ID 2
#define RTE_GPIO_20_ID 2
#define RTE_GPIO_21_ID 2
#define RTE_GPIO_22_ID 2
#define RTE_GPIO_23_ID 2
#define RTE_GPIO_24_ID 0
#define RTE_GPIO_25_ID 0
#define RTE_GPIO_26_ID 0
#define RTE_GPIO_27_ID 0
#define RTE_GPIO_28_ID 1
#define RTE_GPIO_29_ID 1
#define RTE_GPIO_30_ID 1
#define RTE_GPIO_31_ID 1
#else
#define RTE_GPIO_0_ID 0
#define RTE_GPIO_1_ID 0
#define RTE_GPIO_2_ID 0
#define RTE_GPIO_3_ID 0
#define RTE_GPIO_4_ID 0
#define RTE_GPIO_5_ID 0
#define RTE_GPIO_6_ID 0
#define RTE_GPIO_7_ID 0
#define RTE_GPIO_8_ID 0
#define RTE_GPIO_9_ID 0
#define RTE_GPIO_10_ID 0
#define RTE_GPIO_11_ID 0
#define RTE_GPIO_12_ID 0
#define RTE_GPIO_13_ID 0
#define RTE_GPIO_14_ID 0
#define RTE_GPIO_15_ID 0
#define RTE_GPIO_16_ID 0
#define RTE_GPIO_17_ID 0
#define RTE_GPIO_18_ID 0
#define RTE_GPIO_19_ID 0
#define RTE_GPIO_20_ID 0
#define RTE_GPIO_21_ID 0
#define RTE_GPIO_22_ID 0
#define RTE_GPIO_23_ID 0
#define RTE_GPIO_24_ID 0
#define RTE_GPIO_25_ID 0
#define RTE_GPIO_26_ID 0
#define RTE_GPIO_27_ID 0
#define RTE_GPIO_28_ID 0
#define RTE_GPIO_29_ID 0
#define RTE_GPIO_30_ID 0
#define RTE_GPIO_31_ID 0
#endif
// </e>

// <e> SPI0 (SPI Master Controller)
// <i> Configuration settings for Driver_SPI0 in component ::Drivers:SPI
//  <o1> SPIM0_CS_N pin <1=>MCU_SPIM0_CS_N
//  <o2> SPIM0_CLK pin  <1=>MCU_SPIM0_CLK
//  <o3> SPIM0_MOSI pin <1=>MCU_SPIM0_MOSI
//  <o4> SPIM0_MISO pin <1=>MCU_SPIM0_MISO
//  <o5> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o6> Internal resistor for MOSI and MISO <0=>Pulldown <1=>None <2=>Pullup
//  <e7> DMA Rx
//   <o8> DMA Channel <0-7>
//   <o9> Handshake Channel <0-7>
//  </e>
//  <e10> DMA Tx
//   <o11> DMA Channel <0-7>
//   <o12> Handshake Channel <0-7>
//  </e>
#define RTE_SPI0 0
#if RTE_SPI0
#define RTE_SPIM0_CS_N_ID 1
#define RTE_SPIM0_CLK_ID 1
#define RTE_SPIM0_MOSI_ID 1
#define RTE_SPIM0_MISO_ID 1
#define RTE_SPIM0_DRIVE_CAPABILITY 0
#define RTE_SPIM0_RESISTOR         1
#define RTE_SPIM0_DMA_RX 0
#define RTE_SPIM0_DMA_RX_CH 0
#define RTE_SPIM0_DMA_RX_HS 0
#define RTE_SPIM0_DMA_TX 0
#define RTE_SPIM0_DMA_TX_CH 0
#define RTE_SPIM0_DMA_TX_HS 0
#else
#define RTE_SPIM0_CS_N_ID 0
#define RTE_SPIM0_CLK_ID 0
#define RTE_SPIM0_MOSI_ID 0
#define RTE_SPIM0_MISO_ID 0
#define RTE_SPIM0_DRIVE_CAPABILITY 0
#define RTE_SPIM0_RESISTOR         1
#define RTE_SPIM0_DMA_RX 0
#define RTE_SPIM0_DMA_RX_CH 0
#define RTE_SPIM0_DMA_RX_HS 0
#define RTE_SPIM0_DMA_TX 0
#define RTE_SPIM0_DMA_TX_CH 0
#define RTE_SPIM0_DMA_TX_HS 0
#endif
// </e>

// <e> SPI1 (SPI Master Controller)
// <i> Configuration settings for Driver_SPI1 in component ::Drivers:SPI
//  <o1> SPIM1_CS_N pin <1=>MCU_SPIM1_CS_N
//  <o2> SPIM1_CLK pin  <1=>MCU_SPIM1_CLK
//  <o3> SPIM1_MOSI pin <1=>MCU_SPIM1_MOSI
//  <o4> SPIM1_MISO pin <1=>MCU_SPIM1_MISO
//  <o5> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o6> Internal resistor for MOSI and MISO <0=>Pulldown <1=>None <2=>Pullup
//  <e7> DMA Rx
//   <o8> DMA Channel <0-7>
//   <o9> Handshake Channel <0-7>
//  </e>
//  <e10> DMA Tx
//   <o11> DMA Channel <0-7>
//   <o12> Handshake Channel <0-7>
//  </e>
#define RTE_SPI1 0
#if RTE_SPI1
#define RTE_SPIM1_CS_N_ID 1
#define RTE_SPIM1_CLK_ID 1
#define RTE_SPIM1_MOSI_ID 1
#define RTE_SPIM1_MISO_ID 1
#define RTE_SPIM1_DRIVE_CAPABILITY 0
#define RTE_SPIM1_RESISTOR         1
#define RTE_SPIM1_DMA_RX 0
#define RTE_SPIM1_DMA_RX_CH 0
#define RTE_SPIM1_DMA_RX_HS 0
#define RTE_SPIM1_DMA_TX 0
#define RTE_SPIM1_DMA_TX_CH 0
#define RTE_SPIM1_DMA_TX_HS 0
#else
#define RTE_SPIM1_CS_N_ID 0
#define RTE_SPIM1_CLK_ID 0
#define RTE_SPIM1_MOSI_ID 0
#define RTE_SPIM1_MISO_ID 0
#define RTE_SPIM1_DRIVE_CAPABILITY 0
#define RTE_SPIM1_RESISTOR         1
#define RTE_SPIM1_DMA_RX 0
#define RTE_SPIM1_DMA_RX_CH 0
#define RTE_SPIM1_DMA_RX_HS 0
#define RTE_SPIM1_DMA_TX 0
#define RTE_SPIM1_DMA_TX_CH 0
#define RTE_SPIM1_DMA_TX_HS 0
#endif
// </e>

// <e> SPI2 (SPI Master Controller)
// <i> Configuration settings for Driver_SPI2 in component ::Drivers:SPI
//  <o1> SPIM2_CS_N pin <1=>MCU_SPIM2_CS_N
//  <o2> SPIM2_CLK pin  <1=>MCU_SPIM2_CLK
//  <o3> SPIM2_MOSI pin <1=>MCU_SPIM2_MOSI
//  <o4> SPIM2_MISO pin <1=>MCU_SPIM2_MISO
//  <o5> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o6> Internal resistor for MOSI and MISO <0=>Pulldown <1=>None <2=>Pullup
//  <e7> DMA Rx
//   <o8> DMA Channel <0-7>
//   <o9> Handshake Channel <0-7>
//  </e>
#define RTE_SPI2 0
#if RTE_SPI2
#define RTE_SPIM2_CS_N_ID 1
#define RTE_SPIM2_CLK_ID 1
#define RTE_SPIM2_MOSI_ID 1
#define RTE_SPIM2_MISO_ID 1
#define RTE_SPIM2_DRIVE_CAPABILITY 0
#define RTE_SPIM2_RESISTOR         0
#define RTE_SPIM2_DMA_RX 0
#define RTE_SPIM2_DMA_RX_CH 0
#define RTE_SPIM2_DMA_RX_HS 0
#else
#define RTE_SPIM2_CS_N_ID 0
#define RTE_SPIM2_CLK_ID 0
#define RTE_SPIM2_MOSI_ID 0
#define RTE_SPIM2_MISO_ID 0
#define RTE_SPIM2_DRIVE_CAPABILITY 0
#define RTE_SPIM2_RESISTOR         1
#define RTE_SPIM2_DMA_RX 0
#define RTE_SPIM2_DMA_RX_CH 0
#define RTE_SPIM2_DMA_RX_HS 0
#endif
// </e>

// <e> SPI3 (SPI Master Controller)
// <i> Configuration settings for Driver_SPI3 in component ::Drivers:SPI
//  <o1> SPIM3_CS_N pin <1=>MCU_SPIM3_CS_N <2=>MCU_GPIO_12
//  <o2> SPIM3_CLK pin  <1=>MCU_SPIM3_CLK  <2=>MCU_GPIO_13
//  <o3> SPIM3_MOSI pin <1=>MCU_SPIM3_MOSI <2=>MCU_GPIO_14
//  <o4> SPIM3_MISO pin <1=>MCU_SPIM3_MISO <2=>MCU_GPIO_15
//  <o5> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o6> Internal resistor for MOSI and MISO <0=>Pulldown <1=>None <2=>Pullup
//  <e7> DMA Rx
//   <o8> DMA Channel <0-7>
//   <o9> Handshake Channel <0-7>
//  </e>
#define RTE_SPI3 1
#if RTE_SPI3
#define RTE_SPIM3_CS_N_ID 2
#define RTE_SPIM3_CLK_ID 2
#define RTE_SPIM3_MOSI_ID 2
#define RTE_SPIM3_MISO_ID 2
#define RTE_SPIM3_DRIVE_CAPABILITY 0
#define RTE_SPIM3_RESISTOR         0
#define RTE_SPIM3_DMA_RX 0
#define RTE_SPIM3_DMA_RX_CH 0
#define RTE_SPIM3_DMA_RX_HS 0
#else
#define RTE_SPIM3_CS_N_ID 0
#define RTE_SPIM3_CLK_ID 0
#define RTE_SPIM3_MOSI_ID 0
#define RTE_SPIM3_MISO_ID 0
#define RTE_SPIM3_DRIVE_CAPABILITY 0
#define RTE_SPIM3_RESISTOR         1
#define RTE_SPIM3_DMA_RX 0
#define RTE_SPIM3_DMA_RX_CH 0
#define RTE_SPIM3_DMA_RX_HS 0
#endif
// </e>

// <e> I2C0
// <i> Configuration settings for Driver_I2C0 in component ::Drivers:I2C
//  <o1> SDA pin <1=>MCU_I2C0_DATA <2=>MCU_UA1_TXD
//  <o2> SCL pin <1=>MCU_I2C0_CLK  <2=>MCU_UA1_RXD
//  <o3> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o4> Internal resistor for SDA and SCL <1=>None <2=>Pullup 
//  <e5> DMA Rx
//   <o6> DMA Channel <0-7>
//   <o7> Handshake Channel <0-7>
//  </e>
#define RTE_I2C0 0
#if RTE_I2C0
#define RTE_I2C0_DATA_ID 1
#define RTE_I2C0_CLK_ID 1
#define RTE_I2C0_DRIVE_CAPABILITY 0
#define RTE_I2C0_RESISTOR 1
#define RTE_I2C0_DMA_RX 0
#define RTE_I2C0_DMA_RX_CH 0
#define RTE_I2C0_DMA_RX_HS 0
#else
#define RTE_I2C0_DATA_ID 0
#define RTE_I2C0_CLK_ID 0
#define RTE_I2C0_DRIVE_CAPABILITY 0
#define RTE_I2C0_RESISTOR 1
#define RTE_I2C0_DMA_RX 0
#define RTE_I2C0_DMA_RX_CH 0
#define RTE_I2C0_DMA_RX_HS 0
#endif
// </e>

// <e> I2C1
// <i> Configuration settings for Driver_I2C1 in component ::Drivers:I2C
//  <o1> SDA pin <1=>MCU_I2C1_DATA
//  <o2> SCL pin <1=>MCU_I2C1_CLK
//  <o3> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o4> Internal resistor for SDA and SCL <1=>None <2=>Pullup
//  <e5> DMA Rx
//   <o6> DMA Channel <0-7>
//   <o7> Handshake Channel <0-7>
//  </e>
#define RTE_I2C1 1
#if RTE_I2C1
#define RTE_I2C1_DATA_ID 1
#define RTE_I2C1_CLK_ID 1
#define RTE_I2C1_DRIVE_CAPABILITY 0
#define RTE_I2C1_RESISTOR 1
#define RTE_I2C1_DMA_RX 0
#define RTE_I2C1_DMA_RX_CH 0
#define RTE_I2C1_DMA_RX_HS 0
#else
#define RTE_I2C1_DATA_ID 0
#define RTE_I2C1_CLK_ID 0
#define RTE_I2C1_DRIVE_CAPABILITY 0
#define RTE_I2C1_RESISTOR 1
#define RTE_I2C1_DMA_RX 0
#define RTE_I2C1_DMA_RX_CH 0
#define RTE_I2C1_DMA_RX_HS 0
#endif
// </e>

// <e> I2C2
// <i> Configuration settings for Driver_I2C2 in component ::Drivers:I2C
//  <o1> SDA pin <1=>MCU_I2C2_DATA <2=>MCU_UA1_RTS_N
//  <o2> SCL pin <1=>MCU_I2C2_CLK  <2=>MCU_UA1_CTS_N
//  <o3> Output drive capability <0=>2mA <1=>4mA <2=>5mA <3=>7mA
//  <o4> Internal resistor for SDA and SCL, when MCU_UA1_RTS_N and MCU_UA1_CTS_N are used. <1=>None <2=>Pullup
//  <e5> DMA Rx
//   <o6> DMA Channel <0-7>
//   <o7> Handshake Channel <0-7>
//  </e>
#define RTE_I2C2 1
#if RTE_I2C2
#define RTE_I2C2_DATA_ID 2
#define RTE_I2C2_CLK_ID 2
#define RTE_I2C2_DRIVE_CAPABILITY 0
#define RTE_I2C2_RESISTOR 1
#define RTE_I2C2_DMA_RX 0
#define RTE_I2C2_DMA_RX_CH 0
#define RTE_I2C2_DMA_RX_HS 0
#else
#define RTE_I2C2_DATA_ID 0
#define RTE_I2C2_CLK_ID 0
#define RTE_I2C2_DRIVE_CAPABILITY 0
#define RTE_I2C2_RESISTOR 1
#define RTE_I2C2_DMA_RX 0
#define RTE_I2C2_DMA_RX_CH 0
#define RTE_I2C2_DMA_RX_HS 0
#endif
// </e>

// <e> UART0
// <i> Configuration settings for Driver_UART0 in component ::Drivers:UART
//  <o1> UA0_RXD pin <1=>MCU_UA0_RXD <2=>MCU_SPIM1_CS_N
//  <o2> UA0_TXD pin <1=>MCU_UA0_TXD <2=>MCU_SPIM1_CLK
#define RTE_UART0 0
#if RTE_UART0
#define RTE_UA0_RXD_ID 1
#define RTE_UA0_TXD_ID 1
#else
#define RTE_UA0_RXD_ID 0
#define RTE_UA0_TXD_ID 0
#endif
//  <e> Hardware flow control
//   <o1> UA0_RTS_N pin <1=>MCU_GPIO_14 <2=>MCU_I2C1_DATA
//   <o2> UA0_CTS_N pin <1=>MCU_GPIO_15 <2=>MCU_I2C1_CLK
//  </e>
#define RTE_UART0_HW_FLOW 0
#if RTE_UART0 && RTE_UART0_HW_FLOW
#define RTE_UA0_RTS_N_ID 1
#define RTE_UA0_CTS_N_ID 1
#else
#define RTE_UA0_RTS_N_ID 0
#define RTE_UA0_CTS_N_ID 0
#endif
//  <e> DMA Rx
//   <o1> Channel <0-7>
//  </e>
#define RTE_UA0_DMA_RX 0
#define RTE_UA0_DMA_RX_CH 0
//  <e> DMA Tx
//   <o1> Channel <0-7>
//  </e>
#define RTE_UA0_DMA_TX 0
#define RTE_UA0_DMA_TX_CH 0
// </e>

// <e> UART1
// <i> Configuration settings for Driver_UART1 in component ::Drivers:UART
//  <o1> UA1_RXD pin <1=>MCU_UA1_RXD
//  <o2> UA1_TXD pin <1=>MCU_UA1_TXD
#define RTE_UART1 1
#if RTE_UART1
#define RTE_UA1_RXD_ID 1
#define RTE_UA1_TXD_ID 1
#else
#define RTE_UA1_RXD_ID 0
#define RTE_UA1_TXD_ID 0
#endif
//  <e> Hardware flow control
//   <o1> UA1_RTS_N pin <1=>MCU_UA1_RTS_N
//   <o2> UA1_CTS_N pin <1=>MCU_UA1_CTS_N
//  </e>
#define RTE_UART1_HW_FLOW 0
#if RTE_UART1 && RTE_UART1_HW_FLOW
#define RTE_UA1_RTS_N_ID 1
#define RTE_UA1_CTS_N_ID 1
#else
#define RTE_UA1_RTS_N_ID 0
#define RTE_UA1_CTS_N_ID 0
#endif
//  <e> DMA Rx
//   <o1> Channel <0-7>
//  </e>
#define RTE_UA1_DMA_RX 0
#define RTE_UA1_DMA_RX_CH 0
//  <e> DMA Tx
//   <o1> Channel <0-7>
//  </e>
#define RTE_UA1_DMA_TX 0
#define RTE_UA1_DMA_TX_CH 0
// </e>

// <e> UART2
// <i> Configuration settings for Driver_UART2 in component ::Drivers:UART
//  <o1> UA2_RXD pin <1=>MCU_UA2_RXD
//  <o2> UA2_TXD pin <1=>MCU_UA2_TXD
#define RTE_UART2 1
#if RTE_UART2
#define RTE_UA2_RXD_ID 1
#define RTE_UA2_TXD_ID 1
#else
#define RTE_UA2_RXD_ID 0
#define RTE_UA2_TXD_ID 0
#endif
//  <e> Hardware flow control
//   <o1> UA2_RTS_N pin <1=>MCU_UA2_RTS_N
//   <o2> UA2_CTS_N pin <1=>MCU_UA2_CTS_N
//  </e>
#define RTE_UART2_HW_FLOW 1
#if RTE_UART2 && RTE_UART2_HW_FLOW
#define RTE_UA2_RTS_N_ID 1
#define RTE_UA2_CTS_N_ID 1
#else
#define RTE_UA2_RTS_N_ID 0
#define RTE_UA2_CTS_N_ID 0
#endif
//  <e> DMA Rx
//   <o1> Channel <0-7>
//  </e>
#define RTE_UA2_DMA_RX 0
#define RTE_UA2_DMA_RX_CH 0
//  <e> DMA Tx
//   <o1> Channel <0-7>
//  </e>
#define RTE_UA2_DMA_TX 0
#define RTE_UA2_DMA_TX_CH 0
// </e>

// <e> USB2FS
// <i> Configuration settings for Driver_USBD0
#define RTE_USB2FS 0
#if RTE_USB2FS
//  <o0>  MaxPacketSize0  <8=>8 <16=>16 <32=>32 <64=>64
//  <o1>  Number of Endpoints <1=>1 <2=>2 <3=>3 <4=>4
//  <o2>  Debug enable <0=>0 <1=>1
#define RTE_USB2FS_MPS0 64
#define RTE_USB2FS_NUM_EPS 4
#define RTE_USB2FS_DEBUG 1
#endif // RTE_USB2FS
// </e>

// <e> ADCC12
#define RTE_ADCC12 0
// </e>

// <e> ADCC24
#define RTE_ADCC24 0
#if RTE_ADCC24
//  <o> ADC24_SYNC pin <0=>(disabled) <1=>MCU_ADC24_SYNC
#define RTE_ADC24_SYNC_ID 0
#else
#define RTE_ADC24_SYNC_ID 0
#endif
// </e>

// <e> TMR (Timer)
#define RTE_TMR 1
// </e>

// <e> ADVTMR (Advanced Timer)
// <i> Configuration settings for Driver_ADVTMR* in component ::Drivers:ADVTMR
#define RTE_ADVTMR 1
//  <e> PWM0
//   <o1> PWM0 pin <1=>MCU_GPIO_8 <2=>MCU_SPIM0_CS_N
//   <e2> Output Compare DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_PWM0 1
#if RTE_ADVTMR && RTE_ADVTMR_PWM0
#define RTE_PWM0_ID 1
#define RTE_PWM0_DMA 0
#define RTE_PWM0_DMA_CH 0
#else
#define RTE_PWM0_ID 0
#define RTE_PWM0_DMA 0
#define RTE_PWM0_DMA_CH 0
#endif
//  <e> PWM1
//   <o1> PWM1 pin <1=>MCU_GPIO_9 <2=>MCU_SPIM0_CLK
//   <e2> Output Compare DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_PWM1 1
#if RTE_ADVTMR && RTE_ADVTMR_PWM1
#define RTE_PWM1_ID 1
#define RTE_PWM1_DMA 0
#define RTE_PWM1_DMA_CH 0
#else
#define RTE_PWM1_ID 0
#define RTE_PWM1_DMA 0
#define RTE_PWM1_DMA_CH 0
#endif
//  <e> PWM2
//   <o1> PWM2 pin <1=>MCU_GPIO_10 <2=>MCU_SPIM0_MOSI
//   <e2> Output Compare DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_PWM2 0
#if RTE_ADVTMR && RTE_ADVTMR_PWM2
#define RTE_PWM2_ID 1
#define RTE_PWM2_DMA 0
#define RTE_PWM2_DMA_CH 0
#else
#define RTE_PWM2_ID 0
#define RTE_PWM2_DMA 0
#define RTE_PWM2_DMA_CH 0
#endif
//  <e> PWM3
//   <o1> PWM3 pin <1=>MCU_GPIO_11 <2=>MCU_GPIO_27
//   <e2> Output Compare DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_PWM3 0
#if RTE_ADVTMR && RTE_ADVTMR_PWM3
#define RTE_PWM3_ID 1
#define RTE_PWM3_DMA 0
#define RTE_PWM3_DMA_CH 0
#else
#define RTE_PWM3_ID 0
#define RTE_PWM3_DMA 0
#define RTE_PWM3_DMA_CH 0
#endif
//  <e> CAPTURE0
//   <o1> CAPTURE0 pin <1=>MCU_GPIO_12 <2=>MCU_UA0_RXD
//   <e2> Input Capture DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_CAPTURE0 0
#if RTE_ADVTMR && RTE_ADVTMR_CAPTURE0
#define RTE_CAPTURE0_ID 1
#define RTE_CAPTURE0_DMA 0
#define RTE_CAPTURE0_DMA_CH 0
#else
#define RTE_CAPTURE0_ID 0
#define RTE_CAPTURE0_DMA 0
#define RTE_CAPTURE0_DMA_CH 0
#endif
//  <e> CAPTURE1
//   <o1> CAPTURE1 pin <1=>MCU_GPIO_13 <2=>MCU_UA0_TXD
//   <e2> Input Capture DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_CAPTURE1 0
#if RTE_ADVTMR && RTE_ADVTMR_CAPTURE1
#define RTE_CAPTURE1_ID 1
#define RTE_CAPTURE1_DMA 0
#define RTE_CAPTURE1_DMA_CH 0
#else
#define RTE_CAPTURE1_ID 0
#define RTE_CAPTURE1_DMA 0
#define RTE_CAPTURE1_DMA_CH 0
#endif
//  <e> CAPTURE2
//   <o1> CAPTURE2 pin <1=>MCU_I2C0_DATA
//   <e2> Input Capture DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_CAPTURE2 0
#if RTE_ADVTMR && RTE_ADVTMR_CAPTURE2
#define RTE_CAPTURE2_ID 1
#define RTE_CAPTURE2_DMA 0
#define RTE_CAPTURE2_DMA_CH 0
#else
#define RTE_CAPTURE2_ID 0
#define RTE_CAPTURE2_DMA 0
#define RTE_CAPTURE2_DMA_CH 0
#endif
//  <e> CAPTURE3
//   <o1> CAPTURE3 pin <1=>MCU_I2C0_CLK
//   <e2> Input Capture DMA
//    <o3> Channel <0-7>
//   </e>
//  </e>
#define RTE_ADVTMR_CAPTURE3 0
#if RTE_ADVTMR && RTE_ADVTMR_CAPTURE3
#define RTE_CAPTURE3_ID 1
#define RTE_CAPTURE3_DMA 0
#define RTE_CAPTURE3_DMA_CH 0
#else
#define RTE_CAPTURE3_ID 0
#define RTE_CAPTURE3_DMA 0
#define RTE_CAPTURE3_DMA_CH 0
#endif
// </e>

// <e> RTC (Real Time Clock)
#define RTE_RTC 0
// </e>

// <e> WDT (Watch Dog Timer)
#define RTE_WDT 0
// </e>

// <e> AESA (AES Accelerator)
#define RTE_AESA 0
// </e>

// <e> RNG (Random Number Generator)
#define RTE_RNG 0
// </e>

// <e> NOR (internal flash)
//  <o1> SPIC_CS_N pin <0=>(disabled) <1=>MCU_SPIC_CS_N
//  <o2> SPIC_CLK pin  <0=>(disabled) <1=>MCU_SPIC_CLK
//  <o3> SPIC_MOSI pin <0=>(disabled) <1=>MCU_SPIC_MOSI
//  <o4> SPIC_MISO pin <0=>(disabled) <1=>MCU_SPIC_MISO
//  <o5> SPIC_IO2 pin  <0=>(disabled) <1=>MCU_SPIC_IO2
//  <o6> SPIC_IO3 pin  <0=>(disabled) <1=>MCU_SPIC_IO3
//  <e7> NOR DMA
//   <o8> Channel(8:assign a shared channel) <0-8>
//  </e>
#define RTE_NOR 0
#if RTE_NOR
#define RTE_SPIC_CS_N_ID 1
#define RTE_SPIC_CLK_ID 1
#define RTE_SPIC_MOSI_ID 1
#define RTE_SPIC_MISO_ID 1
#define RTE_SPIC_IO2_ID 1
#define RTE_SPIC_IO3_ID 1
#define RTE_SPIC_DMA 1
#define RTE_SPIC_DMA_CH 8
#else
#define RTE_SPIC_CS_N_ID 1
#define RTE_SPIC_CLK_ID 1
#define RTE_SPIC_MOSI_ID 1
#define RTE_SPIC_MISO_ID 1
#define RTE_SPIC_IO2_ID 1
#define RTE_SPIC_IO3_ID 1
#define RTE_SPIC_DMA 0
#define RTE_SPIC_DMA_CH 0
#endif
// </e>

// <e> SDMAC (DMA Controler)
//  <h> DMA channel shared setting
//   <o1.0> 0ch   <0=> not shared <1=> shared
//   <o1.1> 1ch   <0=> not shared <1=> shared
//   <o1.2> 2ch   <0=> not shared <1=> shared
//   <o1.3> 3ch   <0=> not shared <1=> shared
//   <o1.4> 4ch   <0=> not shared <1=> shared
//   <o1.5> 5ch   <0=> not shared <1=> shared
//   <o1.6> 6ch   <0=> not shared <1=> shared
//   <o1.7> 7ch   <0=> not shared <1=> shared
//  </h>
//  <h> HW Handshake channel shared setting
//   <o2.0> 0ch   <0=> not shared <1=> shared
//   <o2.1> 1ch   <0=> not shared <1=> shared
//   <o2.2> 2ch   <0=> not shared <1=> shared
//   <o2.3> 3ch   <0=> not shared <1=> shared
//   <o2.4> 4ch   <0=> not shared <1=> shared
//   <o2.5> 5ch   <0=> not shared <1=> shared
//   <o2.6> 6ch   <0=> not shared <1=> shared
//   <o2.7> 7ch   <0=> not shared <1=> shared
//  </h>
#define RTE_SDMAC 1
#if RTE_SDMAC
#define RTE_SDMAC_SHARED_DMA_CHANNEL       0xF0
#define RTE_SDMAC_SHARED_HANDSHAKE_CHANNEL 0xF0
#else
#define RTE_SDMAC_SHARED_DMA_CHANNEL       0x00
#define RTE_SDMAC_SHARED_HANDSHAKE_CHANNEL 0x00
#endif
// </e>

// <e> SRAMC
#define RTE_SRAMC 0
// </e>

// <e> CPU TRACE
//  <o1> TRACECLK pin     <1=>MCU_SPIM0_MISO
//  <o2> TRACEDATA[0] pin <1=>MCU_GPIO_8
//  <o3> TRACEDATA[1] pin <1=>MCU_GPIO_9
//  <o4> TRACEDATA[2] pin <1=>MCU_GPIO_10
//  <o5> TRACEDATA[3] pin <1=>MCU_GPIO_11
#define RTE_CPU_TRACE 0
#if RTE_CPU_TRACE 
#define RTE_TRACECLK_ID   1
#define RTE_TRACEDATA0_ID 1
#define RTE_TRACEDATA1_ID 1
#define RTE_TRACEDATA2_ID 1
#define RTE_TRACEDATA3_ID 1
#else
#define RTE_TRACECLK_ID   0
#define RTE_TRACEDATA0_ID 0
#define RTE_TRACEDATA1_ID 0
#define RTE_TRACEDATA2_ID 0
#define RTE_TRACEDATA3_ID 0
#endif
// </e>

// <e> BLE (Bluetooth Low Energy)
#define RTE_BLE 0
// </e>

// <e> ACCEL (internal accelerometer)
#define RTE_ACCEL 0
// </e>

// <e> GYRO (internal gyrometer)
#define RTE_GYRO 0
//  <e> connect external gyrometer
//   <o1> GPIO pin number for Gyro INT1 <0-24>
//   <o2> GPIO pin number for Gyro INT2 <0-24>
#define RTE_GYRO_EXTERNAL 0
#if RTE_GYRO_EXTERNAL
#define RTE_GYRO_INT1_PIN 6
#define RTE_GYRO_INT2_PIN 7
#else
#define RTE_GYRO_INT1_PIN 26
#define RTE_GYRO_INT2_PIN 27
#endif
//  </e>
// </e>

// <e> MAG (internal magnetometer)
#define RTE_MAG 0
// </e>

// <e> ACCEL_GYRO (internal accelerometer)
#define RTE_ACCEL_GYRO 0
// </e>

// <<< end of configuration section >>>

#endif  /* __RTE_DEVICE_H */
