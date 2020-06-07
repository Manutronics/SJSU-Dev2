/**************************************************************************//**
 * @file      LPC18xx.h
 * @brief     CMSIS Cortex-M3 Core Peripheral Access Layer Header File for
 *            NXP LPC18xx Device Series
 * @version:  V1.09
 * @date:     17. March 2010
 * @modified: August 9, 2019
 *
 * @note
 * Copyright (C) 2009 ARM Limited. All rights reserved.
 *
 * @par
 * ARM Limited (ARM) is supplying this software for use with Cortex-M
 * processor based microcontrollers.  This file can be freely distributed
 * within development tools that are supporting such ARM based processors.
 *
 * @par
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 ******************************************************************************/


#ifndef __LPC18xx_H__
#define __LPC18xx_H__
#pragma GCC system_header

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#ifdef __cplusplus
  #define     _I     volatile                /*!< defines 'read only' permissions      */
#else
  #define     _I     volatile const          /*!< defines 'read only' permissions      */
#endif
#define     _O     volatile                  /*!< defines 'write only' permissions     */
#define     _IO    volatile                  /*!< defines 'read / write' permissions   */
#include <stdint.h>

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT             1         /*!< MPU present or not                               */
#define __NVIC_PRIO_BITS          5         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    1         /*!< Set to 1 if different SysTick Config is used     */


/* Cortex-M3 processor and core peripherals           */
#include "L0_Platform/arm_cortex/m4/core_cm4.h"
// #include "system_LPC18xx.h"                 /* System Header */


#if defined (__cplusplus)
// SJSU-Dev2: Putting contents of this include in sjsu::lpc18xx
namespace sjsu::lpc18xx
{
extern "C" {
#endif
/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

enum IRQn
{
/******  Cortex-M3 Processor Exceptions Numbers ***************************************************/
     Reset_IRQn                = -15,      /*!< 1  Reset Vector, invoked on Power up and warm reset */
     NonMaskableInt_IRQn       = -14,      /*!< 2 Non Maskable Interrupt                         */
     MemoryManagement_IRQn     = -12,      /*!< 4 Cortex-M3 Memory Management Interrupt          */
     BusFault_IRQn             = -11,      /*!< 5 Cortex-M3 Bus Fault Interrupt                  */
     UsageFault_IRQn           = -10,      /*!< 6 Cortex-M3 Usage Fault Interrupt                */
     SVCall_IRQn               = -5,       /*!< 11 Cortex-M3 SV Call Interrupt                   */
     DebugMonitor_IRQn         = -4,       /*!< 12 Cortex-M3 Debug Monitor Interrupt             */
     PendSV_IRQn               = -2,       /*!< 14 Cortex-M3 Pend SV Interrupt                   */
     SysTick_IRQn              = -1,       /*!< 15 Cortex-M3 System Tick Interrupt               */

     /* ---------------------------  LPC18xx/43xx Specific Interrupt Numbers  ------------------------------- */
     DAC_IRQn                  =   0,/*!<   0  DAC                     */
     RESERVED0_IRQn            =   1,
     DMA_IRQn                  =   2,/*!<   2  DMA                     */
     RESERVED1_IRQn            =   3,/*!<   3  EZH/EDM                 */
     RESERVED2_IRQn            =   4,
     ENET_IRQn                 =   5,/*!<   5  ETHERNET                */
     SDIO_IRQn                 =   6,/*!<   6  SDIO                    */
     LCD_IRQn                  =   7,/*!<   7  LCD                     */
     USB0_IRQn                 =   8,/*!<   8  USB0                    */
     USB1_IRQn                 =   9,/*!<   9  USB1                    */
     SCT_IRQn                  =  10,/*!<  10  SCT                     */
     RIT_IRQn                  =  11,/*!<  11  RITIMER                 */
     TIMER0_IRQn               =  12,/*!<  12  TIMER0                  */
     TIMER1_IRQn               =  13,/*!<  13  TIMER1                  */
     TIMER2_IRQn               =  14,/*!<  14  TIMER2                  */
     TIMER3_IRQn               =  15,/*!<  15  TIMER3                  */
     MCPWM_IRQn                =  16,/*!<  16  MCPWM                   */
     ADC0_IRQn                 =  17,/*!<  17  ADC0                    */
     I2C0_IRQn                 =  18,/*!<  18  I2C0                    */
     I2C1_IRQn                 =  19,/*!<  19  I2C1                    */
     RESERVED3_IRQn            =  20,
     ADC1_IRQn                 =  21,/*!<  21  ADC1                    */
     SSP0_IRQn                 =  22,/*!<  22  SSP0                    */
     SSP1_IRQn                 =  23,/*!<  23  SSP1                    */
     UART0_IRQn                =  24,/*!<  24  UART0                   */
     UART1_IRQn                =  25,/*!<  25  UART1                   */
     UART2_IRQn                =  26,/*!<  26  UART2                   */
     UART3_IRQn                =  27,/*!<  27  UART3                   */
     I2S0_IRQn                 =  28,/*!<  28  I2S0                    */
     I2S1_IRQn                 =  29,/*!<  29  I2S1                    */
     RESERVED4_IRQn            =  30,
     RESERVED5_IRQn            =  31,
     EINT0_IRQn                =  32,/*!<  32  PIN_INT0                */
     EINT1_IRQn                =  33,/*!<  33  PIN_INT1                */
     EINT3_IRQn                =  35,/*!<  35  PIN_INT3                */
     EINT4_IRQn                =  36,/*!<  36  PIN_INT4                */
     EINT5_IRQn                =  37,/*!<  37  PIN_INT5                */
     EINT6_IRQn                =  38,/*!<  38  PIN_INT6                */
     EINT7_IRQn                =  39,/*!<  39  PIN_INT7                */
     GINT0_IRQn                =  40,/*!<  40  GINT0                   */
     GINT1_IRQn                =  41,/*!<  41  GINT1                   */
     EVENTROUTER_IRQn          =  42,/*!<  42  EVENTROUTER             */
     CAN1_IRQn                 =  43,/*!<  43  C_CAN1                  */
     RESERVED6_IRQn            =  44,
     RESERVED7_IRQn            =  45,/*!<                              */
     ATIMER_IRQn               =  46,/*!<  46  ATIMER                  */
     RTC_IRQn                  =  47,/*!<  47  RTC                     */
     RESERVED8_IRQn            =  48,
     WDT_IRQn                  =  49,/*!<  49  WWDT                    */
     RESERVED9_IRQn            =  50,
     CAN0_IRQn                 =  51,/*!<  51  C_CAN0                  */
     QEI_IRQn                  =  52,/*!<  52  QEI                     */
     kNumberOfIrqs
};
  
  /******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/*------------- System Control (SC) ------------------------------------------*/
typedef struct
{
     _IO uint32_t SFSP[16][32];
     _I  uint32_t RESERVED0[256];
     _IO uint32_t SFSCLK[4];         /*!< Pin configuration register for pins CLK0-3 */
     _I  uint32_t RESERVED16[28];
     _IO uint32_t SFSUSB;            /*!< Pin configuration register for USB */
     _IO uint32_t SFSI2C0;           /*!< Pin configuration register for I2C0-bus pins */
     _IO uint32_t ENAIO[3];          /*!< Analog function select registerS */
     _I  uint32_t RESERVED17[27];
     _IO uint32_t EMCDELAYCLK;       /*!< EMC clock delay register */
     _I  uint32_t RESERVED18[63];
     _IO uint32_t PINTSEL[2];        /*!< Pin interrupt select register for pin int 0 to 3 index 0, 4 to 7 index 1. */
} LPC_SC_TypeDef;

/*------------- Pin Connect Block (PINCON) -----------------------------------*/
typedef struct
{
     _IO uint32_t PINSEL0;
     _IO uint32_t PINSEL1;
     _IO uint32_t PINSEL2;
     _IO uint32_t PINSEL3;
     _IO uint32_t PINSEL4;
     _IO uint32_t PINSEL5;
     _IO uint32_t PINSEL6;
     _IO uint32_t PINSEL7;
     _IO uint32_t PINSEL8;
     _IO uint32_t PINSEL9;
     _IO uint32_t PINSEL10;
         uint32_t RESERVED0[5];
     _IO uint32_t PINMODE0;
     _IO uint32_t PINMODE1;
     _IO uint32_t PINMODE2;
     _IO uint32_t PINMODE3;
     _IO uint32_t PINMODE4;
     _IO uint32_t PINMODE5;
     _IO uint32_t PINMODE6;
     _IO uint32_t PINMODE7;
     _IO uint32_t PINMODE8;
     _IO uint32_t PINMODE9;
     _IO uint32_t PINMODE_OD0;
     _IO uint32_t PINMODE_OD1;
     _IO uint32_t PINMODE_OD2;
     _IO uint32_t PINMODE_OD3;
     _IO uint32_t PINMODE_OD4;
     _IO uint32_t I2CPADCFG;
} LPC_PINCON_TypeDef;

/*------------- General Purpose Input/Output (GPIO) --------------------------*/
typedef struct
{                    /*!< GPIO_PORT Structure */
     _IO uint8_t B[128][32];          /*!< Offset 0x0000: Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31 */
     _IO uint32_t W[32][32];          /*!< Offset 0x1000: Word pin registers port 0 to n */
     _IO uint32_t DIR[32];	           /*!< Offset 0x2000: Direction registers port n */
     _IO uint32_t MASK[32];           /*!< Offset 0x2080: Mask register port n */
     _IO uint32_t PIN[32];	           /*!< Offset 0x2100: Portpin register port n */
     _IO uint32_t MPIN[32];           /*!< Offset 0x2180: Masked port register port n */
     _IO uint32_t SET[32];	           /*!< Offset 0x2200: Write: Set register for port n Read: output bits for port n */
     _O  uint32_t CLR[32];	           /*!< Offset 0x2280: Clear port n */
     _O  uint32_t NOT[32];	           /*!< Offset 0x2300: Toggle port n */
} LPC_GPIO_TypeDef;

typedef struct
{                    /*!< GPIO_GROUP_INTn Structure */
     _IO uint32_t  CTRL;           /*!< GPIO grouped interrupt control register */
     _I  uint32_t  RESERVED0[7];
     _IO uint32_t  PORT_POL[8];    /*!< GPIO grouped interrupt port polarity register */
     _IO uint32_t  PORT_ENA[8];    /*!< GPIO grouped interrupt port m enable register */
     uint32_t       RESERVED1[1000];
} LPC_GPIOGROUPINT_TypeDef;

typedef struct
{                    /*!< PIN_INT Structure */
     _IO uint32_t ISEL;               /*!< Pin Interrupt Mode register */
     _IO uint32_t IENR;               /*!< Pin Interrupt Enable (Rising) register */
     _IO uint32_t SIENR;              /*!< Set Pin Interrupt Enable (Rising) register */
     _IO uint32_t CIENR;              /*!< Clear Pin Interrupt Enable (Rising) register */
     _IO uint32_t IENF;               /*!< Pin Interrupt Enable Falling Edge / Active Level register */
     _IO uint32_t SIENF;              /*!< Set Pin Interrupt Enable Falling Edge / Active Level register */
     _IO uint32_t CIENF;              /*!< Clear Pin Interrupt Enable Falling Edge / Active Level address */
     _IO uint32_t RISE;               /*!< Pin Interrupt Rising Edge register */
     _IO uint32_t FALL;               /*!< Pin Interrupt Falling Edge register */
     _IO uint32_t IST;                /*!< Pin Interrupt Status register */
} LPC_GPIOINT_TypeDef;

/*------------- Timer (TIM) --------------------------------------------------*/
typedef struct
{                    /*!< TIMERn Structure       */
     _IO uint32_t IR;                 /*!< Interrupt Register. The IR can be written to clear interrupts. The IR can be read to identify which of eight possible interrupt sources are pending. */
     _IO uint32_t TCR;                /*!< Timer Control Register. The TCR is used to control the Timer Counter functions. The Timer Counter can be disabled or reset through the TCR. */
     _IO uint32_t TC;                 /*!< Timer Counter. The 32 bit TC is incremented every PR+1 cycles of PCLK. The TC is controlled through the TCR. */
     _IO uint32_t PR;                 /*!< Prescale Register. The Prescale Counter (below) is equal to this value, the next clock increments the TC and clears the PC. */
     _IO uint32_t PC;                 /*!< Prescale Counter. The 32 bit PC is a counter which is incremented to the value stored in PR. When the value in PR is reached, the TC is incremented and the PC is cleared. The PC is observable and controllable through the bus interface. */
     _IO uint32_t MCR;                /*!< Match Control Register. The MCR is used to control if an interrupt is generated and if the TC is reset when a Match occurs. */
     _IO uint32_t MR[4];              /*!< Match Register. MR can be enabled through the MCR to reset the TC, stop both the TC and PC, and/or generate an interrupt every time MR matches the TC. */
     _IO uint32_t CCR;                /*!< Capture Control Register. The CCR controls which edges of the capture inputs are used to load the Capture Registers and whether or not an interrupt is generated when a capture takes place. */
     _IO uint32_t CR[4];              /*!< Capture Register. CR is loaded with the value of TC when there is an event on the CAPn.0 input. */
     _IO uint32_t EMR;                /*!< External Match Register. The EMR controls the external match pins MATn.0-3 (MAT0.0-3 and MAT1.0-3 respectively). */
     _I  uint32_t RESERVED0[12];
     _IO uint32_t CTCR;               /*!< Count Control Register. The CTCR selects between Timer and Counter mode, and in Counter mode selects the signal and edge(s) for counting. */
} LPC_TIM_TypeDef;

/*------------- Motor Control Pulse-Width Modulation (MCPWM) -----------------*/
typedef struct
{                   /*!< MCPWM Structure        */
     _I  uint32_t CON;               /*!< PWM Control read address */
     _O  uint32_t CON_SET;           /*!< PWM Control set address */
     _O  uint32_t CON_CLR;           /*!< PWM Control clear address */
     _I  uint32_t CAPCON;            /*!< Capture Control read address */
     _O  uint32_t CAPCON_SET;        /*!< Capture Control set address */
     _O  uint32_t CAPCON_CLR;        /*!< Event Control clear address */
     _IO uint32_t TC[3];             /*!< Timer Counter register */
     _IO uint32_t LIM[3];            /*!< Limit register         */
     _IO uint32_t MAT[3];            /*!< Match register         */
     _IO uint32_t DT;                /*!< Dead time register     */
     _IO uint32_t CCP;               /*!< Communication Pattern register */
     _I  uint32_t CAP[3];            /*!< Capture register       */
     _I  uint32_t INTEN;             /*!< Interrupt Enable read address */
     _O  uint32_t INTEN_SET;         /*!< Interrupt Enable set address */
     _O  uint32_t INTEN_CLR;         /*!< Interrupt Enable clear address */
     _I  uint32_t CNTCON;            /*!< Count Control read address */
     _O  uint32_t CNTCON_SET;        /*!< Count Control set address */
     _O  uint32_t CNTCON_CLR;        /*!< Count Control clear address */
     _I  uint32_t INTF;              /*!< Interrupt flags read address */
     _O  uint32_t INTF_SET;          /*!< Interrupt flags set address */
     _O  uint32_t INTF_CLR;          /*!< Interrupt flags clear address */
     _O  uint32_t CAP_CLR;           /*!< Capture clear address  */
} LPC_MCPWM_TypeDef;

/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
typedef struct
{                        /*!< USARTn Structure       */
     union
     {
        _IO uint32_t  DLL;      /*!< Divisor Latch LSB. Least significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
        _O  uint32_t  THR;      /*!< Transmit Holding Register. The next character to be transmitted is written here (DLAB = 0). */
        _I  uint32_t  RBR;      /*!< Receiver Buffer Register. Contains the next received character to be read (DLAB = 0). */
     };   

     union
     {
        _IO uint32_t IER;       /*!< Interrupt Enable Register. Contains individual interrupt enable bits for the 7 potential UART interrupts (DLAB = 0). */
        _IO uint32_t DLM;       /*!< Divisor Latch MSB. Most significant byte of the baud rate divisor value. The full divisor is used to generate a baud rate from the fractional rate divider (DLAB = 1). */
     };   

     union
     {
        _O  uint32_t FCR;       /*!< FIFO Control Register. Controls UART FIFO usage and modes. */
        _I  uint32_t IIR;       /*!< Interrupt ID Register. Identifies which interrupt(s) are pending. */
     };   

     _IO uint32_t LCR;            /*!< Line Control Register. Contains controls for frame formatting and break generation. */
     _IO uint32_t MCR;            /*!< Modem Control Register. Only present on USART ports with full modem support. */
     _I  uint32_t LSR;            /*!< Line Status Register. Contains flags for transmit and receive status, including line errors. */
     _I  uint32_t MSR;            /*!< Modem Status Register. Only present on USART ports with full modem support. */
     _IO uint32_t SCR;            /*!< Scratch Pad Register. Eight-bit temporary storage for software. */
     _IO uint32_t ACR;            /*!< Auto-baud Control Register. Contains controls for the auto-baud feature. */
     _IO uint32_t ICR;            /*!< IrDA control register (not all UARTS) */
     _IO uint32_t FDR;            /*!< Fractional Divider Register. Generates a clock input for the baud rate divider. */
     _IO uint32_t OSR;            /*!< Oversampling Register. Controls the degree of oversampling during each bit time. Only on some UARTS. */
     _IO uint32_t TER1;           /*!< Transmit Enable Register. Turns off USART transmitter for use with software flow control. */
     _I  uint32_t RESERVED0[3];
     _IO uint32_t HDEN;            /*!< Half-duplex enable Register- only on some UARTs */
     _I  uint32_t RESERVED1[1];
     _IO uint32_t SCICTRL;        /*!< Smart card interface control register- only on some UARTs */     
     _IO uint32_t RS485CTRL;      /*!< RS-485/EIA-485 Control. Contains controls to configure various aspects of RS-485/EIA-485 modes. */
     _IO uint32_t RS485ADRMATCH;  /*!< RS-485/EIA-485 address match. Contains the address match value for RS-485/EIA-485 mode. */
     _IO uint32_t RS485DLY;       /*!< RS-485/EIA-485 direction control delay. */   

     union
     {
        _IO uint32_t SYNCCTRL;  /*!< Synchronous mode control register. Only on USARTs. */
        _I  uint32_t FIFOLVL;   /*!< FIFO Level register. Provides the current fill levels of the transmit and receive FIFOs. */
     };

     _IO uint32_t TER2;           /*!< Transmit Enable Register. Only on LPC177X_8X UART4 and LPC18XX/43XX USART0/2/3. */
} LPC_USART0_2_3_TypeDef;

/*------------- Serial Peripheral Interface (SPI) ----------------------------*/
typedef struct
{         /*!< SPI Structure          */
     _IO uint32_t CR;      /*!< SPI Control Register. This register controls the operation of the SPI. */
     _I  uint32_t SR;      /*!< SPI Status Register. This register shows the status of the SPI. */
     _IO uint32_t DR;      /*!< SPI Data Register. This bi-directional register provides the transmit and receive data for the SPI. Transmit data is provided to the SPI0 by writing to this register. Data received by the SPI0 can be read from this register. */
     _IO uint32_t CCR;     /*!< SPI Clock Counter Register. This register controls the frequency of a master's SCK0. */
     _I  uint32_t RESERVED0[3];
     _IO uint32_t INT;     /*!< SPI Interrupt Flag. This register contains the interrupt flag for the SPI interface. */
} LPC_SPI_TypeDef;

/*------------- Synchronous Serial Communication (SSP) -----------------------*/
typedef struct
{              /*!< SSPn Structure         */
     _IO uint32_t CR0;       /*!< Control Register 0. Selects the serial clock rate, bus type, and data size. */
     _IO uint32_t CR1;       /*!< Control Register 1. Selects master/slave and other modes. */
     _IO uint32_t DR;        /*!< Data Register. Writes fill the transmit FIFO, and reads empty the receive FIFO. */
     _I  uint32_t SR;        /*!< Status Register        */
     _IO uint32_t CPSR;      /*!< Clock Prescale Register */
     _IO uint32_t IMSC;      /*!< Interrupt Mask Set and Clear Register */
     _I  uint32_t RIS;       /*!< Raw Interrupt Status Register */
     _I  uint32_t MIS;       /*!< Masked Interrupt Status Register */
     _O  uint32_t ICR;       /*!< SSPICR Interrupt Clear Register */
     _IO uint32_t DMACR;      /*!< SSPn DMA control register */
} LPC_SSP_TypeDef;

/*------------- Inter-Integrated Circuit (I2C) -------------------------------*/
typedef struct
{              /* I2C0 Structure         */
     _IO uint32_t CONSET;         /*!< I2C Control Set Register. When a one is written to a bit of this register, the corresponding bit in the I2C control register is set. Writing a zero has no effect on the corresponding bit in the I2C control register. */
     _I  uint32_t STAT;           /*!< I2C Status Register. During I2C operation, this register provides detailed status codes that allow software to determine the next action needed. */
     _IO uint32_t DAT;            /*!< I2C Data Register. During master or slave transmit mode, data to be transmitted is written to this register. During master or slave receive mode, data that has been received may be read from this register. */
     _IO uint32_t ADR0;           /*!< I2C Slave Address Register 0. Contains the 7-bit slave address for operation of the I2C interface in slave mode, and is not used in master mode. The least significant bit determines whether a slave responds to the General Call address. */
     _IO uint32_t SCLH;           /*!< SCH Duty Cycle Register High Half Word. Determines the high time of the I2C clock. */
     _IO uint32_t SCLL;           /*!< SCL Duty Cycle Register Low Half Word. Determines the low time of the I2C clock. SCLL and SCLH together determine the clock frequency generated by an I2C master and certain times used in slave mode. */
     _O  uint32_t CONCLR;         /*!< I2C Control Clear Register. When a one is written to a bit of this register, the corresponding bit in the I2C control register is cleared. Writing a zero has no effect on the corresponding bit in the I2C control register. */
     _IO uint32_t MMCTRL;         /*!< Monitor mode control register. */
     _IO uint32_t ADR1;           /*!< I2C Slave Address Register. Contains the 7-bit slave address for operation of the I2C interface in slave mode, and is not used in master mode. The least significant bit determines whether a slave responds to the General Call address. */
     _IO uint32_t ADR2;           /*!< I2C Slave Address Register. Contains the 7-bit slave address for operation of the I2C interface in slave mode, and is not used in master mode. The least significant bit determines whether a slave responds to the General Call address. */
     _IO uint32_t ADR3;           /*!< I2C Slave Address Register. Contains the 7-bit slave address for operation of the I2C interface in slave mode, and is not used in master mode. The least significant bit determines whether a slave responds to the General Call address. */
     _I  uint32_t DATA_BUFFER;    /*!< Data buffer register. The contents of the 8 MSBs of the DAT shift register will be transferred to the DATA_BUFFER automatically after every nine bits (8 bits of data plus ACK or NACK) has been received on the bus. */
     _IO uint32_t MASK[4];        /*!< I2C Slave address mask register */
} LPC_I2C_TypeDef;

/*------------- Inter IC Sound (I2S) -----------------------------------------*/
typedef enum
{
     I2S_DMA_REQUEST_CHANNEL_1,    /*!< DMA request channel 1 */
     I2S_DMA_REQUEST_CHANNEL_2,    /*!< DMA request channel 2 */
     I2S_DMA_REQUEST_CHANNEL_NUM,  /*!< The number of DMA request channels */
} LPC_I2S_DMA_CHANNEL_Typedef;

typedef struct
{              /*!< I2S Structure */
     _IO uint32_t DAO;            /*!< I2S Digital Audio Output Register. Contains control bits for the I2S transmit channel */
     _IO uint32_t DAI;            /*!< I2S Digital Audio Input Register. Contains control bits for the I2S receive channel */
     _O uint32_t TXFIFO;          /*!< I2S Transmit FIFO. Access register for the 8 x 32-bit transmitter FIFO */
     _I uint32_t RXFIFO;          /*!< I2S Receive FIFO. Access register for the 8 x 32-bit receiver FIFO */
     _I uint32_t STATE;           /*!< I2S Status Feedback Register. Contains status information about the I2S interface */
     _IO uint32_t DMA[I2S_DMA_REQUEST_CHANNEL_NUM];   /*!< I2S DMA Configuration Registers. Contains control information for DMA request channels */
     _IO uint32_t IRQ;            /*!< I2S Interrupt Request Control Register. Contains bits that control how the I2S interrupt request is generated */
     _IO uint32_t TXRATE;         /*!< I2S Transmit MCLK divider. This register determines the I2S TX MCLK rate by specifying the value to divide PCLK by in order to produce MCLK */
     _IO uint32_t RXRATE;         /*!< I2S Receive MCLK divider. This register determines the I2S RX MCLK rate by specifying the value to divide PCLK by in order to produce MCLK */
     _IO uint32_t TXBITRATE;      /*!< I2S Transmit bit rate divider. This register determines the I2S transmit bit rate by specifying the value to divide TX_MCLK by in order to produce the transmit bit clock */
     _IO uint32_t RXBITRATE;      /*!< I2S Receive bit rate divider. This register determines the I2S receive bit rate by specifying the value to divide RX_MCLK by in order to produce the receive bit clock */
     _IO uint32_t TXMODE;         /*!< I2S Transmit mode control */
     _IO uint32_t RXMODE;         /*!< I2S Receive mode control */
} LPC_I2S_TypeDef;

/*------------- Repetitive Interrupt Timer (RIT) -----------------------------*/
typedef struct
{                   /*!< RITIMER Structure      */
     _IO uint32_t  COMPVAL;       /*!< Compare register       */
     _IO uint32_t  MASK;          /*!< Mask register. This register holds the 32-bit mask value. A 1 written to any bit will force a compare on the corresponding bit of the counter and compare register. */
     _IO uint32_t  CTRL;          /*!< Control register.      */
     _IO uint32_t  COUNTER;       /*!< 32-bit counter         */
} LPC_RIT_TypeDef;

/*------------- Real-Time Clock (RTC) ----------------------------------------*/
typedef enum
{
     RTC_TIMETYPE_SECOND,          /*!< Second */
     RTC_TIMETYPE_MINUTE,          /*!< Month */
     RTC_TIMETYPE_HOUR,	          /*!< Hour */
     RTC_TIMETYPE_DAYOFMONTH,      /*!< Day of month */
     RTC_TIMETYPE_DAYOFWEEK,       /*!< Day of week */
     RTC_TIMETYPE_DAYOFYEAR,       /*!< Day of year */
     RTC_TIMETYPE_MONTH,	          /*!< Month */
     RTC_TIMETYPE_YEAR,	          /*!< Year */
     RTC_TIMETYPE_LAST
} RTC_TIMEINDEX_TypeDef;

/**
 * @brief Event Channel Identifier definitions
 */
typedef enum
{
     RTC_EV_CHANNEL_1 = 0,
     RTC_EV_CHANNEL_2,
     RTC_EV_CHANNEL_3,
     RTC_EV_CHANNEL_NUM,
} RTC_EV_CHANNEL_TypeDef;

typedef struct
{                        /*!< RTC Structure          */
     _IO uint32_t  ILR;                          /*!< Interrupt Location Register */
     _I  uint32_t  RESERVED0;
     _IO uint32_t  CCR;                          /*!< Clock Control Register */
     _IO uint32_t  CIIR;                         /*!< Counter Increment Interrupt Register */
     _IO uint32_t  AMR;                          /*!< Alarm Mask Register    */
     _I  uint32_t  CTIME[3];                     /*!< Consolidated Time Register 0,1,2 */
     _IO uint32_t  TIME[RTC_TIMETYPE_LAST];      /*!< Timer field registers */
     _IO uint32_t  CALIBRATION;                  /*!< Calibration Value Register */
     _I  uint32_t  RESERVED1[7];
     _IO uint32_t  ALRM[RTC_TIMETYPE_LAST];      /*!< Alarm field registers */
     _IO uint32_t ERSTATUS;                      /*!< Event Monitor/Recorder Status register*/
     _IO uint32_t ERCONTROL;                     /*!< Event Monitor/Recorder Control register*/
     _I  uint32_t ERCOUNTERS;                    /*!< Event Monitor/Recorder Counters register*/
     _I  uint32_t RESERVED2;
     _I  uint32_t ERFIRSTSTAMP[RTC_EV_CHANNEL_NUM];        /*!<Event Monitor/Recorder First Stamp registers*/
     _I  uint32_t RESERVED3;
     _I  uint32_t ERLASTSTAMP[RTC_EV_CHANNEL_NUM];         /*!<Event Monitor/Recorder Last Stamp registers*/
} LPC_RTC_TypeDef;

/*------------- Watchdog Timer (WDT) -----------------------------------------*/
typedef struct
{              /*!< WWDT Structure         */
     _IO uint32_t MOD;            /*!< Watchdog mode register. This register contains the basic mode and status of the Watchdog Timer. */
     _IO uint32_t TC;             /*!< Watchdog timer constant register. This register determines the time-out value. */
     _O  uint32_t FEED;           /*!< Watchdog feed sequence register. Writing 0xAA followed by 0x55 to this register reloads the Watchdog timer with the value contained in WDTC. */
     _I  uint32_t TV;             /*!< Watchdog timer value register. This register reads out the current value of the Watchdog timer. */
     _I  uint32_t RESERVED0;
     _IO uint32_t WARNINT;       /*!< Watchdog warning interrupt register. This register contains the Watchdog warning interrupt compare value. */
     _IO uint32_t WINDOW;        /*!< Watchdog timer window register. This register contains the Watchdog window value. */
} LPC_WDT_TypeDef;

/*------------- Analog-to-Digital Converter (ADC) ----------------------------*/
typedef struct
{              /*!< ADCn Structure */
     _IO uint32_t CR;             /*!< A/D Control Register. The AD0CR register must be written to select the operating mode before A/D conversion can occur. */
     _I  uint32_t GDR;            /*!< A/D Global Data Register. Contains the result of the most recent A/D conversion. */
     _I  uint32_t RESERVED0;
     _IO uint32_t INTEN;          /*!< A/D Interrupt Enable Register. This register contains enable bits that allow the DONE flag of each A/D channel to be included or excluded from contributing to the generation of an A/D interrupt. */
     _I  uint32_t DR[8];          /*!< A/D Channel Data Register. This register contains the result of the most recent conversion completed on channel n. */
     _I  uint32_t STAT;	          /*!< A/D Status Register. This register contains DONE and OVERRUN flags for all of the A/D channels, as well as the A/D interrupt flag. */
} LPC_ADC_TypeDef;

/*------------- Digital-to-Analog Converter (DAC) ----------------------------*/
typedef struct
{              /*!< DAC Structure          */
     _IO uint32_t CR;       /*!< DAC register. Holds the conversion data. */
     _IO uint32_t CTRL;     /*!< DAC control register.  */
     _IO uint32_t CNTVAL;   /*!< DAC counter value register. */
} LPC_DAC_TypeDef;

/*------------- Quadrature Encoder Interface (QEI) ---------------------------*/
typedef struct
{         /*!< QEI Structure          */
     _O  uint32_t CON;           /*!< Control register       */
     _I  uint32_t STAT;          /*!< Encoder status register */
     _IO uint32_t CONF;          /*!< Configuration register */
     _I  uint32_t POS;           /*!< Position register      */
     _IO uint32_t MAXPOS;        /*!< Maximum position register */
     _IO uint32_t CMPOS0;        /*!< position compare register 0 */
     _IO uint32_t CMPOS1;        /*!< position compare register 1 */
     _IO uint32_t CMPOS2;        /*!< position compare register 2 */
     _I  uint32_t INXCNT;        /*!< Index count register   */
     _IO uint32_t INXCMP0;       /*!< Index compare register 0 */
     _IO uint32_t LOAD;          /*!< Velocity timer reload register */
     _I  uint32_t TIME;          /*!< Velocity timer register */
     _I  uint32_t VEL;           /*!< Velocity counter register */
     _I  uint32_t CAP;           /*!< Velocity capture register */
     _IO uint32_t VELCOMP;       /*!< Velocity compare register */
     _IO uint32_t FILTERPHA;     /*!< Digital filter register on input phase A (QEI_A) */
     _IO uint32_t FILTERPHB;     /*!< Digital filter register on input phase B (QEI_B) */
     _IO uint32_t FILTERINX;     /*!< Digital filter register on input index (QEI_IDX) */
     _IO uint32_t WINDOW;        /*!< Index acceptance window register */
     _IO uint32_t INXCMP1;       /*!< Index compare register 1 */
     _IO uint32_t INXCMP2;       /*!< Index compare register 2 */
     _I  uint32_t RESERVED0[993];
     _O  uint32_t IEC;           /*!< Interrupt enable clear register */
     _O  uint32_t IES;           /*!< Interrupt enable set register */
     _I  uint32_t INTSTAT;       /*!< Interrupt status register */
     _I  uint32_t IE;            /*!< Interrupt enable register */
     _O  uint32_t CLR;           /*!< Interrupt status clear register */
     _O  uint32_t SET;           /*!< Interrupt status set register */
} LPC_QEI_TypeDef;

/*------------- Controller Area Network (CAN) --------------------------------*/
/*         CAN message interface register block structure       */
typedef struct {	/*!< C_CAN message interface Structure       */
     _IO uint32_t CMDREQ;         /*!< Message interface command request  */
     _IO uint32_t CMDMSK;         /*!< Message interface command mask*/
     _IO uint32_t MSK1;           /*!< Message interface mask 1 */
     _IO uint32_t MSK2;           /*!< Message interface mask 2 */
     _IO uint32_t ARB1;           /*!< Message interface arbitration 1 */
     _IO uint32_t ARB2;           /*!< Message interface arbitration 2 */
     _IO uint32_t MCTRL;           /*!< Message interface message control */
     _IO uint32_t DA1;            /*!< Message interface data A1 */
     _IO uint32_t DA2;            /*!< Message interface data A2 */
     _IO uint32_t DB1;            /*!< Message interface data B1 */
     _IO uint32_t DB2;            /*!< Message interface data B2 */
     _I  uint32_t  RESERVED[13];
} CAN_IF_TypeDef;

typedef struct                               /* Controller Registers               */
{
     _IO uint32_t CNTL;                     /*!< CAN control            */
     _IO uint32_t STAT;                     /*!< Status register        */
     _I  uint32_t EC;                       /*!< Error counter          */
     _IO uint32_t BT;                       /*!< Bit timing register    */
     _I  uint32_t INT;                      /*!< Interrupt register     */
     _IO uint32_t TEST;                     /*!< Test register          */
     _IO uint32_t BRPE;                     /*!< Baud rate prescaler extension register */
     _I  uint32_t RESERVED0;
     CAN_IF_TypeDef IF[2];
     _I  uint32_t RESERVED2[8];
     _I  uint32_t TXREQ1;                   /*!< Transmission request 1 */
     _I  uint32_t TXREQ2;                   /*!< Transmission request 2 */
     _I  uint32_t RESERVED3[6];
     _I  uint32_t ND1;                      /*!< New data 1             */
     _I  uint32_t ND2;                      /*!< New data 2             */
     _I  uint32_t RESERVED4[6];
     _I  uint32_t IR1;                      /*!< Interrupt pending 1    */
     _I  uint32_t IR2;                      /*!< Interrupt pending 2    */
     _I  uint32_t RESERVED5[6];
     _I  uint32_t MSGV1;                    /*!< Message valid 1        */
     _I  uint32_t MSGV2;                    /*!< Message valid 2        */
     _I  uint32_t RESERVED6[6];
     _IO uint32_t CLKDIV;                   /*!< CAN clock divider register */
} LPC_CAN_TypeDef;

/*------------- General Purpose Direct Memory Access (GPDMA) -----------------*/
typedef struct
{
     _IO uint32_t  SRCADDR;                 /*!< DMA Channel Source Address Register */
     _IO uint32_t  DESTADDR;                /*!< DMA Channel Destination Address Register */
     _IO uint32_t  LLI;                     /*!< DMA Channel Linked List Item Register */
     _IO uint32_t  CONTROL;                 /*!< DMA Channel Control Register */
     _IO uint32_t  CONFIG;                  /*!< DMA Channel Configuration Register */
     _I  uint32_t  RESERVED1[3];
} LPC_GPDMACH_TypeDef;

typedef struct
{               /*!< GPDMA Structure */
	_I  uint32_t INTSTAT;                  /*!< DMA Interrupt Status Register */
	_I  uint32_t INTTCSTAT;                /*!< DMA Interrupt Terminal Count Request Status Register */
	_O  uint32_t INTTCCLEAR;               /*!< DMA Interrupt Terminal Count Request Clear Register */
	_I  uint32_t INTERRSTAT;               /*!< DMA Interrupt Error Status Register */
	_O  uint32_t INTERRCLR;                /*!< DMA Interrupt Error Clear Register */
	_I  uint32_t RAWINTTCSTAT;             /*!< DMA Raw Interrupt Terminal Count Status Register */
	_I  uint32_t RAWINTERRSTAT;            /*!< DMA Raw Error Interrupt Status Register */
	_I  uint32_t ENBLDCHNS;                /*!< DMA Enabled Channel Register */
	_IO uint32_t SOFTBREQ;                 /*!< DMA Software Burst Request Register */
	_IO uint32_t SOFTSREQ;                 /*!< DMA Software Single Request Register */
	_IO uint32_t SOFTLBREQ;                /*!< DMA Software Last Burst Request Register */
	_IO uint32_t SOFTLSREQ;                /*!< DMA Software Last Single Request Register */
	_IO uint32_t CONFIG;                   /*!< DMA Configuration Register */
	_IO uint32_t SYNC;                     /*!< DMA Synchronization Register */
	_I  uint32_t RESERVED0[50];
	LPC_GPDMACH_TypeDef     CH[8];
}LPC_GPDMA_TypeDef;

/*------------- Universal Serial Bus (USB) -----------------------------------*/
typedef struct
{
  _I  uint32_t HcRevision;             /* USB Host Registers                 */
  _IO uint32_t HcControl;
  _IO uint32_t HcCommandStatus;
  _IO uint32_t HcInterruptStatus;
  _IO uint32_t HcInterruptEnable;
  _IO uint32_t HcInterruptDisable;
  _IO uint32_t HcHCCA;
  _I  uint32_t HcPeriodCurrentED;
  _IO uint32_t HcControlHeadED;
  _IO uint32_t HcControlCurrentED;
  _IO uint32_t HcBulkHeadED;
  _IO uint32_t HcBulkCurrentED;
  _I  uint32_t HcDoneHead;
  _IO uint32_t HcFmInterval;
  _I  uint32_t HcFmRemaining;
  _I  uint32_t HcFmNumber;
  _IO uint32_t HcPeriodicStart;
  _IO uint32_t HcLSTreshold;
  _IO uint32_t HcRhDescriptorA;
  _IO uint32_t HcRhDescriptorB;
  _IO uint32_t HcRhStatus;
  _IO uint32_t HcRhPortStatus1;
  _IO uint32_t HcRhPortStatus2;
       uint32_t RESERVED0[40];
  _I  uint32_t Module_ID;

  _I  uint32_t OTGIntSt;               /* USB On-The-Go Registers            */
  _IO uint32_t OTGIntEn;
  _O  uint32_t OTGIntSet;
  _O  uint32_t OTGIntClr;
  _IO uint32_t OTGStCtrl;
  _IO uint32_t OTGTmr;
       uint32_t RESERVED1[58];

  _I  uint32_t USBDevIntSt;            /* USB Device Interrupt Registers     */
  _IO uint32_t USBDevIntEn;
  _O  uint32_t USBDevIntClr;
  _O  uint32_t USBDevIntSet;

  _O  uint32_t USBCmdCode;             /* USB Device SIE Command Registers   */
  _I  uint32_t USBCmdData;

  _I  uint32_t USBRxData;              /* USB Device Transfer Registers      */
  _O  uint32_t USBTxData;
  _I  uint32_t USBRxPLen;
  _O  uint32_t USBTxPLen;
  _IO uint32_t USBCtrl;
  _O  uint32_t USBDevIntPri;

  _I  uint32_t USBEpIntSt;             /* USB Device Endpoint Interrupt Regs */
  _IO uint32_t USBEpIntEn;
  _O  uint32_t USBEpIntClr;
  _O  uint32_t USBEpIntSet;
  _O  uint32_t USBEpIntPri;

  _IO uint32_t USBReEp;                /* USB Device Endpoint Realization Reg*/
  _O  uint32_t USBEpInd;
  _IO uint32_t USBMaxPSize;

  _I  uint32_t USBDMARSt;              /* USB Device DMA Registers           */
  _O  uint32_t USBDMARClr;
  _O  uint32_t USBDMARSet;
       uint32_t RESERVED2[9];
  _IO uint32_t USBUDCAH;
  _I  uint32_t USBEpDMASt;
  _O  uint32_t USBEpDMAEn;
  _O  uint32_t USBEpDMADis;
  _I  uint32_t USBDMAIntSt;
  _IO uint32_t USBDMAIntEn;
       uint32_t RESERVED3[2];
  _I  uint32_t USBEoTIntSt;
  _O  uint32_t USBEoTIntClr;
  _O  uint32_t USBEoTIntSet;
  _I  uint32_t USBNDDRIntSt;
  _O  uint32_t USBNDDRIntClr;
  _O  uint32_t USBNDDRIntSet;
  _I  uint32_t USBSysErrIntSt;
  _O  uint32_t USBSysErrIntClr;
  _O  uint32_t USBSysErrIntSet;
       uint32_t RESERVED4[15];

  union {
  _I  uint32_t I2C_RX;                 /* USB OTG I2C Registers              */
  _O  uint32_t I2C_TX;
  };
  _I  uint32_t I2C_STS;
  _IO uint32_t I2C_CTL;
  _IO uint32_t I2C_CLKHI;
  _O  uint32_t I2C_CLKLO;
       uint32_t RESERVED5[824];

  union {
  _IO uint32_t USBClkCtrl;             /* USB Clock Control Registers        */
  _IO uint32_t OTGClkCtrl;
  };
  union {
  _I  uint32_t USBClkSt;
  _I  uint32_t OTGClkSt;
  };
} LPC_USB_TypeDef;

/*------------- Ethernet Media Access Controller (EMAC) ----------------------*/
typedef struct
{         /*!< ETHERNET Structure */
     _IO uint32_t  MAC_CONFIG;              /*!< MAC configuration register */
     _IO uint32_t  MAC_FRAME_FILTER;        /*!< MAC frame filter */
     _IO uint32_t  MAC_HASHTABLE_HIGH;      /*!< Hash table high register */
     _IO uint32_t  MAC_HASHTABLE_LOW;       /*!< Hash table low register */
     _IO uint32_t  MAC_MII_ADDR;            /*!< MII address register */
     _IO uint32_t  MAC_MII_DATA;            /*!< MII data register */
     _IO uint32_t  MAC_FLOW_CTRL;           /*!< Flow control register */
     _IO uint32_t  MAC_VLAN_TAG;            /*!< VLAN tag register */
     _I  uint32_t  RESERVED0;
     _I  uint32_t  MAC_DEBUG;               /*!< Debug register */
     _IO uint32_t  MAC_RWAKE_FRFLT;         /*!< Remote wake-up frame filter */
     _IO uint32_t  MAC_PMT_CTRL_STAT;       /*!< PMT control and status */
     _I  uint32_t  RESERVED1[2];
     _I  uint32_t  MAC_INTR;                /*!< Interrupt status register */
     _IO uint32_t  MAC_INTR_MASK;           /*!< Interrupt mask register */
     _IO uint32_t  MAC_ADDR0_HIGH;          /*!< MAC address 0 high register */
     _IO uint32_t  MAC_ADDR0_LOW;           /*!< MAC address 0 low register */
     _I  uint32_t  RESERVED2[430];
     _IO uint32_t  MAC_TIMESTP_CTRL;        /*!< Time stamp control register */
     _IO uint32_t  SUBSECOND_INCR;          /*!< Sub-second increment register */
     _I  uint32_t  SECONDS;                 /*!< System time seconds register */
     _I  uint32_t  NANOSECONDS;             /*!< System time nanoseconds register */
     _IO uint32_t  SECONDSUPDATE;           /*!< System time seconds update register */
     _IO uint32_t  NANOSECONDSUPDATE;       /*!< System time nanoseconds update register */
     _IO uint32_t  ADDEND;                  /*!< Time stamp addend register */
     _IO uint32_t  TARGETSECONDS;           /*!< Target time seconds register */
     _IO uint32_t  TARGETNANOSECONDS;       /*!< Target time nanoseconds register */
     _IO uint32_t  HIGHWORD;                /*!< System time higher word seconds register */
     _I  uint32_t  TIMESTAMPSTAT;           /*!< Time stamp status register */
     _IO uint32_t  PPSCTRL;                 /*!< PPS control register */
     _I  uint32_t  AUXNANOSECONDS;          /*!< Auxiliary time stamp nanoseconds register */
     _I  uint32_t  AUXSECONDS;              /*!< Auxiliary time stamp seconds register */
     _I  uint32_t  RESERVED3[562];
     _IO uint32_t  DMA_BUS_MODE;            /*!< Bus Mode Register      */
     _IO uint32_t  DMA_TRANS_POLL_DEMAND;   /*!< Transmit poll demand register */
     _IO uint32_t  DMA_REC_POLL_DEMAND;     /*!< Receive poll demand register */
     _IO uint32_t  DMA_REC_DES_ADDR;        /*!< Receive descriptor list address register */
     _IO uint32_t  DMA_TRANS_DES_ADDR;      /*!< Transmit descriptor list address register */
     _IO uint32_t  DMA_STAT;                /*!< Status register */
     _IO uint32_t  DMA_OP_MODE;             /*!< Operation mode register */
     _IO uint32_t  DMA_INT_EN;              /*!< Interrupt enable register */
     _I  uint32_t  DMA_MFRM_BUFOF;          /*!< Missed frame and buffer overflow register */
     _IO uint32_t  DMA_REC_INT_WDT;         /*!< Receive interrupt watchdog timer register */
     _I  uint32_t  RESERVED4[8];
     _I  uint32_t  DMA_CURHOST_TRANS_DES;   /*!< Current host transmit descriptor register */
     _I  uint32_t  DMA_CURHOST_REC_DES;     /*!< Current host receive descriptor register */
     _I  uint32_t  DMA_CURHOST_TRANS_BUF;   /*!< Current host transmit buffer address register */
     _I  uint32_t  DMA_CURHOST_REC_BUF;     /*!< Current host receive buffer address register */
} LPC_EMAC_TypeDef;

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* Base addresses                                                             */
constexpr intptr_t LPC_FLASH_BASE = 0x00000000UL;
constexpr intptr_t LPC_RAM_BASE   = 0x10000000UL;
constexpr intptr_t LPC_GPIO_BASE  = 0x400F4000UL;
constexpr intptr_t LPC_RTC_BASE   = 0x40040000UL;
constexpr intptr_t LPC_APB0_BASE  = 0x40080000UL;
constexpr intptr_t LPC_APB1_BASE  = 0x400A0000UL;
constexpr intptr_t LPC_APB2_BASE  = 0x400C0000UL;
constexpr intptr_t LPC_APB3_BASE  = 0x400E0000UL;
constexpr intptr_t LPC_AHB_BASE   = 0x40000000UL;
constexpr intptr_t LPC_CM3_BASE   = 0xE0000000UL;

/* APB0 peripherals                                                           */
constexpr intptr_t LPC_WDT_BASE                = (LPC_APB0_BASE + 0x00000);
constexpr intptr_t LPC_UART0_BASE              = (LPC_APB0_BASE + 0x01000);
constexpr intptr_t LPC_UART1_BASE              = (LPC_APB0_BASE + 0x02000);
constexpr intptr_t LPC_SSP0_BASE               = (LPC_APB0_BASE + 0x03000);
constexpr intptr_t LPC_TIM0_BASE               = (LPC_APB0_BASE + 0x04000);
constexpr intptr_t LPC_TIM1_BASE               = (LPC_APB0_BASE + 0x05000);
constexpr intptr_t LPC_SCU_BASE                = (LPC_APB0_BASE + 0x06000);
constexpr intptr_t LPC_GPIOINT_BASE            = (LPC_APB0_BASE + 0x07000);
constexpr intptr_t LPC_GPIO0_GROUP_BASE        = (LPC_APB0_BASE + 0x09000);
constexpr intptr_t LPC_GPIO1_GROUP_BASE        = (LPC_APB0_BASE + 0x08000);

/* APB1 peripherals                                                                */
constexpr intptr_t LPC_MCPWM_BASE              = (LPC_APB1_BASE + 0x00000);
constexpr intptr_t LPC_I2C0_BASE               = (LPC_APB1_BASE + 0x01000);
constexpr intptr_t LPC_I2S0_BASE               = (LPC_APB1_BASE + 0x02000);
constexpr intptr_t LPC_I2S1_BASE               = (LPC_APB1_BASE + 0x03000);
constexpr intptr_t LPC_CAN1_BASE               = (LPC_APB1_BASE + 0x04000);

/* APB2 peripherals                                                                */
constexpr intptr_t LPC_RIT_BASE                = (LPC_APB2_BASE + 0x00000);
constexpr intptr_t LPC_UART2_BASE              = (LPC_APB2_BASE + 0x01000);
constexpr intptr_t LPC_UART3_BASE              = (LPC_APB2_BASE + 0x02000);
constexpr intptr_t LPC_TIM2_BASE               = (LPC_APB2_BASE + 0x03000);
constexpr intptr_t LPC_TIM3_BASE               = (LPC_APB2_BASE + 0x04000);
constexpr intptr_t LPC_SSP1_BASE               = (LPC_APB2_BASE + 0x05000);
constexpr intptr_t LPC_QEI_BASE                = (LPC_APB2_BASE + 0x06000);
constexpr intptr_t LPC_GIMA_BASE               = (LPC_APB2_BASE + 0x07000);

/* APB3 peripherals                                                                */
constexpr intptr_t LPC_I2C1_BASE               = (LPC_APB3_BASE + 0x00000);
constexpr intptr_t LPC_DAC_BASE                = (LPC_APB3_BASE + 0x01000);
constexpr intptr_t LPC_CAN0_BASE               = (LPC_APB3_BASE + 0x02000);
constexpr intptr_t LPC_ADC0_BASE               = (LPC_APB3_BASE + 0x03000);
constexpr intptr_t LPC_ADC1_BASE               = (LPC_APB3_BASE + 0x04000);

/* RTC peripherals                                                                */
constexpr intptr_t LPC_ALARM_BASE              = (LPC_RTC_BASE  + 0x00000);
constexpr intptr_t LPC_BACKUP_BASE             = (LPC_RTC_BASE  + 0x01000);
constexpr intptr_t LPC_POWER_MODE_BASE         = (LPC_RTC_BASE  + 0x02000);
constexpr intptr_t LPC_CREG_BASE               = (LPC_RTC_BASE  + 0x03000);
constexpr intptr_t LPC_EVENT_ROUTER_BASE       = (LPC_RTC_BASE  + 0x04000);
constexpr intptr_t LPC_OTP_CONTROLLER_BASE     = (LPC_RTC_BASE  + 0x05000);
constexpr intptr_t LPC_EVENT_RECORDER_BASE     = (LPC_RTC_BASE  + 0x06000);





/* AHB peripherals                                                            */
constexpr intptr_t LPC_SCT_BASE                   = (LPC_AHB_BASE + 0x00000);
constexpr intptr_t LPC_GPDMA_BASE                 = (LPC_AHB_BASE + 0x02000);
constexpr intptr_t LPC_GPDMACH0_BASE              = (LPC_GPDMA_BASE + 0x00100);
constexpr intptr_t LPC_GPDMACH1_BASE              = (LPC_GPDMA_BASE + 0x00120);
constexpr intptr_t LPC_GPDMACH2_BASE              = (LPC_GPDMA_BASE + 0x00140);
constexpr intptr_t LPC_GPDMACH3_BASE              = (LPC_GPDMA_BASE + 0x00160);
constexpr intptr_t LPC_GPDMACH4_BASE              = (LPC_GPDMA_BASE + 0x00180);
constexpr intptr_t LPC_GPDMACH5_BASE              = (LPC_GPDMA_BASE + 0x001A0);
constexpr intptr_t LPC_GPDMACH6_BASE              = (LPC_GPDMA_BASE + 0x001C0);
constexpr intptr_t LPC_GPDMACH7_BASE              = (LPC_GPDMA_BASE + 0x001E0);

constexpr intptr_t LPC_SPIFI_BASE              = (LPC_AHB_BASE + 0x03000);
constexpr intptr_t LPC_SDMMC_BASE              = (LPC_AHB_BASE + 0x04000);
constexpr intptr_t LPC_EMC_BASE                = (LPC_AHB_BASE + 0x05000);
constexpr intptr_t LPC_USB0_BASE               = (LPC_AHB_BASE + 0x06000);
constexpr intptr_t LPC_USB1_BASE               = (LPC_AHB_BASE + 0x07000);
constexpr intptr_t LPC_LCD_BASE                = (LPC_AHB_BASE + 0x08000);
constexpr intptr_t LPC_FLASHA_BASE             = (LPC_AHB_BASE + 0x0C000);
constexpr intptr_t LPC_FLASHB_BASE             = (LPC_AHB_BASE + 0x0D000);
constexpr intptr_t LPC_EEPROM_BASE             = (LPC_AHB_BASE + 0x0E000);
constexpr intptr_t LPC_ETHERNET_BASE           = (LPC_AHB_BASE + 0x10000);

/* GPIOs                                                                      */
//constexpr intptr_t LPC_GPIO0_BASE = (LPC_GPIO_BASE + 0x00000);
//constexpr intptr_t LPC_GPIO1_BASE = (LPC_GPIO_BASE + 0x00020);
//constexpr intptr_t LPC_GPIO2_BASE = (LPC_GPIO_BASE + 0x00040);
//constexpr intptr_t LPC_GPIO3_BASE = (LPC_GPIO_BASE + 0x00060);
//constexpr intptr_t LPC_GPIO4_BASE = (LPC_GPIO_BASE + 0x00080);

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
inline auto * const LPC_SC = reinterpret_cast<LPC_SC_TypeDef*>(LPC_SCU_BASE);
inline auto * const LPC_GPIO = reinterpret_cast<LPC_GPIO_TypeDef*>(LPC_GPIO_BASE);
// inline auto * const LPC_GPIO1 = reinterpret_cast<LPC_GPIO_TypeDef*>(LPC_GPIO1_BASE);
// inline auto * const LPC_GPIO2 = reinterpret_cast<LPC_GPIO_TypeDef*>(LPC_GPIO2_BASE);
// inline auto * const LPC_GPIO3 = reinterpret_cast<LPC_GPIO_TypeDef*>(LPC_GPIO3_BASE);
// inline auto * const LPC_GPIO4 = reinterpret_cast<LPC_GPIO_TypeDef*>(LPC_GPIO4_BASE);
inline auto * const LPC_WDT = reinterpret_cast<LPC_WDT_TypeDef*>(LPC_WDT_BASE);
inline auto * const LPC_TIM0 = reinterpret_cast<LPC_TIM_TypeDef*>(LPC_TIM0_BASE);
inline auto * const LPC_TIM1 = reinterpret_cast<LPC_TIM_TypeDef*>(LPC_TIM1_BASE);
inline auto * const LPC_TIM2 = reinterpret_cast<LPC_TIM_TypeDef*>(LPC_TIM2_BASE);
inline auto * const LPC_TIM3 = reinterpret_cast<LPC_TIM_TypeDef*>(LPC_TIM3_BASE);
inline auto * const LPC_RIT = reinterpret_cast<LPC_RIT_TypeDef*>(LPC_RIT_BASE);
inline auto * const LPC_UART0 = reinterpret_cast<LPC_USART0_2_3_TypeDef*>(LPC_UART0_BASE);
// inline auto * const LPC_UART1 = reinterpret_cast<LPC_UART1_TypeDef*>(LPC_UART1_BASE);
inline auto * const LPC_UART2 = reinterpret_cast<LPC_USART0_2_3_TypeDef*>(LPC_UART2_BASE);
inline auto * const LPC_UART3 = reinterpret_cast<LPC_USART0_2_3_TypeDef*>(LPC_UART3_BASE);
// inline auto * const LPC_PWM1 = reinterpret_cast<LPC_PWM_TypeDef*>(LPC_PWM1_BASE);
inline auto * const LPC_I2C0 = reinterpret_cast<LPC_I2C_TypeDef*>(LPC_I2C0_BASE);
inline auto * const LPC_I2C1 = reinterpret_cast<LPC_I2C_TypeDef*>(LPC_I2C1_BASE);
// inline auto * const LPC_I2C2 = reinterpret_cast<LPC_I2C_TypeDef*>(LPC_I2C2_BASE);
inline auto * const LPC_I2S0 = reinterpret_cast<LPC_I2S_TypeDef*>(LPC_I2S0_BASE);
inline auto * const LPC_I2S1 = reinterpret_cast<LPC_I2S_TypeDef*>(LPC_I2S1_BASE);
// inline auto * const LPC_SPI = reinterpret_cast<LPC_SPI_TypeDef*>(LPC_SPI_BASE);
inline auto * const LPC_RTC = reinterpret_cast<LPC_RTC_TypeDef*>(LPC_RTC_BASE);
inline auto * const LPC_GPIOINT = reinterpret_cast<LPC_GPIOINT_TypeDef*>(LPC_GPIOINT_BASE);
// inline auto * const LPC_PINCON = reinterpret_cast<LPC_PINCON_TypeDef*>(LPC_PINCON_BASE);
inline auto * const LPC_SSP0 = reinterpret_cast<LPC_SSP_TypeDef*>(LPC_SSP0_BASE);
inline auto * const LPC_SSP1 = reinterpret_cast<LPC_SSP_TypeDef*>(LPC_SSP1_BASE);
// inline auto * const LPC_ADC = reinterpret_cast<LPC_ADC_TypeDef*>(LPC_ADC_BASE);
inline auto * const LPC_DAC = reinterpret_cast<LPC_DAC_TypeDef*>(LPC_DAC_BASE);
// inline auto * const LPC_CANAF_RAM = reinterpret_cast<LPC_CANAF_RAM_TypeDef*>(LPC_CANAF_RAM_BASE);
// inline auto * const LPC_CANAF = reinterpret_cast<LPC_CANAF_TypeDef*>(LPC_CANAF_BASE);
// inline auto * const LPC_CANCR = reinterpret_cast<LPC_CANCR_TypeDef*>(LPC_CANCR_BASE);
inline auto * const LPC_CAN0 = reinterpret_cast<LPC_CAN_TypeDef*>(LPC_CAN0_BASE);
inline auto * const LPC_CAN1 = reinterpret_cast<LPC_CAN_TypeDef*>(LPC_CAN1_BASE);
inline auto * const LPC_MCPWM = reinterpret_cast<LPC_MCPWM_TypeDef*>(LPC_MCPWM_BASE);
inline auto * const LPC_QEI = reinterpret_cast<LPC_QEI_TypeDef*>(LPC_QEI_BASE);
// inline auto * const LPC_EMAC = reinterpret_cast<LPC_EMAC_TypeDef*>(LPC_EMAC_BASE);
inline auto * const LPC_GPDMA = reinterpret_cast<LPC_GPDMA_TypeDef*>(LPC_GPDMA_BASE);
inline auto * const LPC_GPDMACH0 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH0_BASE);
inline auto * const LPC_GPDMACH1 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH1_BASE);
inline auto * const LPC_GPDMACH2 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH2_BASE);
inline auto * const LPC_GPDMACH3 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH3_BASE);
inline auto * const LPC_GPDMACH4 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH4_BASE);
inline auto * const LPC_GPDMACH5 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH5_BASE);
inline auto * const LPC_GPDMACH6 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH6_BASE);
inline auto * const LPC_GPDMACH7 = reinterpret_cast<LPC_GPDMACH_TypeDef*>(LPC_GPDMACH7_BASE);
// inline auto * const LPC_USB = reinterpret_cast<LPC_USB_TypeDef*>(LPC_USB_BASE);


#if defined (__cplusplus)
// SJSU-Dev2: Putting contents of this include in sjsu::lpc18xx
}  // namespace sjsu::lpc18xx
}  // extern "C"
#endif

#endif  // __LPC18xx_H__
