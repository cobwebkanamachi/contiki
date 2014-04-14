/*****************************************************************************
 *
 * MODULE:             Application Hardware API
 *
 * COMPONENT:          AppHardwareApi_JN514x.h
 *
 * AUTHOR:             CJG
 *
 * DESCRIPTION:        Abstraction of the hardware peripherals available on the
 *                     802.15.4 chip that are not used directly for 802.15.4,
 *                     such as UARTs and timers.
 *
 * $HeadURL: http://svn/sware/Projects/Jeneric/Modules/HardwareApi/Tags/JENERIC_HARDWAREAPI_1v3_RC1/Include/AppHardwareApi_JN514x.h $
 *
 * $Revision: 22263 $
 *
 * $LastChangedBy: jahme $
 *
 * $LastChangedDate: 2009-11-23 11:53:34 +0000 (Mon, 23 Nov 2009) $
 *
 * $Id: AppHardwareApi_JN514x.h 22263 2009-11-23 11:53:34Z jahme $
 *
 *****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2009. All rights reserved
 *
 ****************************************************************************/

#ifndef  AHI_H_INCLUDED
#define  AHI_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* System Controller RND Generator */
#define E_AHI_RND_SINGLE_SHOT   TRUE
#define E_AHI_RND_CONTINUOUS    FALSE
#define E_AHI_INTS_ENABLED      TRUE
#define E_AHI_INTS_DISABLED     FALSE

/* System Controller CPU clock control */
#define E_AHI_XTAL_OFF TRUE
#define E_AHI_XTAL_ON FALSE
#define E_AHI_XTAL_4MHZ     0x00
#define E_AHI_XTAL_8MHZ     0x01
#define E_AHI_XTAL_16MHZ    0x02
#define E_AHI_XTAL_32MHZ    0x03


/* System Controller 32Khz Clock Mode Control */
#define E_AHI_INTERNAL_RC 0
#define E_AHI_EXTERNAL_RC 1
#define E_AHI_XTAL 2

/* Device enumerations */
#define E_AHI_WAKE_TIMER_0         0
#define E_AHI_WAKE_TIMER_1         1
#define E_AHI_AP_DAC_1             0
#define E_AHI_AP_DAC_2             1
#define E_AHI_AP_COMPARATOR_1      0
#define E_AHI_AP_COMPARATOR_2      1
#define E_AHI_UART_0               0
#define E_AHI_UART_1               1
#define E_AHI_TIMER_0              0
#define E_AHI_TIMER_1              1
#define E_AHI_TIMER_2              2
#define E_AHI_PC_0                 0
#define E_AHI_PC_1                 1

#define E_AHI_VBOREF_2V0    0
#define E_AHI_VBOREF_2V3    1
#define E_AHI_VBOREF_2V7    2
#define E_AHI_VBOREF_3V0    3


/*********** obsolete *************/
#define E_AHI_DAC_0                0
#define E_AHI_DAC_1                1
#define E_AHI_COMPARATOR_0         0
#define E_AHI_COMPARATOR_1         1
/**********************************/

/* Base Band Controller - Higher data rate */
#define E_AHI_BBC_CTRL_DATA_RATE_250_KBPS   0
#define E_AHI_BBC_CTRL_DATA_RATE_500_KBPS   1
#define E_AHI_BBC_CTRL_DATA_RATE_666_KBPS   2
#define E_AHI_BBC_CTRL_DATA_RATE_1_MBPS     3

/* Phy Controller - Rf Power Settings */
#define E_AHI_ZERO_dB       3
#define E_AHI_MINUS_12_dB   2
#define E_AHI_MINUS_24_dB   1
#define E_AHI_MINUS_32_dB   0

/* Value enumerations: wake timer */
#define E_AHI_WAKE_TIMER_MASK_0    1
#define E_AHI_WAKE_TIMER_MASK_1    2

/* Value enumerations: Analogue Peripherals */
#define E_AHI_ADC_SRC_ADC_1             0
#define E_AHI_ADC_SRC_ADC_2             1
#define E_AHI_ADC_SRC_ADC_3             2
#define E_AHI_ADC_SRC_ADC_4             3
#define E_AHI_ADC_SRC_TEMP              4
#define E_AHI_ADC_SRC_VOLT              5
#define E_AHI_AP_REGULATOR_ENABLE       TRUE
#define E_AHI_AP_REGULATOR_DISABLE      FALSE
#define E_AHI_AP_SAMPLE_2               0
#define E_AHI_AP_SAMPLE_4               1
#define E_AHI_AP_SAMPLE_6               2
#define E_AHI_AP_SAMPLE_8               3
#define E_AHI_AP_CLOCKDIV_2MHZ          0
#define E_AHI_AP_CLOCKDIV_1MHZ          1
#define E_AHI_AP_CLOCKDIV_500KHZ        2
#define E_AHI_AP_CLOCKDIV_250KHZ        3
#define E_AHI_AP_INPUT_RANGE_2          TRUE
#define E_AHI_AP_INPUT_RANGE_1          FALSE
#define E_AHI_AP_GAIN_2                 TRUE
#define E_AHI_AP_GAIN_1                 FALSE
#define E_AHI_AP_EXTREF                 TRUE
#define E_AHI_AP_INTREF                 FALSE
#define E_AHI_ADC_CONVERT_ENABLE        TRUE
#define E_AHI_ADC_CONVERT_DISABLE       FALSE
#define E_AHI_ADC_CONTINUOUS            TRUE
#define E_AHI_ADC_SINGLE_SHOT           FALSE
#define E_AHI_AP_CAPT_INT_STATUS_MASK   0x01
#define E_AHI_AP_ACC_INT_STATUS_MASK    0x2
#define E_AHI_ADC_ACC_SAMPLE_2          0x00
#define E_AHI_ADC_ACC_SAMPLE_4          0x01
#define E_AHI_ADC_ACC_SAMPLE_8          0x02
#define E_AHI_ADC_ACC_SAMPLE_16         0x03
#define E_AHI_AP_ADCACC_INT_ENABLE      0x2U
#define E_AHI_AP_CAPT_INT_ENABLE        0x1U
#define E_AHI_AP_INT_ENABLE             TRUE
#define E_AHI_AP_INT_DISABLE            FALSE
#define E_AHI_DAC_RETAIN_ENABLE         TRUE
#define E_AHI_DAC_RETAIN_DISABLE        FALSE

#define E_AHI_AP_BANDGAP_ENABLE         TRUE
#define E_AHI_AP_BANDGAP_DISABLE        FALSE

/* Value enumerations: Comparator */
#define E_AHI_COMP_HYSTERESIS_0MV  0
#define E_AHI_COMP_HYSTERESIS_10MV 1
#define E_AHI_COMP_HYSTERESIS_20MV 2
#define E_AHI_COMP_HYSTERESIS_40MV 3
#define E_AHI_AP_COMPARATOR_MASK_1 1
#define E_AHI_AP_COMPARATOR_MASK_2 2
/******* obsolete *****************/
#define E_AHI_COMPARATOR_MASK_0    1
#define E_AHI_COMPARATOR_MASK_1    2
/**********************************/
#define E_AHI_COMP_SEL_EXT         0x00
#define E_AHI_COMP_SEL_DAC         0x01
#define E_AHI_COMP_SEL_BANDGAP     0x03

/* Value enumerations: UART */
#define E_AHI_UART_RATE_4800       0
#define E_AHI_UART_RATE_9600       1
#define E_AHI_UART_RATE_19200      2
#define E_AHI_UART_RATE_38400      3
#define E_AHI_UART_RATE_76800      4
#define E_AHI_UART_RATE_115200     5
#define E_AHI_UART_WORD_LEN_5      0
#define E_AHI_UART_WORD_LEN_6      1
#define E_AHI_UART_WORD_LEN_7      2
#define E_AHI_UART_WORD_LEN_8      3
#define E_AHI_UART_FIFO_LEVEL_1    0
#define E_AHI_UART_FIFO_LEVEL_4    1
#define E_AHI_UART_FIFO_LEVEL_8    2
#define E_AHI_UART_FIFO_LEVEL_14   3
#define E_AHI_UART_LS_ERROR        0x80
#define E_AHI_UART_LS_TEMT         0x40
#define E_AHI_UART_LS_THRE         0x20
#define E_AHI_UART_LS_BI           0x10
#define E_AHI_UART_LS_FE           0x08
#define E_AHI_UART_LS_PE           0x04
#define E_AHI_UART_LS_OE           0x02
#define E_AHI_UART_LS_DR           0x01
#define E_AHI_UART_MS_CTS          0x10
#define E_AHI_UART_MS_DCTS         0x01
#define E_AHI_UART_INT_MODEM       0
#define E_AHI_UART_INT_TX          1
#define E_AHI_UART_INT_RXDATA      2
#define E_AHI_UART_INT_RXLINE      3
#define E_AHI_UART_INT_TIMEOUT     6
#define E_AHI_UART_TX_RESET        TRUE
#define E_AHI_UART_RX_RESET        TRUE
#define E_AHI_UART_TX_ENABLE       FALSE
#define E_AHI_UART_RX_ENABLE       FALSE
#define E_AHI_UART_EVEN_PARITY     TRUE
#define E_AHI_UART_ODD_PARITY      FALSE
#define E_AHI_UART_PARITY_ENABLE   TRUE
#define E_AHI_UART_PARITY_DISABLE  FALSE
#define E_AHI_UART_1_STOP_BIT      TRUE
#define E_AHI_UART_2_STOP_BITS     FALSE
#define E_AHI_UART_RTS_HIGH        TRUE
#define E_AHI_UART_RTS_LOW         FALSE
#define E_AHI_UART_FIFO_ARTS_LEVEL_8    0
#define E_AHI_UART_FIFO_ARTS_LEVEL_11   1
#define E_AHI_UART_FIFO_ARTS_LEVEL_13   2
#define E_AHI_UART_FIFO_ARTS_LEVEL_15   3   /* not supported in hardware dcook */

/* Value enumerations: Sample FIFO Interrupt Status */
#define E_AHI_INT_RX_FIFO_HIGH_MASK     0x08
#define E_AHI_INT_TX_FIFO_LOW_MASK      0x04
#define E_AHI_INT_RX_FIFO_OVERFLOW_MASK 0x02
#define E_AHI_INT_TX_FIFO_EMPTY_MASK    0x01

/* Value enumerations: Sample FIFO Status */
#define E_AHI_RX_FIFO_EMPTY     0x00
#define E_AHI_RX_FIFO_FULL      0x0F
#define E_AHI_TX_FIFO_EMPTY     0x00
#define E_AHI_TX_FIFO_FULL      0x0F

/* Data Source for fifo */
#define E_AHI_DAI_CONNECT       TRUE
#define E_AHI_AP_CONNECT        FALSE

/* Default settings for timer2 when used with fifo */
#define E_AHI_LOW_PERIOD    24
#define E_AHI_HIGH_PERIOD   245

/* Default setting Sample fifo ctrl */
#define E_AHI_RX_INT_LEVEL_DEFAULT 8
#define E_AHI_TX_INT_LEVEL_DEFAULT 2

#define E_AHI_DIV_8 3

/* Value enumerations: SPI */
#define E_AHI_SPIM_MSB_FIRST       FALSE
#define E_AHI_SPIM_LSB_FIRST       TRUE
#define E_AHI_SPIM_TXPOS_EDGE      FALSE
#define E_AHI_SPIM_TXNEG_EDGE      TRUE
#define E_AHI_SPIM_RXPOS_EDGE      FALSE
#define E_AHI_SPIM_RXNEG_EDGE      TRUE
#define E_AHI_SPIM_INT_ENABLE      TRUE
#define E_AHI_SPIM_INT_DISABLE     FALSE
#define E_AHI_SPIM_AUTOSLAVE_ENBL  TRUE
#define E_AHI_SPIM_AUTOSLAVE_DSABL FALSE
#define E_AHI_SPIM_SLAVE_ENBLE_0   0x1
#define E_AHI_SPIM_SLAVE_ENBLE_1   0x2
#define E_AHI_SPIM_SLAVE_ENBLE_2   0x4
#define E_AHI_SPIM_SLAVE_ENBLE_3   0x8

/* Value enumerations: Serial Interface */
#define E_AHI_SI_INT_AL            0x20
#define E_AHI_SI_SLAVE_RW_SET      FALSE
#define E_AHI_SI_START_BIT         TRUE
#define E_AHI_SI_NO_START_BIT      FALSE
#define E_AHI_SI_STOP_BIT          TRUE
#define E_AHI_SI_NO_STOP_BIT       FALSE
#define E_AHI_SI_SLAVE_READ        TRUE
#define E_AHI_SI_NO_SLAVE_READ     FALSE
#define E_AHI_SI_SLAVE_WRITE       TRUE
#define E_AHI_SI_NO_SLAVE_WRITE    FALSE
#define E_AHI_SI_SEND_ACK          FALSE
#define E_AHI_SI_SEND_NACK         TRUE
#define E_AHI_SI_IRQ_ACK           TRUE
#define E_AHI_SI_NO_IRQ_ACK        FALSE

/* Value enumerations: Intelligent Peripheral */
#define E_AHI_IP_MAX_MSG_SIZE      0x3E
#define E_AHI_IP_TXPOS_EDGE        FALSE
#define E_AHI_IP_TXNEG_EDGE        TRUE
#define E_AHI_IP_RXPOS_EDGE        FALSE
#define E_AHI_IP_RXNEG_EDGE        TRUE
#define E_AHI_IP_BIG_ENDIAN        TRUE
#define E_AHI_IP_LITTLE_ENDIAN     FALSE

/* Value enumerations: Timer */
#define E_AHI_TIMER_INT_PERIOD     1
#define E_AHI_TIMER_INT_RISE       2

/* Value enumerations: Tick Timer */
#define E_AHI_TICK_TIMER_DISABLE   0x00 /* Disable tick timer */
#define E_AHI_TICK_TIMER_RESTART   0x01 /* Restart timer when match occurs */
#define E_AHI_TICK_TIMER_STOP      0x02 /* Stop timer when match occurs */
#define E_AHI_TICK_TIMER_CONT      0x03 /* Timer does not stop when match occurs */

/* Value enumerations: DIO */
#define E_AHI_DIO0_INT             0x00000001
#define E_AHI_DIO1_INT             0x00000002
#define E_AHI_DIO2_INT             0x00000004
#define E_AHI_DIO3_INT             0x00000008
#define E_AHI_DIO4_INT             0x00000010
#define E_AHI_DIO5_INT             0x00000020
#define E_AHI_DIO6_INT             0x00000040
#define E_AHI_DIO7_INT             0x00000080
#define E_AHI_DIO8_INT             0x00000100
#define E_AHI_DIO9_INT             0x00000200
#define E_AHI_DIO10_INT            0x00000400
#define E_AHI_DIO11_INT            0x00000800
#define E_AHI_DIO12_INT            0x00001000
#define E_AHI_DIO13_INT            0x00002000
#define E_AHI_DIO14_INT            0x00004000
#define E_AHI_DIO15_INT            0x00008000
#define E_AHI_DIO16_INT            0x00010000
#define E_AHI_DIO17_INT            0x00020000
#define E_AHI_DIO18_INT            0x00040000
#define E_AHI_DIO19_INT            0x00080000
#define E_AHI_DIO20_INT            0x00100000

/* Interrupt Item Bitmap Masks */
#define E_AHI_SYSCTRL_PC0_MASK     (1 << E_AHI_SYSCTRL_PC0)
#define E_AHI_SYSCTRL_PC1_MASK     (1 << E_AHI_SYSCTRL_PC1)
#define E_AHI_SYSCTRL_WK0_MASK     (1 << E_AHI_SYSCTRL_WK0)
#define E_AHI_SYSCTRL_WK1_MASK     (1 << E_AHI_SYSCTRL_WK1)
#define E_AHI_SYSCTRL_COMP0_MASK   (1 << E_AHI_SYSCTRL_COMP0)
#define E_AHI_SYSCTRL_COMP1_MASK   (1 << E_AHI_SYSCTRL_COMP1)

#define E_AHI_SYSCTRL_CKEM_MASK     (1 << 31)
#define E_AHI_SYSCTRL_RNDEM_MASK    (1 << 30)
#define E_AHI_SYSCTRL_VREM_MASK     (1 << 25)
#define E_AHI_SYSCTRL_VFEM_MASK     (1 << 24)



#define E_AHI_UART_TIMEOUT_MASK    (1 << E_AHI_UART_INT_TIMEOUT)
#define E_AHI_UART_RXLINE_MASK     (1 << E_AHI_UART_INT_RXLINE)
#define E_AHI_UART_RXDATA_MASK     (1 << E_AHI_UART_INT_RXDATA)
#define E_AHI_UART_TX_MASK         (1 << E_AHI_UART_INT_TX)
#define E_AHI_UART_MODEM_MASK      (1 << E_AHI_UART_INT_MODEM)

#define E_AHI_TIMER_RISE_MASK      E_AHI_TIMER_INT_RISE
#define E_AHI_TIMER_PERIOD_MASK    E_AHI_TIMER_INT_PERIOD

#define E_AHI_SIM_RXACK_MASK        (1 << 7)
#define E_AHI_SIM_BUSY_MASK         (1 << 6)
#define E_AHI_SIM_AL_MASK           (1 << 5)
#define E_AHI_SIM_ICMD_MASK         (1 << 2)
#define E_AHI_SIM_TIP_MASK          (1 << 1)
#define E_AHI_SIM_INT_STATUS_MASK   (1 << 0)

#define E_AHI_SIS_ERROR_MASK        (1 << 4)
#define E_AHI_SIS_LAST_DATA_MASK    (1 << 3)
#define E_AHI_SIS_DATA_WA_MASK      (1 << 2)
#define E_AHI_SIS_DATA_RTKN_MASK    (1 << 1)
#define E_AHI_SIS_DATA_RR_MASK      (1 << 0)

#define E_AHI_SPIM_TX_MASK         (1 << 0)

#define E_AHI_IP_INT_STATUS_MASK   (1 << 6)
#define E_AHI_IP_TXGO_MASK         (1 << 1)
#define E_AHI_IP_RXGO_MASK         (1 << 0)

#define E_AHI_AP_INT_STATUS_MASK   (1 << 0)

/*  DAI enumerations */
// BIT[5:0] DAI_DIV @ 0x0200B008
#define E_AHI_DAI_DIV2  0
#define E_AHI_DAI_DIV4  2

// BIT[2:1] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_I2S_MODE  0x00
#define E_AHI_DAI_LHJ_MODE  0x01
#define E_AHI_DAI_RHJ_MODE  0x10

// BIT[6:3] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_CHARLEN_1BIT  0
#define E_AHI_DAI_CHARLEN_2BIT  1
#define E_AHI_DAI_CHARLEN_3BIT  2
#define E_AHI_DAI_CHARLEN_4BIT  3
#define E_AHI_DAI_CHARLEN_5BIT  4
#define E_AHI_DAI_CHARLEN_6BIT  5
#define E_AHI_DAI_CHARLEN_7BIT  6
#define E_AHI_DAI_CHARLEN_8BIT  7
#define E_AHI_DAI_CHARLEN_9BIT  8
#define E_AHI_DAI_CHARLEN_10BIT 9
#define E_AHI_DAI_CHARLEN_11BIT 10
#define E_AHI_DAI_CHARLEN_12BIT 11
#define E_AHI_DAI_CHARLEN_13BIT 12
#define E_AHI_DAI_CHARLEN_14BIT 13
#define E_AHI_DAI_CHARLEN_15BIT 14
#define E_AHI_DAI_CHARLEN_16BIT 15

// BIT[7] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_EXTRA_PAD_EN  TRUE
#define E_AHI_DAI_EXTRA_PAD_DIS FALSE

// BIT[11:8] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_EXTRA_PAD_LEN_1BIT    0
#define E_AHI_DAI_EXTRA_PAD_LEN_2BIT    1
#define E_AHI_DAI_EXTRA_PAD_LEN_3BIT    2
#define E_AHI_DAI_EXTRA_PAD_LEN_4BIT    3
#define E_AHI_DAI_EXTRA_PAD_LEN_5BIT    4
#define E_AHI_DAI_EXTRA_PAD_LEN_6BIT    5
#define E_AHI_DAI_EXTRA_PAD_LEN_7BIT    6
#define E_AHI_DAI_EXTRA_PAD_LEN_8BIT    7
#define E_AHI_DAI_EXTRA_PAD_LEN_9BIT    8
#define E_AHI_DAI_EXTRA_PAD_LEN_10BIT   9
#define E_AHI_DAI_EXTRA_PAD_LEN_11BIT   10
#define E_AHI_DAI_EXTRA_PAD_LEN_12BIT   11
#define E_AHI_DAI_EXTRA_PAD_LEN_13BIT   12
#define E_AHI_DAI_EXTRA_PAD_LEN_14BIT   13
#define E_AHI_DAI_EXTRA_PAD_LEN_15BIT   14
#define E_AHI_DAI_EXTRA_PAD_LEN_16BIT   15

// BIT[13] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_WS_POL_LHC_1  1
#define E_AHI_DAI_WS_POL_LHC_0  0

// BIT[14] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_WS_IDLE_RHC   0
#define E_AHI_DAI_WS_IDLE_LHC   1

// BIT[15] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_PAD_DIS TRUE
#define E_AHI_DAI_PAD_EN  FALSE

// BIT[16] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_AUX_MODE_EN TRUE
#define E_AHI_DAI_AUX_MODE_DIS FALSE

// BIT[17] DAI _CTRL @ 0x0200B004
#define E_AHI_DAI_AUX_RCHAN TRUE
#define E_AHI_DAI_AUX_LCHAN FALSE

/* Version number of module */
#define AHI_VERSION                0x02040337

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

#ifndef AHI_DEVICE_ENUM
#define AHI_DEVICE_ENUM

/* Device types, used to identify interrupt source */
typedef enum
{
    E_AHI_DEVICE_AUDIOFIFO  = 0,    /* Sample FIFO */
    E_AHI_DEVICE_I2S        = 1,    /* 4-wire Digital Audio Interface */
    E_AHI_DEVICE_SYSCTRL    = 2,    /* System controller */
    E_AHI_DEVICE_BBC        = 3,    /* Baseband controller */
    E_AHI_DEVICE_AES        = 4,    /* Encryption engine */
    E_AHI_DEVICE_PHYCTRL    = 5,    /* Phy controller */
    E_AHI_DEVICE_UART0      = 6,    /* UART 0 */
    E_AHI_DEVICE_UART1      = 7,    /* UART 1 */
    E_AHI_DEVICE_TIMER0     = 8,    /* Timer 0 */
    E_AHI_DEVICE_TIMER1     = 9,    /* Timer 1 */
    E_AHI_DEVICE_SI         = 10,   /* Serial Interface (2 wire) */
    E_AHI_DEVICE_SPIM       = 11,   /* SPI master */
    E_AHI_DEVICE_INTPER     = 12,   /* Intelligent peripheral */
    E_AHI_DEVICE_ANALOGUE   = 13,   /* Analogue peripherals */
    E_AHI_DEVICE_TIMER2     = 14,   /* Timer 2 */
    E_AHI_DEVICE_TICK_TIMER = 15    /* Tick timer */
} teAHI_Device;

/* Individual interrupts. GPIO defined elswhere in this file */
typedef enum
{
    E_AHI_SYSCTRL_PC0   = 22,   /* Pulse Counter 0 */
    E_AHI_SYSCTRL_PC1   = 23,   /* Pulse Counter 1 */
    E_AHI_SYSCTRL_VFES  = 24,   /* VBO Falling  */
    E_AHI_SYSCTRL_VRES  = 25,   /* VBO Rising */
    E_AHI_SYSCTRL_WK0   = 26,   /* Wake timer 0 */
    E_AHI_SYSCTRL_WK1   = 27,   /* Wake timer 1 */
    E_AHI_SYSCTRL_COMP0 = 28,   /* Comparator 0 */
    E_AHI_SYSCTRL_COMP1 = 29,   /* Comparator 1 */
    E_AHI_SYSCTRL_RNDES = 30,   /* Random number generator */
    E_AHI_SYSCTRL_CKES  = 31    /* Clock change  */
} teAHI_Item;

#endif

typedef void (*PR_HWINT_APPCALLBACK)(uint32 u32Device, uint32 u32ItemBitmap);

typedef uint32 tSpiConfiguration;

/* Sleep Modes */
typedef enum
{
    E_AHI_SLEEP_OSCON_RAMON,     /*32Khz Osc on and Ram On*/
    E_AHI_SLEEP_OSCON_RAMOFF,    /*32Khz Osc on and Ram off*/
    E_AHI_SLEEP_OSCOFF_RAMON,    /*32Khz Osc off and Ram on*/
    E_AHI_SLEEP_OSCOFF_RAMOFF,   /*32Khz Osc off and Ram off*/
    E_AHI_SLEEP_DEEP,            /*Deep Sleep*/
} teAHI_SleepMode;


/*Flash Chips*/
typedef enum {
    E_FL_CHIP_ST_M25P10_A,
    E_FL_CHIP_SST_25VF010,
    E_FL_CHIP_ATMEL_AT25F512,
    E_FL_CHIP_ST_M25P40_A,
    E_FL_CHIP_CUSTOM,
    E_FL_CHIP_AUTO
} teFlashChipType;



/// Type definitions for SPI Flash access functions
typedef void    (*tpfvZSPIflashInit)(int iDivisor, uint8 u8SlaveSel);
typedef void    (*tpfvZSPIflashSetSlaveSel)(uint8 u8SlaveSel);
typedef void    (*tpfvZSPIflashWREN)(void);
typedef void    (*tpfvZSPIflashEWRSR)(void);
typedef uint8   (*tpfu8ZSPIflashRDSR)(void);
typedef uint16  (*tpfu16ZSPIflashRDID)(void);
typedef void    (*tpfvZSPIflashWRSR)(uint8 u8Data);
typedef void    (*tpfvZSPIflashPP)(uint32 u32Addr, uint16 u16Len, uint8* pu8Data);
typedef void    (*tpfvZSPIflashRead)(uint32 u32Addr,uint16 u16Len,uint8* pu8Data);
typedef void    (*tpfvZSPIflashBE)(void);
typedef void    (*tpfvZSPIflashSE)(uint8 u8Sector);

/// Table of SPI Flash access functions
typedef struct tagSPIflashFncTable {
    uint32                      u32Signature;
    uint16                      u16FlashId;
    uint16                      u16Reserved;

    tpfvZSPIflashInit           vZSPIflashInit;
    tpfvZSPIflashSetSlaveSel    vZSPIflashSetSlaveSel;
    tpfvZSPIflashWREN           vZSPIflashWREN;
    tpfvZSPIflashEWRSR          vZSPIflashEWRSR;
    tpfu8ZSPIflashRDSR          u8ZSPIflashRDSR;
    tpfu16ZSPIflashRDID         u16ZSPIflashRDID;
    tpfvZSPIflashWRSR           vZSPIflashWRSR;
    tpfvZSPIflashPP             vZSPIflashPP;
    tpfvZSPIflashRead           vZSPIflashRead;
    tpfvZSPIflashBE             vZSPIflashBE;
    tpfvZSPIflashSE             vZSPIflashSE;
} tSPIflashFncTable;





/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/* Initialisation */
PUBLIC uint32 u32AHI_Init(void);

/* System controller */
PUBLIC void vAHI_ClearSystemEventStatus(uint32 u32BitMask);
PUBLIC void vAHI_SetStackOverflow(bool_t bStkOvfEn, uint32 u32Addr);
PUBLIC uint8  u8AHI_PowerStatus(void);
PUBLIC void   vAHI_ProtocolPower(bool_t bOnNotOff);
/*[I SP001340_sfr 5]*/ /*[PR984 ]*/
PUBLIC void   vAHI_MemoryHold(bool_t bHoldDuringSleep);
PUBLIC void   vAHI_CpuDoze(void);
PUBLIC void   vAHI_PowerDown(bool_t bDeepNotNormalSleep);
/* [PR379] */
PUBLIC void   vAHI_Sleep(teAHI_SleepMode eSleepMode);
PUBLIC void   vAHI_AntennaDiversityOutputEnable(bool_t,  bool_t);
PUBLIC void  vAHI_ETSIHighPowerModuleEnable(bool_t bOnNotOff);
PUBLIC void   vAHI_HighPowerModuleEnable(bool_t,  bool_t);
PUBLIC void   vAHI_SysCtrlRegisterCallback(PR_HWINT_APPCALLBACK prSysCtrlCallback);
/*[I SP001340_sfr 1]*/ /*[PR202] */
PUBLIC void vAHI_BrownOutConfigure(uint8 const u8VboSelect,
                                    bool_t const bVboRstEn,
                                    bool_t const bVboEn,
                                    bool_t const bVboIntEnFalling,
                                    bool_t const bVboIntEnRising);
PUBLIC bool_t bAHI_BrownOutStatus(void);
PUBLIC bool_t bAHI_BrownOutEventResetStatus(void);
PUBLIC uint32 u32AHI_BrownOutPoll(void);
/*[I SP001340_sfr 6]*/
PUBLIC void vAHI_StartRandomNumberGenerator(bool_t const bMode,bool_t const bIntEn);
/*[I SP001340_sfr 7]*/
PUBLIC void vAHI_StopRandomNumberGenerator(void);
/*[I SP001340_sfr 8]*/
PUBLIC uint16 u16AHI_ReadRandomNumber(void);
/*[I SP001340_sfr 9]*/
PUBLIC bool_t bAHI_RndNumPoll(void);

/* Base Band Controller  */
/* [I SP001340_sfr 10] */
PUBLIC void vAHI_BbcSetHigherDataRate(uint8 const u8DataRate);
/*[I SP001340_sfr 11] */
PUBLIC void vAHI_BbcSetInterFrameGap(uint8 const u8Lifs);

/* 32Khz Clock Mode selection*/
/*[I SP001340_sfr 4]*/ /*[PR835] */
PUBLIC bool_t bAHI_Set32KhzClockMode(uint8 const u8Mode);

/* Phy Controller */
/*[I SP001340_sfr 3]*/ /*[PR263]*/
PUBLIC bool_t bAHI_PhyRadioSetPower(uint8 const u8PowerLevel);

/* ADC/DAC */
PUBLIC void vAHI_ApSetBandGap(bool_t bBandGapEnable); /* RefPR1488*/

/*[I SP001340_sfr 24] */ /* [PR833] */
PUBLIC void   vAHI_ApConfigure( bool_t bAPRegulator, bool_t bIntEnable,
                              uint8 u8SampleSelect, uint8 u8ClockDivRatio,
                              bool_t bRefSelect);
/*[I SP001340_sfr 24] */ /* [PR833] */
PUBLIC void vAHI_AdcStartAccumulateSamples(uint8 const u8AccSamples);
PUBLIC bool_t bAHI_APRegulatorEnabled(void);
PUBLIC void   vAHI_APRegisterCallback(PR_HWINT_APPCALLBACK prApCallback);
/*[I SP001340_sfr 24] */ /* [PR833] */
PUBLIC void   vAHI_AdcEnable(bool_t bContinuous, bool_t bInputRange, uint8 u8Source);
PUBLIC void   vAHI_AdcDisable(void);
/*[I SP001340_sfr 24] */ /* [PR833] */
PUBLIC void   vAHI_AdcStartSample(void);
/*[I SP001340_sfr 24] */ /* [PR833] */
PUBLIC bool_t bAHI_AdcPoll(void);
PUBLIC uint16 u16AHI_AdcRead(void);
PUBLIC void   vAHI_DacEnable(uint8 u8Dac, bool_t bInputRange, bool_t bRetainOutput,
                           uint16 u16InitialVal);
PUBLIC bool_t bAHI_DacPoll(void);
PUBLIC void   vAHI_DacOutput(uint8 u8Dac, uint16 u16Value);
PUBLIC void   vAHI_DacDisable(uint8 u8Dac);

/* Comparators */
PUBLIC void   vAHI_ComparatorEnable(uint8 u8Comparator, uint8 u8Hysteresis,
                                  uint8 u8SignalSelect);
 PUBLIC void vAHI_ComparatorLowPowerMode(bool_t bLowPowerMode);
PUBLIC void   vAHI_ComparatorDisable(uint8 u8Comparator);
PUBLIC void   vAHI_ComparatorIntEnable(uint8 u8Comparator, bool_t bIntEnable,
                                        bool_t bRisingNotFalling);
PUBLIC void   vAHI_ComparatorWakeEnable(uint8 u8Comparator, bool_t bIntEnable,
                                        bool_t bRisingNotFalling);
PUBLIC uint8  u8AHI_ComparatorStatus(void);
PUBLIC uint8  u8AHI_ComparatorWakeStatus(void);

/* Comparator aliases */
#define vAHI_CompEnable(u8Hysteresis, u8SignalSelect) \
        vAHI_ComparatorEnable(E_AHI_AP_COMPARATOR_2, u8Hysteresis, u8SignalSelect);
#define vAHI_CompDisable() vAHI_ComparatorDisable(E_AHI_AP_COMPARATOR_2)
#define vAHI_CompIntEnable(bIntEnable, bRisingNotFalling) \
            vAHI_ComparatorIntEnable(E_AHI_AP_COMPARATOR_2, bIntEnable, bRisingNotFalling);
#define vAHI_CompWakeEnable(bIntEnable, bRisingNotFalling) \
            vAHI_ComparatorWakeEnable(E_AHI_AP_COMPARATOR_2, bIntEnable, bRisingNotFalling);
#define u8AHI_CompStatus() (u8AHI_ComparatorStatus() & E_AHI_AP_COMPARATOR_MASK_2)
#define u8AHI_CompWakeStatus() (u8AHI_ComparatorWakeStatus() & E_AHI_AP_COMPARATOR_MASK_2)

/* Wake timers */
PUBLIC void   vAHI_WakeTimerEnable(uint8 u8Timer, bool_t bIntEnable);
PUBLIC void   vAHI_WakeTimerStart(uint8 u8Timer, uint32 u32Count);
PUBLIC void   vAHI_WakeTimerStop(uint8 u8Timer);
PUBLIC uint8  u8AHI_WakeTimerStatus(void);
/*[I SP001340_sfr 20]*/ /*[PR844] & [PR836]*/
PUBLIC uint32 u32AHI_WakeTimerCalibrate(void);
PUBLIC uint32 u32AHI_WakeTimerCalibrateEnhanced(uint16 u16CalValue);
PUBLIC uint8  u8AHI_WakeTimerFiredStatus(void);
PUBLIC uint32 u32AHI_WakeTimerRead(uint8 u8Timer);
/*[I SP001340_sfr 16]*/
PUBLIC uint64 u64AHI_WakeTimerReadLarge(uint8 u8Timer);
PUBLIC void vAHI_WakeTimerStartLarge(uint8 u8Timer, uint64 u64Count);

/* GPIO */
PUBLIC void   vAHI_DioSetDirection(uint32 u32Inputs, uint32 u32Outputs);
PUBLIC void   vAHI_DioSetOutput(uint32 u32On, uint32 u32Off);
PUBLIC void   vAHI_DioSetPullup(uint32 u32On, uint32 u32Off);
PUBLIC uint32 u32AHI_DioReadInput(void);
PUBLIC void   vAHI_DioWakeEdge(uint32 u32Rising, uint32 u32Falling);
PUBLIC void   vAHI_DioWakeEnable(uint32 u32Enable, uint32 u32Disable);
PUBLIC uint32 u32AHI_DioWakeStatus(void);

/*[I SP001340_sfr 27]*/ /*[PR400]*/
PUBLIC uint8 u8AHI_DioReadByte(bool_t bDIOSelect);
/*[I SP001340_sfr 28]*/ /*[PR400] */
PUBLIC void vAHI_DioSetByte(bool_t bDIOSelect, uint8 u8DataByte);

#define vAHI_DioInterruptEdge(u32Rising, u32Falling)    (vAHI_DioWakeEdge(u32Rising, u32Falling))
#define vAHI_DioInterruptEnable(u32Enable, u32Disable)  (vAHI_DioWakeEnable(u32Enable, u32Disable))
/*[I SP001340_sfr 29] */ /*[PR676 ] */
#define u32AHI_DioInterruptStatus()                     (u32AHI_DioWakeStatus())

/* UARTs */
PUBLIC void   vAHI_UartEnable(uint8 u8Uart);
PUBLIC void   vAHI_UartDisable(uint8 u8Uart);
PUBLIC void   vAHI_UartSetBaudDivisor(uint8 u8Uart, uint16 u16Divisor);
PUBLIC void   vAHI_UartSetClockDivisor(uint8 u8Uart, uint8 u8BaudRate);
PUBLIC void   vAHI_UartSetControl(uint8 u8Uart, bool_t bEvenParity, bool_t bEnableParity,
                                  uint8 u8WordLength, bool_t bOneStopBit, bool_t bRtsValue);
PUBLIC void   vAHI_UartSetInterrupt(uint8 u8Uart, bool_t bEnableModemStatus,
                                    bool_t bEnableRxLineStatus, bool_t bEnableTxFifoEmpty,
                                    bool_t bEnableRxData, uint8 u8FifoLevel);
PUBLIC void   vAHI_UartReset(uint8 u8Uart, bool_t bTxReset, bool_t bRxReset);
PUBLIC uint8  u8AHI_UartReadLineStatus(uint8 u8Uart);
PUBLIC uint8  u8AHI_UartReadModemStatus(uint8 u8Uart);
PUBLIC uint8  u8AHI_UartReadInterruptStatus(uint8 u8Uart);
PUBLIC void   vAHI_UartWriteData(uint8 u8Uart, uint8 u8Data);
PUBLIC uint8  u8AHI_UartReadData(uint8 u8Uart);
PUBLIC void   vAHI_UartSetRTSCTS(uint8 u8Uart, bool_t bRTSCTSEn);
PUBLIC void   vAHI_Uart0RegisterCallback(PR_HWINT_APPCALLBACK prUart0Callback);
PUBLIC void   vAHI_Uart1RegisterCallback(PR_HWINT_APPCALLBACK prUart1Callback);
/*[I SP001340_sfr 30]*/
PUBLIC void   vAHI_UartSetAutoFlowCtrl(uint8 const u8Uart, uint8 const u8RxFifoLevel,
                                                        bool_t const bAutoRts,
                                                        bool_t const bAutoCts,
                                                        bool_t const bFlowCtrlPolarity);
PUBLIC void vAHI_UartSetClocksPerBit(uint8 u8Uart, uint8 u8Cpb);
PUBLIC uint8  u8AHI_UartReadRxFifoLevel(uint8 u8Uart);
PUBLIC uint8  u8AHI_UartReadTxFifoLevel(uint8 u8Uart);
PUBLIC void vAHI_UartSetRTS(uint8 u8Uart, bool_t bRtsValue);
PUBLIC void vAHI_UartSetBreak(uint8 u8Uart, bool_t bBreak);
#define vAHI_UartSetBaudRate(u8Uart, u8BaudRate) (vAHI_UartSetClockDivisor(u8Uart, u8BaudRate))

/* Timers */
PUBLIC void vAHI_TimerConfigureInputs(uint8 u8Timer, bool_t bInvCapt,bool_t bEventEdge);
PUBLIC void vAHI_TimerConfigure(uint8 u8Timer, bool_t bInvertPwmOutput,bool_t bGateDisable);
#define vAHI_TimerConfigureOutputs(u8Timer, bInvertPwmOutput, bGateDisable) vAHI_TimerConfigure(u8Timer, bInvertPwmOutput, bGateDisable)
PUBLIC void  vAHI_TimerEnable(uint8 u8Timer, uint8 u8Prescale,
                              bool_t bIntRiseEnable, bool_t bIntPeriodEnable,
                              bool_t bOutputEnable);
/*[I SP001340_sfr 33]*/
PUBLIC void  vAHI_TimerDisable(uint8 u8Timer);
/*[I SP001340_sfr 34]*/
PUBLIC void  vAHI_TimerClockSelect(uint8 u8Timer, bool_t bExternalClock, bool_t bInvertClock);
/*[I SP001340_sfr 35]*/
PUBLIC void  vAHI_TimerStartSingleShot(uint8 u8Timer, uint16 u16Hi, uint16 u16Lo);
/*[I SP001340_sfr 36]*/
PUBLIC void  vAHI_TimerStartRepeat(uint8 u8Timer, uint16 u16Hi, uint16 u16Lo);
/*[I SP001340_sfr 37]*/
PUBLIC void  vAHI_TimerStartDeltaSigma(uint8 u8Timer,uint16 u16Hi,uint16 u16Lo, bool_t bRtzEnable);
/*[I SP001340_sfr 38]*/
PUBLIC void  vAHI_TimerStartCapture(uint8 u8Timer);
/*[I SP001340_sfr 39] */
PUBLIC void  vAHI_TimerReadCapture(uint8 u8Timer, uint16 *pu16Hi, uint16 *pu16Lo);
/*[I SP001340_sfr 39] */ /*[PR385] */
PUBLIC void vAHI_TimerReadCaptureFreeRunning(uint8 u8Timer, uint16 *pu16Hi, uint16 *pu16Lo);
/*[I SP001340_sfr 41]*/
PUBLIC void  vAHI_TimerStop(uint8 u8Timer);
/*[I SP001340_sfr 42]*/
PUBLIC uint8 u8AHI_TimerFired(uint8 u8Timer);
/*[I SP001340_sfr 40]*/ /* [PR469] */
PUBLIC uint16 u16AHI_TimerReadCount(uint8 u8Timer);
/*[I SP001340_sfr 43]*/
PUBLIC void  vAHI_TimerDIOControl(uint8, bool_t);
PUBLIC void vAHI_TimerFineGrainDIOControl(uint8 const u8BitMask);
/*[I SP001340_sfr 44]*/
PUBLIC void  vAHI_Timer0RegisterCallback(PR_HWINT_APPCALLBACK prTimer0Callback);
/*[I SP001340_sfr 45]*/
PUBLIC void  vAHI_Timer1RegisterCallback(PR_HWINT_APPCALLBACK prTimer1Callback);
/*[I SP001340_sfr 46]*/
PUBLIC void vAHI_Timer2RegisterCallback(PR_HWINT_APPCALLBACK prTimer2Callback);
/*[I SP001340_sfr 47]*/
PUBLIC void vAHI_IntHandlerTimer0(void);
/*[I SP001340_sfr 48]*/
PUBLIC void vAHI_IntHandlerTimer1(void);
/*[I SP001340_sfr 49]*/
PUBLIC void vAHI_IntHandlerTimer2(void);

/* Tick Timers */
PUBLIC uint32 u32AHI_TickTimerRead(void);
PUBLIC void   vAHI_TickTimerIntPendClr(void);
PUBLIC bool_t bAHI_TickTimerIntStatus(void);
PUBLIC void   vAHI_TickTimerWrite(uint32 u32Count);
PUBLIC void   vAHI_TickTimerConfigure(uint8 u8Mode);
PUBLIC void   vAHI_TickTimerInterval(uint32 u32Interval);
PUBLIC void   vAHI_TickTimerIntEnable(bool_t bIntEnable);
PUBLIC void   vAHI_TickTimerInit(PR_HWINT_APPCALLBACK prTickTimerCallback);

#define  vAHI_TickTimerRegisterCallback(prTickTimerCallback) (vAHI_TickTimerInit(prTickTimerCallback))

/* SPI master */
PUBLIC void   vAHI_SpiConfigure(uint8 u8SlaveEnable, bool_t bLsbFirst, bool_t bTxNegEdge,
                                bool_t bRxNegEdge, uint8 u8ClockDivider, bool_t bInterruptEnable,
                                bool_t bAutoSlaveSelect);
PUBLIC void   vAHI_SpiReadConfiguration(tSpiConfiguration *ptConfiguration);
PUBLIC void   vAHI_SpiRestoreConfiguration(tSpiConfiguration *ptConfiguration);
PUBLIC void   vAHI_SpiSelect(uint8 u8SlaveMask);
PUBLIC void   vAHI_SpiStop(void);
/*[I SP001340_sfr 60]*/
PUBLIC void   vAHI_SpiStartTransfer32(uint32 u32Out);
PUBLIC uint32 u32AHI_SpiReadTransfer32(void);
/*[I SP001340_sfr 60]*/
PUBLIC void   vAHI_SpiStartTransfer16(uint16 u16Out);
PUBLIC uint16 u16AHI_SpiReadTransfer16(void);
/*[I SP001340_sfr 60]*/
PUBLIC void   vAHI_SpiStartTransfer8(uint8 u8Out);
PUBLIC uint8  u8AHI_SpiReadTransfer8(void);
PUBLIC bool_t bAHI_SpiPollBusy(void);
PUBLIC void   vAHI_SpiWaitBusy(void);
PUBLIC void   vAHI_SpiRegisterCallback(PR_HWINT_APPCALLBACK prSpiCallback);
PUBLIC void   vAHI_SpiSetContinuousMode(bool_t bEnable);
/*[I SP001340_sfr 59]*/
PUBLIC void vAHI_SpiSetDelayReadEdge(bool_t const bSetDreBit);

PUBLIC void   vAHI_SpiStartTransfer(uint8 u8CharLen, uint32 u32Out);
PUBLIC void   vAHI_SpiContinuous(bool_t bEnable, uint8 u8CharLen);

/* Serial 2-wire interface Master */
PUBLIC void vAHI_SiMasterConfigure( bool_t bPulseSuppressionEnable,
                                    bool_t bInterruptEnable,
                                    uint8 u8PreScaler);
PUBLIC void vAHI_SiMasterSetCmdReg( bool_t bSetSTA,
                                    bool_t bSetSTO,
                                    bool_t bSetRD,
                                    bool_t bSetWR,
                                    bool_t bSetAckCtrl,
                                    bool_t bSetIACK);
PUBLIC bool_t bAHI_SiMasterSetCmdReg( bool_t bSetSTA,
                                    bool_t bSetSTO,
                                    bool_t bSetRD,
                                    bool_t bSetWR,
                                    bool_t bSetAckCtrl,
                                    bool_t bSetIACK);
PUBLIC void   vAHI_SiMasterWriteData8(uint8 u8Out);
PUBLIC void vAHI_SiMasterWriteSlaveAddrExtended(uint16 u16SlaveAddress,
                                                bool_t bReadStatus,
                                                bool_t bExtendAddress );
PUBLIC void   vAHI_SiMasterWriteSlaveAddr(uint8 u8SlaveAddress,bool_t bReadStatus);
PUBLIC uint8  u8AHI_SiMasterReadData8(void);
PUBLIC bool_t bAHI_SiMasterPollBusy(void);
PUBLIC bool_t bAHI_SiMasterPollTransferInProgress(void);
PUBLIC bool_t bAHI_SiMasterPollRxNack(void);
PUBLIC bool_t bAHI_SiMasterPollArbitrationLost(void);
PUBLIC void vAHI_SiMasterDisable(void);
PUBLIC bool_t bAHI_SiMasterCheckRxNack(void);

#define vAHI_SiSlaveDisable() vAHI_SiMasterDisable()

/* Serial 2-wire interface Master wrappers for backwards compatability */
PUBLIC bool_t bAHI_SiPollArbitrationLost(void);
PUBLIC bool_t bAHI_SiCheckRxNack(void);
PUBLIC bool_t bAHI_SiPollTransferInProgress(void);
PUBLIC bool_t bAHI_SiPollBusy(void);
PUBLIC void vAHI_SiWriteSlaveAddr(uint8 u8SlaveAddress, bool_t bReadStatus);
PUBLIC uint8 u8AHI_SiReadData8(void);
PUBLIC void vAHI_SiWriteData8(uint8 u8Out);
PUBLIC void vAHI_SiSetCmdReg(bool_t bSetSTA, bool_t bSetSTO, bool_t bSetRD,
                             bool_t bSetWR, bool_t bSetAckCtrl, bool_t bSetIACK);
PUBLIC void vAHI_SiConfigure(bool_t bPulseSuppressionEnable, bool_t bInterruptEnable, uint8 u8PreScaler);



/* Serial 2-wire interface Slave */
PUBLIC void vAHI_SiSlaveConfigure(  uint16 u16SlaveAddress,
                                    bool_t bExtendAddr,
                                    bool_t bPulseSuppressionEnable,
                                    bool_t bInMaskEnable,
                                    bool_t bFlowCtrlMode);

PUBLIC void vAHI_SiSlaveWriteData8(uint8 u8Out);
PUBLIC uint8 u8AHI_SiSlaveReadData8(void);
PUBLIC bool_t bAHI_SiSlavePollProtocolError(void);

/* Serial 2-wire interface common */
PUBLIC void   vAHI_SiRegisterCallback(PR_HWINT_APPCALLBACK prSiCallback);

/* Intelligent Peripheral PR1443 TR62, TR63 and TR4 */
PUBLIC void vAHI_IpEnable(bool_t bTxEdge, bool_t bRxEdge, bool_t bIntEn);
PUBLIC void vAHI_IpDisable(void);
PUBLIC bool_t bAHI_IpSendData(uint8 u8Length, uint8 *pau8Data, bool_t bEndian);
PUBLIC bool_t bAHI_IpReadData(uint8 *pu8Length, uint8 *pau8Data, bool_t bEndian);
PUBLIC bool_t bAHI_IpTxDone(void);
PUBLIC bool_t bAHI_IpRxDataAvailable(void);
PUBLIC void   vAHI_IpRegisterCallback(PR_HWINT_APPCALLBACK prIpCallback);
PUBLIC void   vAHI_IpReadyToReceive(void);

/* Flash access */
/* [I SP001340_sfr 72]   */ /*[PR748] */
PUBLIC bool_t bAHI_FlashInit(teFlashChipType flashType, tSPIflashFncTable *pCustomFncTable);
PUBLIC bool_t bAHI_FlashEraseSector(uint8 u8Sector);
/*[I SP001340_sfr 70] */ /*[PR566]  */
PUBLIC bool_t bAHI_FullFlashProgram(uint32 u32Addr, uint16 u16Len, uint8 *pu8Data);
/*[I SP001340_sfr 70] */ /*[PR566]  */
PUBLIC bool_t bAHI_FullFlashRead(uint32 u32Addr, uint16 u16Len, uint8 *pu8Data);
/*[I SP001340_sfr 71] */ /*[PR379]  */
PUBLIC void vAHI_FlashPowerUp(void);
/*[I SP001340_sfr 71] */ /*[PR379]  */
PUBLIC void vAHI_FlashPowerDown(void);

/* Reset functions */
PUBLIC void vAHI_SwReset(void);
PUBLIC void vAHI_DriveResetOut(uint8 u8Period);

/* eFuse functions */
//removed.. to be released through app note at later date

/*Watchdog functions*/
/*[I SP001340_sfr 12]*/
PUBLIC void vAHI_WatchdogStart(uint8 u8Prescale);
/*[I SP001340_sfr 13]*/
PUBLIC void vAHI_WatchdogStop(void);
/*[I SP001340_sfr 14]*/
PUBLIC void vAHI_WatchdogRestart(void);
/*[I SP001340_sfr 15]*/
PUBLIC bool_t bAHI_WatchdogResetEvent(void);
PUBLIC uint16 u16AHI_WatchdogReadValue(void);

/* Pulse Counter Prototypes */
/*[I SP001340_sfr 50] */
PUBLIC bool_t bAHI_PulseCounterConfigure(uint8 const u8Counter,bool_t const bEdgeType,
                                        uint8 const u8Debounce,bool_t const bCombine,
                                        bool_t const bIntEnable);
/*[I SP001340_sfr 51] */
PUBLIC bool_t bAHI_StartPulseCounter(uint8 const u8Counter);
/*[I SP001340_sfr 52] */
PUBLIC bool_t bAHI_StopPulseCounter(uint8 const u8Counter);
/*[I SP001340_sfr 53] */
PUBLIC bool_t bAHI_Read16BitCounter(uint8 const u8Counter, volatile uint16 * pu16Count);
/*[I SP001340_sfr 54] */
PUBLIC bool_t bAHI_Clear16BitPulseCounter(uint8 const u8Counter);
/*[I SP001340_sfr 55] */
PUBLIC bool_t bAHI_Read32BitCounter(volatile uint32 * pu32Count);
/*[I SP001340_sfr 56] */
PUBLIC bool_t bAHI_Clear32BitPulseCounter(void);
/*[I SP001340_sfr 57] */
PUBLIC bool_t bAHI_SetPulseCounterRef(uint8 const u8Counter,uint32 u32RefValue);
PUBLIC uint32 u32AHI_PulseCounterStatus(void);

/* Digital Audio Iinterface Prototypes */
/*[I SP001340_sfr 73] */
PUBLIC void vAHI_DaiEnable(bool_t const bEnable);
/*[I SP001340_sfr 74] */
PUBLIC void vAHI_DaiInterruptEnable(bool_t const bEnable);
/*[I SP001340_sfr 75] */
PUBLIC void vAHI_DaiSetBitClock(uint8 const u8Div, bool_t const bConClock);
/*[I SP001340_sfr 76] */
PUBLIC void vAHI_DaiSetAudioData(uint8 const u8CharLen,
                                bool_t const bPadDis,
                                bool_t const bExPadEn,
                                uint8 u8ExPadLen);
/*[I SP001340_sfr 77] */
PUBLIC void vAHI_DaiSetAudioFormat(uint8 const u8Mode,
                                    bool_t const bWsIdle,
                                    bool_t const bWsPolarity);
/*[I SP001340_sfr 78] */
PUBLIC void vAHI_DaiConnectToFIFO(bool_t const bMode, bool_t const bChannel);
/*[I SP001340_sfr 79] */
PUBLIC void vAHI_DaiStartTransaction(void);
/*[I SP001340_sfr 80] */
PUBLIC bool_t bAHI_DaiPollBusy(void);
/*[I SP001340_sfr 81] */
PUBLIC void vAHI_DaiReadAudioData(uint16 * u16RxDataR,uint16 * u16RxDataL);
/*[I SP001340_sfr 82] */
PUBLIC void vAHI_DaiWriteAudioData(uint16 const u16TxDataR,uint16 const u16TxDataL);
/*[I SP001340_sfr 83] */
PUBLIC void vAHI_IntHandlerDai(void);
/*[I SP001340_sfr 84] */
PUBLIC void vAHI_DaiRegisterCallback(PR_HWINT_APPCALLBACK prDaiCallback);

/* FIFO Prototypes */
/*[I SP001340_sfr 90] */
PUBLIC void vAHI_FifoEnable(bool_t const bEnable);
/*[I SP001340_sfr 91] */
PUBLIC void vAHI_FifoSetInterruptLevel(uint8 const u8RxIntLevel,
                                        uint8 const u8TxIntLevel,
                                        bool_t const bDataSource);
/*[I SP001340_sfr 92] */
PUBLIC void vAHI_FifoEnableInterrupts(bool_t const bRxAbove,
                                    bool_t const bTxBelow,
                                    bool_t const bRxOverflow,
                                    bool_t const bTxEmpty);

/*[I SP001340_sfr 94] */
PUBLIC bool_t bAHI_FifoRead(uint16 * u16RxData);
/*[I SP001340_sfr 95] */
PUBLIC void vAHI_FifoWrite(uint16 const u16TxBuffer);
/*[I SP001340_sfr 96] */
PUBLIC uint8  u8AHI_FifoReadRxLevel(void);
/*[I SP001340_sfr 97] */
PUBLIC uint8  u8AHI_FifoReadTxLevel(void);
PUBLIC void vAHI_IntHandlerAudioFifo(void);
PUBLIC void vAHI_FifoRegisterCallback(PR_HWINT_APPCALLBACK prFifoCallback);

/* Sys Ctrl - CPU Clock  */
/*[I SP001340_sfr 110] */ /*[PR983] */
PUBLIC  bool_t bAHI_SetClockRate(uint8 u8Speed);
/*[I SP001340_sfr 111]*/ /*[PR983] */
PUBLIC  void vAHI_SelectClockSource(bool_t const bClkSource, bool_t const bPowerDown);
/*[I SP001340_sfr 112]*/ /*[PR983] */
PUBLIC  void vAHI_EnableFastStartUp(bool_t const bMode,bool_t const bIsPowerDown);
/*[I SP001340_sfr 113]*/ /*[PR983] */
PUBLIC  void vAHI_PowerXTAL(bool_t const bIsOn);
/*[I SP001340_sfr 114]*/ /*[PR983] */
PUBLIC bool_t bAHI_GetClkSource(void);
/*[I SP001340_sfr 115]*/ /*[PR983] */
PUBLIC  uint8 u8AHI_GetSystemClkSpeed(void);
#define u8AHI_GetSystemClkRate() u8AHI_GetSystemClkSpeed()
/*[I SP001340_sfr 116]*/ /*[PR983] */
PUBLIC bool_t bAHI_Calibrate24MhzClock(uint8 const u8RcCalibration);
/*[I SP001340_sfr 117]*/
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* AHI_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
