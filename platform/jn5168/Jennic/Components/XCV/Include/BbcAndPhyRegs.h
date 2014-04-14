/****************************************************************************
 *
 * MODULE:             MAC
 *
 * DESCRIPTION:
 * BBC and Phy device register addresses
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5148, JN5142, JN5139].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2012. All rights reserved
 *
 ***************************************************************************/

#ifndef _bbc_and_phy_reg_h_
#define _bbc_and_phy_reg_h_

#ifdef __cplusplus
extern "C" {
#endif

/***********************/
/**** INCLUDE FILES ****/
/***********************/
#include "jendefs.h"
#include "PeripheralRegs.h" /* Required for register access functions on
                               JN514x */

/************************/
/**** MACROS/DEFINES ****/
/************************/
#ifndef JENNIC_HW_BBC_DMA
#error Build environment must define JENNIC_HW_BBC_DMA
#endif

#ifndef BIT_W_1
#define BIT_W_1                         0x1
#define BIT_W_2                         0x3
#define BIT_W_3                         0x7
#define BIT_W_4                         0xF
#define BIT_W_5                         0x1F
#define BIT_W_6                         0x3F
#define BIT_W_7                         0x7F
#define BIT_W_8                         0xFF
#define BIT_W_12                        0xFFF
#define BIT_W_16                        0xFFFF
#define BIT_W_21                        0x1FFFFF
#endif

/**** BBC IER/ISR ****/
#define REG_BBC_INT_TX_BIT      0
#define REG_BBC_INT_TX_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_TX_BIT))
#define REG_BBC_INT_RX_BIT      1
#define REG_BBC_INT_RX_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_RX_BIT))
#define REG_BBC_INT_RX_H_BIT    2
#define REG_BBC_INT_RX_H_MASK   ((uint32)(BIT_W_1 << REG_BBC_INT_RX_H_BIT))
#define REG_BBC_INT_M0_BIT      4
#define REG_BBC_INT_M0_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_M0_BIT))
#define REG_BBC_INT_M1_BIT      5
#define REG_BBC_INT_M1_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_M1_BIT))
#define REG_BBC_INT_M2_BIT      6
#define REG_BBC_INT_M2_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_M2_BIT))
#define REG_BBC_INT_M3_BIT      7
#define REG_BBC_INT_M3_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_M3_BIT))
#define REG_BBC_INT_T0_BIT      8
#define REG_BBC_INT_T0_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_T0_BIT))
#define REG_BBC_INT_T1_BIT      9
#define REG_BBC_INT_T1_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_T1_BIT))
#define REG_BBC_INT_T2_BIT      10
#define REG_BBC_INT_T2_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_T2_BIT))
#define REG_BBC_INT_T3_BIT      11
#define REG_BBC_INT_T3_MASK     ((uint32)(BIT_W_1 << REG_BBC_INT_T3_BIT))

#define REG_BBC_INT_TIMER_MASK  ((uint32)(BIT_W_8 << REG_BBC_INT_M0_BIT))

/**** PHYCTRL PHY_RC_CAL ******/
#define REG_PHY_RC_CAL_BIT      12
#define REG_PHY_RC_CAL_MASK     ((uint32)(BIT_W_4 << REG_PHY_RC_CAL_BIT))

#if (defined JENNIC_CHIP_FAMILY_JN516x)
#define REG_PHY_AD_EN_TX_EN_BIT         0
#define REG_PHY_AD_EN_TX_EN_MASK        ((uint8)(BIT_W_1 << REG_PHY_AD_EN_TX_EN_BIT))
#define REG_PHY_AD_EN_RX_EN_BIT         1
#define REG_PHY_AD_EN_RX_EN_MASK        ((uint8)(BIT_W_1 << REG_PHY_AD_EN_RX_EN_BIT))

#define REG_PHY_AD_STAT_TX_ANT_BIT      0
#define REG_PHY_AD_STAT_TX_ANT_MASK     ((uint8)(BIT_W_1 << REG_PHY_AD_STAT_TX_ANT_BIT))
#define REG_PHY_AD_STAT_RX_ANT_BIT      1
#define REG_PHY_AD_STAT_RX_ANT_MASK     ((uint8)(BIT_W_1 << REG_PHY_AD_STAT_RX_ANT_BIT))
#define REG_PHY_AD_STAT_ANT_BIT         2
#define REG_PHY_AD_STAT_ANT_MASK        ((uint8)(BIT_W_1 << REG_PHY_AD_STAT_ANT_BIT))

#define REG_PHY_AD_CTRL_RSSI_THR_BIT    0
#define REG_PHY_AD_CTRL_RSSI_THR_MASK   ((uint8)(BIT_W_5 << REG_PHY_AD_CTRL_RSSI_THR_BIT))
#define REG_PHY_AD_CTRL_SW_ANT_BIT      8
#define REG_PHY_AD_CTRL_SW_ANT_MASK     ((uint32)(BIT_W_1 << REG_PHY_AD_CTRL_SW_ANT_BIT))

#define REG_PHY_MRX_CTRL_RX_COR_HI_BIT  6
#define REG_PHY_MRX_CTRL_RX_COR_HI_MASK ((uint32)(BIT_W_6 << REG_PHY_MRX_CTRL_RX_COR_HI_BIT))

#define REG_PHY_MCCA_CCA_ED_THR_BIT     4
#define REG_PHY_MCCA_CCA_ED_THR_MASK    ((uint32)(BIT_W_8 << REG_PHY_MCCA_CCA_ED_THR_BIT))

#define REG_PHY_PA_CTRL_PAP_BIT         12
#define REG_PHY_PA_CTRL_PAP_MASK        ((uint32)(BIT_W_3 << REG_PHY_PA_CTRL_PAP_BIT))
#endif

#define REG_SYS_BASE   0x02000000UL
#define BBC_TI_OFFSET  0x500
#define BBC_TX_OFFSET  0x540
#define BBC_RX_OFFSET  0x580
#define PHY_OFFSET     0x780

/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/
typedef enum
{
    REG_BBC_SCMR0    = BBC_TI_OFFSET + 0,
    REG_BBC_SCMR1    = BBC_TI_OFFSET + 1,
    REG_BBC_SCMR2    = BBC_TI_OFFSET + 2,
    REG_BBC_SCMR3    = BBC_TI_OFFSET + 3,
    REG_BBC_SCTR0    = BBC_TI_OFFSET + 4,
    REG_BBC_SCTR1    = BBC_TI_OFFSET + 5,
#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_SCTR2    = BBC_TI_OFFSET + 6,
    REG_BBC_SCTR3    = BBC_TI_OFFSET + 7,
#endif
    REG_BBC_SCTCR    = BBC_TI_OFFSET + 8,
    REG_BBC_SCFRC    = BBC_TI_OFFSET + 9,
    REG_BBC_SCSSR    = BBC_TI_OFFSET + 10,
    REG_BBC_SCESL    = BBC_TI_OFFSET + 11,
    REG_BBC_RXETST   = BBC_TI_OFFSET + 12,
    REG_BBC_RXTSTP   = BBC_TI_OFFSET + 13,
    REG_BBC_TXTSTP   = BBC_TI_OFFSET + 14,
    REG_BBC_IER      = BBC_TI_OFFSET + 15,
    REG_BBC_ISR      = BBC_TI_OFFSET + 16,
    REG_BBC_SCTL     = BBC_TI_OFFSET + 17,
    REG_BBC_PTTT     = BBC_TI_OFFSET + 18,
    REG_BBC_PRTT     = BBC_TI_OFFSET + 19,
#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_POFT     = BBC_TI_OFFSET + 20,
#endif
    REG_BBC_TAT      = BBC_TI_OFFSET + 21,
    REG_BBC_BOT      = BBC_TI_OFFSET + 22,
#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_CTID     = BBC_TI_OFFSET + 23,
    REG_BBC_RXNOT    = BBC_TI_OFFSET + 24,
    REG_BBC_RX       = BBC_TI_OFFSET + 25,
    REG_BBC_TXPOST   = BBC_TI_OFFSET + 26,
    REG_BBC_TXPRE    = BBC_TI_OFFSET + 27,
#endif
    REG_BBC_PRBSS    = BBC_TI_OFFSET + 28,
#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_ENDIAN   = BBC_TI_OFFSET + 29,
    REG_BBC_STAT_CTR = BBC_TI_OFFSET + 30,
#endif
    REG_BBC_SM_STATE = BBC_TI_OFFSET + 31,
    REG_BBC_LIFS_TURNAROUND = BBC_TI_OFFSET + 32,
    REG_BBC_HDR_CTRL = BBC_TI_OFFSET + 33,
#if JENNIC_HW_BBC_DMA
    REG_BBC_MISR       = BBC_TI_OFFSET + 34,
#endif

#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_TXLEN    = BBC_TX_OFFSET + 0,
    REG_BBC_TXFCTL   = BBC_TX_OFFSET + 1,
    REG_BBC_TXSEQ    = BBC_TX_OFFSET + 2,
    REG_BBC_TXDPID   = BBC_TX_OFFSET + 3,
    REG_BBC_TXDSAD   = BBC_TX_OFFSET + 4,
    REG_BBC_TXDEADL  = BBC_TX_OFFSET + 5,
    REG_BBC_TXDEADH  = BBC_TX_OFFSET + 6,
    REG_BBC_TXSPID   = BBC_TX_OFFSET + 7,
    REG_BBC_TXSSAD   = BBC_TX_OFFSET + 8,
    REG_BBC_TXSEADL  = BBC_TX_OFFSET + 9,
    REG_BBC_TXSEADH  = BBC_TX_OFFSET + 10,
    REG_BBC_TXMSDU   = BBC_TX_OFFSET + 11,
#endif
    REG_BBC_TXCTL    = BBC_TX_OFFSET + 48,
    REG_BBC_TXSTAT   = BBC_TX_OFFSET + 49,
    REG_BBC_TXMBEBT  = BBC_TX_OFFSET + 50,
    REG_BBC_TXCSMAC  = BBC_TX_OFFSET + 51,
    REG_BBC_TXRETRY  = BBC_TX_OFFSET + 52,
    REG_BBC_TXPEND   = BBC_TX_OFFSET + 53,
#if JENNIC_HW_BBC_DMA
    REG_BBC_TXBUFAD  = BBC_TX_OFFSET + 55,
#endif

#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_RXLEN    = BBC_RX_OFFSET + 0,
    REG_BBC_RXFCTL   = BBC_RX_OFFSET + 1,
    REG_BBC_RXSEQ    = BBC_RX_OFFSET + 2,
    REG_BBC_RXDPID   = BBC_RX_OFFSET + 3,
    REG_BBC_RXDSAD   = BBC_RX_OFFSET + 4,
    REG_BBC_RXDEADL  = BBC_RX_OFFSET + 5,
    REG_BBC_RXDEADH  = BBC_RX_OFFSET + 6,
    REG_BBC_RXSPID   = BBC_RX_OFFSET + 7,
    REG_BBC_RXSSAD   = BBC_RX_OFFSET + 8,
    REG_BBC_RXSEADL  = BBC_RX_OFFSET + 9,
    REG_BBC_RXSEADH  = BBC_RX_OFFSET + 10,
    REG_BBC_RXMSDU   = BBC_RX_OFFSET + 11,
#endif
    REG_BBC_RXCTL    = BBC_RX_OFFSET + 48,
    REG_BBC_RXSTAT   = BBC_RX_OFFSET + 49,
    REG_BBC_RXMPID   = BBC_RX_OFFSET + 50,
    REG_BBC_RXMSAD   = BBC_RX_OFFSET + 51,
    REG_BBC_RXMEADL  = BBC_RX_OFFSET + 52,
    REG_BBC_RXMEADH  = BBC_RX_OFFSET + 53,
#if JENNIC_HW_BBC_DMA
    REG_BBC_RXBUFAD  = BBC_RX_OFFSET + 55,
#endif
    REG_BBC_RXPROM   = BBC_RX_OFFSET + 56,
#if !(JENNIC_HW_BBC_DMA)
    REG_BBC_RXHOFST  = BBC_RX_OFFSET + 57,
#endif

    /* Phy */

    REG_PHY_CTRL         = PHY_OFFSET,
    REG_PHY_STAT         = PHY_OFFSET + 1,
    REG_PHY_CHAN         = PHY_OFFSET + 2,
    REG_PHY_PA_CTRL      = PHY_OFFSET + 3,
    REG_PHY_VCO_AUTO_CAL = PHY_OFFSET + 4,
    REG_PHY_VCO_CAL_CTRL = PHY_OFFSET + 5,
    //REG_PHY_VCO_CAL_STAT = PHY_OFFSET + 6,
    REG_PHY_VCO_CHG      = PHY_OFFSET + 7,
    //REG_PHY_VCO_CHO      = PHY_OFFSET + 8,
    REG_PHY_VCO_FT0      = PHY_OFFSET + 9,
    REG_PHY_VCO_TXO      = PHY_OFFSET + 10,
    REG_PHY_VCO_RXO      = PHY_OFFSET + 11,
    REG_PHY_MCTRL        = PHY_OFFSET + 12,
    REG_PHY_MSTAT0       = PHY_OFFSET + 13,
    REG_PHY_MCCA         = PHY_OFFSET + 14,
    REG_PHY_MRX_CTRL     = PHY_OFFSET + 15,
    REG_PHY_MTX_CTRL     = PHY_OFFSET + 16,
    //REG_PHY_MTX_TST_SEQ  = PHY_OFFSET + 17,
    //REG_PHY_MRST_STAT    = PHY_OFFSET + 18,
    #if JENNIC_HW_BBC_DMA
    REG_PHY_MISR         = PHY_OFFSET + 18,
    #endif
    REG_PHY_IS           = PHY_OFFSET + 19,
    REG_PHY_IE           = PHY_OFFSET + 20,
    REG_PHY_RC_CAL       = PHY_OFFSET + 21,
    REG_PHY_RC_CAL0      = PHY_OFFSET + 22,
    //REG_PHY_LOCK_DET     = PHY_OFFSET + 23,
    REG_PHY_SYNTH_CTRL   = PHY_OFFSET + 24,
    REG_PHY_TST_SIG_CTRL = PHY_OFFSET + 26,
    REG_PHY_MAN_MODE_EN  = PHY_OFFSET + 27,
    REG_PHY_TST0         = PHY_OFFSET + 28,
    REG_PHY_TST1         = PHY_OFFSET + 29,
    REG_PHY_TST2         = PHY_OFFSET + 30,
    REG_PHY_TST3         = PHY_OFFSET + 31,
    #if (defined JENNIC_CHIP_FAMILY_JN516x)
    REG_PHY_AD_EN        = PHY_OFFSET + 32,
    REG_PHY_AD_STAT      = PHY_OFFSET + 33,
    REG_PHY_AD_CTRL      = PHY_OFFSET + 34,
    REG_PHY_TSTART       = PHY_OFFSET + 59,
    REG_PHY_TCOR         = PHY_OFFSET + 60,
    REG_PHY_TTOT         = PHY_OFFSET + 61,
    REG_PHY_TTAT         = PHY_OFFSET + 62,
    REG_PHY_TCTRL        = PHY_OFFSET + 63,
    #endif

} teBbcAndPhyReg;

/* Register access functions - use the NC versions where the address is Not Constant e.g. it changes within a loop */
#define u32REG_PhyRead(eOffset)             u32REG_Read(((eOffset) << 2) + REG_SYS_BASE)
#define vREG_PhyWrite(eOffset, u32Data)     vREG_Write(((eOffset) << 2) + REG_SYS_BASE, u32Data)
#define u32REG_BbcRead(eOffset)             u32REG_Read(((eOffset) << 2) + REG_SYS_BASE)
#define vREG_BbcWrite(eOffset, u32Data)     vREG_Write(((eOffset) << 2) + REG_SYS_BASE, u32Data)

#define u32REG_PhyReadNC(eOffset)             u32REG_ReadNC(((eOffset) << 2) + REG_SYS_BASE)
#define vREG_PhyWriteNC(eOffset, u32Data)     vREG_WriteNC(((eOffset) << 2) + REG_SYS_BASE, u32Data)
#define u32REG_BbcReadNC(eOffset)             u32REG_ReadNC(((eOffset) << 2) + REG_SYS_BASE)
#define vREG_BbcWriteNC(eOffset, u32Data)     vREG_WriteNC(((eOffset) << 2) + REG_SYS_BASE, u32Data)

#ifdef HDK_BUILD
#include "HdkSpi.h"
#define u32REG_XcvrPhyRead(eOffset)         u32HdkSpiRead(eOffset)
#define vREG_XcvrPhyWrite(eOffset, u32Data) vHdkSpiWrite(eOffset, u32Data)
#else
#define u32REG_XcvrPhyRead(eOffset)         u32REG_PhyRead(eOffset)
#define vREG_XcvrPhyWrite(eOffset, u32Data) vREG_PhyWrite(eOffset, u32Data)
#endif

#define vREG_PhyReadModWrite32(eOffset, u32Mask, u32Data) \
    vREG_PhyWrite(eOffset, (((u32Mask) & (u32Data)) | (~(u32Mask) & (u32REG_PhyRead(eOffset)))))

/****************************/
/**** EXPORTED VARIABLES ****/
/****************************/

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/

#ifdef __cplusplus
};
#endif

#endif /* _bbc_phy_reg_h_ */
