/*****************************************************************************
 *
 * MODULE:              AHI AES engine
 *
 * DESCRIPTION:
 * AES coprocessor API bit masks, defines and function prototypes.
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

#ifndef  AHI_AES_H_INCLUDED
#define  AHI_AES_H_INCLUDED

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

/* CCM parameter limits */
#define M_MAX       (16)
#define ALENGTH_MAX (31)
#define MLENGTH_MAX (127)

/* AES Encryption Modes */
#define XCV_REG_AES_SET_MODE_ECB          (0)
#define XCV_REG_AES_SET_MODE_CTR          (2)
#define XCV_REG_AES_SET_MODE_CCM          (3)
#define XCV_REG_AES_SET_MODE_CCM_D        (4)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* 128-bit data and config data structure */
typedef struct
{
    uint32 u32register0;
    uint32 u32register1;
    uint32 u32register2;
    uint32 u32register3;
} tsReg128;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/* ECB Encoding API Calls */
PUBLIC bool_t bACI_ECBencodeBuffer(tsReg128 *psKeyData,
                                   bool_t    bLoadKey,
                                   uint32    u32stripes,
                                   uint8    *pau8inputData,
                                   uint8    *pau8outputData,
                                   uint32   *pu32stripesComplete);
PUBLIC bool_t bACI_ECBencodeStripe(tsReg128 *psKeyData,
                                   bool_t    bLoadKey,
                                   tsReg128 *psInputData,
                                   tsReg128 *psOutputData);

/* CCM* Encode/Decode API Calls */
/* full CCM operation with authentication and Encryption/Decryption */
PUBLIC bool_t bACI_CCMstar(tsReg128 *psKeyData,
                           bool_t    bLoadKey,
                           uint8     u8AESmode,
                           uint8     u8M,
                           uint8     u8alength,
                           uint8     u8mlength,
                           tsReg128 *psNonce,
                           uint8    *pau8authenticationData,
                           uint8    *pau8inputData,
                           uint8    *pau8outputData,
                           uint8    *pau8checksumData,
                           bool_t   *pbChecksumVerify);

/* CCM* Data Authentication only */
PUBLIC bool_t bACI_CCMstarAuthenticateData(tsReg128 *psKeyData,
                                           bool_t    bLoadKey,
                                           uint8     u8AESmode,
                                           uint8     u8M,
                                           uint8     u8alength,
                                           tsReg128 *psNonce,
                                           uint8    *pau8authenticationData,
                                           uint8    *pau8checksumData,
                                           bool_t   *pbChecksumVerify);

/* CCM* Data Encode Decode Only */
PUBLIC bool_t bACI_CCMstarEncodeData(tsReg128 *psKeyData,
                                     bool_t    bLoadKey,
                                     uint8     u8mlength,
                                     tsReg128 *psNonce,
                                     uint8    *pau8inputData,
                                     uint8    *pau8outputData);

/* IEEE CTR Data Encode Decode Only */
PUBLIC bool_t bACI_IEEECTRencodeData(tsReg128 *psKeyData,
                                     bool_t    bLoadKey,
                                     uint8     u8mlength,
                                     tsReg128 *psNonce,
                                     uint8    *pau8inputData,
                                     uint8    *pau8outputData);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* AHI_AES_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
