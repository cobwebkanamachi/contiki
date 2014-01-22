/*****************************************************************************
 *
 * MODULE:              JenNet security API
 *
 * DESCRIPTION:
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

#ifndef SEC2006_H_
#define SEC2006_H_

#if defined __cplusplus
extern "C" {
#endif

/* Key indeces: note that these are _not_ the same as the key index passed to
   vSecurityUpdateKey, vSecurityInvalidateKey and bSecurityKeyValid! */
#define NWK_KEY_INDEX                (1) // Network key index
#define COM_KEY_INDEX                (2) // Commissioning key index
#define COM_STANDALONE_KEY_INDEX     (3) // Commissioning key index for when in standalone

/* Key indeces: these _are_ suitable for use with the API functions */
#define NWK_KEY_API_INDEX            (0) // Network key index
#define COM_KEY_API_INDEX            (1) // Commissioning key index
#define COM_STANDALONE_KEY_API_INDEX (2) // Commissioning key index for when in standalone

#define TOTAL_SECURITY_KEYS (3)
#define TOTAL_SEC_ENTRIES   (MAX_NEIGHBOUR+MAX_CHILDREN+2)

typedef struct
{
    uint32 u32KeyVal_1;
    uint32 u32KeyVal_2;
    uint32 u32KeyVal_3;
    uint32 u32KeyVal_4;
} tsSecurityKey;

extern void vSetupDefaultSecurity(void);
extern void vSecurityAddDevice(uint8 u8NodeId,MAC_ExtAddr_s *psAddr,uint16 u16PanId);
extern void vSecurityDeleteDevice(uint8 u8NodeId);
extern void vSecurityUpdateKey(uint8 u8KeyIndex,tsSecurityKey *psSecurityKey);
extern void vSecurityInvalidateKey(uint8 u8KeyIndex);
extern uint8 u8SecurityFreeEntry(void);
extern uint8 u8SecurityIsNeighbour(MAC_ExtAddr_s *psAddr);
extern bool_t bSecurityKeyValid(uint8 u8KeyIndex);
extern MAC_DeviceDescriptor_s *psSecurityGetDescriptor(uint8 u8DevId);
extern bool_t bSecuritySetDescriptor(uint8 u8DevId,MAC_DeviceDescriptor_s *psDevDesc);

#if defined __cplusplus
}
#endif
#endif /* SEC2006_H_ */
