/****************************************************************************
 *
 * MODULE:             JIP: Access functions
 *
 * DESCRIPTION:        Variable access functions
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

#ifndef  ACCESS_FUNCTIONS_H_INCLUDED
#define  ACCESS_FUNCTIONS_H_INCLUDED

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "JIP.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define eSetInt8   ((teJIP_Status (*)(int8,  void *))  eSetUint8)
#define eSetInt16  ((teJIP_Status (*)(int16,  void *)) eSetUint16)
#define eSetInt32  ((teJIP_Status (*)(int32,  void *)) eSetUint32)
#define eSetInt64  ((teJIP_Status (*)(int64,  void *)) eSetUint64)
#define eSetFloat  ((teJIP_Status (*)(float,  void *)) eSetUint32)
#define eSetDouble ((teJIP_Status (*)(double, void *)) eSetUint64)

#define vGetInt8   vGetUint8
#define vGetInt16  vGetUint16
#define vGetInt32  vGetUint32
#define vGetInt64  vGetUint64
#define vGetFloat  vGetUint32
#define vGetDouble vGetUint64

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC void vGetUint8(thJIP_Packet hPacket, void *pvCbData);
PUBLIC void vGetUint16(thJIP_Packet hPacket, void *pvCbData);
PUBLIC void vGetUint32(thJIP_Packet hPacket, void *pvCbData);
PUBLIC void vGetUint64(thJIP_Packet hPacket, void *pvCbData);
PUBLIC void vGetString(thJIP_Packet hPacket, void *pvCbData);

PUBLIC teJIP_Status eSetUint8(uint8 u8Val, void *pvCbData);
PUBLIC teJIP_Status eSetUint16(uint16 u16Val, void *pvCbData);
PUBLIC teJIP_Status eSetUint32(uint32 u32Val, void *pvCbData);
PUBLIC teJIP_Status eSetUint64(uint64 u64Val, void *pvCbData);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif  /* ACCESS_FUNCTIONS_H */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

