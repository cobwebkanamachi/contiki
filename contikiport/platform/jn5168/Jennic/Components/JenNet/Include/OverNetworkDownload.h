/*****************************************************************************
 *
 * MODULE:              JenNet Over Network Download (OND) API
 *
 * DESCRIPTION:         Provides ability to update images over JenNet
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

#ifndef  OND_H_INCLUDED
#define  OND_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "mac_sap.h"
#include "6LP.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define PACKED  __attribute__ ((__packed__))

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum PACKED
{
    E_OND_SUCCESS,
    E_OND_NO_SPACE,
    E_OND_OUT_OF_RANGE,
    E_OND_FUNCTION_NOT_SUPPORTED,
    E_OND_INIT_ERROR,
    E_OND_BAD_PARAMETER
} teOND_Result;

typedef enum PACKED
{
    E_IMG_BLANK,
    E_IMG_CURRENT,
    E_IMG_UPLOADING,
    E_IMG_VALID,
    E_IMG_COMPLETE
} teImageStatus;

typedef struct
{
    MAC_ExtAddr_s sServerAddr;
    uint32        u32DeviceType;
    uint16        u16Chipset;
    uint16        u16Revision;
    uint16        u16BlocksLeft;
    uint16        u16TotalBlocks;
    teImageStatus eStatus;
} tsGetImageInfo;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
/* Device (i.e. non-server) API */
PUBLIC teOND_Result eOND_DevInit(void);
PUBLIC teOND_Result eOND_DevCommenceUpdate(uint32 u32DeviceType,
                                           uint16 u16Chipset,
                                           uint16 u16Revision,
                                           MAC_ExtAddr_s *psServerAddr);
PUBLIC teOND_Result eOND_DevGetImage(uint8 u8ImageId,
                                     tsGetImageInfo *psImageInfo);
PUBLIC teOND_Result eOND_DevSwitchToImage(uint8 u8ImageId);
PUBLIC void vOND_DevCommenceReset(uint32 u32DeviceType,
                                  uint16 u16Timeout,
                                  uint16 u16DepthInfluence);

/* Server API: not included with JN5142J01 */
#ifndef JENNIC_CHIP_JN5142J01
PUBLIC teOND_Result eOND_SrvInit(uint16 u16Port);
PUBLIC teOND_Result eOND_SrvInitiate(uint32          u32DeviceType,
                                     uint16          u16Chipset,
                                     uint16          u16Revision,
                                     ts6LP_SockAddr *psServerAddr);
#endif

/* Sleeping device API: only available on JN516x */
#ifdef JENNIC_CHIP_FAMILY_JN516x
PUBLIC teOND_Result eOND_SleepingDevInit(void);
PUBLIC void vOND_SleepConfigure(uint32 u32PollDelay,
                                uint8  u8WakesPerPoll,
                                uint8  u8PollsPerRefresh);
#endif

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
#ifdef JENNIC_CHIP_JN5142J01
/* Variables are only present on JN5142J01 to allow support for different
   flash parts. For JN5148 and JN5148J01 a M25P40-sized flash part is
   assumed */
extern PUBLIC uint8 u8OND_SectorsAvailable;
extern PUBLIC uint8 u8OND_SectorSize;
#endif

#if defined __cplusplus
}
#endif

#endif  /* OND_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
