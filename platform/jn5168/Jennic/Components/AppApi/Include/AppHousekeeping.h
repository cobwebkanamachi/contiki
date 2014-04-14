/*****************************************************************************
 *
 * MODULE:              Application API Housekeeping functions header
 *
 * DESCRIPTION:
 * Access functions and structures used by the application to interact with
 * the Jennic 802.15.4 stack.
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

/**
 * @defgroup g_app_house_sap Application MAC Service Access Point (SAP)
 */

#ifndef  APP_HOUSE_H_INCLUDED
#define  APP_HOUSE_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "tsv_pub.h"
#include "MacVector.h"
#include "tsv_pub.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
typedef enum
{
    E_HOUSE_WAIT_FOR_NEXT_PERIOD,
    E_HOUSE_WAIT_FOR_ED_SCAN,
    E_HOUSE_WAIT_FOR_MAC_FINISH
} teHousekeepingState;

typedef void (*tprvAppHousekeepingAttemptFunc)(void);

typedef struct tagHousekeeping_s
{
    TSV_Timer_s                    sTimer;
    teHousekeepingState            eState;
    tprvAppHousekeepingAttemptFunc prvAppHousekeepingAttemptFunc;
} Housekeeping_s;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
#define vAppHousekeepingAttemptCalibration (sAppHousekeeping.prvAppHousekeepingAttemptFunc)
PUBLIC void vAppHousekeepingInit(void);
PUBLIC void vAppHousekeepingTimerExpiry(void *pvParam);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern PUBLIC Housekeeping_s sAppHousekeeping;

#if defined __cplusplus
}
#endif

#endif  /* APP_HOUSE_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

