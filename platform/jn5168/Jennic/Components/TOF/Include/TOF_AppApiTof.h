/*****************************************************************************
 *
 * MODULE:              Application API header
 *
 * COMPONENT:           $RCSfile: AppApiTof.h,v $
 *
 * VERSION:             $Name: libMacPatch_LIB_V01060235_IRC01 $
 *
 * REVISION:            $Revision: 1.2 $
 *
 * DATED:               $Date: 2009/04/22 17:15:48 $
 *
 * STATUS:              $State: Exp $
 *
 * AUTHOR:              CJG
 *
 * DESCRIPTION:
 * Access functions and structures used by the application to interact with
 * the Jennic 802.15.4 stack.
 *
 * LAST MODIFIED BY:    $Author: dclar $
 *                      $Modtime: $
 *
 ****************************************************************************
 *
 *  (c) Copyright JENNIC Ltd 2006
 *
 ****************************************************************************/
 /****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on
 * each copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2006. All rights reserved
 *
 ***************************************************************************/

/**
 * @defgroup g_app_sap Application MAC Service Access Point (SAP)
 */

#ifndef  APP_API_TOF_H_INCLUDED
#define  APP_API_TOF_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <mac_sap.h>
//#include "RunTimeConfig.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

//STATUS byte return values
#define MAC_TOF_STATUS_SUCCESS 1      // Success TOF reading
#define MAC_TOF_STATUS_RTE 2          // Remote Time Error
#define MAC_TOF_STATUS_LTE 4          // Local Time Error
#define MAC_TOF_STATUS_NOACK 8        // No ACk received for Tof Poll
#define MAC_TOF_STATUS_DATAERROR 16   // No Remote data received for tof reading

//TOF Direction
#define API_TOF_FORWARDS 0            
#define API_TOF_REVERSE 1
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

//TOF Callback definitions
typedef enum
{
 TOF_SUCCESS,
 TOF_FAIL,
 TOF_NO_VALID_SEQUENCES,
 TOF_NOT_STARTED,
 TOF_TX_ERROR,
 TOF_RX_ERROR
}eTofReturn;


//Callback primitive
typedef void (*PR_TOF_APPCALLBACK)(eTofReturn eStatus);



 typedef struct 
{
    int32   s32Tof; 
    int8    s8LocalRSSI;
    uint8   u8LocalSQI;
    int8    s8RemoteRSSI;
    uint8   u8RemoteSQI;
    uint32  u32Timestamp;
    uint8   u8Status;
}tsAppApiTof_Data;


/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC bool_t bAppApiGetTof(tsAppApiTof_Data *pTofData, MAC_Addr_s *pAddr, uint8 u8NumReadings, bool_t bDirection, PR_TOF_APPCALLBACK prTofCallback);
PUBLIC int32 s32AppApiTofGetCalloffset(void);
PUBLIC void vAppApiTofSetCalloffset(int32 s32pSecs);
PUBLIC void vAppApiTofInit(bool_t bEnable);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* APP_API_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

