/*****************************************************************************
 *
 * MODULE:             Application API header
 *
 * COMPONENT:          AppApi_JN514x.h
 *
 * AUTHOR:             CJG
 *
 * DESCRIPTION:        Access functions and structures used by the application
 *                     to interact with the Jennic 802.15.4 stack.
 *
 * $HeadURL: http://svn/sware/Projects/Jeneric/Modules/AppApi/Tags/JENERIC_APPAPI_0v3_RC2/Include/AppApi_JN514x.h $
 *
 * $Revision: 13257 $
 *
 * $LastChangedBy: mwild $
 *
 * $LastChangedDate: 2009-05-26 16:11:24 +0100 (Tue, 26 May 2009) $
 *
 * $Id: AppApi_JN514x.h 13257 2009-05-26 15:11:24Z mwild $
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
 * Copyright Jennic Ltd. 2008 All rights reserved
 *
 ****************************************************************************/

/**
 * @defgroup g_app_sap Application MAC Service Access Point (SAP)
 */
#ifndef  APP_API_H_INCLUDED
#define  APP_API_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <ieee_mac_sap.h>
//#include "RunTimeConfig.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define prRTC_GetComponentInitFunc(A) (((void *(*)(uint8))(*(uint32 *)0x48))(A))
#define prRTC_GetApiFunc(A, B)        (((void *(*)(uint8, uint8))(*(uint32 *)0x4c))(A, B))
/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
#ifndef AHI_DEVICE_ENUM
#define AHI_DEVICE_ENUM

/* Device types, used to identify interrupt source */
typedef enum
{
    E_AHI_DEVICE_SYSCTRL = 2, /* System controller */
    E_AHI_DEVICE_BBC,         /* Baseband controller */
    E_AHI_DEVICE_AES,         /* Encryption engine */
    E_AHI_DEVICE_PHYCTRL,     /* Phy controller */
    E_AHI_DEVICE_UART0,       /* UART 0 */
    E_AHI_DEVICE_UART1,       /* UART 1 */
    E_AHI_DEVICE_TIMER0,      /* Timer 0 */
    E_AHI_DEVICE_TIMER1,      /* Timer 1 */
    E_AHI_DEVICE_SI,          /* Serial Interface (2 wire) */
    E_AHI_DEVICE_SPIM,        /* SPI master */
    E_AHI_DEVICE_INTPER,      /* Intelligent peripheral */
    E_AHI_DEVICE_ANALOGUE     /* Analogue peripherals */
} teAHI_Device;

/* Individual interrupts */
typedef enum
{
    E_AHI_SYSCTRL_WK0 = 26,   /* Wake timer 0 */
    E_AHI_SYSCTRL_WK1 = 27    /* Wake timer 1 */
} teAHI_Item;

#endif /* !AHI_DEVICE_ENUM */

/**
 * @ingroup g_app_sap
 * @brief Get Buffer routine type
 *
 * Type of Get Buffer callback routine
 */
typedef MAC_DcfmIndHdr_s * (*PR_GET_BUFFER)(void *);

/**
 * @ingroup g_app_sap
 * @brief Post routine type
 *
 * Type of Post callback routine
 */
typedef void (*PR_POST_CALLBACK)(void *, MAC_DcfmIndHdr_s *);

/* Types for calls used to re-direct MAC request entry points */
typedef void (*PR_MLME_CALL)(void *, MAC_MlmeReqRsp_s *, MAC_MlmeSyncCfm_s *);
typedef void (*PR_MCPS_CALL)(void *, MAC_McpsReqRsp_s *, MAC_McpsSyncCfm_s *);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
//#ifdef BUILD_FOR_ROM
//#define pvAppApiGetMacHandle ((void * (*)(void))prRTC_GetApiFunc(COMP_ID_MAC, 0))
//#define u32AppApiInit ((uint32 (*)(PR_GET_BUFFER, PR_POST_CALLBACK, void *, PR_GET_BUFFER, PR_POST_CALLBACK, void *))prRTC_GetComponentInitFunc(COMP_ID_MAC))
//#define vAppApiMlmeRequest ((void (*)(MAC_MlmeReqRsp_s *, MAC_MlmeSyncCfm_s *))prRTC_GetApiFunc(COMP_ID_MAC, 1))
//#define vAppApiMcpsRequest ((void (*)(MAC_McpsReqRsp_s *, MAC_McpsSyncCfm_s *))prRTC_GetApiFunc(COMP_ID_MAC, 2))
//#define eAppApiPlmeSet ((PHY_Enum_e (*)(PHY_PibAttr_e, uint32))prRTC_GetApiFunc(COMP_ID_MAC, 3))
//#define eAppApiPlmeGet ((PHY_Enum_e (*)(PHY_PibAttr_e, uint32 *))prRTC_GetApiFunc(COMP_ID_MAC, 4))
//#define pvAppApiGetMacAddrLocation ((void * (*)(void))prRTC_GetApiFunc(COMP_ID_MAC, 5))
//#define vAppApiSaveMacSettings ((void (*)(void))prRTC_GetApiFunc(COMP_ID_MAC, 6))
//#define vAppApiRestoreMacSettings ((void (*)(void))prRTC_GetApiFunc(COMP_ID_MAC, 7))

//#define MAC_COMP_ID 0

//#define pvAppApiGetMacHandle ((void * (*)(void))prRTC_GetApiFunc(MAC_COMP_ID, 0))

//#define u32AppApiInit ((uint32 (*)(PR_GET_BUFFER, PR_POST_CALLBACK, void *, PR_GET_BUFFER, PR_POST_CALLBACK, void *))prRTC_GetComponentInitFunc(MAC_COMP_ID))

/* MLME calls */
//#define vAppApiMlmeRequest ((void (*)(MAC_MlmeReqRsp_s *, MAC_MlmeSyncCfm_s *))prRTC_GetApiFunc(MAC_COMP_ID, 1))
//#define vAppApiMcpsRequest ((void (*)(MAC_McpsReqRsp_s *, MAC_McpsSyncCfm_s *))prRTC_GetApiFunc(MAC_COMP_ID, 2))

/* PLME calls */
//#define eAppApiPlmeSet ((PHY_Enum_e (*)(PHY_PibAttr_e, uint32))prRTC_GetApiFunc(MAC_COMP_ID, 3))
//#define eAppApiPlmeGet ((PHY_Enum_e (*)(PHY_PibAttr_e, uint32 *))prRTC_GetApiFunc(MAC_COMP_ID, 4))

//#define pvAppApiGetMacAddrLocation ((void * (*)(void))prRTC_GetApiFunc(MAC_COMP_ID, 5))
//#define vAppApiSaveMacSettings ((void (*)(void))prRTC_GetApiFunc(MAC_COMP_ID, 6))
//#define vAppApiRestoreMacSettings ((void (*)(void))prRTC_GetApiFunc(MAC_COMP_ID, 7))

//#else
PUBLIC void *pvAppApiGetMacHandle(void);

PUBLIC uint32
u32AppApiInit(PR_GET_BUFFER prMlmeGetBuffer,
              PR_POST_CALLBACK prMlmeCallback,
              void *pvMlmeParam,
              PR_GET_BUFFER prMcpsGetBuffer,
              PR_POST_CALLBACK prMcpsCallback,
              void *pvMcpsParam);

/* MLME calls */
PUBLIC void
vAppApiMlmeRequest(MAC_MlmeReqRsp_s *psMlmeReqRsp,
                   MAC_MlmeSyncCfm_s *psMlmeSyncCfm);

PUBLIC void
vAppApiMcpsRequest(MAC_McpsReqRsp_s *psMcpsReqRsp,
                   MAC_McpsSyncCfm_s *psMcpsSyncCfm);

/* PLME calls */
PUBLIC PHY_Enum_e eAppApiPlmeSet(PHY_PibAttr_e ePhyPibAttribute,
                                 uint32 u32PhyPibValue);
PUBLIC PHY_Enum_e eAppApiPlmeGet(PHY_PibAttr_e ePhyPibAttribute,
                                 uint32 *pu32PhyPibValue);


PUBLIC void *pvAppApiGetMacAddrLocation(void);
PUBLIC void  vAppApiSaveMacSettings(void);
PUBLIC void  vAppApiRestoreMacSettings(void);
//#endif

PUBLIC void vAppApiSetMacEntryPoints(PR_MLME_CALL, PR_MCPS_CALL);
PUBLIC void vAppApiGetMacEntryPoints(PR_MLME_CALL *, PR_MCPS_CALL *);
PUBLIC void vAppApiRegisterTxStartCallback(void (*prvUserCallback)(uint8 u8Handle));
PUBLIC void vAppApiEnableBeaconResponse(bool_t bEnable);

PUBLIC void vMAC_RestoreSettings(void);

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

