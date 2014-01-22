/****************************************************************************
 *
 * MODULE:             MAC
 *
 * COMPONENT:          $RCSfile: mac_tof.h,v $
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.2 $
 *
 * DATED:              $Date: 2009/09/24 15:15:36 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             dclar
 *
 * DESCRIPTION:
 * MAC - The 802.15.4 Media Access Controller
 * Private header file to be included by module files only
 *
 * LAST MODIFIED BY:   $Author: dclar $
 *                     $Modtime$
 *
 *
 ****************************************************************************
 *
 *  (c) Copyright 2005, Jennic Limited
 *
 ****************************************************************************/

#ifndef _mac_tof_h_
#define _mac_tof_h_

#ifdef __cplusplus
extern "C" {
#endif


/***********************/
/**** INCLUDE FILES ****/
/***********************/
#include "jendefs.h"
#include "mac_sap.h"
#include "mac_prv.h"
#include "TOF_AppApiTof.h"

/************************/
/**** MACROS/DEFINES ****/
/************************/

/***************/
/**** Frame ****/
/***************/

/**** Broadcast IDs ****/


/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/

/**
 * MAC Result code
 * @ingroup grp_MAC
 */


/****************************/
/**** EXPORTED VARIABLES ****/
/****************************/

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/
PUBLIC void vTOF_Init(bool_t bEnable);
PUBLIC void vTOF_SetCalloffset(int32 s32pSecs);

#ifdef JENNIC_CHIP_FAMILY_JN514x
PUBLIC void MAC_vHandleTOFMlmeReqRsp(void *pvMac,
                      MAC_MlmeReqRsp_s *psMlmeReqRsp,
                      MAC_MlmeSyncCfm_s *psMlmeSyncCfm);
PUBLIC void vTOFProcessFrameRXComplete(tsGenFrameHeader *psRXframe);
#endif

PUBLIC bool_t bTOF_StartForwards(MAC_Addr_s *pAddr, uint8 u8TofNumberReadings, tsAppApiTof_Data *pTofData, PR_TOF_APPCALLBACK prTofCallback);
PUBLIC bool_t bTOF_StartReverse(MAC_Addr_s *pAddr, uint8 u8TofNumberReadings, tsAppApiTof_Data *pTofData, PR_TOF_APPCALLBACK prTofCallback);
PUBLIC void vTOF_SetCalloffset(int32 s32pSecs);
PUBLIC int32 s32TOF_GetCalloffset(void);

#ifdef JENNIC_CHIP_FAMILY_JN514x

/* Lookup table to access TOF functions */
#define TOF_VECTOR_TABLE_ENTRIES 8

#define TOF_POLLREQUEST_VECTOR       0
#define TOF_PRIMEREQUEST_VECTOR      1
#define TOF_DATAPOLLREQUEST_VECTOR   2
#define TOF_DATAREQUEST_VECTOR       3
#define TOF_POLLRECEIVED_VECTOR      4
#define TOF_DATARECEIVED_VECTOR      5
#define TOF_DATAPOLLRECEIVED_VECTOR  6
#define TOF_PRIMERECEIVED_VECTOR     7

extern void *aprTofVectorRamTable[TOF_VECTOR_TABLE_ENTRIES];

typedef void (*tprTOFRequest)(MAC_s *, MAC_MlmeReqRsp_s *, MAC_MlmeSyncCfm_s *);
typedef void (*tprTOFReceived)(tsGenFrameHeader *);

#define vTOFPollRequestVector ((tprTOFRequest)(aprTofVectorRamTable[TOF_POLLREQUEST_VECTOR]))
#define vTOFPrimeRequestVector ((tprTOFRequest)(aprTofVectorRamTable[TOF_PRIMEREQUEST_VECTOR]))
#define vTOFDataPollRequestVector ((tprTOFRequest)(aprTofVectorRamTable[TOF_DATAPOLLREQUEST_VECTOR]))
#define vTOFDataRequestVector ((tprTOFRequest)(aprTofVectorRamTable[TOF_DATAREQUEST_VECTOR]))
#define vTOFPollReceivedVector ((tprTOFReceived)(aprTofVectorRamTable[TOF_POLLRECEIVED_VECTOR]))
#define vTOFDataReceivedVector ((tprTOFReceived)(aprTofVectorRamTable[TOF_DATARECEIVED_VECTOR]))
#define vTOFDataPollReceivedVector ((tprTOFReceived)(aprTofVectorRamTable[TOF_DATAPOLLRECEIVED_VECTOR]))
#define vTOFPrimeReceivedVector ((tprTOFReceived)(aprTofVectorRamTable[TOF_PRIMERECEIVED_VECTOR]))

#endif

#ifdef __cplusplus
};
#endif

#endif /* _mac_prv_h_ */

/* End of file $RCSfile: mac_tof.h,v $ *******************************************/
