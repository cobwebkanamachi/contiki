/*****************************************************************************
 *
 * MODULE:              JenNet API
 *
 * DESCRIPTION:         Tree-based routing layer
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

#ifndef  API_H_INCLUDED
#define  API_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "mac_sap.h"
#include "Sec2006.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifndef PACK
#define PACK      __attribute__ ((packed))        /* align to byte boundary  */
#endif

/* stack events */
#define DATA_TO_COORD               0
#define DATA_TO_PEER                1
#define DATA_TO_COORD_RSP           2
#define DATA_TO_PEER_RSP            3
//#define REGISTER_SERVICES_RSP       4 // Services now in Jenie
//#define SERVICE_REQUEST_RSP         5 // Services now in Jenie
#define POLL_COMPLETE               6
#define DATA_TO_NWK                 9
#define NETWORK_UP                  10
//#define DATA_TO_SERVICE             11    // Services now in Jenie
//#define DATA_TO_SERVICE_RSP         12    // Services now in Jenie
#define CHILD_JOINED                13
#define STACK_RESET                 14
#define CHILD_LEAVE                 15
#define CHILD_REJECTED              16
#define UNROUTED_PACKET             17
#define COORD_ROUTE_ADDED           18      // Coordinator only. Passed with tsRouteInfo
#define COORD_ROUTE_DELETED         19      // Coordinator only. Passed with tsRouteInfo
#define COORD_ROUTE_CHANGE          20
#define NODE_AUTHORISE              21
#define CMSG_TMO                    22
#define NETWORK_ANNOUNCE_EV         23
#define ABOUT_TO_SLEEP              24
#define LOW_ENERGY_SEEN             25
#define OND_FT_BLOCK_BCAST          0x40 /* Defined and used by OND plug-in */
#define OND_FT_BLOCK_UNICAST        0x41 /*  ...  */
#define OND_FT_BLOCK_REQUEST        0x42 /*  ...  */
#define OND_FT_RESET                0x43 /*  ...  */
#define MLD_REPORT                  0x50 /* Defined and used by 6LP multicast plug-in */
#define PACKET_SENT_OK              0xfe
#define PACKET_FAILED               0xff

/* poll status */
#define POLL_NO_DATA                0
#define POLL_DATA_READY             1
#define POLL_TIMEOUT                2

#define PAN_COORD                   TRUE
#define ROUTER                      FALSE

#define RESET_STACK                 TRUE    // PR1125
#define CONTINUE_SCAN               FALSE
#define DEV_ID_UNKNOWN              0xffffffff


#define PROFILE_DFLT    0x00
#define PROFILE_USER    0xff

#define NONE_GATEWAY_MODE       (0x0001)
#define CMSN_MODE               (0x0002)


#define MAX_BEACON_USER_DATA (6)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
// PR1132 Api access to child table
typedef enum    // These are used in the u8ChildStatus
{
    ENTRY_FREE,
    CHILD_ASSOCIATED,
    CHILD_ACTIVE
}teChildStatus;

typedef struct
{
    MAC_ExtAddr_s   sAddr;
    bool_t          bSleepingED;
    uint8           u8ChildStatus;
    uint8           u8LinkQuality;      // Last received link quality info
    uint16          u16PktsLost;        // No of packets not acknowledged
    uint16          u16PktsSent;        // Pkts sent to node
    uint16          u16PktsRcvd;        // Pkts rx’d from node
    uint8           u8CnscPktsLost;     // number of consecutive pkts lost
    uint32          u32ActivityTmo;
    uint32          u32DeviceId;
    bool_t          bRoutesToImport;
    bool_t          bDepthValid;
} tsChild;
#define MAX_CHILDREN                16      // Do not change!
#define MAX_NEIGHBOUR               8

typedef struct
{
    tsChild         sChild[MAX_CHILDREN + 1];
    bool_t          bDeviceStateIsInit;
    uint8           u8ChildCount;
} tsChildren;

/* Network information passed in a NETWORK_UP event */
typedef struct
{
    MAC_ExtAddr_s   sLocalAddr;
    MAC_ExtAddr_s   sParentAddr;
    uint16          u16Depth;
    uint16          u16PanID;
    uint8           u8Channel;
    uint8           *pau8ExtData;
}tsNwkInfo;

#define NWK_INFO_LENGTH (sizeof(tsNwkInfo))

typedef struct
{
    uint16          u16EntryNum;        // Entry number
    uint16          u16TotalEntries;    // Total number of entries
    uint32          u32DeviceId;        // Device id
    MAC_ExtAddr_s   sDestAddr;          // Jenie dest address
    MAC_ExtAddr_s   sNextHopAddr;       // Jenie next hop address
}tsRoutingEntry;

typedef struct
{
    uint8           u8EntryNum;         // Entry
    uint8           u8TotalEntries;     // Total number of entries
    MAC_ExtAddr_s   sAddr;              // Address of node
    bool_t          bSleepingED;        // If device is a sleeping node
    uint8           u8LinkQuality;      // Last received link quality info
    uint16          u16PktsLost;        // No of packets not acknowledged
    uint16          u16PktsSent;        // Acknowledged packets sent to node
    uint16          u16PktsRcvd;        // Pkts rx’d from node
    uint32          u32DeviceId;        // Device id of an associated child
}tsNeighbourEntry;
typedef struct
{
    uint32 u32DeviceId;
    uint16 u16TableEntry;
    uint8 u8NextHop;
    uint8 *pau8ExtData;
}tsRouteInfo;

typedef enum
{
    NETWORK_VERSION,
    MAC_VERSION,
    CHIP_VERSION
}teJenNetComponent;

typedef struct
{
    MAC_ExtAddr_s   sExtAddr;
    uint16          u16PanId;
    uint16          u16Depth;
    uint8           u8Channel;
    uint8           u8LinkQuality;
    uint8           u8NumChildren;
    uint16          u16StackMode;
    uint32          u32DevType;
    uint8           au8UserDefined[MAX_BEACON_USER_DATA];
    uint8           u8JoinProfile;
}tsScanElement;

typedef enum
{
    E_JENNET_SUCCESS,
    E_JENNET_DEFERRED,
    E_JENNET_ERROR
}teJenNetStatusCode;

/* Sleep Modes */
typedef enum
{
    E_JENNET_SLEEP_OSCON_RAMON,     /*32Khz Osc on and Ram On*/
    E_JENNET_SLEEP_OSCON_RAMOFF,    /*32Khz Osc on and Ram off*/
    E_JENNET_SLEEP_OSCOFF_RAMON,    /*32Khz Osc off and Ram on*/
    E_JENNET_SLEEP_OSCOFF_RAMOFF,   /*32Khz Osc off and Ram off*/
    E_JENNET_SLEEP_DEEP,            /*Deep Sleep*/
} teJenNetSleepMode;

typedef enum
{
    E_JENNET_CR,
    E_JENNET_ED
} teJenNetStackType;

#ifdef JENNIC_CHIP_FAMILY_JN514x
typedef struct
{
    teJenNetStackType eStackType;
    uint16  u16PanID;
    uint8   u8Channel;
    uint32  u32ScanChannels;
    bool_t  bPurgeInactiveED;
    uint32 u32RoutePurgeInterval;
    uint32 u32RouteImportInterval;
    bool_t  bSleepDuringBackoff;
    uint8   u8EndDevicePingInterval;
    uint32  u32EndDeviceScanTimeout;
    uint32  u32EndDeviceScanSleep;
    uint32  u32EndDevicePollPeriod;
    uint32  u32EndDeviceActivityTimeout;
    uint32  u32RouterActivityTimeout;
    bool_t  bPermitExtNwkPkts;
    uint32  u32RoutingTableEntries; /* Routing table size (not ED) */
    void    *pvRoutingTableSpace;   /* Routing table buffer (not ED) */
    uint8   u8InternalTimer;
    bool_t  bRecoveredFromJPDM;
    uint16  u16CommWindow;
} tsJenNetConfigData;
#else
typedef struct
{
    teJenNetStackType eStackType;
    uint32            u32ScanChannels;
    uint32            u32RoutePurgeInterval;
    uint32            u32RouteImportInterval;
    uint32            u32EndDeviceScanTimeout;
    uint32            u32EndDeviceScanSleep;
    uint32            u32EndDevicePollPeriod;
    uint32            u32EndDeviceActivityTimeout;
    uint32            u32RouterActivityTimeout;
    uint32            u32RoutingTableEntries; /* Routing table size (not ED) */
    void             *pvRoutingTableSpace;   /* Routing table buffer (not ED) */
    uint16            u16PanID;
    uint16            u16CommWindow;
    uint8             u8Channel;
    uint8             u8EndDevicePingInterval;
    uint8             u8InternalTimer;
    bool_t            bPurgeInactiveED;
    bool_t            bSleepDuringBackoff;
    bool_t            bPermitExtNwkPkts;
    bool_t            bRecoveredFromJPDM;
} tsJenNetConfigData;
#endif

typedef struct
{
    uint8 u8MaxChildren;         /* Set in bJnc_SetRunProfile */
    uint8 u8MaxSleepingChildren; /* Set in bJnc_SetRunProfile */
    uint8 u8MaxFailedPkts;       /* Set in bJnc_SetRunProfile */
    uint8 u8MaxBcastTTL;         /* Set in bJnc_SetRunProfile */
    uint16 u16RouterPingPeriod;  /* Set in bJnc_SetRunProfile */
    uint8 u8MinBeaconLQI;        /* Set in bJnc_SetJoinProfile */
    uint16 u16ScanBackOffMin;    /* Set in bJnc_SetJoinProfile */
    uint16 u16ScanBackOffMax;    /* Set in bJnc_SetJoinProfile */
    uint16 u16EstRtBackOffMin;   /* Set in bJnc_SetJoinProfile */
    uint16 u16EstRtBackOffMax;   /* Set in bJnc_SetJoinProfile */
}tsNwkProfile;


#define tsJenNetInitData tsJenNetConfigData

// This wants compressing down to do. Mick
typedef struct
{
    MAC_ExtAddr_s   sDestAddr;
    uint8           u8NextHop;
    uint8           u8RouteUsage;   // GNATS PR545
    bool_t          bEntryActive;
    bool_t          bRoutesToImport;
#ifndef NON_COORD_BUILD /* Not included on builds that don't support a coordinator */
    uint32          u32DeviceId;
#endif
} tsRoutingTable;

typedef struct
{
    uint32 u32Protocol;
    char *pau8VersionString;
}tsVersionInfo;

typedef struct
{
    uint8 u8StackCurrentState;
    uint8 u8StackResumeState;
    uint8 u8LostPacketCount;
    uint8 u8UnackPings;
    uint8 u8EstablishRouteAttempts;
}tsStackReset;

typedef bool_t (*trSortScanCallback)(tsScanElement *pasScanResult,
                                   uint8 u8ScanListSize,
                                   uint8 *pau8ScanListOrder);

typedef bool_t (*trBeaconNotifyCallback)(tsScanElement *psBeaconInfo,
                                       uint16 u16ProtocolVersion);

typedef bool_t (*trLocalAuthoriseCallback)(MAC_ExtAddr_s *psAddr);
typedef bool_t (*trNwkAuthoriseCallback)(MAC_ExtAddr_s *psAddr,
                                         uint8 u8DataLength,
                                         uint8 *pu8Data);

typedef bool_t (*trEndDeviceResumeCallback)(bool_t bBusy);

typedef void (*tvAboutToSleep)(void);

typedef enum PACK
{
    E_LEF_ADD,
    E_LEF_DELETE
} teLowEnergyStatus;

typedef bool_t (*tprSafeToSleepCallback)(void);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void     vJnc_JenNetColdStart(void);
PUBLIC void     vNed_JenNetWarmStart(void);

PUBLIC void     vApi_Start_ED(bool_t bPanCoord);
//PUBLIC void     vApi_Start_CR(bool_t bPanCoord);
PUBLIC void     vApi_Recover(bool_t bPanCoord);
PUBLIC void     vApi_Resume(void);
PUBLIC void     vApi_SetSleepPeriod(uint32 u32SleepPeriodInMs);
PUBLIC void     vApi_SetScanSleep(uint32 u32ScanSleepDuration);
PUBLIC void     vApi_Sleep(teJenNetSleepMode eSleepMode);
PUBLIC void     vApi_SetBcastTTL(uint8 u8MaxTTL);

PUBLIC teJenNetStatusCode eApi_LeaveNetwork(void);
PUBLIC teJenNetStatusCode eApi_RepairRoute(MAC_ExtAddr_s *psDestAddr);
PUBLIC teJenNetStatusCode eApi_PollForData(void);
PUBLIC teJenNetStatusCode eApi_SendDataToCoord(uint8 *pu8Payload, uint8 u8Length,
                    bool_t bGetResponse);
PUBLIC teJenNetStatusCode eApi_SendDataToPeer(MAC_ExtAddr_s *spDestAddr, uint8 *pu8Payload,
                    uint8 u8Length, bool_t bGetResponse);
PUBLIC teJenNetStatusCode eApi_SendDataToNwk(uint8 *pu8Payload, uint8 u8Length);
PUBLIC teJenNetStatusCode eApi_SendDataToExtNwk(MAC_Addr_s *psDestAddr, uint8 *pu8Payload,
                    uint8 u8Length);

PUBLIC void     vApi_SetAssociationState(bool_t bAssociate);
PUBLIC bool_t   bApi_GetAssociationState(void);

PUBLIC void     vApi_SetNetworkKey(uint32 *pu32Key);
PUBLIC uint32   u32Api_GetVersion(teJenNetComponent eComponent,tsVersionInfo* psVersionInfo);

#ifndef ROM_BUILD
PUBLIC bool_t   bApi_IsRouterLib(void);
#endif

PUBLIC void     vApi_SetPurgeRoute(bool_t bPurge);
PUBLIC void     vApi_SetImportRoute(bool_t bImport);

PUBLIC void     vApi_SetUserBeaconBits(uint8 *pu8Bits);
//PUBLIC uint32   u32Api_GetUserBeaconBits(void);

PUBLIC void     vApi_RegScanSortCallback(trSortScanCallback prCallback);
PUBLIC void     vApi_RegBeaconNotifyCallback(trBeaconNotifyCallback prCallback);
PUBLIC void     vApi_RegLocalAuthoriseCallback(trLocalAuthoriseCallback prCallback);
PUBLIC void     vApi_RegNwkAuthoriseCallback(trNwkAuthoriseCallback prCallback);
PUBLIC void     vApi_SetEndDeviceResumeCallback(trEndDeviceResumeCallback prNewHandler,
                                                trEndDeviceResumeCallback *prOldHandler);
PUBLIC void     vApi_RegPreSleepCallback(tvAboutToSleep pvCallback);

PUBLIC void     vApi_GetNetworkConfigBuffer(void  **ppvBuffer,
                                            uint32 *pu32BufferSize,
                                            void  **ppvChildBuffer,
                                            uint32 *pu32ChildBufferSize);

PUBLIC uint16   u16Api_GetRoutingTableSize(void);
PUBLIC bool_t   bApi_GetRoutingTableEntry(uint16 u16EntryNum,
                                          tsRoutingEntry *psRoutingEntry);
PUBLIC uint8    u8Api_GetNeighbourTableSize(void);
PUBLIC bool_t   bApi_GetNeighbourTableEntry(uint8 u8EntryNum,
                                            tsNeighbourEntry *psNeighbourEntry);
PUBLIC bool_t   bApi_ResetNeighbourStats(uint8 u8EntryNum);

PUBLIC void     vApp_Init(bool_t bWarmStart);
PUBLIC void     vApp_Main(void);

#ifdef SIX_LOWPAN_ONLY
PUBLIC void     v6LP_ProcessStackEvent(uint8 u8StackEvent, MAC_ExtAddr_s *psAddr,
                                       uint8 *au8Payload, uint8 u8Length);

#define vApp_StackEvent(A,B,C,D) v6LP_ProcessStackEvent((A),(B),(uint8 *)(C),(D))
#define vApp_HwEvent             v6LP_PeripheralEvent
#define vApp_ConfigureNetwork(A) v6LP_ConfigureNetwork((tsNetworkConfigData *)(A))
#else
PUBLIC void     vApp_StackEvent(uint8 u8StackEvent, MAC_ExtAddr_s *psAddr,
                                uint8 *au8Payload, uint8 u8Length);

PUBLIC void     vApp_HwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap);

PUBLIC void     vApp_ConfigureNetwork(tsJenNetConfigData *psJenNetConfigData);
#endif

PUBLIC void     vNwk_SetBeaconCalming(bool_t bState);
PUBLIC uint16   u16Api_GetDepth(void);
PUBLIC          tsJenNetConfigData *psApi_GetJenNetConfig(void);
PUBLIC tsChildren *psApi_GetChildTable(void);
PUBLIC void vAddRcvdDataStats(MAC_RxFrameData_s *psFrame);
PUBLIC bool_t  bCheckRxSeqNo(MAC_ExtAddr_s *psAddr, uint8 u8SeqNo);
PUBLIC uint8 u8Api_GetLastPktLqi(void); //PR633 & 1067
PUBLIC bool_t bApi_GetLastPktDirect(void); //lpsw2054
PUBLIC uint8    u8Api_GetStackState(void);      // PR1274
PUBLIC uint32 u32Api_GetDeviceId(void);
PUBLIC void vApi_SetDeviceId(uint32 u32DeviceId);
PUBLIC teJenNetStatusCode eApi_CommissionNode(MAC_ExtAddr_s *psDeviceAddr,tsSecurityKey *psSecurityKey);
PUBLIC void vApi_SetStackMode(uint16 u16ModeMask);
PUBLIC uint16 u16Api_GetStackMode(void);
PUBLIC void vApi_SetNwkKey(uint8 u8KeyIndex,tsSecurityKey *psSecurityKey);
PUBLIC tsSecurityKey *psApi_GetNwkKey(void);
PUBLIC void vApi_SkipJoin(uint16 u16PanId,uint8 u8Channel);
PUBLIC void vApi_EnableJoin(void);
PUBLIC void vApi_RegisterExtData(void *pvExtData);
PUBLIC bool_t bJnc_ChangeJoinProfile(uint8 u8ProfileIndex, uint8 u8NumBroadcasts, uint8 u8BroadcastPeriod);
PUBLIC bool_t bJnc_SetJoinProfile(uint8 u8ProfileIndex,const tsNwkProfile *psProfile);
PUBLIC bool_t bJnc_SetRunProfile(uint8 u8ProfileIndex,const tsNwkProfile *psProfile);
PUBLIC void vJnc_GetNwkProfile(tsNwkProfile *psProfile);
PUBLIC void vNed_SortScanList(void);    // lpsw2372
PUBLIC void vNcr_SortScanList(void);
PUBLIC uint8 u8GetCurJoinProfile(void);
PUBLIC uint8 u8GetCurRunProfile(void);
PUBLIC bool_t bApi_ResetStack(bool_t bKeepRouting,uint8 u8NewState);
PUBLIC void vApi_ReStartCR(bool_t bPanCoord);
PUBLIC void vApi_DeleteChild(MAC_ExtAddr_s *psAddr);
PUBLIC teJenNetStatusCode eApi_SendNetworkAnnounce(void);
PUBLIC teJenNetStatusCode eApi_SendNetworkAnnounceEnhanced(void);
PUBLIC teJenNetStatusCode eApi_SendLowEnergyInform(MAC_ExtAddr_s    *psAddr,
                                                   uint8            *pu8Key,
                                                   teLowEnergyStatus eStatus);
PUBLIC uint32 u32Jnc_GetNumberOfDevicesInBranch(void);
PUBLIC void vApi_ConfigureFastCommission(uint8 u8Channel, uint16 u16PanId);
PUBLIC tprSafeToSleepCallback prJnc_SetSafeToSleepCallback(tprSafeToSleepCallback prNewCallback);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* API_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
