/*****************************************************************************
 *
 * MODULE:              Jennic IP stack API
 *
 * DESCRIPTION:         The API for the Jennic IPv6 stack
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

#ifndef  SIXLP_H_INCLUDED
#define  SIXLP_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "jendefs.h"
#include "mac_sap.h"
#include "Api.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifndef PACK
#define PACK      __attribute__ ((packed))        /* align to byte boundary  */
#endif

/* Error codes returned directly by functions (incomplete list) */
#define E_WOULD_BLOCK (-2)

/* Normal sockets are in the range 0 to (u8Sockets - 1), where u8Sockets is
   defined during initialisation. The error socket is used as a special case
   for incoming packets that need to be communicated but for which no valid
   socket has been found */
#define SIXLP_ERROR_SOCKET (-2)

#define SIXLP_DEFAULT_MTU_SIZE    (1280)

/* Support for legacy API function names and enumerations */
#ifdef LEGACY_SUPPORT
#define i6LP_GatewaySendTo                       i6LP_IpSendTo
#define i6LP_GatewayRecvFrom                     i6LP_IpRecvFrom
#define v6LP_setPacketDefragTimeOut              v6LP_SetPacketDefragTimeout
#define u16_6LP_GetSizeOfIPv6record              u16_6LP_GetSizeOfIPv6Record
#define iEUI_GenerateInterfaceIdFrom64           i6LP_CreateInterfaceIdFrom64
#define iEUI_GenerateLinkLocalAddress            i6LP_CreateLinkLocalAddress
#define iEUI_GetDeviceAddress                    i6LP_GetOwnDeviceAddress
#define e6LP_getLinkLayer                        e6LP_GetLinkLayer
#define e6LP_getDeviceType                       e6LP_GetDeviceType
#define v6LP_setDefaultHopCount                  v6LP_SetDefaultMaxHopCount
#define v6LP_setPacketDefragTimeOut              v6LP_SetPacketDefragTimeout
#define v6LP_setAllowUnrecognisedNextHeaderFlag  v6LP_SetAllowUnrecognisedNextHeaderFlag
#define bo6LP_getAllowUnrecognisedNextHeaderFlag b6LP_GetAllowUnrecognisedNextHeaderFlag
#define bo6LP_setMaximumAllowableMTU             b6LP_SetMaxAllowableMTU
#define u16_6LP_getMaximumAllowableMTU           u16_6LP_GetMaxAllowableMTU
#define u16_6LP_getRouteOverIPstatus             b6LP_GetRouteOverIPstatus
#define v6LP_setRouteOverIPstatus                v6LP_SetRouteOverIPstatus
#define E_GATEWAY_DATA_RECEIVED                  E_IP_DATA_RECEIVED
#define E_6LP_SOCKET_GATEWAY_SENDTO_FAIL         E_6LP_SOCKET_IP_SENDTO_FAIL
#define E_6LP_SOCKET_GATEWAY_RECV_FROM_FAIL      E_6LP_SOCKET_IP_RECV_FROM_FAIL
#define E_6LP_GATEWAY_SENDTO_FAIL                E_6LP_IP_SENDTO_FAIL
#define E_6LP_GATEWAY_RECV_FROM_FAIL             E_6LP_IP_RECV_FROM_FAIL

#endif

/* For library build, map function names to default names */
#ifdef LIBRARY_BUILD
#define i6LP_InitStack                 i6LP_DefInitStack
#define v6LP_InitHardware              v6LP_DefInitHardware
#define i6LP_Socket                    i6LP_DefSocket
#define i6LP_Bind                      i6LP_DefBind
#define v6LP_Sleep                     v6LP_DefSleep
#define i6LP_ResumeStack               i6LP_DefResumeStack
#define i6LP_Close                     i6LP_DefClose
#define e6LP_Poll                      e6LP_DefPoll
#define u32_6LP_GetErrNo               u32_6LP_DefGetErrNo
#define i6LP_SendTo                    i6LP_DefSendTo
#define i6LP_RecvFrom                  i6LP_DefRecvFrom
#define i6LP_IpSendTo                  i6LP_DefIpSendTo
#define i6LP_IpRecvFrom                i6LP_DefIpRecvFrom
#define i6LP_GetDataBuffer             i6LP_DefGetDataBuffer
#define v6LP_SetPacketDefragTimeout    v6LP_DefSetPacketDefragTimeout
#define u16_6LP_GetSizeOfIPv6Record    u16_6LP_DefGetSizeOfIPv6Record
#define u16_6LP_GetNumberOfIPv6Buffers u16_6LP_DefGetNumberOfIPv6Buffers
#define i6LP_GetNextPacketSize         i6LP_DefGetNextPacketSize
#define i6LP_CreateInterfaceIdFrom64   i6LP_DefCreateInterfaceIdFrom64
#define i6LP_CreateInterfaceIdFrom16   i6LP_DefCreateInterfaceIdFrom16
#define i6LP_CreateLinkLocalAddress    i6LP_DefCreateLinkLocalAddress
#define i6LP_GetOwnDeviceAddress       i6LP_DefGetOwnDeviceAddress
#endif

/* Following definitions are for building the ROM core library. They should
   not be used for user applications */
#ifdef ROM_CORE_LIBRARY
#define v6LP_ConfigureNetwork v6LP_ConfigureNetworkHook
#define v6LP_PeripheralEvent  v6LP_PeripheralEventHook
#define v6LP_StackEvent       v6LP_StackEventHook
#define v6LP_DataEvent        v6LP_DataEventHook
#endif

/* Flags to OR together to pass to i6LP_RecvFrom */
#define SIXLP_MSG_PEEK          (2)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/* Socket address */
typedef struct
{
    union {
        uint8  u6_addr8[16];
        uint16 u6_addr16[8];
        uint32 u6_addr32[4];
    } in6_u;

    #define s6_addr                 in6_u.u6_addr8
    #define s6_addr16               in6_u.u6_addr16
    #define s6_addr32               in6_u.u6_addr32

} in6_addr;

typedef uint16 in_port_t;
typedef uint8 socklen_t;

typedef struct
{
    int       sin6_family;       /* E_6LP_PF_INET6 */
    in_port_t sin6_port;         /* Transport layer port # */
    uint32    sin6_flowinfo;     /* IPv6 flow information */
    in6_addr  sin6_addr;         /* IPv6 address */
    uint32    sin6_scope_id;     /* IPv6 scope-id */
} ts6LP_SockAddr;


/* Stack events, as passed up to callback */
typedef enum PACK
{
    E_STACK_STARTED,         /* Coordinator has started (only seen at coordinator) */
    E_STACK_JOINED,          /* This end device has joined (see Api.h: tsNwkInfo) */
    E_STACK_NODE_JOINED,     /* A device has joined this parent */
    E_STACK_NODE_LEFT,       /* A device has left this parent */
    E_STACK_TABLES_RESET,    /* Coordinator has reset tables (e.g. associated devices table); called as part of initialisation */
    E_STACK_RESET,           /* The networking stack has decided to reset itself */
    E_STACK_POLL,            /* A deferred poll response has arrived (see te6LP_PollResponse) */
    E_STACK_NODE_JOINED_NWK, /* A device has joined the network (only seen at coordinator) */
    E_STACK_NODE_LEFT_NWK,   /* A device has left the network (only seen at coordinator) */
    E_STACK_NODE_AUTHORISE,  /* A device is trying to join but needs key */
    E_STACK_ROUTE_CHANGE,    /* A device has moved (only seen at coordinator) */
    E_STACK_GROUP_CHANGE,    /* One or more multicast addresses have been added or removed from network (see ts6LP_StackGroupChange) */
    E_STACK_NETWORK_ANNOUNCE,/* A network has started and sent out an event */
    E_STACK_LOW_ENERGY_SEEN, /* Possible low energy frame has been seen in the network */
} te6LP_StackEvent;

typedef enum PACK
{
    E_6LP_POLL_NO_DATA = 0, /* Poll complete but no data pending */
    E_6LP_POLL_DATA_READY,  /* Poll complete and data received */
    E_6LP_POLL_TIMEOUT,     /* Poll timed out, no data received */
    E_6LP_POLL_ERROR,       /* Problem with request */
    E_6LP_POLL_PENDING      /* Request accepted but not complete yet */
} te6LP_PollResponse;

/* Structure passed with E_STACK_GROUP_CHANGE event, when the set of multicast
   addresses registered on a node has been added to or removed from. The event
   is generated on the coordinator for all changes within the wireless network
   and is also triggered on the node itself */
typedef struct
{
    MAC_ExtAddr_s  sDeviceAddress;   /* MAC address of device on which change
                                        has occurred. Address is all 0s if it
                                        is this device */
    in6_addr      *psAddressList;    /* List of multicast addresses */
    uint8          u8AddressListLen; /* Number of entries in list */
    bool_t         bAddNotRemove;    /* TRUE if addresses were added, FALSE if
                                        they were removed */
} ts6LP_StackGroupChange;

/* Data events, as passed up to callback */
typedef enum PACK
{
    E_DATA_SENT,            /* Datagram sent successfully */
    E_DATA_SEND_FAILED,     /* Datagram send failed (see u32_6LP_GetErrNo) */
    E_DATA_RECEIVED,        /* Datagram received */
    E_IP_DATA_RECEIVED,     /* IP packet received at border router, to pass on */
    E_6LP_ICMP_MESSAGE      /* ICMP message passed up to the user */
} te6LP_DataEvent;


/* Device types */
typedef enum PACK
{
    E_6LP_NULL_TYPE,    /* Not a valid choice, used prior to initialisation */
    E_6LP_COORDINATOR,
    E_6LP_ROUTER,       /* Router is not supported by Star (E_LINKLAYER_MAC) */
    E_6LP_END_DEVICE
} te6LP_DeviceType;


/* Protocol values: only one is supported */
typedef enum PACK
{
    E_6LP_PF_INET6
} te6LP_ProtocolFamily;

typedef enum PACK
{
    E_6LP_SOCK_DGRAM,   /* UDP */
#ifdef TCP_MESSAGING
    E_6LP_SOCK_STREAM   /* TCP */
#endif
} te6LP_ProtocolType;

typedef enum PACK
{
    E_6LP_PROTOCOL_ONLY_ONE
} te6LP_Protocol;

/* Error codes, appear as bits 0-7 of value returned by u32_6LP_GetErrNo */
typedef enum PACK
{
    E_6LP_NETWORK_FORMATION =1,
    E_6LP_6LP_BUILD,         // 2
    E_6LP_SEND_FAIL,         // 3
    E_6LP_ICMP_BUILD,        // 4
    E_6LP_COMPRESS_FAIL,     // 5
    E_6LP_CHECKSUM_FAIL,     // 6
    E_6LP_TRANSMIT_FAIL,     // 7
    E_6LP_BITSTREAM_FAIL,    // 8
    E_6LP_IPV6_BUFFER_FAIL,  // 9
    E_6LP_IPV6_HEADER_BUILD, // 0xa
    E_6LP_IPV6_HEADER_INFO,  // 0xb
    E_6LP_PING_FAIL,         // 0xc
    E_6LP_RX_FAIL,           // 0xd
    E_6LP_RX_DEFRAG_TIMER_FAIL, // 0xe
    E_6LP_SOCKET_FAIL,          // 0xf
    E_6LP_SOCKET_BIND_FAIL,     // 0x10
    E_6LP_SOCKET_SENDTO_FAIL,   // 0x11
    E_6LP_SOCKET_GET_NEXT_PACKET_FAIL, // 0x12
    E_6LP_SOCKET_IP_SENDTO_FAIL,       // 0x13
    E_6LP_SOCKET_RECV_FROM_FAIL,       // 0x14
    E_6LP_SOCKET_IP_RECV_FROM_FAIL,    // 0x15
    E_6LP_SOCKET_CLOSE,                // 0x16
    E_6LP_SOCKET_COMPRESSION_OPT_FAIL, // 0x17
    E_6LP_SOCKET_SET_NEXT_HEADER_FAIL, // 0x18
    E_6LP_SOCKET_EVT_HANDLER,    // 0x19
    E_6LP_IP_SENDTO_FAIL,        // 0x1a
    E_6LP_IP_RECV_FROM_FAIL,      // 0x1b
    E_6LP_ND_FAIL
} te6LP_ErrorCode;

/* Error info, appear as bits 15-8 of value returned by u32_6LP_GetErrNo */
typedef enum PACK
{
    I_6LP_NONE  = 0,
    I_6LP_PARAM,                          // 0x01
    I_6LP_START_FAIL,                     // 0x02
    I_6LP_MTU_ERROR,                      // 0x03
    I_6LP_BUFFER_UNKNOWN,                 // 0x04
    I_6LP_DEVICE_UNKNOWN,                 // 0x05
    I_6LP_SOCKET_PROTOCOL_UNSUPPORTED,    // 0x06
    I_6LP_PACKET_TOO_LARGE,               // 0x07
    I_6LP_NO_TCP_HANDLER,                 // 0x08
    I_6LP_TCP_ERROR,                      // 0x09
    I_6LP_NEXT_HEADER_UNSUPPORTED,        // 0x0a
    I_6LP_BUFFER_TOO_SMALL,               // 0x0b
    I_6LP_BUFFER_RECOVERY,                // 0x0c
    I_6LP_BUFFER_NONE,                    // 0x0d
    I_6LP_PING_ID_ERROR,                  // 0x0e
    I_6LP_PING_TIMER_FAIL,                // 0x0f
    I_6LP_PING_NO_FREE_SERVER_INSTANCES,  // 0x10
    I_6LP_PING_NO_FREE_BUFFERS,           // 0x11
    I_6LP_PING_SEND_PING_FAILED,          // 0x12
    I_6LP_FRAME_FAIL,                     // 0x13
    I_6LP_MESH_CB_FAIL,                   // 0x14
    I_6LP_BCAST_CB_FAIL,                  // 0x15
    I_6LP_IPV6_HEADER_FAIL,               // 0x16
    I_6LP_6LP_FRAGMENT_FAIL,              // 0x17
    I_6LP_SOCKETS_NONAVAILABLE,           // 0x18
    I_6LP_SOCKET_NOT_FOUND,               // 0x19
    I_6LP_SOCKET_RANGE,                   // 0x1a
    I_6LP_SOCKET_FAMILY,                  // 0x1b
    I_6LP_DEST_PREFIX_MISMATCH,           // 0x1c
    I_6LP_NO_COMP_OPTIONS,                // 0x1d
    I_6LP_ND_NO_FREE_BUFFERS,             // 0x1e
    I_6LP_ND_TIMER_FAIL,                  // 0x1f
    I_6LP_CHANNEL_INVALID,                // 0x20
    I_6LP_FORMATION_UNSUPPORTED,          // 0x21
    I_6LP_INTERNAL_1 = 0x80,              // 0x80
    I_6LP_INTERNAL_2,                     // 0x81
    I_6LP_INTERNAL_3,                     // 0x82
    I_6LP_INTERNAL_4,                     // 0x83
    I_6LP_INTERNAL_5,                     // 0x84
    I_6LP_INTERNAL_6,                     // 0x85
    I_6LP_INTERNAL_7,                     // 0x86
    I_6LP_INTERNAL_8                      // 0x87
} te6LP_ErrorInfo;

typedef enum PACK
{
    E_LINKLAYER_UNDEFINED,                   /* Not defined yet */
    E_LINKLAYER_MAC,                         /* Star, on top of 802.15.4 MAC */
    E_LINKLAYER_JENNET                       /* Tree, on top of JenNet */
} te6LP_LinkLayer;

typedef  struct EUI64_s_Tag
{
    uint32 u32L;  /**< Low word */
    uint32 u32H;  /**< High word */
} EUI64_s;


/* Used to track an association between extended address and short address */
typedef struct
{
    MAC_ExtAddr_s sMacAddr;
    uint32        u32DeviceClass;
    uint16        u16NetworkAddr;
} tsAssocNodeInfo;


/* Stack initialisation data */
typedef struct
{
    uint64           u64AddressPrefix;       /* IPv6 address prefix (C only) */
    uint32           u32Channel;             /* Channel 'bitmap' */
    uint16           u16PanId;               /* PAN ID to use or look for (0xffff to search/generate) */
    uint16           u16maxMTU;              /* max MTU size, 0 will default to 1280 */
    uint16           u16NumPacketBuffers;    /* Number of individual 6LP packet
                                                buffers */
    uint8            u8Sockets;              /* Number of sockets supported */

    te6LP_DeviceType eDeviceType;            /* Device type (C, R, or ED) */

    /* Following is only used in combined library and ROM */
    te6LP_LinkLayer  eLinkLayer;             /* Link layer (15.4, JenNet...) */

    union
    {
        struct
        {
#ifndef JENNIC_CHIP_JN5139T01
            /* Following fields added by Daniel Gavelle Jan 2010.
             * Only used in Star  (E_LINKLAYER_MAC) networks end device. */
            uint16          u16ShortAddres;         /* Our short address if bDoAssociate is false */
            bool_t          bDoBeaconScan;          /* Do an active scan for beacons.
                                                       If FALSE, skip E_6LP_STATE_SCANNING.
                                                       Uses lowest channel bit in  u32Channel which should only have one bit set.
                                                       Coord will have fixed 16 bit of SIXLP_COORD_ADDR */

            bool_t          bDoAssociate;           /* Do an associate and get short addr from response.
                                                       If FALSE skip E_6LP_STATE_ASSOCIATING */

            /* End of additions for skipping scan / associate */
#endif

            /* Following is only used in Star (E_LINKLAYER_MAC) networks, on coordinators */
            uint16           u16MaxAssociations;     /* Number of associations to accept */
        } sStar;

        struct
        {
            /* Following are only used in Tree (E_LINKLAYER_JENNET) networks */
            uint32           u32RoutingTableEntries; /* Routing table size (not ED) */
        } sTree;
    } uLinkLayer;

} tsStackInitData;


typedef void (*tprDataEventHandler)(int iSocket,
                                   te6LP_DataEvent eEvent,
                                   ts6LP_SockAddr *psAddr,
                                   uint8 u8AddrLen);

/* Following can be used to adjust configuration of JenNet in 
   v6LP_ConfigureNetwork callback. Definition used to be copied here from 
   JenNet/Include/Api.h, to avoid having a direct dependency on JenNet for 
   6LP builds. To avoid potential mismatches the original structure is now 
   referenced directly and JenNet/Include/Api.h should be referred to for the
   definition */
typedef tsJenNetConfigData tsNetworkConfigData;


typedef bool_t (*tprNwkCallback)(MAC_ExtAddr_s *psAddr,
                                 uint8 u8DataLength,
                                 uint8 *pu8Data);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/* Error handling */
extern PUBLIC uint32 u32_6LP_GetErrNo(void);

/* Network formation */
extern PUBLIC void v6LP_InitHardware(void);
extern PUBLIC int  i6LP_InitStack   (tsStackInitData *psStackInitData);
/* Following are used in Tree (E_LINKLAYER_JENNET) networks only, and only
   with end devices */
extern PUBLIC int  i6LP_ResumeStack(void);
extern PUBLIC void v6LP_Sleep(bool_t bMemoryHold, uint32 u32SleepPeriodInMs);

/* Socket formation */
extern PUBLIC int i6LP_Socket(te6LP_ProtocolFamily,
                              te6LP_ProtocolType,
                              te6LP_Protocol);

extern PUBLIC int i6LP_Close (int iSocket);

extern PUBLIC int i6LP_Bind  (int iSocket,
                              ts6LP_SockAddr *psAddr,
                              uint8 u8AddrLen);

extern PUBLIC int i6LP_SetSocketDataEventHandler(int iSocket,
                                 tprDataEventHandler prDataEventHandler);

/* Buffer management */
extern PUBLIC int    i6LP_GetDataBuffer(uint8 **ppu8dataBuffer);
extern PUBLIC uint16 u16_6LP_GetSizeOfIPv6Record(void);
extern PUBLIC uint16 u16_6LP_GetNumberOfIPv6Buffers(void);
extern PUBLIC uint16 u16_6LP_GetNumberOfAvailableIPv6Buffers(void);
extern PUBLIC int    i6LP_GetNextPacketSize(int iSocket, uint16 *pu16PacketSize);

/* Data transfer */
extern PUBLIC int i6LP_SendTo  (int iSocket,
                                uint8 *pu8Data,
                                uint16 u16DataLen,
                                uint32 u32Flags,
                                ts6LP_SockAddr *psDestAddr,
                                uint8 u8AddrLen);

extern PUBLIC int i6LP_RecvFrom(int iSocket,
                                uint8 *pu8Data,
                                uint16 u16DataLen,
                                uint32 u32Flags,
                                ts6LP_SockAddr *psSrcAddr,
                                uint8 *pu8AddrLen);

/* Data transfer: end devices on Tree only */
extern PUBLIC te6LP_PollResponse e6LP_Poll(void);

/* IP (Boundary Router) data transfer */
extern PUBLIC int i6LP_IpSendTo(uint8 *pu8Data);

extern PUBLIC int i6LP_IpRecvFrom(uint8 *pu8Data, uint16 u16DataLen);

/* Regular call to give 6LP processor time */
PUBLIC void v6LP_Tick(void);

/* Miscellaneous configuration */
PUBLIC int i6LP_SetSocketNextHeader(int iSocket, uint8 u8nextHeader);
PUBLIC void v6LP_SetAllowUnrecognisedNextHeaderFlag(bool_t bAllowUnrecognisedNextHeader);
PUBLIC bool_t b6LP_GetAllowUnrecognisedNextHeaderFlag(void);

PUBLIC int i6LP_GetSocketLastDestination(int iSocket, in6_addr *psLastDestAddr);
PUBLIC int i6LP_SocketAddGroupAddr(int iSocket, in6_addr *psNewAddr);
PUBLIC int i6LP_SocketRemoveGroupAddr(int iSocket, in6_addr *psAddr);
PUBLIC int i6LP_SocketListGroupAddr(int iSocket, uint8 u8Index, in6_addr *psAddr);
PUBLIC int i6LP_SocketSetMulticastLoop(int iSocket, bool_t bAllowLoop);

PUBLIC te6LP_LinkLayer e6LP_GetLinkLayer(void);
PUBLIC te6LP_DeviceType e6LP_GetDeviceType(void);

PUBLIC void v6LP_SetPacketDefragTimeout(uint8 u8userTimeoutInSeconds);

PUBLIC void v6LP_SetDefaultMaxHopCount(uint8 u8userDefaultHopCount);

PUBLIC bool_t b6LP_SetMaxAllowableMTU(uint16 u16maxMTU);
PUBLIC uint16 u16_6LP_GetMaxAllowableMTU(void);

PUBLIC bool_t b6LP_GetRouteOverIPstatus(void);
PUBLIC void v6LP_SetRouteOverIPstatus(bool_t bRouteOverIP);

PUBLIC void v6LP_EnableSecurity(void);

/* Sets user data passed with JenNet Establish Route messages. Length limited
   to 16 bytes */
PUBLIC void v6LP_SetUserData(uint8 u8DataLength, uint8 *pu8Data);

/* Declare callback used during establish route process. Return from callback
   of TRUE is necessary to allow joining node to succeed */
PUBLIC void v6LP_SetNwkCallback(tprNwkCallback prCallback);

/****************************************************************************/
/***        Callback Functions                                            ***/
/****************************************************************************/

extern PUBLIC void v6LP_PeripheralEvent(uint32 u32Device,
                                        uint32 u32ItemBitmap);

extern PUBLIC void v6LP_StackEvent     (te6LP_StackEvent eEvent,
                                        uint8 *pu8Data,
                                        uint8 u8DataLen);

extern PUBLIC void v6LP_DataEvent      (int iSocket,
                                        te6LP_DataEvent,
                                        ts6LP_SockAddr *psAddr,
                                        uint8 u8AddrLen);

extern PUBLIC void v6LP_ConfigureNetwork(tsNetworkConfigData *psNetworkConfigData);

/****************************************************************************/
/***        Association Functions for parent devices                      ***/
/****************************************************************************/
/* Following are used by Star (E_LINKLAYER_MAC) networks only */
extern PUBLIC tsAssocNodeInfo *ps6LP_AssocGetNodeInfo(uint32 u32TableIndex);
extern PUBLIC uint32 u32_6LP_AssocGetTotalEntries(void);
extern PUBLIC uint32 u32_6LP_AssocGetMaxEntries(void);
extern PUBLIC int i6LP_AssocAddEntry            (tsAssocNodeInfo *psNewEntry);
extern PUBLIC int i6LP_AssocRemoveEntry         (uint32 u32TableIndex);
extern PUBLIC uint32 u32_6LP_AssocFindMacAddr    (uint16 u16NetworkAddr,
                                                 MAC_ExtAddr_s *psMacAddr);
extern PUBLIC uint32 u32_6LP_AssocFindNetworkAddr(MAC_ExtAddr_s *psMacAddr,
                                                 uint16 *pu16NetworkAddr);

/****************************************************************************/
/***        Helper Functions                                              ***/
/****************************************************************************/

extern PUBLIC int i6LP_CreateInterfaceIdFrom64(EUI64_s *psDeviceInterfaceId,
                                               EUI64_s *psDeviceEUI64);
extern PUBLIC int i6LP_CreateInterfaceIdFrom16(EUI64_s *psDeviceInterfaceId,
                                               uint16 u16PanId, uint16 u16ShortAddr);
extern PUBLIC int i6LP_CreateLinkLocalAddress (in6_addr *psDeviceLLaddress,
                                               EUI64_s *psDeviceInterfaceId);
extern PUBLIC int i6LP_GetOwnDeviceAddress(ts6LP_SockAddr *psDeviceAddress,
                                           bool_t bUseGlobal);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
extern PUBLIC bool_t b6LP_AlwaysBroadcast;

#if defined __cplusplus
}
#endif

#endif  /* SIXLP_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
