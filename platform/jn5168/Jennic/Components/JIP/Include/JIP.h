/****************************************************************************
 *
 * MODULE:             JIP: API
 *
 * DESCRIPTION:        Main functional API for the JIP
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

#ifndef JIP_H_
#define JIP_H_

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "6LP.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#define JIP_MAX_WATCHERS (0xff)

#define JIP_DEFAULT_PORT (1873)

#define WEAK              __attribute__ ((weak))

#define JIP_START_DECLARE_MIB(MIB, NAME)    ts##MIB##Mib_Inst s##NAME##Mib = { .sMib = {NULL, #NAME, &s##MIB##MibDef.sMib, 0},
#define JIP_CALLBACK(VAR, GET, SET, DATA) .sVar##VAR = { GET, SET, {DATA, TRUE} },
#define JIP_END_DECLARE_MIB(NAME, HANDLE) }; static const thJIP_Mib HANDLE = &s##NAME##Mib.sMib;

#define JIP_EXTERN_MIB(NAME, HANDLE) extern tsJIP_MibInst s##NAME##Mib; static const thJIP_Mib HANDLE = &s##NAME##Mib;

#define VAR_ID(MIB, VAR) s##MIB##MibDef.sVar##VAR.u8VarID

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef enum PACK
{
    E_JIP_DEVICE_COORDINATOR = E_6LP_COORDINATOR,
    E_JIP_DEVICE_ROUTER      = E_6LP_ROUTER,
    E_JIP_DEVICE_END_DEVICE  = E_6LP_END_DEVICE
} teJIP_Device;

typedef struct
{
    uint64        u64AddressPrefix;       /* IPv6 address prefix (C only) */
    uint32        u32Channel;             /* Channel 'bitmap' */
    uint16        u16PanId;               /* PAN ID to use or look for (0xffff to search/generate) */
    uint16        u16MaxIpPacketSize;     /* Max IP packet size, 0 defaults to 1280 */
    uint16        u16NumPacketBuffers;    /* Number of IP packet buffers */
    uint8         u8UdpSockets;           /* Number of UDP sockets supported */
    teJIP_Device eDeviceType;            /* Device type (C, R, or ED) */
    uint32        u32RoutingTableEntries; /* Routing table size (not ED) */

    uint32        u32DeviceId;
    uint8         u8UniqueWatchers;
    uint8         u8MaxTraps;
    uint8         u8QueueLength;
    uint8         u8MaxNameLength;
    uint16        u16Port;
    const char   *pcVersion;
} tsJIP_InitData;

/** Structure that is passed to application with a read table request.
 *  JIP fills in the requested starting index, the app fills in the other fields
 */
typedef struct
{
    uint16  u16FirstEntry;          /**< Index of the first requested entry (populated by JIP) */
    uint8   u8EntryCount;           /**< Number of entries requested (populated by JIP) */
    uint16  u16RemainingEntries;    /**< Number of entries in the table after the last entry added to packet (populated by application) */
    uint16  u16TableVersion;        /**< Table version - used to detect inconsistencies between multiple table read packets (populated by application) */
} tsJIP_TableData;

enum _eJIP_Status
{
    E_JIP_OK                        = 0x00,
    E_JIP_ERROR_TIMEOUT             = 0x7f,
    E_JIP_ERROR_BAD_MIB_INDEX       = 0x8f,
    E_JIP_ERROR_BAD_VAR_INDEX       = 0x9f,
    E_JIP_ERROR_NO_ACCESS           = 0xaf,
    E_JIP_ERROR_BAD_BUFFER_SIZE     = 0xbf,
    E_JIP_ERROR_WRONG_TYPE          = 0xcf,
    E_JIP_ERROR_BAD_VALUE           = 0xdf,
    E_JIP_ERROR_DISABLED            = 0xef,
    E_JIP_ERROR_FAILED              = 0xff
} PACK;
#ifdef WIN32
typedef uint8 teJIP_Status;
#else
typedef enum _eJIP_Status teJIP_Status;
#endif

enum _eJIP_VarType
{
#define E_JIP_VAR_TYPE_TABLE 64
    E_JIP_VAR_TYPE_INT8,
    E_JIP_VAR_TYPE_INT16,
    E_JIP_VAR_TYPE_INT32,
    E_JIP_VAR_TYPE_INT64,
    E_JIP_VAR_TYPE_UINT8,
    E_JIP_VAR_TYPE_UINT16,
    E_JIP_VAR_TYPE_UINT32,
    E_JIP_VAR_TYPE_UINT64,
    E_JIP_VAR_TYPE_FLOAT,
    E_JIP_VAR_TYPE_DOUBLE,
    E_JIP_VAR_TYPE_STRING,
    E_JIP_VAR_TYPE_BLOB,

    E_JIP_VAR_TYPE_TABLE_BLOB = E_JIP_VAR_TYPE_TABLE + E_JIP_VAR_TYPE_BLOB,
} PACK;
#ifdef WIN32
typedef uint8 teJIP_VarType;
#else
typedef enum _eJIP_VarType teJIP_VarType;
#endif

enum _eJIP_AccessType
{
    E_JIP_ACCESS_TYPE_CONST,
    E_JIP_ACCESS_TYPE_READ_ONLY,
    E_JIP_ACCESS_TYPE_READ_WRITE,

} PACK;
#ifdef WIN32
typedef uint8 teJIP_AccessType;
#else
typedef enum _eJIP_AccessType teJIP_AccessType;
#endif

enum _eJIP_Security
{
    E_JIP_SECURITY_NONE,

} PACK;
#ifdef WIN32
typedef uint8 teJIP_Security;
#else
typedef enum _eJIP_Security teJIP_Security;
#endif

enum _eJIP_GroupEvent
{
    E_JIP_GROUP_JOIN,
    E_JIP_GROUP_LEAVE
} PACK;
#ifdef WIN32
typedef uint8 teJIP_GroupEvent;
#else
typedef enum _eJIP_GroupEvent teJIP_GroupEvent;
#endif

typedef struct sTag_tsJIP_Var *thJIP_Var;
typedef struct sTag_tsJIP_Mib *thJIP_Mib;
typedef struct _tsJIP_Packet *thJIP_Packet;

typedef struct sTag_tsJIP_Static *thJIP_Static;
typedef struct sTag_tsJIP_VarDynamic *thJIP_Functions;

typedef struct
{
    uint32          u32MibID;
    uint8           u8MibIndex;
    const char     *pcName;
} tsJIP_QueryMibResponse;

typedef struct
{
    uint8            u8VarIndex;
    const char      *pcName;
    teJIP_VarType    eVarType;
    teJIP_AccessType eAccessType;
    teJIP_Security   eSecurity;
} tsJIP_QueryVarResponse;

typedef bool_t  (*tprJIP_Set)(const void   *pvVal,
                               uint32           u32ValSize,
                               void         *pvCbData);

typedef void    (*tprJIP_Get)(const void   **ppvOutVal,
                               uint32       *pu32OutValSize,
                               void         *pvCbData);

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

PUBLIC teJIP_Status eJIP_Init(tsJIP_InitData *psInitData);

PUBLIC void vJIP_Tick(void);

PUBLIC teJIP_Status eJIP_RegisterMib(thJIP_Mib phMib);

PUBLIC void vJIP_SetEnabled(thJIP_Mib phMib, uint8 u8VarID, bool_t bEnabled);
PUBLIC void vJIP_NotifyChanged(thJIP_Mib phMib, uint8 u8VarID);
PUBLIC teJIP_Status eJIP_AddTrap(thJIP_Mib phMib, uint8 u8VarID, ts6LP_SockAddr *psAddr, uint8 u8Handle);
PUBLIC teJIP_Status eJIP_RemoveTrap(thJIP_Mib phMib, uint8 u8VarID, ts6LP_SockAddr *psAddr);

PUBLIC void vJIP_SuspendTrapNotifies(void);
PUBLIC void vJIP_ResumeTrapNotifies(void);

/* Default MIB access */
PUBLIC void vJIP_SetDeviceTypes(uint8 u8Entries, uint16 *pu16DeviceTypeList);
PUBLIC void vJIP_SetNodeName(char *pcName);
PUBLIC void vJIP_RegisterSetNameCallback(void (*prSetNameCallback)(char *pcName));

/* remote variables */

#define eJIP_Remote_Mib_Set(ADDR, HANDLE, ID, VAR, TYPE, VAL, SIZE) eJIP_Remote_ID_Set(ADDR, HANDLE, ID, VAR, TYPE, VAL, SIZE, FALSE);
#define eJIP_Remote_Set(ADDR, HANDLE, ID, VAR, TYPE, VAL, SIZE) eJIP_Remote_ID_Set(ADDR, HANDLE, ID, VAR, TYPE, VAL, SIZE, TRUE);

PUBLIC teJIP_Status eJIP_Remote_ID_Set(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint32 u32MibID,
        uint8 u8VarIndex,
        teJIP_VarType eVarType,
        void *pvVal,
        uint32 u32ValSize,
        bool_t bUseMibIndex);

#define eJIP_Remote_Get(ADDR, HANDLE, MIB, VAR) eJIP_Remote_TableGet(ADDR, HANDLE, MIB, VAR, 0, 255, TRUE);
#define eJIP_Remote_Mib_Get(ADDR, HANDLE, MIB, VAR) eJIP_Remote_TableGet(ADDR, HANDLE, MIB, VAR, 0, 255, FALSE);

PUBLIC teJIP_Status eJIP_Remote_TableGet(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint32 u32MibID,
        uint8 u8VarIndex,
        uint16 u16FirstEntry,
        uint8 u8EntryCount,
        bool_t bUseMibIndex);

PUBLIC teJIP_Status eJIP_Remote_Trap(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8NotificationHandle,
        uint8 u8MIBIndex,
        uint8 u8VarIndex);

PUBLIC teJIP_Status eJIP_Remote_Untrap(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarIndex);

PUBLIC teJIP_Status eJIP_Remote_QueryMib(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibStartIndex,
        uint8 u8NumMibs);

PUBLIC teJIP_Status eJIP_Remote_QueryVar(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarStartIndex,
        uint8 u8NumVars);

/* callbacks - responses to the above */
PUBLIC WEAK void vJIP_Remote_SetResponse(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarIndex,
        teJIP_Status eStatus);

PUBLIC WEAK void vJIP_Remote_GetResponse(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarIndex,
        teJIP_Status eStatus,
        teJIP_VarType eVarType,
        const void *pvVal,
        uint32 u32ValSize);

PUBLIC WEAK void vJIP_Remote_TableGetResponse(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarIndex,
        teJIP_Status eStatus,
        teJIP_VarType eVarType,

        uint16 u16Entry,
        uint16 u16Remaining,
        uint8 u8PacketRemaining,

        const void *pvVal,
        uint32 u32ValSize);

PUBLIC WEAK void vJIP_Remote_TrapResponse(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarIndex,
        teJIP_Status eStatus);

PUBLIC WEAK void vJIP_Remote_TrapNotify(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        uint8 u8VarIndex,
        teJIP_Status eStatus,
        teJIP_VarType eVarType,
        void *pvVal,
        uint32 u32ValSize);

PUBLIC WEAK void vJIP_Remote_QueryMibResponse(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        teJIP_Status eStatus,
        uint16 u16MibsOutstanding,
        uint8 u8MibsRemaining,
        tsJIP_QueryMibResponse *psMib);

PUBLIC WEAK void vJIP_Remote_QueryVarResponse(ts6LP_SockAddr *psAddr,
        uint8 u8Handle,
        uint8 u8MibIndex,
        teJIP_Status eStatus,
        uint16 u16VarsOutstanding,
        uint8 u8VarsRemaining,
        tsJIP_QueryVarResponse *psVar);

PUBLIC WEAK void vJIP_Remote_DataSent(ts6LP_SockAddr *psAddr,
        teJIP_Status eStatus);

PUBLIC WEAK bool_t bJIP_GroupCallback(teJIP_GroupEvent eEvent,
        in6_addr *psAddr);

PUBLIC WEAK void vJIP_StayAwakeRequest(void);

PUBLIC teJIP_Status eJIP_PacketAddData(thJIP_Packet hHandle,
        void *pvData,
        uint32 u32Len,
        uint32 u32Entry);

PUBLIC uint32 u32JIP_GetDeviceId(void);

PUBLIC bool_t bJIP_JoinGroup(uint16 u16GroupID);
PUBLIC bool_t bJIP_LeaveGroup(uint16 u16GroupID);

PUBLIC bool_t bJIP_AddGroupAddr(in6_addr *psAddr);
PUBLIC bool_t bJIP_RemoveGroupAddr(in6_addr *psAddr);

PUBLIC int iJIP_GetLastDestinationAddr(ts6LP_SockAddr *psAddr);
PUBLIC int iJIP_GetLastSourceAddr(ts6LP_SockAddr *psAddr);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif /*JIP_H_*/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
