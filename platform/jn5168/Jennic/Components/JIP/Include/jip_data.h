/****************************************************************************
 *
 * MODULE:             JIP: Data types
 *
 * DESCRIPTION:        Data types suported by JIP
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

#ifndef JIP_DATA_H_
#define JIP_DATA_H_

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

#include "6LP.h"
#include "jip.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef enum
{
    E_JIP_ACCESS_TYPE_READ		= 0x01,
    E_JIP_ACCESS_TYPE_WRITE	= 0x02,
    E_JIP_ACCESS_TYPE_TRAP		= 0x04,
} PACK teJIP_Access;

#define JIP_CACHE_CONST			0xFF
#define JIP_CACHE_NONE           0x00

#define JIP_CACHE_SECONDS(A)		(0x00 | (0x3F & A))
#define JIP_CACHE_MINUTES(A)     (0x40 | (0x3F & A))
#define JIP_CACHE_HOURS(A)       (0x80 | (0x3F & A))
#define JIP_CACHE_DAYS(A)        (0xC0 | (0x3F & A))

typedef const struct {
	const char			*pcName;
	uint8				u8VarID;

	uint8				u8Flags;
	teJIP_VarType		eVarType;
	teJIP_Access		eAccess;
	uint8				u8CacheHint;
	teJIP_Security		eSecurity;
} PACK tsJIP_VarDef;

typedef const struct {
	uint32					u32MibID;
	uint8					u8Variables;

	tsJIP_VarDef		    asVariables[];
} PACK tsJIP_MibDef;

typedef struct {
    void            *pvCbData;
    bool_t          bEnabled;
} tsJIP_Var;

typedef struct {
	teJIP_Status	(*prSet)	(const uint8 *pu8Val, uint8 u8Len, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
	tsJIP_Var        sVar;
} tsJIP_BLOB;

typedef struct {
	teJIP_Status	(*prSet)	(const char *pcVal, uint8 u8Len, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_STRING;

typedef struct {
	teJIP_Status	(*prSet)	(const uint8 *pu8Val, uint8 u8Len, void *pvCbData, uint16 u16Entry);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData, tsJIP_TableData *psTableData);
    tsJIP_Var        sVar;
} tsJIP_TABLE_BLOB;

typedef struct {
    teJIP_Status   (*prSet)    (double dVal, void *pvCbData);
    void            (*prGet)    (thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_DOUBLE;

typedef struct {
    teJIP_Status   (*prSet)    (float fVal, void *pvCbData);
    void            (*prGet)    (thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_FLOAT;

typedef struct {
	teJIP_Status	(*prSet)	(uint64 u64Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_UINT64;

typedef struct {
	teJIP_Status	(*prSet)	(uint32 u32Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_UINT32;

typedef struct {
	teJIP_Status	(*prSet)	(uint16 u16Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_UINT16;

typedef struct {
	teJIP_Status	(*prSet)	(uint8 u8Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_UINT8;

typedef struct {
	teJIP_Status	(*prSet)	(int64 i64Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_INT64;

typedef struct {
	teJIP_Status	(*prSet)	(int32 i32Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_INT32;

typedef struct {
	teJIP_Status	(*prSet)	(int16 i16Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_INT16;

typedef struct {
	teJIP_Status	(*prSet)	(int8 i8Val, void *pvCbData);
	void			(*prGet)	(thJIP_Packet hPacket, void *pvCbData);
    tsJIP_Var        sVar;
} tsJIP_INT8;

typedef union {
    tsJIP_TABLE_BLOB  sTABLE_BLOB;
	tsJIP_BLOB		   sBLOB;
	tsJIP_STRING  	   sSTRING;
	tsJIP_DOUBLE      sDOUBLE;
	tsJIP_FLOAT       sFLOAT;
	tsJIP_UINT64      sUINT64;
	tsJIP_UINT32      sUINT32;
	tsJIP_UINT16	   sUINT16;
	tsJIP_UINT8	   sUINT8;
	tsJIP_INT64	   sINT64;
	tsJIP_INT32	   sINT32;
	tsJIP_INT16	   sINT16;
	tsJIP_INT8		   sINT8;
} tsJIP_VarDynamic;

typedef struct sTag_tsJIP_Mib {
    thJIP_Mib         psNext;
	char              *pcName;
	tsJIP_MibDef     *psDef;
	uint8              u8Index;
	tsJIP_VarDynamic  asFuncs[];
} PACK tsJIP_MibInst;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif /*JIP_DATA_H_*/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
