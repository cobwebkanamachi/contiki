/****************************************************************************
 *
 * MODULE:             JIP: Network Endian functions
 *
 * DESCRIPTION:        Functions to enable big or little endian processors
 *                     to be supported by the same code
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

#ifndef ENDIAN_H_
#define ENDIAN_H_

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <endian.h>
#include "dbg.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifndef NETWORK_ENDIAN_DBG_STREAM
#define NETWORK_ENDIAN_DBG_STREAM FALSE
#endif

#ifndef INLINE
#ifdef __GNUC__
#define INLINE inline
#else
#ifdef WIN32
#define INLINE __forceinline
#else
#define INLINE static
#endif
#endif
#endif /* INLINE */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

#if __BYTE_ORDER == __LITTLE_ENDIAN
#if __BYTE_ORDER == __BIG_ENDIAN
#error Unable to get endianess
#endif
#else

#endif

#if __BYTE_ORDER == __BIG_ENDIAN

#define bIsLittleEndian(A) 	(FALSE)
#define u64ToNS(A) 			(A)
#define u64FromNS(A) 		(A)
#define u32ToNS(A)			(A)
#define u32FromNS(A)		(A)
#define u16ToNS(A)			(A)
#define u16FromNS(A)		(A)

#else

#define bIsLittleEndian(A)  (TRUE)

#define u32ToNS(A)          htonl(A)
#define u32FromNS(A)        ntohl(A)
#define u16ToNS(A)          htons(A)
#define u16FromNS(A)        ntohs(A)

#endif

#if 0
INLINE uint64 u64ToNS(uint64 u64Val)
{
	uint64 u64Ret;
	uint8 *pu8In  = (uint8 *)&u64Val;
	uint8 *pu8Out = (uint8 *)&u64Ret;

	pu8Out[0] = pu8In[7];
	pu8Out[1] = pu8In[6];
	pu8Out[2] = pu8In[5];
	pu8Out[3] = pu8In[4];
	pu8Out[4] = pu8In[3];
	pu8Out[5] = pu8In[2];
	pu8Out[6] = pu8In[1];
	pu8Out[7] = pu8In[0];

	DBG_vPrintf(NETWORK_ENDIAN_DBG_STREAM, "%s(): in = %llu; out = %llu\n", __FUNCTION__,
			u64Val, u64Ret);

	return u64Ret;
}

INLINE uint64 u64FromNS(uint64 u64Val)
{
    return u64ToNS(u64Val);
}

INLINE uint32 u32ToNS(uint32 u32Val)
{
	uint32 u32Ret;
	uint8 *pu8In  = (uint8 *)&u32Val;
	uint8 *pu8Out = (uint8 *)&u32Ret;

	pu8Out[0] = pu8In[3];
	pu8Out[1] = pu8In[2];
	pu8Out[2] = pu8In[1];
	pu8Out[3] = pu8In[0];

	DBG_vPrintf(NETWORK_ENDIAN_DBG_STREAM, "%s(): in = %u; out = %u\n", __FUNCTION__,
			u32Val, u32Ret);

	return u32Ret;
}

INLINE uint32 u32FromNS(uint32 u32Val)
{
    return u32ToNS(u32Val);
}

INLINE uint16 u16ToNS(uint16 u16Val)
{
	uint16 u16Ret;
	uint8 *pu8In  = (uint8 *)&u16Val;
	uint8 *pu8Out = (uint8 *)&u16Ret;

	pu8Out[0] = pu8In[1];
	pu8Out[1] = pu8In[0];

	DBG_vPrintf(NETWORK_ENDIAN_DBG_STREAM, "%s(): in = %u; out = %u\n", __FUNCTION__,
			u16Val, u16Ret);

	return u16Ret;
}

INLINE uint16 u16FromNS(uint16 u16Val)
{
    return u16ToNS(u16Val);
}
#endif

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /*ENDIAN_H_*/

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
