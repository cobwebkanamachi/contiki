/****************************************************************************
 *
 * MODULE:             JenNet-IP Border Router
 *
 * COMPONENT:          Buffered, interrupt driven serial I/O
 *
 * VERSION:            $Name:  $
 *
 * REVISION:           $Revision: 1.1 $
 *
 * DATED:              $Date: 2008/10/17 10:17:56 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:             LJM
 *
 * DESCRIPTION:
 * Just some simple common uart functions (header file)
 *
 * CHANGE HISTORY:
 *
 * LAST MODIFIED BY:   $Author: lmitch $
 *                     $Modtime: $
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

#ifndef  UARTBUFFERED_H_INCLUDED
#define  UARTBUFFERED_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include "contiki-conf.h"
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#if !defined PUBLIC
#define PUBLIC
#endif

#if !defined PRIVATE
#define PRIVATE static
#endif

//#define UART_EXTRAS

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

//PUBLIC void vUartInit(uint8_t u8Uart, uint32_t u32BaudRate, uint8_t *pu8TxBuffer, uint32_t u32TxBufferLen, uint8_t *pu8RxBuffer, uint32_t u32RxBufferLen);
PUBLIC void vUartInit(uint8_t u8Uart, uint8_t u8BaudRateEnum, uint8_t *pu8TxBuffer, uint32_t u32TxBufferLen, int (*uart_input_function)(unsigned char c));
PUBLIC void vUartInitAdvancedBR(uint8_t u8Uart, uint32_t u32BaudRate, uint8_t *pu8TxBuffer, uint32_t u32TxBufferLen, int (*uart_input_function)(unsigned char c));
PUBLIC void vUartWrite(uint8_t u8Uart, uint8_t u8Data);
PUBLIC bool bUartRead(uint8_t u8Uart, uint8_t *pu8Data);

PUBLIC void vUartClear(uint8_t u8Uart);

#ifdef UART_EXTRAS
PUBLIC uint8_t u8UartRead(uint8_t u8Uart);
PUBLIC bool_t bUartReadBinary(uint8_t u8Uart, uint8_t *pu8Ptr, uint32_t u32Len, uint32_t u32TimeoutTime);
PUBLIC bool_t bUartReadWithTimeout(uint8_t u8Uart, uint8_t *pu8Data, uint32_t u32TimeoutTime);
PUBLIC bool_t bUartTxInProgress(uint8_t u8Uart);
PUBLIC bool_t bUartRxDataAvailable(uint8_t u8Uart);
PUBLIC void vUartWriteBinary(uint8_t u8Uart, uint8_t *pu8Ptr, uint32_t u32Len);
PUBLIC void vUartWriteString(uint8_t u8Uart, uint8_t *pu8String);
PUBLIC void vUartDeInit(uint8_t u8Uart);
PUBLIC void vUartFlush(uint8_t u8Uart);
#endif
/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* UARTBUFFERED_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

