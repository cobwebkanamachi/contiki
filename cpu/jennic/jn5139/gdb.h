/****************************************************************************
 *
 * MODULE:             ZED-MAC
 *
 * COMPONENT:          $RCSfile: gdb.h,v $
 *
 * VERSION:            $Name: RD_RELEASE_6thMay09 $
 *
 * REVISION:           $Revision: 1.5 $
 *
 * DATED:              $Date: 2008/10/22 12:09:43 $
 *
 * STATUS:             $State: Exp $
 *
 * AUTHOR:
 *
 * DESCRIPTION:
 * gdb header
 *
 * LAST MODIFIED BY:   $Author: pjtw $
 *                     $Modtime$
 *
 *
 ****************************************************************************
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
 * Copyright Jennic Ltd 2005, 2006. All rights reserved
 *
 ***************************************************************************/

#ifndef GDB_H
#define GDB_H


#ifdef __cplusplus
extern "C"
#endif

#ifdef GDB_PATCH
	// patched gdb
	void gdb_startup(void);	/* This is void in the chiplib
								   See ZED-SDK\Sw\GDBstub\Source
								   on lable Bal02aPatchV01040631a
								*/

	//void gdb_startup (uint8 u8Uart, uint16 u16Divisor);
	#define gdb_breakpoint() asm volatile ("l.trap %0 " : :"I"(1));
#endif

#ifdef GDB
	// original gdb
	void gdb_startup(void);
	#define HAL_GDB_INIT()   gdb_startup()
	#define HAL_BREAKPOINT() asm volatile ("l.trap %0 " : :"I"(1))
#else
	#define HAL_GDB_INIT()
	#define HAL_BREAKPOINT()
#endif


#endif
