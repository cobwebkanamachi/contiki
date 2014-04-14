/*****************************************************************************
 *
 * MODULE:             libZPSAPLPatch_JN5148.ld
 *
 * DESCRIPTION:        Linker command file for ZPSAPL component patch
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
 * Copyright Jennic Ltd. 2009 All rights reserved
 *
 ****************************************************************************/

SECTIONS {
    .rom.u32NumberOfEmptyMasksInGroupTable ABSOLUTE(0x00016324) : { *(.text.u32NumberOfEmptyMasksInGroupTable) } > rom
    .rom.vSetAllEndpointBitmasksForGroup ABSOLUTE(0x0001634c) : { *(.text.vSetAllEndpointBitmasksForGroup) } > rom
    .rom.eHandleAddEntryToGroupTable ABSOLUTE(0x00016367) : { *(.text.eHandleAddEntryToGroupTable) } > rom
}
