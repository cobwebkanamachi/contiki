/*****************************************************************************
 *
 * MODULE:             libZPSNWKPatch_JN5148.ld
 *
 * DESCRIPTION:        Linker command file for ZPSNWK component patch
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
    .rom.zps_vNwkLibU16ToPayload ABSOLUTE(0x00003c0e) : { *(.text.zps_vNwkLibU16ToPayload) } > rom
    .rom.zps_vNwkLibU32ToPayload ABSOLUTE(0x00003c1a) : { *(.text.zps_vNwkLibU32ToPayload) } > rom
    .rom.zps_vNwkLibU64ToPayload ABSOLUTE(0x00003c32) : { *(.text.zps_vNwkLibU64ToPayload) } > rom
    .rom.zps_u16NwkLibFromPayload ABSOLUTE(0x00003cf1) : { *(.text.zps_u16NwkLibFromPayload) } > rom
    .rom.zps_u32NwkLibFromPayload ABSOLUTE(0x00003d00) : { *(.text.zps_u32NwkLibFromPayload) } > rom
    .rom.zps_u64NwkLibFromPayload ABSOLUTE(0x00003d23) : { *(.text.zps_u64NwkLibFromPayload) } > rom
    .rom.zps_vNwkSlistInit ABSOLUTE(0x000068f6) : { *(.text.zps_vNwkSlistInit) } > rom
    .rom.zps_vNwkSlistAddToHead ABSOLUTE(0x000068ff) : { *(.text.zps_vNwkSlistAddToHead) } > rom
    .rom.zps_vNwkSlistAddToTail ABSOLUTE(0x00006916) : { *(.text.zps_vNwkSlistAddToTail) } > rom
    .rom.zps_vNwkSlistInsertAfter ABSOLUTE(0x00006930) : { *(.text.zps_vNwkSlistInsertAfter) } > rom
    .rom.zps_psNwkSlistRemoveFromHead ABSOLUTE(0x00006949) : { *(.text.zps_psNwkSlistRemoveFromHead) } > rom
    .rom.zps_psNwkSlistRemove ABSOLUTE(0x00006960) : { *(.text.zps_psNwkSlistRemove) } > rom
    .rom.zps_vNwkSlistMakeLimboNode ABSOLUTE(0x0000699c) : { *(.text.zps_vNwkSlistMakeLimboNode) } > rom
    .rom.zps_bNwkSlistIsLimboNode ABSOLUTE(0x000069a4) : { *(.text.zps_bNwkSlistIsLimboNode) } > rom
}
