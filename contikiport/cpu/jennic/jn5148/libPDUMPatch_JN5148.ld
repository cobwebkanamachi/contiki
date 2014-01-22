/*****************************************************************************
 *
 * MODULE:             libPDUMPatch_JN5148.ld
 *
 * DESCRIPTION:        Linker command file for PDUM component patch
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
    .rom_workspace.pdum_nwk.o.s_psFreeListHead ABSOLUTE(0x040002dc) : { *pdum_nwk.o(.bss.s_psFreeListHead) } > ram
    .rom_workspace.pdum_nwk.o.s_psNPduPoolStart ABSOLUTE(0x040002e0) : { *pdum_nwk.o(.bss.s_psNPduPoolStart) } > ram
    .rom_workspace.pdum_nwk.o.s_psNPduPoolEnd ABSOLUTE(0x040002e4) : { *pdum_nwk.o(.bss.s_psNPduPoolEnd) } > ram
    .rom_workspace.pdum_nwk.o.s_hMutex ABSOLUTE(0x040002e8) : { *pdum_nwk.o(.bss.s_hMutex) } > ram
    .rom_workspace.pdum_apl.o.s_hMutex ABSOLUTE(0x040002ec) : { *pdum_apl.o(.bss.s_hMutex) } > ram
    .rom_workspace.pdum_apl.o.s_u16NumAPdus ABSOLUTE(0x040002f0) : { *pdum_apl.o(.bss.s_u16NumAPdus) } > ram
    .rom_workspace.pdum_apl.o.s_asAPduPool ABSOLUTE(0x040002f4) : { *pdum_apl.o(.bss.s_asAPduPool) } > ram
    .rom.PDUM_eNPduReadFooterNBO ABSOLUTE(0x00001034) : { *(.text.PDUM_eNPduReadFooterNBO) } > rom
    .rom.PDUM_eNPduReadHeaderNBO ABSOLUTE(0x00001080) : { *(.text.PDUM_eNPduReadHeaderNBO) } > rom
    .rom.PDUM_eNPduAppendDataNBO ABSOLUTE(0x000010ce) : { *(.text.PDUM_eNPduAppendDataNBO) } > rom
    .rom.PDUM_eNPduAppendFooterNBO ABSOLUTE(0x0000112a) : { *(.text.PDUM_eNPduAppendFooterNBO) } > rom
    .rom.PDUM_eNPduPrependHeaderNBO ABSOLUTE(0x00001186) : { *(.text.PDUM_eNPduPrependHeaderNBO) } > rom
    .rom.PDUM_eNPduAllocate ABSOLUTE(0x000011e0) : { *(.text.PDUM_eNPduAllocate) } > rom
    .rom.PDUM_eNPduInitDescending ABSOLUTE(0x0000122e) : { *(.text.PDUM_eNPduInitDescending) } > rom
    .rom.PDUM_eNPduInitAscending ABSOLUTE(0x00001249) : { *(.text.PDUM_eNPduInitAscending) } > rom
    .rom.PDUM_eNPduFree ABSOLUTE(0x00001269) : { *(.text.PDUM_eNPduFree) } > rom
    .rom.PDUM_eNPduClaimHeader ABSOLUTE(0x000012d0) : { *(.text.PDUM_eNPduClaimHeader) } > rom
    .rom.PDUM_eNPduClaimFooter ABSOLUTE(0x000012f9) : { *(.text.PDUM_eNPduClaimFooter) } > rom
    .rom.PDUM_eNPduClaimData ABSOLUTE(0x0000131f) : { *(.text.PDUM_eNPduClaimData) } > rom
    .rom.PDUM_eNPduAscend ABSOLUTE(0x00001380) : { *(.text.PDUM_eNPduAscend) } > rom
    .rom.PDUM_eNPduDescend ABSOLUTE(0x000013a4) : { *(.text.PDUM_eNPduDescend) } > rom
    .rom.PDUM_eNPduWriteToBuffer ABSOLUTE(0x000013c8) : { *(.text.PDUM_eNPduWriteToBuffer) } > rom
    .rom.PDUM_eNPduWriteFromBuffer ABSOLUTE(0x00001413) : { *(.text.PDUM_eNPduWriteFromBuffer) } > rom
    .rom.PDUM_eNPduWriteDataToBuffer ABSOLUTE(0x0000143c) : { *(.text.PDUM_eNPduWriteDataToBuffer) } > rom
    .rom.PDUM_eNPduWriteDataFromBuffer ABSOLUTE(0x0000149d) : { *(.text.PDUM_eNPduWriteDataFromBuffer) } > rom
    .rom.pdum_vNPduInit ABSOLUTE(0x000014fa) : { *(.text.pdum_vNPduInit) } > rom
    .rom.PDUM_pvAPduInstanceGetPayload ABSOLUTE(0x00001537) : { *(.text.PDUM_pvAPduInstanceGetPayload) } > rom
    .rom.PDUM_u16APduInstanceGetPayloadSize ABSOLUTE(0x00001567) : { *(.text.PDUM_u16APduInstanceGetPayloadSize) } > rom
    .rom.PDUM_eAPduInstanceSetPayloadSize ABSOLUTE(0x00001574) : { *(.text.PDUM_eAPduInstanceSetPayloadSize) } > rom
    .rom.PDUM_u16APduGetSize ABSOLUTE(0x0000159c) : { *(.text.PDUM_u16APduGetSize) } > rom
    .rom.PDUM_u16APduInstanceWriteStrNBO ABSOLUTE(0x000015a9) : { *(.text.PDUM_u16APduInstanceWriteStrNBO) } > rom
    .rom.PDUM_u16APduInstanceWriteNBO ABSOLUTE(0x000015f7) : { *(.text.PDUM_u16APduInstanceWriteNBO) } > rom
    .rom.PDUM_u16APduInstanceReadNBO ABSOLUTE(0x00001653) : { *(.text.PDUM_u16APduInstanceReadNBO) } > rom
    .rom.PDUM_hAPduAllocateAPduInstance ABSOLUTE(0x00001698) : { *(.text.PDUM_hAPduAllocateAPduInstance) } > rom
    .rom.PDUM_eAPduFreeAPduInstance ABSOLUTE(0x000016ec) : { *(.text.PDUM_eAPduFreeAPduInstance) } > rom
    .rom.pdum_vAPduInit ABSOLUTE(0x00001764) : { *(.text.pdum_vAPduInit) } > rom
    .rom.PDUM_u16SizeNBO ABSOLUTE(0x000017c7) : { *(.text.PDUM_u16SizeNBO) } > rom
    .rom.pdum_u16WriteNBO ABSOLUTE(0x00001826) : { *(.text.pdum_u16WriteNBO) } > rom
    .rom.pdum_u16WriteStrNBO ABSOLUTE(0x00001946) : { *(.text.pdum_u16WriteStrNBO) } > rom
    .rom.pdum_u16ReadNBO ABSOLUTE(0x00001a6e) : { *(.text.pdum_u16ReadNBO) } > rom
    .rom.vDumpPayload ABSOLUTE(0x00001bc5) : { *(.text.vDumpPayload) } > rom
    .rom.PDUM_vDBGPrintNPdu ABSOLUTE(0x00001bf3) : { *(.text.PDUM_vDBGPrintNPdu) } > rom
}
_pdum_g_aprPatchTable = 0x04000084;
