/*****************************************************************************
 *
 * MODULE:             libZPSMACPatch_JN5148.ld
 *
 * DESCRIPTION:        Linker command file for ZPSMAC component patch
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
    .rom_workspace.zps_mac.o.s_hMacMutex ABSOLUTE(0x04000330) : { *zps_mac.o(.bss.s_hMacMutex) } > ram
    .rom_workspace.zps_mac.o.s_sDcfmIndMgr ABSOLUTE(0x04000334) : { *zps_mac.o(.bss.s_sDcfmIndMgr) } > ram
    .rom_workspace.zps_mac.o.prHandleMlmeReqRsp ABSOLUTE(0x040001ac) : { *zps_mac.o(.data.prHandleMlmeReqRsp) } > ram
    .rom_workspace.zps_mac.o.prHandleMcpsReqRsp ABSOLUTE(0x040001b0) : { *zps_mac.o(.data.prHandleMcpsReqRsp) } > ram
    .rom.MAC_vRegisterMcpsVsDcfmIndCallbacks ABSOLUTE(0x00001f5c) : { *(.text.MAC_vRegisterMcpsVsDcfmIndCallbacks) } > rom
    .rom.MAC_vHandleMlmeVsReqRsp ABSOLUTE(0x0000202f) : { *(.text.MAC_vHandleMlmeVsReqRsp) } > rom
    .rom.MAC_vHandleMcpsVsReqRsp ABSOLUTE(0x0000206b) : { *(.text.MAC_vHandleMcpsVsReqRsp) } > rom
    .rom.MAC_vSetMutex ABSOLUTE(0x00002165) : { *(.text.MAC_vSetMutex) } > rom
}
_zpsmac_g_aprPatchTable = 0x040001b4;
