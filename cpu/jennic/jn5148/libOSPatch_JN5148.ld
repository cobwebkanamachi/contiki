/*****************************************************************************
 *
 * MODULE:             libOSPatch_JN5148.ld
 *
 * DESCRIPTION:        Linker command file for OS component patch
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
    .rom_workspace.os.o.s_u32CurrentPriorityMask ABSOLUTE(0x040002f8) : { *os.o(.bss.s_u32CurrentPriorityMask) } > ram
    .rom_workspace.os.o.s_psCurrentTask ABSOLUTE(0x040002fc) : { *os.o(.bss.s_psCurrentTask) } > ram
    .rom_workspace.os.o.s_prvUnclaimedISR ABSOLUTE(0x04000300) : { *os.o(.bss.s_prvUnclaimedISR) } > ram
    .rom_workspace.os.o.s_u32SuspendIRQNesting ABSOLUTE(0x04000304) : { *os.o(.bss.s_u32SuspendIRQNesting) } > ram
    .rom_workspace.os.o.s_u8SuspendedIPL ABSOLUTE(0x04000308) : { *os.o(.bss.s_u8SuspendedIPL) } > ram
    .rom_workspace.os.o.s_u32PendingQueue ABSOLUTE(0x0400030c) : { *os.o(.bss.s_u32PendingQueue) } > ram
    .rom_workspace.os.o.s_asTasks ABSOLUTE(0x04000310) : { *os.o(.bss.s_asTasks) } > ram
    .rom_workspace.os.o.s_u8OSLevelIPL ABSOLUTE(0x04000314) : { *os.o(.bss.s_u8OSLevelIPL) } > ram
    .rom.OS_eGetCurrentTask ABSOLUTE(0x00013844) : { *(.text.OS_eGetCurrentTask) } > rom
    .rom.OS_eGetMessageStatus ABSOLUTE(0x00013859) : { *(.text.OS_eGetMessageStatus) } > rom
    .rom.OS_eSuspendOSInterrupts ABSOLUTE(0x0001388b) : { *(.text.OS_eSuspendOSInterrupts) } > rom
    .rom.OS_eResumeOSInterrupts ABSOLUTE(0x000138cc) : { *(.text.OS_eResumeOSInterrupts) } > rom
    .rom.OS_eDisableAllInterrupts ABSOLUTE(0x00013903) : { *(.text.OS_eDisableAllInterrupts) } > rom
    .rom.OS_eEnableAllInterrupts ABSOLUTE(0x0001391a) : { *(.text.OS_eEnableAllInterrupts) } > rom
    .rom.OS_eGetSWTimerStatus ABSOLUTE(0x0001392e) : { *(.text.OS_eGetSWTimerStatus) } > rom
    .rom.os_vUnclaimedInterrupt ABSOLUTE(0x00013949) : { *(.text.os_vUnclaimedInterrupt) } > rom
    .rom.eExpireSWTimer ABSOLUTE(0x00013a25) : { *(.text.eExpireSWTimer) } > rom
    .rom.OS_eEnterCriticalSection ABSOLUTE(0x0001421e) : { *(.text.OS_eEnterCriticalSection) } > rom
}
_os_g_aprPatchTable = 0x04000130;
