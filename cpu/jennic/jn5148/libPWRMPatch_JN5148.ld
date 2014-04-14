/*****************************************************************************
 *
 * MODULE:             libPWRMPatch_JN5148.ld
 *
 * DESCRIPTION:        Linker command file for PWRM component patch
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
    .rom_workspace.pwrm.o.s_bWakeTimerActive ABSOLUTE(0x04000340) : { *pwrm.o(.bss.s_bWakeTimerActive) } > ram
    .rom_workspace.pwrm.o.s_ePowerMode ABSOLUTE(0x04000344) : { *pwrm.o(.bss.s_ePowerMode) } > ram
    .rom_workspace.pwrm.o.s_psPreSleepCallbacks ABSOLUTE(0x04000348) : { *pwrm.o(.bss.s_psPreSleepCallbacks) } > ram
    .rom_workspace.pwrm.o.u32CalibrationValue ABSOLUTE(0x04000374) : { *pwrm.o(.bss.u32CalibrationValue) } > ram
    .rom_workspace.pwrm.o.psNextWake ABSOLUTE(0x04000384) : { *pwrm.o(.bss.psNextWake) } > ram
/*    .rom.pwrm_vRegisterWakeupCallback ABSOLUTE(0x0001d868) : { *(.text.pwrm_vRegisterWakeupCallback) } > rom */
    .rom.pwrm_vRegisterPreSleepCallback ABSOLUTE(0x0001d887) : { *(.text.pwrm_vRegisterPreSleepCallback) } > rom
    .rom.PWRM_vSetupDozeMonitor ABSOLUTE(0x0001d8a6) : { *(.text.PWRM_vSetupDozeMonitor) } > rom
    .rom.PWRM_vResetDozeTimers ABSOLUTE(0x0001d8ca) : { *(.text.PWRM_vResetDozeTimers) } > rom
    .rom.PWRM_u32GetDozeTime ABSOLUTE(0x0001d8d3) : { *(.text.PWRM_u32GetDozeTime) } > rom
    .rom.PWRM_u32GetDozeElapsedTime ABSOLUTE(0x0001d8dc) : { *(.text.PWRM_u32GetDozeElapsedTime) } > rom
    .rom.PWRM_vWakeInterruptCallback ABSOLUTE(0x0001d8e5) : { *(.text.PWRM_vWakeInterruptCallback) } > rom
    .rom.PWRM_eScheduleActivity ABSOLUTE(0x0001d937) : { *(.text.PWRM_eScheduleActivity) } > rom
}
pwrm_psWakeupCallbacks = 0x04000380;
