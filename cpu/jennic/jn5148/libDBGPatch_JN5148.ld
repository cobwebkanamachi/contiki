SECTIONS {
    .rom_workspace.dbg.o.vResetFn ABSOLUTE(0x04000354) : { *dbg.o(.bss.vResetFn) } > ram
    .rom_workspace.dbg.o.vPutchFn ABSOLUTE(0x04000358) : { *dbg.o(.bss.vPutchFn) } > ram
    .rom_workspace.dbg.o.vFlushFn ABSOLUTE(0x0400035c) : { *dbg.o(.bss.vFlushFn) } > ram
    .rom_workspace.dbg.o.vStopFn ABSOLUTE(0x04000360) : { *dbg.o(.bss.vStopFn) } > ram
    .rom_workspace.dbg_uart.o.u8AhiUart ABSOLUTE(0x04000364) : { *dbg_uart.o(.bss.u8AhiUart) } > ram
    .rom_workspace.dbg.o.bIsNewline ABSOLUTE(0x0400010c) : { *dbg.o(.data.bIsNewline) } > ram
    .rom_workspace.dbg_uart.o.u8AhiBaudRate ABSOLUTE(0x0400010d) : { *dbg_uart.o(.data.u8AhiBaudRate) } > ram
    .rom.DBG_vReset ABSOLUTE(0x00006eb4) : { *(.text.DBG_vReset) } > rom
    .rom.vWriteUIntToBuffer ABSOLUTE(0x00006ef3) : { *(.text.vWriteUIntToBuffer) } > rom
    .rom.vNewLine ABSOLUTE(0x00006fcb) : { *(.text.vNewLine) } > rom
    .rom.dbg_vAssertImpl ABSOLUTE(0x00007468) : { *(.text.dbg_vAssertImpl) } > rom
    .rom.DBG_vUartFlush ABSOLUTE(0x00007538) : { *(.text.DBG_vUartFlush) } > rom
    .rom.DBG_vUartPutch ABSOLUTE(0x00007550) : { *(.text.DBG_vUartPutch) } > rom
    .rom.DBG_vUartInitHardware ABSOLUTE(0x00007579) : { *(.text.DBG_vUartInitHardware) } > rom
}
_dbg_g_aprPatchTable = 0x04000110;
