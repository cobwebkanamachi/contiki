/*****************************************************************************
 *
 * MODULE:             JN5148
 *
 * DESCRIPTION:        Linker command file for an application using JN5148
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

INCLUDE Rom_Functions_JN5148.ld

_minimum_heap_size = 2048;
       _stack_size = 8192;


/* RAM is defined as the whole RAM */
MEMORY
{
        ram     : ORIGIN = 0x04000000, LENGTH = 0x20000
        flash   : ORIGIN = 0x04000000, LENGTH = 0x80000
}

INCLUDE ZPS_ROM.ld

/* The start of RAM available to applications is given by __app_ram_start,
   put into the linker file as part of the ROM build process. As usual 
   the flash header starts 0x30 bytes ahead of this so the MAC address 
   appears at the start of the application RAM space */
   
SECTIONS
{
        .flashheader ABSOLUTE(__app_ram_start - 0x30):
        {
        _flash_beg = ABSOLUTE(.);
        LONG(0x0000e0e0)
        LONG(ABSOLUTE(_data_beg))
        LONG(_data_len)
        LONG(0x01060035)
        LONG(0)
        LONG(ABSOLUTE(_bss_beg))
        LONG(_bss_len)
        LONG(ABSOLUTE(_AppWarmStart))
        LONG(ABSOLUTE(_AppColdStart))
        } > ram

        .oad :
        {
        LONG(0xffffffff)
        LONG(0xffffffff)
        LONG(0xf0f0f0f0)
        } > ram

        .mac :
        {
        _enc_start = ABSOLUTE(.);
        _data_beg = ABSOLUTE(.);
        LONG(0xffffffff) /* MAC address  8 bytes*/
        LONG(0xffffffff) 
        LONG(0xffffffff) 
        LONG(0xffffffff)
        _FlsLinkKey = ABSOLUTE(.);
        LONG(0xffffffff) /* Initial Link Key 16 bytes*/
        LONG(0xffffffff)
        LONG(0xffffffff)
        LONG(0xffffffff)
        } > ram
        _enc_offset = (_enc_start & 0x0000000F);
        _enc_start = _enc_start - _flash_beg;
        
        .heap_location :
        {
        _heap_location = .;
        LONG(ABSOLUTE(_heap_start))
        LONG(ABSOLUTE(_heap_end))
        } > ram
        
        /* RAM CLT: a pointer for each component (NULL if component not patched and not using callbacks) */
        .rtc_clt :
        {
        LONG(0x00000000) /* MAC */
        LONG(0x00000000) /* Peripheral API */
        LONG(0x00000000) /* 6LoWPAN */
        LONG(0x00000000) /* JenNet */
        LONG(0x00000000)
        LONG(0x00000000)
        LONG(0x00000000)
        LONG(0x00000000)
        LONG(0x00000000)
        LONG(0x00000000)
        } > ram  
        
        /* start location 0x80 to 0xCF */
        .ro_se_cert ALIGN (0x4) :
        {
        _FlsZcCert = ABSOLUTE(.);
        *(.ro_se_cert)  /* Zigbee certificate 48 bytes*/
        }> ram

        .ro_se_pvKey ALIGN (0x4) :
        {
        _FlsPrivateKey = ABSOLUTE(.);
        *(.ro_se_pvKey) /* Private Key 21 Bytes */
        } > ram

        .rodata :
        {
        *(.rodata*)
        } > ram

        .data :
        {
        *(.data*)
        } > ram

        .gdb :
        {
        libSWDebug.a(.text)
        *(.dbgtxt*)
        } > ram
        
        .text :
        {
        _text_start = ABSOLUTE(.);
        *(.text*)
        _text_end = ABSOLUTE(.);
        _data_end = ABSOLUTE(.);
        } > ram
        
        _data_len = (_data_end - _data_beg + 3) & 0xfffffffc;
        
        _overlay_start = ABSOLUTE(ALIGN(0x4));
        _overlay_end = ABSOLUTE(ALIGN(0x4));
        
        _overlays = 0;

        .overlay_location (ALIGN((. - _enc_offset), 16) + _enc_offset):
        {
        } > flash
        
        _overlay_len = 0;
        _overlay_lma = (ALIGN(ADDR(.overlay_location) + SIZEOF(.overlay_location) - (_enc_offset), 16) + _enc_offset);
        

        .bss ALIGN (0x4) (NOLOAD):
        {
        _bss_beg = ABSOLUTE(.);
        _bss_start = ABSOLUTE(.);
        *(.bss*)
        *(.scommon*)
        } > ram
        _bss_end = ABSOLUTE(.);

        /* reserve minimum heap size */
        .heap ALIGN (0x4) (NOLOAD):
        {
        _heap_start = ABSOLUTE(.);
        . += _minimum_heap_size;
        } > ram


        /* make end of heap the bottom of the stack */
        .stack ABSOLUTE (ORIGIN(ram) + LENGTH(ram) - _stack_size) (NOLOAD):
        {
        _heap_end = ABSOLUTE(.);
        _stack_low_water_mark = ABSOLUTE(.);
        . += _stack_size;
        } > ram

        /*
        ASSERT(((_heap_start - ORIGIN(ram) + _minimum_heap_size + _stack_size) <= LENGTH(ram)), "RAM Exhausted")
        */

        .stab 0 (INFO):
        {
        *(.stab)
        *(.stab.*)
        }

        .comment 0 (INFO):
        {
        *(.comment*)
        }

        _bss_len = (_bss_end - _bss_beg + 3) & 0xfffffffc;
}

/* ENTRY(_DebugInit) */

_apvReg       = 0x2001000;
_apvSysReg    = 0x2001000;
_apvAesReg    = 0x2001c00;
_apvAnaReg    = 0x2001f00;
_apvGpioReg   = 0x2002000;
_apvUart0Reg  = 0x2003000;
_apvUart1Reg  = 0x2004000;
_apvTimer0Reg = 0x2005000;
_apvTimer1Reg = 0x2006000;
_apvTimer2Reg = 0x200A000;
_apvSiReg     = 0x2007000;
_apvSpiReg    = 0x2008000;
_apvSpiIpReg  = 0x2009000;
_apvSpiIpMem  = 0x2009800;
_apvDaiReg    = 0x200B000;
_apvSfifoReg  = 0x200C000;
