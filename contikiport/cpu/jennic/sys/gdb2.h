/*
 * Copyright (c) 2008
 * Telecooperation Office (TecO), Universitaet Karlsruhe (TH), Germany.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. Neither the name of the Universitaet Karlsruhe (TH) nor the names
 *    of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#ifndef __GDB2_H__
#define __GDB2_H__
# include "uart0.h"

#ifdef GDB
 #ifdef __BA1__
  void gdb2_console_output(int len, char *buf);
  void gdb_startupPatched(int uart, int div);

   #define GDB2_STARTUP(uart,div) gdb_startupPatched(uart, div)
   #define GDB2_PUTS(buf)         gdb2_console_output(strlen(buf), buf)
   #define GDB2_PUTC(c)           gdb2_console_output(1,&c)
   #define HAL_BREAKPOINT()       asm volatile ("l.trap %0 " : :"I"(1))
 #endif

 #ifdef __BA2__ /* only printf debugging */
   #define GDB2_STARTUP(uart,div) uart0_init(38400)
   #define GDB2_PUTS(buf)         do{size_t gdbi; for(gdbi=0;gdbi<strlen(buf);gdbi++){uart0_writeb(buf[gdbi]);}} while(0);
   #define GDB2_PUTC(c)           do{uart0_writeb(c);} while(0);
   #define HAL_BREAKPOINT()
 #endif
#else
 #define GDB2_STARTUP(uart,div)
 #define GDB2_PUTS(buf)
 #define GDB2_PUTC(c)
 #define HAL_BREAKPOINT()
#endif

#endif

