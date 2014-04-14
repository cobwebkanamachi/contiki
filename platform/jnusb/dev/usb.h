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

#ifndef __MAX_3420E_USB_H__
#define __MAX_3420E_USB_H__

#include "contiki.h"
#include "contiki-net.h"
#include "usb_dev.h"
#include "stdint.h"

#ifndef HTOUS /* usb is little endian */
# if UIP_CONF_BYTE_ORDER == UIP_BIG_ENDIAN
#      define HTOUS(n) (uint16_t)((((uint16_t) (n)) << 8) | (((uint16_t) (n)) >> 8))
#      define HTOUL(n) (((uint32_t)HTOUS(n) << 16) | HTOUS((uint32_t)(n) >> 16))
# else
#      define HTOUS(n) n
#      define HTOUL(n) n
# endif
#endif

#define USBBUFSIZE 64

PROCESS_NAME(usb_process);
process_event_t usb_event;

typedef enum {
  EVENT_USB_ENUMERATED   = 0x01,
  EVENT_USB_DISCONNECTED = 0x02,
} usb_event_t;

extern usbdev_t usbdev;

size_t usb_read(endpoint_t ep, uint8_t *buf, size_t n);
size_t usb_write(endpoint_t ep, uint8_t *buf, size_t n);
size_t usb_write_nonblock(endpoint_t ep, uint8_t *buf, size_t n);

#endif

