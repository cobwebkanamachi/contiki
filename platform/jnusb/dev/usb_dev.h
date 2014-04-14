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

#ifndef __USB_DEV_H__
#define __USB_DEV_H__
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <contiki.h>

typedef enum
{
  EP1,
  EP2,
  EP3
} endpoint_t;

typedef struct {
  struct process *process;
  struct {
    uint8_t configuration;
    volatile bool rwu_enabled, self_powered, suspended;
  } status;
  uint8_t usbdesc[];
} __attribute((__packed__)) usbdev_t;

typedef struct {
    uint8_t bLength, bDescriptorType;
    uint8_t buf[];
} __attribute((__packed__)) usbdescr_t;

typedef enum {
    DEVICE_DESCRIPTOR        = 0x01,
    CONFIGURATION_DESCRIPTOR = 0x02,
    STRING_DESCRIPTOR        = 0x03,
    INTERFACE_DESCRIPTOR     = 0x04,
    ENDPOINT_DESCRIPTOR      = 0x05
} __attribute((__packed__)) usbdescr_type_t;

#endif
