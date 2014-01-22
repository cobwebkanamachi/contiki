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
 * theory OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author(s): Philipp Scholl <scholl@teco.edu>
 */

#include "gdb2.h"

typedef enum
{
  SR_GET_STATUS = 0x00,
  SR_CLEAR_FEATURE,
  SR_RESERVED,
  SR_SET_FEATURE,
  SR_RESERVERD2,
  SR_SET_ADDRESS,
  SR_GET_DESCRIPTOR,
  SR_SET_DESCRIPTOR,
  SR_GET_CONFIGURATION,
  SR_SET_CONFIGURATION,
  SR_GET_INTERFACE,
  SR_SET_INTERFACE
} __attribute((__packed__)) req_t;

typedef enum
{
  TO_DEVICE    = 0x00,
  TO_INTERFACE = 0x01,
  TO_ENDPOINT  = 0x02
} __attribute((__packed__)) recp_t;

typedef struct
{
  struct   { uint8_t direction:1, type:2, recp:5; } __attribute((__packed__)) reqtype;
  req_t    req:8;
  union
  {
    struct { uint8_t idx, val; uint16_t langid, len; } __attribute((__packed__)) asdescreq;
    struct { uint8_t conf, unused[5]; }                __attribute((__packed__)) asconfreq;
    uint16_t                                           asstatresp;
    struct { uint8_t altidx, res1, ifidx; }            __attribute((__packed__)) asifreq;
  };
} __attribute((__packed__)) setup_t;

#define STALLEP0() do { PRINTF("*** stall EP0 ***\n"); wregb2(EPSTALLS, STLEP0IN=true, STLEP0OUT=true); } while(0);
#define ACK()      do { rregn(FNADDR, NULL, 0, true); } while(0); /* dummy read to set ackstat */

static void
usb_send_desc(setup_t *sp)
{
  // initialize with:
  //  * len = requested length
  //  * dsc = configuration descriptor
  //  * wTotalLength = wTotalLength field of configuration descriptor
  uint8_t     len = HTOUS(sp->asdescreq.len);
  usbdescr_t *dsc = (usbdescr_t*) &(usbdev.usbdesc[((usbdescr_t*) usbdev.usbdesc)->bLength]);
  uint16_t    wTotalLength = HTOUS(*(uint16_t*) dsc->buf);
  size_t      i;

  switch(sp->asdescreq.val)
  {
    case DEVICE_DESCRIPTOR:
      PRINTF("dev_desc\n");
      dsc = (usbdescr_t*) usbdev.usbdesc;
      len = dsc->bLength;
      break;

    case CONFIGURATION_DESCRIPTOR:
      PRINTF("conf_desc\n");
      len = MIN(len, wTotalLength);
      break;

    case STRING_DESCRIPTOR:
      PRINTF("str_desc\n");
      dsc = (usbdescr_t*) &dsc->buf[wTotalLength-sizeof(usbdescr_t)]; /* at the first string now */
      for(i=0; i<sp->asdescreq.idx && dsc->bDescriptorType==STRING_DESCRIPTOR; i++)
          dsc = (usbdescr_t*) &dsc->buf[dsc->bLength-sizeof(usbdescr_t)];
      len = dsc->bLength;
      break;

    default:
      PRINTF("unknown setup request\n");
      STALLEP0();
  }

  {
  uint8_t wlen = MIN(64, len),
          *pt  = (uint8_t*) dsc;
  reg_t   st   = {0x00};

  do
  {
    wlen = MIN(64, len);

    /* wait until asserted */
    do { rreg(EPIRQ, &st); } while( !st.EPIRQ.IN0BAVIRQ );

    /* writing to EP0, also clrs irq */
    wregn(EP0FIFO, pt, wlen, false);
    wregn(EP0BC, &wlen, sizeof(uint8_t), len<64);

    len -= wlen;
    pt  += wlen;
  } while(len>0);
  }
}

static void
get_status(setup_t *sp)
{
  uint8_t len = sizeof(sp->asstatresp);
  reg_t st = {0x00};

  switch(sp->reqtype.recp)
  {
    case TO_DEVICE:
      sp->asstatresp = (usbdev.status.rwu_enabled)<<9 |
                       (usbdev.status.self_powered)<<8;
      break;
    case TO_ENDPOINT: /* XXX: for endpoint the stall status should be returned */
    case TO_INTERFACE:
      sp->asstatresp = 0x0000;
      break;
    default:
      STALLEP0();
  }

  do { rreg(EPIRQ, &st); } while( !st.EPIRQ.IN0BAVIRQ );
  wregn(EP0FIFO, (uint8_t*) &sp->asstatresp, len, false);
  wregn(EP0BC, &len, sizeof(len), true);
}

static bool
std_req(setup_t *sp)
{
  switch(sp->req)
  {
    case SR_SET_ADDRESS:
      PRINTF("set_addr\n");
      rregn(FNADDR, NULL, 0, true);
      break;
    case SR_GET_DESCRIPTOR:
      PRINTF("get_desc: ");
      usb_send_desc(sp);
      break;
    case SR_SET_FEATURE:
      /* this either sets that RWU is to be enabled or to halt an EP,
       * which is mandatory for bulk and interrupt EPs */
      PRINTF("set_feature\n");
      break;
    case SR_CLEAR_FEATURE:
      /* clear one of the features which have been set by SET_FEATURE */
      PRINTF("clear_feature\n");
      break;
    case SR_GET_STATUS:
      PRINTF("get_status\n");
      get_status(sp);
      break;
    case SR_SET_INTERFACE:
      /* this is used to set alternative interfaces. For example when
       * the CDC device will be put up */
      PRINTF("setif");
      ACK();
      break;
    case SR_GET_INTERFACE:
      PRINTF("getif");
      /* always report the same alternative: 1 */
      {
        uint8_t i = 0x01;
        reg_t st = {0x00};
        do { rreg(EPIRQ, &st); } while( !st.EPIRQ.IN0BAVIRQ );
        wregn(EP0FIFO, &i, sizeof(uint8_t), false);
        i=sizeof(uint8_t);
        wregn(EP0BC, &i, i, true);
      }
      break;
    case SR_SET_CONFIGURATION:
      PRINTF("set_conf\n");
      usbdev.status.configuration = sp->asconfreq.conf;
      ACK();
      return true;
      break;
    case SR_GET_CONFIGURATION:
      PRINTF("get_conf\n");
      {
        uint8_t len = sizeof(usbdev.status.configuration);
        reg_t st = {0x00};
        do { rreg(EPIRQ, &st); } while( !st.EPIRQ.IN0BAVIRQ );
        wregn(EP0FIFO, &usbdev.status.configuration, len, false);
        wregn(EP0BC, &len, sizeof(len), true);
      }
      break;
    default:
      PRINTF("unknown std_req()");
      STALLEP0();
  }

  return false;
}

static bool
usb_setup(setup_t *sp)
{
  switch(sp->reqtype.type)
  {
    case 0x00: PRINTF("usb: std_req - "); return std_req(sp); break;
    case 0x01: PRINTF("usb: class_req - "); break;
    case 0x02: PRINTF("usb: vendor_req - "); break;
    default:   PRINTF("unknown req - "); STALLEP0();
  }

  PRINTF(".\n");
  return false;
}
