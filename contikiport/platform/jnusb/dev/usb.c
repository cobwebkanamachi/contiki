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

#include "usb.h"
#include <stdint.h>
#include <stdbool.h>
#include <AppHardwareApi.h>
#include "gdb2.h"
#include "sys/pt.h"
#include "sys/timer.h"
#include "leds.h"

#define DEBUG 0
#if DEBUG
# define PRINTF(...) printf(__VA_ARGS__)
#else
# define PRINTF(...)
#endif

/* define the connections for the usb chip */
#ifndef JENNIC_CONF_USB_IRQPIN
# error JENNIC_CONF_USB_IRQPIN missing
#endif

#ifndef JENNIC_CONF_USB_SPISLAVE
# error JENNIC_CONF_USB_SPISLAVE missing
#endif

#define MAX3420_INT_PIN   JENNIC_CONF_USB_IRQPIN
#define MAX3420_SPI_SLAVE JENNIC_CONF_USB_SPISLAVE

uint16_t usb_len;
char usb_buf[sizeof(uip_buf)];
uint16_t usb_in_len = 0,
         usb_in_buf_size = UIP_BUFSIZE;
char *usb_in_buf    = usb_buf;

/* For register documentation, see http://pdfserv.maxim-ic.com/en/an/AN3598.pdf */
typedef union
{
  uint8_t raw;
  union
  {
    uint8_t EP0FIFO;
    uint8_t EP1OUTFIFO;
    uint8_t EP2INFIFO;
    uint8_t EP3INFIFO;
    uint8_t SUDFIFO;
    uint8_t EP0BC;
    uint8_t EP1OUTBC;
    uint8_t EP2INBC;
    uint8_t EP3INBC;
    struct { bool zero:1, ACKSTAT:1, STLSTAT:1, STLEP3IN:1, STLEP2IN:1, STLEP1OUT:1, STLEP0OUT:1, STLEP0IN:1; } EPSTALLS;
    struct { bool EP3DISAB:1, EP2DISAB:1, EP1DISAB:1, CTGEP3IN:1, CTGEP2IN:1, CTGEP1OUT:1, zero:2; } CLRTOGS;
    struct { bool zero:2, SUDAVIRQ:1, IN3BAVIRQ:1, IN2BAVIRQ:1, OUT1DAVIRQ:1, OUT0DAVIRQ:1, IN0BAVIRQ:1; } EPIRQ;
    struct { bool zero:2, SUDAVIE:1, IN3BAVIE:1, IN2BAVIE:1, OUT1DAVIE:1, OUT0DAVIE:1, IN0BAVIE:1; } EPIEN;
    struct { bool URESDNIRQ:1, VBUSIRQ:1, NOVBUSIRQ:1, SUSPIRQ:1, URESIRQ:1, BUSACTIRQ:1, RWUDNIRQ:1, OSCOKIRQ:1; } USBIRQ;
    struct { bool URESDNIE:1, VBUSIE:1, NOVBUSIE:1, SUSPIE:1, URESIE:1, BUSACTIE:1, RWUDNIE:1, OSCOKIE:1; } USBIEN;
    struct { bool HOSCSTEN:1, VBGATE:1, CHIPRES:1, PWRDOWN:1, CONNECT:1, SIGRWU:1, zero:2; } USBCTL;
    struct { bool zero:7, IE:1; } CPUCTL;
    struct { bool EP3INAK:1, EP2INAK:1, EP0INAK:1, FDUPSPI:1, INTLEVEL:1, POSINT:1, GPXB:1, GPXA:1; } PINCTL;
    uint8_t REVISION;
    uint8_t FNADDR;
    struct { bool GPIN3:1, GPIN2:1, GPIN1:1, GPIN0:1, GPOUT3:1, GPOUT2:1, GPOUT1:1, GPOUT0:1; } IOPINS;
  };
} __attribute((__packed__)) reg_t;

typedef enum
{
  EP0FIFO = 0x00,
  EP1OUTFIFO,
  EP2INFIFO,
  EP3INFIFO,
  SUDFIFO,
  EP0BC,
  EP1OUTBC,
  EP2INBC,
  EP3INBC,
  EPSTALLS,
  CLRTOGS,
  EPIRQ,
  EPIEN,
  USBIRQ,
  USBIEN,
  USBCTL,
  CPUCTL,
  PINCTL,
  REVISION,
  FNADDR,
  IOPINS
} __attribute((__packed__)) regid_t;

typedef union
{
  uint16_t raw;
  struct
  {
    regid_t reg:5;
    bool    zero:1, dir:1, ackstat:1;
    reg_t   regval;
  } ascmd;
} __attribute((__packed__)) cmd_t;

typedef union
{
  uint8_t raw;
  struct { bool SUSPIRQ:1, URESIRQ:1, SUDAVIRQ:1, IN3BAVIRQ:1, IN2BAVIRQ:1, OUT1DAVIRQ:1, OUT0DAVIRQ:1, IN0BAVIRQ:1; } status;
} __attribute((__packed__)) status_byte_t;

#define WRITE 1
#define READ  0

/* helper functions to write single register bits */
#define wregb(REGID, BIT) do { reg_t val={0x00}; val.REGID.BIT; wreg(REGID, val);} while(0);
#define wregb2(REGID, BIT, BIT2) do { reg_t val={0x00}; val.REGID.BIT; val.REGID.BIT2; wreg(REGID, val);} while(0);
#define wregb4(REGID, BIT, BIT2, BIT3, BIT4) do { reg_t val={0x00}; val.REGID.BIT; val.REGID.BIT2; val.REGID.BIT3; val.REGID.BIT4; wreg(REGID, val);} while(0);
#define wregb5(REGID, BIT, BIT2, BIT3, BIT4, BIT5) do { reg_t val={0x00}; val.REGID.BIT; val.REGID.BIT2; val.REGID.BIT3; val.REGID.BIT4; val.REGID.BIT5; wreg(REGID, val);} while(0);
#define wregb6(REGID, BIT, BIT2, BIT3, BIT4, BIT5, BIT6) do { reg_t val={0x00}; val.REGID.BIT; val.REGID.BIT2; val.REGID.BIT3; val.REGID.BIT4; val.REGID.BIT5; val.REGID.BIT6; wreg(REGID, val);} while(0);
#define wregb8(REGID, BIT, BIT2, BIT3, BIT4, BIT5, BIT6, BIT7, BIT8) do { reg_t val={0x00}; val.REGID.BIT; val.REGID.BIT2; val.REGID.BIT3; val.REGID.BIT4; val.REGID.BIT5; val.REGID.BIT6; val.REGID.BIT7; val.REGID.BIT8; wreg(REGID, val);} while(0);

tSpiConfiguration usb_conf;

static void
wregn(regid_t id, uint8_t *w, size_t n, bool ackstat)
{
  static cmd_t cmd;
  size_t i;

  cmd.ascmd.reg = id;
  cmd.ascmd.dir = WRITE;
  cmd.ascmd.ackstat = ackstat;
  cmd.ascmd.regval.raw = (w==NULL||n==0) ? 0x00 : w[0];

  //vAHI_SpiRestoreConfiguration(&usb_conf);
  vAHI_SpiWaitBusy();
  vAHI_SpiSelect(MAX3420_SPI_SLAVE); vAHI_SpiWaitBusy();
  vAHI_SpiStartTransfer16(cmd.raw);  vAHI_SpiWaitBusy();

  for(i=1; i<n; i++)
  {
    vAHI_SpiStartTransfer8( w[i] );
    vAHI_SpiWaitBusy();
  }

  vAHI_SpiSelect(0);
}

static void
wreg(regid_t id, reg_t val)
{
  wregn(id, (uint8_t*) &val.raw, sizeof(val.raw), false);
}

static void
rregn(regid_t id, uint8_t *r, size_t n, bool ackstat)
{
  static cmd_t cmd;
  size_t i;

  cmd.raw = 0x00;
  cmd.ascmd.reg = id;
  cmd.ascmd.dir = READ;
  cmd.ascmd.ackstat = ackstat;

  //vAHI_SpiRestoreConfiguration(&usb_conf);
  vAHI_SpiWaitBusy();
  vAHI_SpiSelect(MAX3420_SPI_SLAVE); vAHI_SpiWaitBusy();
  vAHI_SpiStartTransfer16(cmd.raw);  vAHI_SpiWaitBusy();

  if (n==0) return;

  // skip status byte
  if(r!=NULL) r[0]  = u16AHI_SpiReadTransfer16() & 0x00FF;
  else { u16AHI_SpiReadTransfer16(); };

  for(i=1; i<n; i++)
  {
    vAHI_SpiStartTransfer8(0xff); vAHI_SpiWaitBusy();
    r[i] = u8AHI_SpiReadTransfer8();
  }

  vAHI_SpiSelect(0);
}

static void
rreg(regid_t id, reg_t *r)
{
  rregn(id, &r->raw, sizeof(reg_t), false);
}

static void
usb_reset()
{
  reg_t val;

  wregb(USBCTL, CHIPRES=true);  // reset
  wregb(USBCTL, CHIPRES=false); // clear reset
  wregb(PINCTL, FDUPSPI=true);  // full duplex mode

  // wait until internal oscillator stabilized.
  do { rreg(USBIRQ, &val); } while(!val.USBIRQ.OSCOKIRQ);
}

static bool
spi_probe()
{
  reg_t val;
  uint8_t j, rd=0;

  usb_reset();
  for(j=0; j<8; j++)
  {
    val.raw = 1<<j; wreg(USBIEN, val);
    rreg(USBIEN, (reg_t*) &rd);
    if((1<<j)!=rd) return false;
  }
  usb_reset();

  return true;
}

#include "usb_aux.c"

static void
usb_irq(uint32_t a, uint32_t b)
{
  process_poll(&usb_process);
}

static bool
usb_init()
{
  static reg_t val;

  // well, configure the spi
  vAHI_SpiConfigure( 2,    // number of spi slave select line
                     E_AHI_SPIM_MSB_FIRST,
                     0,0,  // polarity and phase
                     1,    // clock divisor, 2x1 on the 16MHz Clock, so clock is at 8 Mhz
                     E_AHI_SPIM_INT_DISABLE,
                     E_AHI_SPIM_AUTOSLAVE_DSABL);

  // save the configuration
  //vAHI_SpiReadConfiguration(&usb_conf);
  if(!spi_probe())
  {
    int i = 250;
    while(i--)
    {
      clock_delay(20);
      leds_toggle(LEDS_ALL);
    }
    return false;
  }

  usb_reset();

  // we have come out of reset so assert irq states, should be 0x19
  rreg(EPIRQ, &val); if(val.raw != 0x19) HAL_BREAKPOINT();

  // enable startup irqs, i.e. plug events.
  wregb2(USBIEN, VBUSIE=true, NOVBUSIE=true);

  // clear irq states, but make sure VBUSIRQ is not cleared, since
  // we would not know if we are connected otherwise.
  val.raw = 0xff; wregn(EPIRQ, &val.raw, sizeof(val.raw), false);
  val.USBIRQ.VBUSIRQ = 0; wregn(USBIRQ, &val.raw, sizeof(val.raw), false);

  // configure irq pin on the µC, we use the default irq configuration
  // of the MAX3420, i.e. falling edge for pending irqs. Irqs will be
  // serviced by usb_irq().
  vAHI_DioSetDirection(MAX3420_INT_PIN, 0x00);
  vAHI_DioInterruptEdge(0x00, MAX3420_INT_PIN);
  vAHI_SysCtrlRegisterCallback(usb_irq);
  vAHI_DioInterruptEnable(MAX3420_INT_PIN, 0x00);

  // Call the irq handler once, prior to enabling irqs to catch the VBUS
  // irq if this line was asserted on startup.
  usb_irq(E_AHI_DEVICE_SYSCTRL, MAX3420_INT_PIN);
  wregb(CPUCTL, IE=1);

  return true;
}

size_t /* poll irq pin until data is available, framing is implemented here.
          This function will only return non-zero, when a read yields less
          than USBBUFSIZE bytes. On buffer overrun 0 will be returned. */
usb_read(endpoint_t target, uint8_t *buf, size_t n)
{
  /* there is only one endpoint to read from -> EP1 */
  size_t r = 0;
  reg_t ep = {0};
  u8_t m   = 0;

  /* check if data is available */
  rreg(EPIRQ, &ep);
  if (!ep.EPIRQ.OUT1DAVIRQ)
    return 0;

  while(r<n && (m%USBBUFSIZE)==0)
  {
    /* block until data available XXX: needs timeout*/
    do { rreg(EPIRQ, &ep); } while (!ep.EPIRQ.OUT1DAVIRQ);

    /* transfer length and buffer */
    m = 0; rregn(EP1OUTBC, &m, sizeof(m), false);
    if (r+m <= n) { rregn(EP1OUTFIFO, buf+r, m, false); r+=m; }
    else { printf("usb: buffer overrun\n"); r = 0; }

    /* clear flag to ack transfer */
    wregb(EPIRQ, OUT1DAVIRQ=true);
  }

  return r;
}

size_t
usb_write(endpoint_t target, uint8_t *buf, size_t n)
{
  size_t m = 0;
  u8_t w;
  reg_t ep = {0};
  regid_t to, tobc;

  switch(target)
  {
    case EP2:
      to   = EP2INFIFO;
      tobc = EP2INBC;
      break;
    case EP3:
      to   = EP3INFIFO;
      tobc = EP3INBC;
      break;
    case EP1: /* this is an OUT-endpoint */
    default:
      return 0;
  }

  while (m<n)
  {
    if (n-m > USBBUFSIZE) w = USBBUFSIZE;
    else w = n-m;

    /* wait unti buffer available */
    if (target==EP2)
      do {  rreg(EPIRQ, &ep); } while (!ep.EPIRQ.IN2BAVIRQ);
    else if (target==EP3)
      do {  rreg(EPIRQ, &ep); } while (!ep.EPIRQ.IN3BAVIRQ);

    /* arm buffer for transfer */
    wregn(to, buf+m, w, false);
    wregn(tobc, &w, sizeof(w), false);

    m += w;
  }

  /* short packet ack */
  if (m%USBBUFSIZE==0)
  {
    if (target==EP2)
      do {  rreg(EPIRQ, &ep); } while (!ep.EPIRQ.IN2BAVIRQ);
    else if (target==EP3)
      do {  rreg(EPIRQ, &ep); } while (!ep.EPIRQ.IN3BAVIRQ);

    w = 0; wregn(tobc, &w, sizeof(w), false);
  }

  return m;
}

size_t
usb_write_nonblock(endpoint_t target, uint8_t *buf, size_t n)
{
  size_t m = 0;
  u8_t w;
  reg_t ep = {0};
  regid_t to, tobc;

  switch(target)
  {
    case EP2:
      to   = EP2INFIFO;
      tobc = EP2INBC;
      break;
    case EP3:
      to   = EP3INFIFO;
      tobc = EP3INBC;
      break;
    case EP1: /* this is an OUT-endpoint */
    default:
      return 0;
  }

  while (m<n)
  {
    if (n-m > USBBUFSIZE) w = USBBUFSIZE;
    else w = n-m;

    /* arm buffer for transfer */
    wregn(to, buf+m, w, false);
    wregn(tobc, &w, sizeof(w), false);

    m += w;
  }

  return m;
}

static void setup_datareq()
{
}

PROCESS(usb_process, "USB MAX3420 driver");
PROCESS_THREAD(usb_process, ev, data)
{
  static reg_t usb, ep;

  if (ev == PROCESS_EVENT_POLL) {
    rreg(USBIRQ, &usb);
    rreg(EPIRQ, &ep);
  } else {
    usb.raw = ep.raw = 0x00;
  }

  /* setup data requests need to be handled in anyway.
   * NOTE: suspend irq will be enabled here, and disabled on reset */
  if (ep.EPIRQ.SUDAVIRQ)
  {
    static setup_t sp;
    rregn(SUDFIFO, (uint8_t*) &sp, sizeof(sp), false);

    if (usb_setup(&sp))
    {
      process_post(usbdev.process, usb_event, (void*) EVENT_USB_ENUMERATED);
      wregb(USBIEN, SUSPIE=true);
      PRINTF("usb: suspend irq enabled\n");
    }

    wregb(EPIRQ, SUDAVIRQ=true);
  }

  PROCESS_BEGIN();
  PROCESS_PAUSE();

  usb_event = process_alloc_event();

  /* setup irq handler and spi communication */
  usb_init();

  /* wait for VBUS irq */
  PROCESS_YIELD_UNTIL(usb.USBIRQ.VBUSIRQ);
  wregb(USBIRQ, VBUSIRQ=true); /* clr irq */

  /* got VBUS, enable irqs, connect to the bus */
  wregb4(USBIEN, URESIE=true, URESDNIE=true, NOVBUSIE=true, SUSPIE=true);
  wregb(USBCTL, CONNECT=true);

  PRINTF("usb: vbus\n");

  /* while not disconnected */
  while(!usb.USBIRQ.NOVBUSIRQ)
  {
    /* wait for BUS RESET */
    PROCESS_YIELD_UNTIL(usb.USBIRQ.URESIRQ);
    wregb(USBIRQ, URESIRQ=true); /* clr irq */

    /* wait for BUS RESET DONE */
    PROCESS_YIELD_UNTIL(usb.USBIRQ.URESDNIRQ);
    wregn(USBIRQ, (uint8_t*) &usb, sizeof(usb), false); /* clr irqs */

    /* reenable irqs, since disabled by bus reset */
    wregb(EPIEN, SUDAVIE=true);

    PRINTF("usb: main loop\n");

    /* stay here until VBUS is gone or BUS RESET happened */
    while(!usb.USBIRQ.URESIRQ && !usb.USBIRQ.NOVBUSIRQ)
    {
      if (usb.USBIRQ.SUSPIRQ)
      {
        /* suspended -> power down, clear bus active and suspended irq */
        //wregb(USBCTL, PWRDOWN=true);
        wregb(USBIRQ, SUSPIRQ=true) //, BUSACTIRQ=true);
        PRINTF("usb: suspended\n");

        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL &&
                (usb.USBIRQ.BUSACTIRQ ||
                 usb.USBIRQ.URESIRQ   ||
                 usb.USBIRQ.NOVBUSIRQ));

        if (usb.USBIRQ.BUSACTIRQ)
        {
          /* resume normal operation, reset irq state */
          PRINTF("usb: wokeup\n");
          wregb(USBIRQ, BUSACTIRQ=true);
        }
        else
        {
          PRINTF("usb: got reset or disconnected while waiting for resume\n");
        }
      }

      PROCESS_YIELD_UNTIL(ev==PROCESS_EVENT_POLL);
    }

    if (usb.USBIRQ.URESIRQ)   { PRINTF("usb: reset\n");   }
    if (usb.USBIRQ.NOVBUSIRQ) { PRINTF("usb: no vbus\n"); }
    process_post(usbdev.process, usb_event, (void*) EVENT_USB_DISCONNECTED);
  }

  /* disconnect */
  wregb(USBCTL, CONNECT=false);
  PRINTF("usb: disconnect\n");

  PROCESS_EXIT();
  PROCESS_END();
}
