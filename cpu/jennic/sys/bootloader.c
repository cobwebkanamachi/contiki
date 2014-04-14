/*
 * Copyright (c) 2008-2010
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

#include <stdint.h>
#include <string.h>
#include "gdb2.h"
#include "contiki-net.h"
#include "AppHardwareApi.h"
#include <stdbool.h>

PROCESS(jennic_bootloader_process, "JN5139 bootloader");

typedef enum
{
  IDLE,
  FLASHING,
  SENDING
} bl_appstate_t;

struct bl_appstate
{
  bl_appstate_t state;
  uint32_t write_addr, write_len, msg_len;
  bool reset_req;
};

typedef enum
{
  REQ_FLASH_ERASE  = 0x07, RSP_FLASH_ERASE,
  REQ_FLASH_PROGR  = 0x09, RSP_FLASH_PROGR,
  REQ_FLASH_READ   = 0x0B, RSP_FLASH_READ,
  REQ_SECTOR_ERASE = 0x0C, RSP_SECTOR_ERASE,
  REQ_SR_WRITE     = 0x0E, RSP_SR_WRITE,
  REQ_RAM_WRITE    = 0x1D, RSP_RAM_WRITE,
  REQ_RAM_READ     = 0x1F, RSP_RAM_READ,
  REQ_RUN          = 0x21, RSP_RUN,
  REQ_FLASH_ID     = 0x25, RSP_FLASH_ID,
  REQ_FLASH_SELECT = 0x2C, RSP_FLASH_SELECT,
  REQ_FLASH_PROG2  = 0x2E, RSP_FLASH_PROG2,
} PACKED cmd_type;

typedef enum
{
  OK,
  NO_RESPONSE = 0xF6,
  AUTH_ERROR,
  TST_ERROR,
  READ_FAIL,
  USER_INTERRUPT,
  ASSERT_FAIL,
  CRC_ERROR,
  INVALID_RESPONSE,
  WRITE_FAIL,
  NOT_SUPPORTED
} PACKED cmd_status;

#define MSG_HDR_SIZE  (3*sizeof(uint8_t))
#define MSG_SIZEOF(x) ((uint8_t) (sizeof(x)+MSG_HDR_SIZE))
#define MSG_MAXDATA   251

typedef struct
{
  uint8_t len, type;
  union
  {
    uint8_t asFlashEraseRsp;
    struct  { uint32_t addr; uint8_t data[MSG_MAXDATA]; } PACKED asProgramReq;
    uint8_t asProgramRsp;
    struct  { uint32_t addr; uint16_t len; } PACKED asReadReq;
    struct  { uint8_t status; uint8_t data[MSG_MAXDATA]; } PACKED asReadRsp;
    uint8_t asSectorReq;
    uint8_t asSectorRsp;
    u8_t asSRReq;
    uint8_t asSRRsp;
    struct  { uint32_t addr; uint8_t data[MSG_MAXDATA]; } PACKED asRamWriteReq;
    uint8_t asRamWriteRsp;
    struct  { uint32_t addr; uint16_t len; } PACKED asRamReadReq;
    struct  { uint8_t status; uint8_t data[MSG_MAXDATA]; } PACKED asRamReadRsp;
    struct  { uint32_t addr; } PACKED asRunReq;
    uint8_t asRunRsp;
    struct  { uint8_t status; uint8_t manufacturer; uint8_t device; } PACKED asFlashIdRsp;
    struct  { uint8_t flashtype; uint32_t addr; } PACKED asSelectReq;
    uint8_t asSelectRsp;
    struct  { uint32_t len; } PACKED asFlashWrite2;
    uint8_t asFlashWrite2Rsp;
  };
  uint8_t chksum;
} PACKED msg_t;

extern char *uip_sappdata;

static void
appcall(void *p)
{
  static struct bl_appstate s = { IDLE, 0x00, 0x00, false };
  clock_time_t time = clock_time();
  msg_t *msg = uip_appdata, *smsg = uip_sappdata;

  if(uip_connected())
  {
    s.state = IDLE;
    s.reset_req = false;
    s.write_addr = 0;
    s.write_len  = 0;
    s.msg_len    = 0;
  }

  if(uip_closed()   ||
     uip_aborted()  ||
     uip_timedout())
  {
    uip_close();

    if(s.reset_req && uip_closed())
      vAHI_SwReset(); /* at this point the system RESETS */

    s.state = IDLE;
  }

  if(uip_acked())
  {
    if (s.state==SENDING) { s.msg_len = 0; s.state = IDLE; };
  }

  if(uip_newdata())
  {
    if (s.state == FLASHING)
    {
      bAHI_FullFlashProgram(s.write_addr, uip_datalen(), (uint8_t*) msg);
      s.write_addr += uip_datalen();

      if(s.write_addr >= s.write_len)
      {
        s.write_len = 0;
        s.state     = SENDING;
        smsg->type  = RSP_FLASH_PROG2;
        smsg->len   = MSG_SIZEOF(smsg->asFlashWrite2Rsp);
        smsg->asFlashWrite2Rsp = OK;
      }
    }
    else if(s.state != SENDING)
    {
      s.msg_len += uip_datalen();

      if(s.msg_len == msg->len)
      {
        s.msg_len = 0;
        s.state   = SENDING;

        switch(msg->type)
        {
          case REQ_FLASH_ERASE:
            smsg->asFlashEraseRsp = bAHI_FlashEraseSector(0) ? OK : NOT_SUPPORTED;
            smsg->asFlashEraseRsp = bAHI_FlashEraseSector(1) ? OK|smsg->asFlashEraseRsp : NOT_SUPPORTED;
            smsg->asFlashEraseRsp = bAHI_FlashEraseSector(2) ? OK|smsg->asFlashEraseRsp : NOT_SUPPORTED;
            smsg->asFlashEraseRsp = bAHI_FlashEraseSector(3) ? OK|smsg->asFlashEraseRsp : NOT_SUPPORTED;
            smsg->asFlashEraseRsp = OK;
            smsg->type = RSP_FLASH_ERASE;
            smsg->len  = MSG_SIZEOF(smsg->asFlashEraseRsp);
            break;

          case REQ_FLASH_PROGR:
            smsg->asProgramRsp = bAHI_FullFlashProgram(msg->asProgramReq.addr,
                msg->len-MSG_HDR_SIZE-sizeof(msg->asProgramReq.addr),
                msg->asProgramReq.data) ? OK : NOT_SUPPORTED;
            smsg->type = RSP_FLASH_PROGR;
            smsg->len  = MSG_SIZEOF(smsg->asProgramRsp);
            break;

          case REQ_FLASH_READ:
            smsg->type = RSP_FLASH_READ;
            smsg->len  = MIN(MSG_SIZEOF(smsg->asReadRsp),
                MSG_HDR_SIZE+(sizeof(smsg->asReadRsp)-sizeof(smsg->asReadRsp.data))
                +msg->asReadReq.len);
            smsg->asReadRsp.status =
              bAHI_FullFlashRead(msg->asReadReq.addr,
                MIN(sizeof(smsg->asReadRsp.data), msg->asReadReq.len),
                smsg->asReadRsp.data) ? OK : NOT_SUPPORTED;
            break;

          case REQ_SECTOR_ERASE:
            smsg->asSelectRsp = OK;
            smsg->asSectorRsp = bAHI_FlashEraseSector(msg->asSectorReq) ? OK : NOT_SUPPORTED;
            smsg->type        = RSP_SECTOR_ERASE;
            smsg->len         = MSG_SIZEOF(smsg->asSectorRsp);
            break;

          case REQ_SR_WRITE:
            smsg->asSRRsp = NOT_SUPPORTED;
            smsg->type    = RSP_SR_WRITE;
            smsg->len     = MSG_SIZEOF(smsg->asSRRsp);
            break;

          case REQ_RAM_WRITE:
            memcpy((uint32_t*) smsg->asRamWriteReq.addr, smsg->asRamWriteReq.data,
                smsg->len-MSG_HDR_SIZE);
            smsg->asRamWriteRsp = OK;
            smsg->type = RSP_RAM_WRITE;
            smsg->len  = MSG_SIZEOF(smsg->asRamWriteRsp);
            break;

          case REQ_RAM_READ:
            smsg->type = RSP_RAM_READ;
            smsg->len  = MIN(MSG_SIZEOF(smsg->asRamReadRsp), MSG_HDR_SIZE+
                (sizeof(smsg->asRamReadRsp)-sizeof(smsg->asRamReadRsp.data))+msg->asRamReadReq.len);
            memcpy(smsg->asRamReadRsp.data, (uint32_t*) msg->asRamReadReq.addr,
                MIN(sizeof(smsg->asRamReadRsp.data), msg->asRamReadReq.len));
            break;

          case REQ_RUN:
            s.reset_req = true;
            smsg->asRunRsp = OK;
            smsg->type = RSP_RUN;
            smsg->len  = MSG_SIZEOF(msg->asRunReq);
            break;

          case REQ_FLASH_ID:
            smsg->asFlashIdRsp.status = OK;
            smsg->asFlashIdRsp.manufacturer = smsg->asFlashIdRsp.device = 0x10;
            smsg->type = RSP_FLASH_ID;
            smsg->len  = MSG_SIZEOF(smsg->asFlashIdRsp);
            break;

          case REQ_FLASH_SELECT:
            smsg->asSelectRsp = bAHI_FlashInit(msg->asSelectReq.flashtype,
                (tSPIflashFncTable*) msg->asSelectReq.addr) ? OK : NOT_SUPPORTED;
            smsg->type = RSP_FLASH_SELECT;
            smsg->len  = MSG_SIZEOF(smsg->asSelectRsp);
            break;

          case REQ_FLASH_PROG2:
            s.state      = FLASHING;
            s.write_addr = 0x00000000;
            s.write_len  = msg->asFlashWrite2.len;
            break;

          default:
            smsg->asSRRsp = NOT_SUPPORTED;
            smsg->type = RSP_SR_WRITE;
            smsg->len  = MSG_SIZEOF(smsg->asSRRsp);
        }
      }
    }
  }

  if(uip_rexmit()    ||
     uip_newdata()   ||
     uip_acked()     ||
     uip_connected() ||
     uip_poll())
  {
    if(s.state==SENDING)
      uip_send(smsg, smsg->len);
  }

  /* this is odd but needed to get tcp throughput rate in sync with flash
   * write rate, otherwise rtt is calculated as the rtt of small packets which
   * is too fast when writing to the flash has started, therefore limiting
   * the overall throughput of the bootloader protocol. */
  time = clock_time() - time;
  if (time < CLOCK_SECOND/12)
    clock_delay(CLOCK_SECOND/12 - time);
}

PROCESS_THREAD(jennic_bootloader_process, ev, data)
{
  PROCESS_BEGIN();
  tcp_listen(UIP_HTONS(2048));

  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(ev == tcpip_event);
    appcall(data);
  }

  PROCESS_END();
}
