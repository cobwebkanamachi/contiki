/*
 * Copyright (c) 2013, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
  *
 */
/**
 * \file
 *         Tools for logging RPL state and tracing data packets
 *
 * \author Simon Duquennoy <simonduq@sics.se>
 */

#include "contiki.h"
#include "deployment.h"
#include "net/rpl/rpl.h"
#include "net/rpl/rpl-private.h"
#include "net/packetbuf.h"
#include "tools/rpl-log.h"
#include <stdio.h>
#include <string.h>

static neighbor_set_size = 0;

/* Print all neighbors (RPL "parents"), their link metric and rank */
static void
rpl_print_neighbor_list() {
	  rpl_parent_t *p = nbr_table_head(rpl_parents);
	  printf("RPL: neighbor list\n");
	  while(p != NULL) {
		  printf("RPL: nbr %d %u + %u = %u %c\n",
				  node_id_from_rimeaddr(nbr_table_get_lladdr(rpl_parents, p)), p->rank, p->link_metric, p->rank + p->link_metric, p==default_instance->current_dag->preferred_parent?'*':' ');
		  p = nbr_table_next(rpl_parents, p);
	  }
	  printf("RPL: end of neighbor list\n");
}

/* Copy an appdata to another with no assumption that the addresses are aligned */
void
appdata_copy(struct app_data *dst, struct app_data *src)
{
  int i;
  for(i=0; i<sizeof(struct app_data); i++) {
    ((char*)dst)[i] = (((char*)src)[i]);
  }
}

/* Get dataptr from the packet currently in uIP buffer */
struct app_data *
appdataptr_from_uip()
{
  return (struct app_data *)((char*)uip_buf + ((uip_len - sizeof(struct app_data))));
}

/* Get dataptr from the current packetbuf */
struct app_data *
appdataptr_from_packetbuf()
{
  if(packetbuf_datalen() < sizeof(struct app_data)) return 0;
  return (struct app_data *)((char*)packetbuf_dataptr() + ((packetbuf_datalen() - sizeof(struct app_data))));
}

/* Log information about a data packet along with RPL routing information */
void
log_appdataptr(struct app_data *dataptr)
{
  struct app_data data;
  int curr_dio_interval = default_instance != NULL ? default_instance->dio_intcurrent : 0;
  int curr_rank = default_instance != NULL ? default_instance->current_dag->rank : 0xffff;

  if(dataptr) {
    appdata_copy(&data, dataptr);

    printf(" [%lx %u_%u %u->%u]",
        data.seqno,
        data.hop,
        data.fpcount,
        data.src,
        data.dest
        );
  }

  if(neighbor_set_size == 0) {
  	neighbor_set_size = uip_ds6_nbr_num();
  }

  printf(" {%u %u %u} \n",
  		  neighbor_set_size,
        curr_rank,
        curr_dio_interval
        );
}

/* Return node id from its rime address */
uint16_t
log_node_id_from_rimeaddr(const void *rimeaddr)
{
  return node_id_from_rimeaddr((const rimeaddr_t *)rimeaddr);
}

/* Return node id from its IP address */
uint16_t
log_node_id_from_ipaddr(const void *ipaddr)
{
  return node_id_from_ipaddr((const uip_ipaddr_t *)ipaddr);
}

PROCESS(rpl_log_process, "RPL Log");
/* Starts logging process */
void
rpl_log_start() {
  process_start(&rpl_log_process, NULL);
}

/* The logging process */
PROCESS_THREAD(rpl_log_process, ev, data)
{
  static struct etimer periodic;
  PROCESS_BEGIN();
  etimer_set(&periodic, 60 * CLOCK_SECOND);
  simple_energest_init();

  while(1) {
  	static int cnt = 0;
  	neighbor_set_size = uip_ds6_nbr_num();

    PROCESS_WAIT_UNTIL(etimer_expired(&periodic));
    etimer_reset(&periodic);
    simple_energest_step();

    if(cnt++ % 8 == 0) {
    	rpl_print_neighbor_list();
    }
  }

  PROCESS_END();
}
