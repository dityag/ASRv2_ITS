/*
 * udp_server.h
 *
 *  Created on: Jan 26, 2023
 *      Author: ismarintan
 */

#ifndef INC_UDP_SERVER_H_
#define INC_UDP_SERVER_H_

#include "main.h"
#include "lwip.h"
#include "lwip/udp.h"

extern err_t udp_serverStat;
extern uint32_t UDP_Recv_TimeStamp;
extern uint32_t epoch_kirim;

void udp_server_init(uint8_t *dataRx, uint8_t *dataTx, uint16_t PORT, int size);
void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);


#endif /* INC_UDP_SERVER_H_ */
