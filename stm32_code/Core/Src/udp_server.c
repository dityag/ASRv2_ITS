/*
 * udp_server.c
 *
 *  Created on: Jan 26, 2023
 *      Author: ismarintan
 */

#include "udp_server.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"


/* Global variable */
// * Pointer Data Buffer Rx and Tx
uint8_t *Buffer_Rx;
uint8_t *Buffer_Tx;
int size_buffer;

uint32_t epoch_kirim;

err_t udp_serverStat;

uint32_t UDP_Recv_TimeStamp;



void udp_server_init(uint8_t *dataRx, uint8_t *dataTx, uint16_t PORT, int size)
{
    struct udp_pcb *upcb;
    err_t err;

    /* Create a new UDP control block  */
    upcb = udp_new();

    if (upcb!=NULL)
    {
        /* Bind the upcb to the UDP_PORT port */
        /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
        err = udp_bind(upcb, IP_ADDR_ANY, PORT);

        if(err == ERR_OK)
        {
            /* Set a receive callback for the upcb */
            udp_recv(upcb, udp_server_receive_callback, NULL);


            /* Copy the dataRx and dataTx pointer to the global variable */
            Buffer_Rx = dataRx;
            Buffer_Tx = dataTx;
            size_buffer = size;

        }

        udp_serverStat = err;
    }
}

void udp_server_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{

    if(p != NULL)
    {

    	UDP_Recv_TimeStamp = HAL_GetTick();

        /* Copy the data from the pbuf to the Buffer_Rx */
        memcpy(Buffer_Rx, p->payload, p->len);
        memcpy(Buffer_Tx, &epoch_kirim, sizeof(epoch_kirim));
        epoch_kirim++;

        // Send data buffer tx to the client
        struct pbuf *p_tx;

        /* Allocate pbuf from pool*/
        p_tx  = pbuf_alloc(PBUF_TRANSPORT, size_buffer, PBUF_RAM);

        if (p_tx != NULL)
        {
            /* Copy the data to the pbuf */


            memcpy(p_tx->payload, Buffer_Tx, size_buffer);

            /* Send the pbuf */
            udp_sendto(upcb, p_tx, addr, port);

            /* Free the pbuf */
            pbuf_free(p_tx);
        }

        /* Free the pbuf */
        pbuf_free(p);

    }

}






