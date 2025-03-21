// server program for udp connection
#include <stdio.h>
#include <strings.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#define PORT 5000
#define MAXLINE 1000

// Driver code
int main()
{
    char buffer[100];
    char message[64];
    int data = 0;
    int listenfd;
    socklen_t len;
    struct sockaddr_in servaddr, cliaddr;
    bzero(&servaddr, sizeof(servaddr));

    int cnt = 0;

    // Create a UDP Socket
    listenfd = socket(AF_INET, SOCK_DGRAM, 0);
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    servaddr.sin_port = htons(PORT);
    servaddr.sin_family = AF_INET;

    // bind server address to socket descriptor
    bind(listenfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

    while(1)
    {
    // receive the datagram in non blocking mode
    len = sizeof(cliaddr);
    int n = recvfrom(listenfd, buffer, sizeof(buffer),
            MSG_DONTWAIT, (struct sockaddr*)&cliaddr,&len);
    if(n>0)
    {
        buffer[n] = 0;
        printf("Client : %s\n", buffer);

        sprintf(message, "Data value is : %d", data);

        sendto(listenfd, message, MAXLINE, 0,
            (struct sockaddr*)&cliaddr, sizeof(cliaddr));


    }

    if(cnt++>1000)
    {
        cnt = 0;
        data ++;
        printf("Data send to client: %d\n", data);
    }


    //delay 1 millisecond
    usleep(1000);

    }

}
