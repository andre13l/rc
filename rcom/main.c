// Main file of the serial port project.
// NOTE: This file must not be changed.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <link_layer.h>

#define BAUDRATE 9600
#define N_TRIES 3
#define TIMEOUT 4

// Arguments:
//   $1: /dev/ttySxx
//   $2: tx | rx
//   $3: filename
int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        printf("Usage: %s /dev/ttySxx tx|rx filename\n", argv[0]);
        exit(1);
    }

    printf("%s %s %s\n", argv[1],argv[2],argv[3]);
    fflush(stdout);

    const char *serialPort = argv[1];
    const char *role = argv[2];
    const char *filename = argv[3];

    LinkLayer l;
    strcpy(l.serialPort,serialPort);
    l.baudRate=BAUDRATE;
    l.nRetransmissions=N_TRIES;
    l.timeout=TIMEOUT;
    if(strcmp(role,"tx")==0){
        l.role=LlTx;
        if(llopen(l)!=1){
            sleep(1);
            llclose(0);
            return;
        }
        unsigned char buf[256];
        for(unsigned int i =0;i<256;++i){
            buf[i]=i;
        }
        printf("llwrite:%i\n",llwrite(buf,256));
    }
    else if(strcmp(role,"rx")==0){
        l.role=LlRx;
        if(llopen(l)!=1){
            sleep(1);
            llclose(0);
            return;
        }
        unsigned char buf[256];
        llread(buf);
        int flag=0;
        for(unsigned int i =0;i<256;++i){
            if(buf[i]!=i){
                printf("llread[%i] error.\n",i);
                flag=1;
            }
        }
        if(!flag)
            printf("llread succeeded.\n");
    }
    sleep(1);
    llclose(0);

    return 0;
}
