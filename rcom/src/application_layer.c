// Application layer protocol implementation

#include <string.h>
#include "application_layer.h"
#include "link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer l;
    strcpy(l.serialPort,serialPort);
    l.role=role;
    l.baudRate=baudRate;
    l.nRetransmissions=nTries;
    l.timeout=timeout;
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
    
}
