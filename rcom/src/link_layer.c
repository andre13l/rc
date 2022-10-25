// Link layer protocol implementation

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>
#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayer connection;
Statistics stats;
struct termios oldtio;
struct termios newtio;

#define FLAG (0b01111110)
#define ESCAPE (0x7d)
#define ESCAPE_FLAG (0x5e)
#define ESCAPE_ESCAPE (0x5d)
#define ADR_TX (0b00000011)
#define ADR_RX (0b00000001)
#define SET (0b00000011)
#define DISC (0b00001011)
#define UA (0b00000111)
#define DATA(S) ((S)%2?0b01000000:0b00000000)
#define C_RR(R) ((R)%2?0b10000101:0b00000101)
#define C_REJ(R) ((R)%2?0b10000001:0b00000001)
#define PACKET_SIZE_LIMIT (128)
int fd;
int DISC_rec = 0;
unsigned char alarmEnabled=0, alarm_count=0;
unsigned char buf[128];
unsigned char *biggerbuf =NULL;
unsigned char flag_dados = 0;

typedef struct {
    enum state_t { S,F,A,C,BCC1,SDATA,ESC,BCC2,E,REJ} state;
    unsigned char adr;
    unsigned char ctrl;
    unsigned char bcc;
    unsigned char *data;
    unsigned int data_size;
} State;
State state;

int stuffing(const unsigned char *buffer, int bufSize, unsigned char* pos, unsigned char *bcc){
    int size=0;
    for(unsigned int i=0; i<bufSize ; i++){
        if(bcc!=NULL)
            *bcc^=buffer[i];
        if(buffer[i]==FLAG){
            pos[size++]=ESCAPE;
            pos[size++]=ESCAPE_FLAG;
            break;
        }
        else if(buffer[i]==ESCAPE){
            pos[size++]=ESCAPE;
            pos[size++]=ESCAPE_ESCAPE;
            break;
        }
        pos[size++]=buffer[i];
    }
    return size;
}
int trama(unsigned char* array, unsigned char A, unsigned char C){
    array[0]= FLAG;
    array[1]= A;
    array[2]= C;
    array[3]= A ^ C;
    array[4]= FLAG;
    return 5;
}


void alarmHandler(int signal){
    ++alarm_count;
    alarmEnabled=0;
}

void state_machine(unsigned char byte,State* state){
    switch (state->state){
        case REJ:
        case E:
            state->state=S;
        case S:
            if(byte==FLAG)
                state->state=F;
            break;
        case F:
            state->data_size=0;
            if(byte==FLAG){
                break;
            }
            if(byte==ADR_TX || byte == ADR_RX){
                state->state=A;
                state->adr=byte;
                break;
            }
            state->state=S;
            break;
        case A:
            if(byte==FLAG){
                state->state=F;
                break;
            }
            if(byte==DISC || byte==SET 
            || byte==UA || byte == C_REJ(0) 
            || byte==C_RR(0) || byte == C_REJ(1) 
            || byte==C_RR(1) || byte == DATA(0)
            || byte==DATA(1)){
                state->state = C;
                state->ctrl = byte;
                state->bcc = state->adr ^ state->ctrl;
                break;
            }
            state->state=S;
            break;
        case C:
            if( byte == state->bcc){
                state->state=BCC1;
                break;
            }
            if(byte==FLAG){
                state->state = F;
                break;
            }
            state->state=S;
            break;
        case BCC1:
            if(byte==FLAG){
                if(state->ctrl==DATA(0) || state->ctrl==DATA(1)){
                    state->state=F;
                    break;
                }
                state->state=E; 
                break;
            }
            if((state->ctrl==DATA(0) || state->ctrl==DATA(1) ) && state->data != NULL){
                state->data_size=0;
                if(byte==ESCAPE){
                    state->bcc=0;
                    state->state=ESC;
                    break;       
                }
                state->data[state->data_size++]=byte;
                state->bcc=byte;
                state->state=SDATA;
                break;
            }
            state->state=S;
            break;
        case SDATA:
            if(byte==ESCAPE){
                state->state=ESC;
                break;       
            }
            if(byte==FLAG){
                state->state=REJ;
                break;
            }
            if(byte==state->bcc){
                state->state=BCC2;
                break;
            }
            state->data[state->data_size++]=byte;
            state->bcc^=byte;
            break;
        case ESC:
            if(byte==FLAG){
                state->state=REJ;
                break;
            }
            if(byte==ESCAPE_FLAG){
                if(state->bcc==FLAG){
                    state->state=BCC2;
                    break;
                }
                state->bcc^=FLAG;
                state->data[state->data_size++]=FLAG;
                state->state=SDATA;
                break;
            }
            if(byte==ESCAPE_ESCAPE){
                if(state->bcc==ESCAPE){
                    state->state=BCC2;
                    break;
                }
                state->bcc^=ESCAPE;
                state->data[state->data_size++]=ESCAPE;
                state->state=SDATA;
                break;
            }
            state->state=S;
            break;
        case BCC2:
            if(byte==FLAG){
                state->state=E;
                break;
            }
            if(byte==0){
                state->data[state->data_size++]=state->bcc;
                state->bcc=0;
                break;
            }
            if(byte==ESCAPE){
                state->data[state->data_size++]=state->bcc;    
                state->bcc=0;
                state->state=ESC;
                break;
            }
            state->data[state->data_size++]=state->bcc;
            state->data[state->data_size++]=byte;
            state->bcc=byte;
            state->state=SDATA;
            break;
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    connection=connectionParameters;
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(connection.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(connection.serialPort);
        exit(-1);
    }
    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connection.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until 1 char received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }
    
    signal(SIGALRM,alarmHandler);

    if(connection.role==LlTx){ //Transmitter
        state.state=S;
        alarm_count=0;
        while(alarm_count<connection.nRetransmissions){
            alarm(connection.timeout);
            alarmEnabled=1;
            if(alarm_count>0){ 
                stats.timeouts++;
                printf("Timed out.\n");}
            int size = trama(buf,ADR_TX,SET);
            printf("Sent SET.\n");
            write(fd,buf,size);
            do{
                int lido = read(fd,buf,PACKET_SIZE_LIMIT);
                if(lido<0)
                    return -1;
                for(unsigned int i=0;i<lido;++i){
                    state_machine(buf[i],&state);
                    if(state.state==E && state.adr==ADR_TX && state.ctrl == UA)
                        {
                       printf("Received UA.\n");
                       return 1;
                    }
                }
            }while(alarmEnabled);
        }
        
    }
    else{ 
        alarm_count=0;
        state.state=S;
        int SET_rec=0;
        do{
            int lido = read(fd,buf,PACKET_SIZE_LIMIT);
            if(lido<0)
                return -1;
            for(unsigned int i=0;i<lido && !SET_rec;++i){
                state_machine(buf[i],&state);
                if(state.state==E && state.adr==ADR_TX && state.ctrl == SET)
                    SET_rec=1;
            }
        }while(!SET_rec);
        if(SET_rec) printf("Received Set.\n");
        int size=trama(buf,ADR_TX,UA);
        write(fd,buf,size);
        printf("Sent UA.\n");
        return 1;
    }
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buffer, int bufSize)
{
    biggerbuf=malloc(bufSize*2+10);

    biggerbuf[0]= FLAG;
    biggerbuf[1]= ADR_TX;
    biggerbuf[2]= DATA(flag_dados);
    biggerbuf[3]= ADR_TX ^ DATA(flag_dados);
    int size=0;
    unsigned char bcc=0;
    for(unsigned int i=0;i<bufSize;++i){
        size+=stuffing(buffer+i,1,biggerbuf+size+4,&bcc);
    }
    size+=stuffing(&bcc,1,biggerbuf+size+4,NULL);
    biggerbuf[4+size]=FLAG;
    size+=5;
    for(unsigned int i=0;i<size;){ 
                int ret=write(fd,biggerbuf+i,size-i);
                if(ret==-1)
                    return -1;
                i+=ret;
            } 
    int got_packet=0, send_again=0, retransmit=0;
    state.data=NULL;
    
    alarmEnabled=1;
    alarm(connection.timeout);
    while(!got_packet){
        if(!alarmEnabled){
            send_again=1;
            alarmEnabled=1;
            alarm(connection.timeout);
            if(retransmit == connection.nRetransmissions){
                printf("Exceeded retransmission limit.\n");
                return -1;
            if(retransmit>0){ 
                stats.timeouts++;
                printf("Timed out.\n");}
            }
            for(unsigned int j=0;j<size;){ 
                int ret=write(fd,biggerbuf+j,size-j);
                if(ret==-1)
                    return -1;
                j+=ret;
            }
            send_again = 0;
            retransmit++;
            stats.retransmissions;
        }        
        int lido = read(fd,buf,PACKET_SIZE_LIMIT);
        if(lido<0)
            return -1;
        for(unsigned int i=0;i<lido;++i){ 
            if(!got_packet && alarmEnabled && state.adr==ADR_TX){
            state_machine(buf[i],&state);
            if(state.state==E){
                if(state.ctrl == C_REJ(flag_dados) ){
                    printf("Waiting.\n");
                }
                if(state.ctrl == C_RR(0) || state.ctrl == C_RR(1)){ 
                    got_packet = 1;
                    printf("Requesting next packet.\n");
                }
            }
            }
        }
    }
    flag_dados = flag_dados?0:1;
    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{

    biggerbuf=malloc(PACKET_SIZE_LIMIT);
        
    int got_packet=0;
    state.data=packet; 
    while(!DISC_rec){
        int lido = read(fd,biggerbuf,PACKET_SIZE_LIMIT);
        if(lido<0)
            return -1;
        for(unsigned int i=0;i<lido;i++){
            state_machine(biggerbuf[i],&state);

            if(state.state==REJ && state.adr==ADR_TX){
                int size=trama(buf,ADR_TX,(state.ctrl==DATA(0)?C_REJ(0):C_REJ(1)));
                write(fd,buf,size);

                printf("Just Sent REJ.\n");
            }
            if(state.state==E && state.adr==ADR_TX){
                if(state.ctrl == SET){
                    int size=trama(buf,ADR_TX,UA);
                    write(fd,buf,size);   
    
                    printf("Just Sent UA.\n");
                }
                else if(state.ctrl == DATA(flag_dados)){
                    if(flag_dados==0){
                        flag_dados=1;
                    }
                    else{
                        flag_dados=0;
                    }
                    int size=trama(buf,ADR_TX,C_RR(flag_dados));
                    write(fd,buf,size);
    
                    printf("Just Sent RR %i.\n",flag_dados);
                    stats.frames_received++;
                    return state.data_size;
                }
                else{
                    int size=trama(buf,ADR_TX,C_RR(flag_dados));
                    write(fd,buf,size);
    
                    printf("Just Sent RR %i requesting retransmission.\n",flag_dados);
                }
            }
            if(state.ctrl==DISC) {
                DISC_rec = 1;
                int size=trama(buf,ADR_TX,(state.ctrl==DATA(0)?C_REJ(0):C_REJ(1)));
                write(fd,buf,size); 

                printf("Just Received DISC.\n");
                return -1; 
                break;
            }

        }
    }
    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    signal(SIGALRM,alarmHandler);
    
    if(connection.role==LlTx) { 

        int DISC_rec=0;

        alarm_count=0;
        while(alarm_count<connection.nRetransmissions && !DISC_rec){
            alarm(connection.timeout);
            alarmEnabled=1;
            if(alarm_count>0){ 
                stats.timeouts++;
                printf("Timed out.\n");}
            int size = trama(buf,ADR_TX,DISC);
            printf("Just Sent DISC.\n");
            write(fd,buf,size);
            while(alarmEnabled && !DISC_rec){
                int lido = read(fd,buf,PACKET_SIZE_LIMIT);
                if(lido<0)
                    return -1;
                for(unsigned int i=0;i<lido;++i){
                    state_machine(buf[i],&state);
                    if(state.state==E && state.adr==ADR_TX && state.ctrl == DISC){
                        if(state.ctrl == DISC){
                            DISC_rec=1;}
                        else if(state.ctrl == C_RR(0) || state.ctrl == C_RR(1) || state.ctrl == C_REJ(0) || state.ctrl == C_REJ(1)){
                            alarm_count=0;
                        }
                    }
                }
            }
        }
        if(DISC_rec) 
            printf("Just Received DISC.\n");
        int size=trama(buf,ADR_TX,UA);
        DISC_rec = 0;
        write(fd,buf,size);
        printf("Just Sent UA.\n");
        sleep(1);

    } else { 
        alarm_count=0;
        while(!DISC_rec){
            int lido = read(fd,buf,PACKET_SIZE_LIMIT);
            if(lido<0)
                return -1;
            for(unsigned int i=0;i<lido && !DISC_rec;++i){
                state_machine(buf[i],&state);
                if(state.state==E && state.adr==ADR_TX && state.ctrl == DISC)
                    DISC_rec=1;
            }
        }
        if(DISC_rec) printf("Just Received DISC.\n");
        int size=trama(buf,ADR_TX,DISC);

        write(fd,buf,size);
        printf("Just Sent DISC.\n");

       while(1){
            int lido = read(fd,buf,PACKET_SIZE_LIMIT);
            if(lido<0)
                return -1;
            for(unsigned int i=0;i<lido;++i){
                state_machine(buf[i],&state);
                if(state.state==E && state.adr==ADR_TX && state.ctrl == UA){
                    printf("Just Received UA.\n");
                    printf("Stats:\n");
                    printf("%d timeouts happened\n", stats.timeouts);
                    printf("%d i_frames were received\n", stats.frames_received);
                    printf("%d retransmissions happened\n", stats.retransmissions);
                    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
                    {
                        perror("tcsetattr");
                        exit(-1);
                    }

                    close(fd);
                    return 1;
                }
            }
        }
    }
    
}