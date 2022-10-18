// Link layer protocol implementation


#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include "link_layer.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

LinkLayer new;
struct termios oldtio;
struct termios newtio;
int fd;

#define FLAG (0b01111110)
#define ESCAPE (0x7d)
#define ESCAPE_FLAG (0x5e)
#define ESCAPE_ESCAPE (0x5d)
#define ADR_TX (0b00000011)
#define ADR_RX (0b00000001)
#define CTRL_SET (0b00000011)
#define CTRL_DISC (0b00001011)
#define CTRL_UA (0b00000111)
#define CTRL_RR(R) (R%2?0b10000101:0b00000101)
#define CTRL_REJ(R) (R%2?0b10000001:0b00000001)
#define CTRL_DATA(S) (S%2?0b01000000:0b00000000)
#define PACKET_SIZE_LIMIT 256
unsigned char alarmcount=0;
bool alarmenabled =FALSE;
unsigned char buf[256];


void alarmHandler(int signal){
    ++alarmcount;
    alarmenabled =FALSE;
}

void trama(unsigned char * array, unsigned char C, unsigned char A,unsigned char* data,unsigned char bcc2,unsigned int size)
{
	array[0] = FLAG;
	array[1] = A;
	array[2] = C;
	array[3] = array[1]^array[2];
	if(data==NULL){
        array[4]= FLAG;
        return 5;
    }
    memcpy(buf+4,data,size);
    array[4+size]=bcc2;
    array[5+size]=FLAG;
    return 6+size;
}

unsigned char recebeTrama(int fd)
{
	unsigned char car, res, Cverif;
	int state=0;

	while( state != 5 )
	{
    res = read(fd,&car,1);
    if( res < 0)
    {
      printf("Read falhou.\n");
      return FALSE;
  	}

		switch(state)
		{
			case 0: //expecting flag
				if(car == FLAG)
					state = 1;
				break;

			case 1: //expecting A
				if(car == ADR_RX || car = ADR_TX)
					state = 2;

				else if(car != FLAG)
				{
					state = 0;
				}

				break;

			case 2: //expecting Cverif
				Cverif = car;
				if(car == CTRL_UA || car == CTRL_DISC || car == CTRL_SET)
					state = 3;

				else if(car != FLAG)
				{
					state = 1;
				}

				else
					state = 0;

				break;

			case 3: //expecting BCC
				if(car == (A^Cverif))
					state = 4;

				else
				{
					state = 0;
				}

				break;

			case 4: //expecting FLAG
				if(car == FLAG)
					state = 5;

				else
				{
					state = 0;
				}

				break;
		}
	}

	alarm(0);

	return Cverif;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    new=connectionParameters;
    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    fd = open(new.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(new.serialPort);
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

    newtio.c_cflag = new.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 1;  // Blocking read until 1 char received

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
    int tries=0;
    signal(SIGALRM,alarmHandler);

    switch(new.role)
    {
      case LlTx:
        bool ua_rec = FALSE;
        while (tries<new.nRetransmissions && !ua_rec)
        {
          alarm(new.timeout);
          alarmenabled=TRUE;
          int send = trama(buf,CTRL_SET,ADR_TX,NULL,0,0);
          write(fd,buf,send);
          printf("SET just sent\n");
          while(alarmenabled && !ua_rec){
                int answer = read(fd,buf,PACKET_SIZE_LIMIT);
                if(answer<0)
                    return -1;
					printf("morreu");
                if(recebeTrama(fd) == CTRL_UA){
                  printf("UA received");
                  ua_rec=TRUE;
                  return 1;
                }
            }
        }
        break;
      case LlRx:
        if(recebeTrama(fd)==CTRL_SET){
          printf("SET received");
          int send_back=trama(buf,CTRL_UA,ADR_RX,NULL,0,0);
          write(fd,buf,send_back);
          printf("UA just sent");
          return 1;
        }
        break;
    }       
    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
unsigned char BCC2_ver(unsigned char *arr, int size)
{
    unsigned char result = arr[0];

    for(int i = 0; i < size; i++)
        result ^= arr[i];

    return result;
}

int llwrite(const unsigned char *buf, int bufSize)
{
  int sizemsg=bufSize+6;
  unsigned char bigbuf[sizemsg];

  bigbuf[0]=FLAG;
  bigbuf[1]=ADR_TX;
  bigbuf[2]=CTRL_DATA(0);
  bigbuf[3]=bigbuf[1]^bigbuf[2];
  
  sizemsg=stuffing(buf,bigbuf,bufSize,sizemsg);
  unsigned char BBC2=BCC2_ver(buf,bufSize);

  bigbuf[sizemsg-2]=BBC2;
  bigbuf[sizemsg-1]=FLAG;

return sizemsg;

}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    while( state != 5 )
	{
    res = read(fd,&car,1);
    if( res < 0)
    {
      printf("Read falhou.\n");
      return FALSE;
  	}

		switch(state)
		{
			case 0: //expecting flag
				if(car == FLAG)
					state = 1;
				break;

			case 1: //expecting A
				if(car == ADR_RX || car = ADR_TX)
					state = 2;

				else if(car != FLAG)
				{
					state = 0;
				}

				break;

			case 2: //expecting Cverif
				Cverif = car;
				if(car == CTRL_UA || car == CTRL_DISC || car == CTRL_SET)
					state = 3;

				else if(car != FLAG)
				{
					state = 1;
				}

				else
					state = 0;

				break;

			case 3: //expecting BCC
				if(car == (A^Cverif))
					state = 4;

				else
				{
					state = 0;
				}

				break;

			case 4: //expecting FLAG
				if(car == FLAG)
					state = 5;

				else
				{
					state = 0;
				}

				break;
		}
	}
  return 0;
}

/* O REJ que é esperado
 * 
 */
unsigned char correctREJ(unsigned char var)	
{
  if( var == C00 )
    return REJ1;

	else
    return REJ0;
}

/* O RR que é esperado
 * 
 */
unsigned char correctRR(unsigned char var)
{
  if(var == C00)
    return RR1;

	else
    return RR0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 1;
}

int stuffing(char * buf, unsigned char * Newmsg, int size, int sizeBuf)
{
	int aux=4;

	for(int i = 0; i < size; i++)
	{
		if( buf[i] == FLAG )
		{
			sizeBuf++;
			Newmsg = (unsigned char *)realloc(Newmsg, sizeBuf);
			Newmsg[aux] = ESCAPE;
			Newmsg[aux+1] = ESCAPE_FLAG;

			aux = aux + 2;
		}

		else if( buf[i] == ESCAPE )
		{
			sizeBuf++;
			Newmsg = (unsigned char *)realloc(Newmsg, sizeBuf);
			Newmsg[aux] = ESCAPE;
			Newmsg[aux+1] = ESCAPE_ESCAPE;

			aux = aux + 2;
		}

		else
		{
			Newmsg[aux] = buf[i];
			aux++;
		}

	}

	return sizeBuf;
}