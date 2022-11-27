// Main file of the serial port project.
// NOTE: This file must not be changed.

#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

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

    printf("Starting link-layer protocol application\n"
           "  - Serial port: %s\n"
           "  - Role: %s\n"
           "  - Baudrate: %d\n"
           "  - Number of tries: %d\n"
           "  - Timeout: %d\n"
           "  - Filename: %s\n",
           serialPort,
           role,
           BAUDRATE,
           N_TRIES,
           TIMEOUT,
           filename);
   if (strcmp(argv[2], "tx") == 0)
   	{
		printf("tx mode\n");

		LinkLayer l;
		sprintf(l.serialPort, "%s", argv[1]);
		l.baudRate = BAUDRATE;
		l.role = LlTx;
		l.nRetransmissions = N_TRIES;
		l.timeout = TIMEOUT;

    		if(llopen(l)==-1) {
        		fprintf(stderr, "Could not initialize link layer connection\n");
        		exit(1);
   		}

		printf("connection opened\n");
		fflush(stdout);
		fflush(stderr);

    		char *file_path = argv[3];
	        int file_desc = open(file_path, O_RDONLY);
    		if(file_desc < 0) {
        		fprintf(stderr, "Error opening file: %s\n", file_path);
        		exit(1);
    		}

		const int buf_size = MAX_PAYLOAD_SIZE-1;
		unsigned char buffer[buf_size+1];
		int write_result = 0;
		int bytes_read = 1;
		while (bytes_read > 0)
		{
			bytes_read = read(file_desc, buffer+1, buf_size);
                        if(bytes_read < 0) {
                                fprintf(stderr, "Error receiving from link layer\n");
                                break;
                        }
			else if (bytes_read > 0) {
				buffer[0] = 1;
				write_result = llwrite(buffer, bytes_read+1);
 		       		if(write_result < 0) {
            				fprintf(stderr, "Error sending data to link layer\n");
            				break;
        			}
	                        printf("%d bytes sent\n", bytes_read);
			}
                        else if (bytes_read == 0) {
                                buffer[0] = 0;
                                llwrite(buffer, 1);
                                printf("THE PENGUIN JUST WENT FLYING\n");
                                break;
                        }

			sleep(1);
		}
		llclose(1);
    		close(file_desc);
    		return 0;
			
	}
	else{
		printf("rx mode\n");

                LinkLayer l;
		        sprintf(l.serialPort, "%s", argv[1]);
		        l.baudRate = BAUDRATE;
                l.role = LlRx;
                l.nRetransmissions = N_TRIES;
                l.timeout = TIMEOUT;

                if(llopen(l)==-1) {
                        fprintf(stderr, "Could not initialize link layer connection\n");
                        exit(1);
                }

                char *file_path = argv[3];
                int file_desc = open(file_path, O_RDWR|O_CREAT, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH);
		if(file_desc < 0) {
                        fprintf(stderr, "Error opening file: %s\n", file_path);
                        exit(1);
                }
		
		int bytes_read = 0;
		int write_result = 0;
                const int buf_size = MAX_PAYLOAD_SIZE;
                unsigned char buffer[buf_size];
                int total_bytes = 0;

		while (bytes_read >= 0)
		{
	        	bytes_read = llread(buffer);
        		if(bytes_read < 0) {
                		fprintf(stderr, "Error receiving from link layer\n");
				break;
			}
			else if (bytes_read > 0) {
				if (buffer[0] == 1) {
					write_result = write(file_desc, buffer+1, bytes_read-1);
        		                if(write_result < 0) {
                			        fprintf(stderr, "Error writing to file\n");
                        		        break;
              	  	        	}
                        		total_bytes = total_bytes + write_result;
					printf("bytes read: %d total bytes read so far: %d\n", write_result, total_bytes);
				}
				else if (buffer[0] == 0) {
					printf("THE PENGUIN HAS ARRIVED\n");
					break;
				}
			}
            	}
		
                llclose(1);
                close(file_desc);
                return 0;


	}
}
