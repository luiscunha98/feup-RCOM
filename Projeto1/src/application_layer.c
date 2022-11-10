#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <stdio.h>
#include <time.h>
#include "application_layer.h"
#include "link_layer.h"
#include "frame.h"


int assembleDataPacket(unsigned char* bytes, unsigned char* packet, int sequenceNum, int readSize){

	int l2 = readSize/256; 
    int l1 = readSize - (l2*256); 

    //Assembles DataPacket Header
    packet[0] = DATA_CTRL;
	packet[1] = sequenceNum%256; 
    packet[2] = l2;
    packet[3] = l1;

    //Assembles DataPacket Data
    for(int i=0; i<readSize; i++){
        packet[i+4] = bytes[i];
    }

	return (readSize+4); //Data Packet Size (data size + header size)
}

int assembleCtrlPacket(char* filename, char CTRL_Field, unsigned char* ctrlPacket){

	if(strlen(filename) > 255){
        perror("Filename couldn't fit in one byte! \n Returning with error\n");
        exit(-1); 
    }
	
    unsigned char hex_string[5]; 
    struct stat file;
    stat(filename, &file); //Get file stats, namely its size
    sprintf(hex_string, "%02lX", file.st_size); //Store the file size in hexadecimal form
	
    int fileSize = file.st_size; //File size 
	int fileSizeBytes = strlen(hex_string) / 2; //File Size in bytes -> 2 Hexadecimal numbers = 1 byte, so to obtain the exact number in bytes we just need
                                                //To divide the string by its hexadecimal value

    printf("Filesize: %d\n Filesize in Hexadecimal form: %s \n", file.st_size, hex_string);
    printf("Bytes needed to represent the filesize: %d \n", fileSizeBytes); 

    if(fileSizeBytes > 256){
        perror("File Size couldn't fit in one byte \n");
        exit(-1); 
    }

    ctrlPacket[0] = CTRL_Field; //Indicates if it is a START or END Packet
    ctrlPacket[1] = 0x00; // File Size (T1)
    ctrlPacket[2] = fileSizeBytes; //File Size in Bytes (L1)

    int index = 3; //Because index0=C, index1=T1, index2=L1

    //Stores the filesize represented in bytes as well
	for(int i=(fileSizeBytes-1); i>-1; i--){
		ctrlPacket[index++] = fileSize >> (8*i);
	}
    
    ctrlPacket[index++] = 0x01; //File Name (T2)
    ctrlPacket[index++] = (strlen(filename)*sizeof(char)); //File Name Size in Bytes (L2)

    //Stores the value of the filename (V2)
	for(int i=0; i<strlen(filename); i++){
		ctrlPacket[index++] = filename[i];
	}
    
    return index;
}


void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename)
{
    LinkLayerRole llrole;
    
    if(strcmp(role, "tx") == 0) //Transmiter
        llrole = LlTx; 
    else if (strcmp(role, "rx") == 0) //Receiver
        llrole = LlRx; 
    else{
        perror(role); 
        return; 
    }

    LinkLayer llayer;
    strcpy(llayer.serialPort, serialPort);
    llayer.baudRate = baudRate;
    llayer.numTransmissions = nTries;
    llayer.timeout = timeout;
    llayer.fd = 0; 
    llayer.role = llrole;

    //Establishes the connection between the receiver and the transmitter
    if(llopen(llayer)==-1) 
        exit(-1); 
        
    if(llrole == LlTx){
        unsigned char packet[300], bytes[200], fileNotOver = TRUE;
        int sizePacket = 0;
       
        FILE *file;

        //Opens the file
        file = fopen(filename, "r");  
        if(file == NULL){
            printf("Couldn't locate the file with the given filename\n");
            exit(-1); 
        }
        
        int nBytes = 200, curByte=0, index=0, nSequence = 0;

        //Assembles START Control Packet
        sizePacket = assembleCtrlPacket(filename,CTRL_START,&packet);

        if(llwrite(packet, sizePacket) < 0){
            perror("Something Went Wrong While Sending the Start Control Packet \n Exiting With Error \n"); 
            exit(-1); 
        }


        while(fileNotOver){
            //Reads the file stream
            //If we reached the end of the file stream
            if(!fread(&curByte, (size_t)1, (size_t) 1, file)){
                fileNotOver = FALSE;
                sizePacket = assembleDataPacket(bytes, &packet, nSequence++, index);

                if(llwrite(packet, sizePacket) == -1){
                    return;
                }
            }

            //At each 200 Bytes we assemble a data packet and pass it to the pipe using llwrite
            else if(nBytes == index) {
                sizePacket = assembleDataPacket(bytes, &packet, nSequence++, index);

                if(llwrite(packet, sizePacket) == -1){
                    perror("Something Went Wrong While Sending a Data Packet \n Exiting With Error \n"); 
                    exit(-1); 
                }

                //Resets the buffers and flag
                memset(bytes,0,sizeof(bytes)); 
                memset(packet,0,sizeof(packet));
                index = 0; 
            }

            bytes[index++] = curByte; //Appends elements to the buffer 
        }

        fclose(file); //Closes the file after reaching the end

        //Assembles the END Control Packet
        sizePacket = assembleCtrlPacket(filename,CTRL_END,&packet);

        if(llwrite(packet, sizePacket) == -1){
            perror("Something Went Wrong While Sending the End Control Packet \n Exiting With Error \n"); 
            exit(-1); 
        }

    }

    else{
        FILE *file;
        int read = TRUE;
        
        while(read){
        
            unsigned char packet[MAX_PAYLOAD_SIZE] = {0};
            int sizeOfPacket = 0, index = 0;
            
            //Calls llread in order to read the information sent by the transmitter
            if(llread(&packet, &sizeOfPacket)==-1){
                continue;
            }
           
           //Analyzes the first byte of the received packet
            switch (packet[0])
            {
            //If it is an End Packet closes the file
            case CTRL_END:
                fclose(file);
                printf("Closing the file\n");
                read = FALSE;
                break;
            //If it is a Start Packet opens the file in write mode
            case CTRL_START:
                file = fopen(filename, "w"); 
                printf("Opening the file\n");
                break; 
            //Otherwise just append the content of the frame to the file
            default:
                //Outputs content into the file
                for(int i=4; i<sizeOfPacket; i++){
                    fputc(packet[i], file);
                }
                break;
            }
        }
    }

    //Closes the Established Connection
    llclose(llayer);
}