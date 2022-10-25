// Application layer protocol implementation

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

//Custom header files
#include "application_layer.h"
#include "link_layer.h"
#include "frame.h"

static fileInformation fileInfo;

void getFileSize(void){
    int size = 0;
    fseek(fileInfo.file, 0L, SEEK_END); //Sets the file position indicator to the end of the file
    size = (int)ftell(fileInfo.file);
    if(size == -1) perror("Something Went Wrong while getting file size;"); exit(-1); 
    rewind(fileInfo.file); //Sets the file position indicator to the begining of the file
    fileInfo.fileSize =  size; 
}

int assembleCtrlPacket(uint8_t CTRL_Field, char* packet){
    unsigned L1 = sizeof(fileInfo.fileSize); //File Size in Bytes
    unsigned L2 = strlen(fileInfo.fileName)*sizeof(char); //File Name in Bytes
    unsigned size_packet = 5 + L1 + L2;

    packet[size_packet];
    packet[0] = CTRL_Field; //Control Field
    packet[1] = 0x00; //File Size
    packet[2] = L1; //File Size in Bytes
    memcpy(&packet[3], &fileInfo.fileSize, L1); //Copies L1 Bytes from fileSize into packet[3]
    packet[3+L1] = 0x01; //File Name
    packet[4+L1] = L2; //File Name in Bytes
    memcpy(&packet[5+L1], fileInfo.fileName, L2); //Copies L2 Bytes from fileSize into packet[5+L1]

    return size_packet;
}

typedef enum{
    SET, 
    START,
    STOP,
} frameType; 

void assembleFrame(char* frame, frameType frametype){
    switch (frametype)
    {
    case SET:
        frame[0] = FRAME_FLAG; 
        frame[1] = FRAME_A; 
        frame[2] = SETFRAME_C; 
        frame[3] = SETFRAME_BCC; 
        frame[4] = FRAME_FLAG;
        break;
    default:
        perror("Invalid frame type"); 
        break;
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    FILE* file; 

    if(strlen(serialPort) > 20){
        perror("Invalid serial Port"); 
        return; 
    }

    int fd = open(serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPort);
        exit(-1);
    }

    LinkLayerRole llrole; 
    
    if(strcmp(role, "tx") == 0) //Transmiter
        llrole = LlTx; 
    else if (strcmp(role, "rx") == 0) //Receiver
        llrole = LlRx; 
    else{
        perror(role); 
        return; 
    }

    appLayer applicationl;
    applicationl.fileDescriptor = fd; 
    applicationl.status = llrole; 

    linkLayer llayer;
    if((sizeof(llayer.port)/sizeof(char)) >= (sizeof(serialPort)/sizeof(char)))
        strcpy(llayer.port, serialPort); 
    else 
        perror("Incorrect serialPort size"); 
    llayer.baudRate = baudRate;
    llayer.sequenceNumber = 0; 
    llayer.timeout = timeout; 
    llayer.numTransmissions = nTries; 
    llayer.role = llrole; 
    llayer.fd = applicationl.fileDescriptor;

    frameType frametype; 

    if(llrole == LlTx){ 
        //llopen() part -> OPENING THE CONNECTION PART
        frametype = SET; 
        assembleFrame(llayer.frame, frametype); 
        if(llopen(llayer) == 1) perror("LLOPEN: Something Went Wrong while trying to establish the connection"); 

        //llwrite() part -> SENDING THE FILE TO THE RECIEVER PART

        //Gets all file info
        file = fopen(filename, "r"); //Opens the file in read mode, once we want to pass it's content to the receiver
        if(file == NULL){
            perror("Invalid file! \n"); 
            //llclose()
            exit(-1); 
        }
        fileInfo.file = file; //Gets the file pointer
        fileInfo.fileName = filename; //Gets the file name
        getFileSize(); //Gets the file size

        //Builds the START PACKET based on file info
        char* startPacket; 
        int startPacketsize = assembleCtrlPacket(CTRL_START, startPacket); 

        //Sends the START Packet
        if(llwrite(startPacket, startPacketsize) < 0){
            perror("Something Went Wrong while trying to send the START Packet"); 
            //Calls llclose()
        }

        //Assembles Data Packet
        int filebytesread = 0; 
        int filereadsz = 0; 

        //Reads the contents of the file
        while(1){
            if(ferror(fileInfo.file) || feof(fileInfo.file)){
                perror("Something Went Wrong while trying to read file's content");
                exit(-1); 
            }

            if(filereadsz == fileInfo.fileSize)
                break;
            
            //Data Field Assemble (P1..Pn)
            filereadsz = fread(llayer.frame+4, sizeof(char), MAX_PAYLOAD_SIZE-4, fileInfo.file); 
            //Data Packet C(0),N(1),L2(2),L1(3) assemble
            llayer.frame[0] = 0x01; //Data
            llayer.frame[1] = llayer.sequenceNumber%256;
            llayer.sequenceNumber++; 
            // Bytes number K = 256*L2+ L1 
            // K = filebytesread
            llayer.frame[2] = filebytesread/256; 
            llayer.frame[3] = filebytesread - (llayer.frame[2] * 256); 

            //Calls llwrite to send the current Data Packet 
            if(llwrite(llayer.frame, filebytesread+4) < 0){
                perror("Something Went Wrong while trying to send the Data Packet"); 
                //Calls llclose()
            }

            filebytesread = filebytesread + filereadsz; //Update bytes read
            memset(llayer.frame, 0, MAX_PAYLOAD_SIZE); //"Resets frame" by setting all its to 0
        }

        //Builds END packet
        char* endPacket;
        int endPacketSize = assembleCtrlPacket(CTRL_END, endPacket);

        //Sends the END Packet
        if(llwrite(endPacket, endPacketSize) < 0){
            perror("Something Went Wrong while trying to send the END Packet");
        }

        //llclose() to finish the tranmission
    } 
    else if(llrole == LlRx){ 
        //llopen() part -> OPENING THE CONNECTION PART
        if(llopen(llayer) == 1) perror("LLOPEN: Something Went Wrong while trying to establish the connection");  

        /* DO THIS PART LATER */

    }
    else
        perror("Invalid Role"); 

}
