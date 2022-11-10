#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <stdio.h>
#include <time.h>
#include <signal.h>
#include "link_layer.h"
#include "frame.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

//State Machine global variables
volatile int STOP = FALSE;

//Alarm Global Variables 
int alarmEnabled=0; 
int alarmCount=0;

//Auxiliary Variables
int txSequenceNumber = 0;  //Numero de sequencia transmissor
int rxSequenceNumber = 1; //Numero de sequencia recetor
int lastSequenceNumber = -1;
LinkLayer conParameters; 

/// ---------- ALARM FUNCTIONS ---------- ///
// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("\nAlarm #%d\n", alarmCount);
}

//Starts the alarm
int startAlarm(int timeout)
{
    // Set alarm function handler
    (void)signal(SIGALRM, alarmHandler);

    if (alarmEnabled == FALSE)
    {
        alarm(timeout);
        alarmEnabled = TRUE;
    }
    

    return 0;
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{   
    conParameters = connectionParameters; 

    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);

    alarmCount = 0;

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    conParameters.fd = fd; 

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = connectionParameters.baudRate | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; 
    newtio.c_cc[VMIN] = 1;  

    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");
    
    if(connectionParameters.role == LlTx){

        unsigned char setFrame[5] = {0}; 
        unsigned char uaFrame[5] = {0};

        //Assembles SET Frame
        setFrame[0] = FRAME_FLAG;
        setFrame[1] = FRAME_A;
        setFrame[2] = SETFRAME_C;
        setFrame[3] = SETFRAME_BCC;
        setFrame[4] = FRAME_FLAG;


        while(alarmCount < conParameters.numTransmissions){

            if(alarmEnabled == FALSE){
                int bytes = write(conParameters.fd, setFrame, 5);//Sends Set Frame
                sleep(1);  // Wait until all bytes have been written to the serial port
                printf("%d bytes written\n", bytes);
                startAlarm(conParameters.timeout);
            }
            
            //Reads 5 bytes from the pipe
            if(read(conParameters.fd, uaFrame, 5) == -1)
                continue; 

            if(uaFrame[0]==FRAME_FLAG){
                if((uaFrame[3] == uaFrame[1]^uaFrame[2]) &&  uaFrame[2] == UAFRAME_C){ //If uaFrame BCC1 is valid and Control Byte is correct we received a valid UA Frame
                    printf("UA Frame Received with success: %x %x %x %x %x \n", uaFrame[0], uaFrame[1], uaFrame[2], uaFrame[3], uaFrame[4]); 
                    printf("LLOPEN: Connection Established Successfully !! \n"); 
                    alarmEnabled = FALSE; 
                    return 1; 
                }
                else{
                    alarmEnabled = FALSE; 
                    printf("UA Frame is Incorrect: %x %x %x %x %x \n", uaFrame[0], uaFrame[1], uaFrame[2], uaFrame[3], uaFrame[4]); 
                    break; 
                }
            }

        }

        if(alarmCount >= conParameters.numTransmissions){
            printf("LLOPEN: MAX ATTEMPTS REACHED ! RETURNING WITH ERROR\n");
            return -1;
        }
    }
    else
    {
        //State Machine Auxiliary Variables
        unsigned char aux_buf[1] = {0}; 
        unsigned char newFrame[5] = {0};
        int currState = STATE0;
        unsigned char readByte = TRUE;
        
        //Reads pipe's content
        while (STOP == FALSE)
        { 
            if(readByte){
                if(read(conParameters.fd, aux_buf, 1) <= 0)
                    continue;
            }
        
            switch (currState)
            {
            case STATE0:
                memset(newFrame, 0, 5);
                readByte = TRUE;
                if(aux_buf[0] == FRAME_FLAG){
                    currState = STATE1;
                    newFrame[0] = aux_buf[0];
                }
                break;

            case STATE1:
                if(aux_buf[0] != FRAME_FLAG){
                    currState = STATE2;
                    newFrame[1] = aux_buf[0];
                }
                else {
                    currState = STATE0;
                }
                break;

            case STATE2:
                if(aux_buf[0] != FRAME_FLAG){
                    currState = STATE3;
                    newFrame[2] = aux_buf[0];
                }
                else {
                    currState = STATE0;
                }
                break;

            case STATE3:
                if(aux_buf[0] != FRAME_FLAG){
                    newFrame[3] = aux_buf[0];
                    currState = STATE4;
                }
                else {
                    currState = STATE0;
                }
                break;

            case STATE4:
                if(aux_buf[0] == FRAME_FLAG){
                    newFrame[4] = aux_buf[0];
                    currState = STATE5;
                    readByte = FALSE;
                }

                else {
                    currState = STATE0;
                }
                break;
            case STATE5:
                //It is a Valid Frame!
                if(((newFrame[1])^(newFrame[2]))==(newFrame[3])){
                    //It is a valid SET Frame!
                    if(newFrame[2] == SETFRAME_C)
                        STOP = TRUE; 
                    else{
                        currState = STATE0;
                    }
                }
                else {
                    currState = STATE0;
                }
                break;
            default:
                break;
            }
        }

        printf("LLOPEN (Rx): SET Frame Received Successfully: %x %x %x %x %x \n", newFrame[0], newFrame[1], newFrame[2], newFrame[3], newFrame[4]);
        printf("LLOPEN (Rx): Sending UA Frame as Reply! \n");

        //If SET Frame was correctly received we are going to reply with a UA Frame
        unsigned char uaFrame[5] = {0}; 
        uaFrame[0] = FRAME_FLAG; 
        uaFrame[1] = FRAME_A; 
        uaFrame[2] = UAFRAME_C; 
        uaFrame[3] = UAFRAME_BCC; 
        uaFrame[4] = FRAME_FLAG; 

        int bytes = write(conParameters.fd, uaFrame, 5); //Sends UA Frame back
        sleep(1);  // Wait until all bytes have been written to the serial port
        printf("%d bytes written\n", bytes);
        return 1; 
    }

    return -1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    printf(" ------ ATEMPTING INFORMATION FRAME TRANSMITION ------ \n");

    alarmCount = 0;

    unsigned char respFrame[5] = {0}; 
    int STOP = FALSE;
    int controlFieldResp=0x00; 

    char info[MAX_PAYLOAD_SIZE] = {0}; 

    //Calculates BCC2 of the received unstuffed data packet 
	char BCC2 = 0x00;
	for(int i=0; i<bufSize; i++){
		BCC2 ^= buf[i];
	}

    int posInfo = 4; 

    //Stores the data inside data packet into the information frame and stuffs it at the same time
    for(int i=0; i<bufSize; i++){
        if(buf[i]==FRAME_FLAG){
            info[posInfo++]=ESC;
            info[posInfo++]=FLAG_ESC;
            continue;
        }
        else if(buf[i]==ESC){
            info[posInfo++]=ESC;
            info[posInfo++]=ESC_ESC;
            continue;
        }
        else
            info[posInfo++]=buf[i];
    }

    //Does byte stuffing on BCC2
    switch (BCC2)
    {
    case FRAME_FLAG:
        info[posInfo++] = ESC; 
        info[posInfo++] = FLAG_ESC; 
        break;
    case ESC: 
        info[posInfo++] = ESC; 
        info[posInfo++] = FLAG_ESC; 
    default:
        info[posInfo++] = BCC2; 
        break;
    }

    //Builds Info frame header & assigns the correct control field value to the aux variable
    info[0]=FRAME_FLAG; 
    info[1]=FRAME_A;
    switch (txSequenceNumber)
    {
    case 0:
        info[2] = IFRAME0; 
        controlFieldResp = C_RR1; //Because (N(r=1), so N(s=0))
        break;
    case 1: 
        info[2] = IFRAME1; 
        controlFieldResp = C_RR0; //Because (N(r=0), so N(s=1))
        break; 
    default:
        perror("LLWRITE ERROR: Invalid transmiter sequence number\n"); 
        break;
    }
    info[3]=info[1]^info[2]; 


    info[posInfo++]=FRAME_FLAG; //Finalizes info packet build by adding the Flag at the end

    while(STOP == FALSE){

        if(alarmEnabled == FALSE){
            write(conParameters.fd, info, posInfo); //Sends Information Frame
            startAlarm(conParameters.timeout);
            printf("\nInformation Frame N(s=%d)\n", txSequenceNumber);
        }
        
        if(read(conParameters.fd, respFrame, 5) == -1)
            continue; 

        //Receives response frame from the receiver
        //If it is a REJ we simply resend the frame
        //If RR BCC1 is correct and Control Field value corresponds to the Tx frame sequence number the RR frame is correct
        //Otherwise we just wait for the timeout 

        //Verifies if response has a valid BCC1 and Response Control Field is correct according to current Tx frame sequence number
        if((respFrame[2] == controlFieldResp) && (respFrame[3] = respFrame[1]^respFrame[2])){
            printf("ACK is Correct: %x %x %x %x %x \n", respFrame[0], respFrame[1], respFrame[2], respFrame[3], respFrame[4]);
            STOP = TRUE;
            alarmEnabled = FALSE;
        }
        else{
            printf("ACK is Wrong: %x %x %x %x %x \n", respFrame[0], respFrame[1], respFrame[2], respFrame[3], respFrame[4]);
            alarmEnabled = FALSE;
        }

        //Did not receive the expected ACK frame (RR0/RR1)
        if(alarmCount >= conParameters.numTransmissions){
            perror("LLWRITE: Number of attempts exceeded\n");
            STOP = TRUE;
            printf(" ------ INFORMATION FRAME TRANSMITION FAILED ------ \n");
            close(conParameters.fd);
            return -1;
        }

    }

    printf(" ------ INFORMATION FRAME TRANSMITION SUCCESSFULLY ------ \n");

    //Updates sequence number
    txSequenceNumber ? txSequenceNumber = 0 : txSequenceNumber++; 

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet, int *sizeOfPacket)
{       
    printf(" ------ ATEMPTING INFORMATION FRAME RECEPTION ------ \n");

    unsigned char receivedFrame[MAX_PAYLOAD_SIZE]={0}; 
    unsigned char supervisionFrame[5]={0}; 

    //Assigns the expected Information Control Byte in order to avoid duplicates
    unsigned char ctrlByte = 0x00; 
    switch (rxSequenceNumber)
    {
    case 0:
        ctrlByte = 0x40; //N(r=0), so its expected to receive info frame 1
        break;
    case 1: 
        ctrlByte = 0x00; //N(r=1), so its expected to receive info frame 0
        break; 
    default:
        perror("LLREAD ERROR: Invalid Receiver Sequence Number\n"); 
        return -1; 
    }

    //State Machine Auxiliary Variables
    int STOP = FALSE;
    char aux_buf[1] = {0}; //Auxiliary buffer to read the contents of the pipe
    int currState = 0; 
    int size = 0; 

    //Reads all the info on the pipe
    while (STOP == FALSE){ 

        //Reads the content sent by the transmitter byte by byte
        if(read(conParameters.fd, aux_buf, 1) <= 0) //If an error occurred while reading or didn't read any bytes
            continue; 
    
        switch (currState)
        {
        case STATE0:
            //Found a flag byte, so adds it to the received buffer
            if(aux_buf[0] == 0x7E){
                currState = STATE1; //Goes into stage1 where it detects if it is not a random byte or it is actually a frame
                receivedFrame[size++] = aux_buf[0];
            }
            else
                currState = STATE0; //Non relevant byte, so we will keep waiting for the desired flag byte
            break;

        case STATE1:
            //We are reading a frame
            if(aux_buf[0] != FRAME_FLAG){
                currState = STATE2; //Advances to read frame content state
                receivedFrame[size++] = aux_buf[0];
            }
            // The byte read before was not part of a frame, so we go back to our starting point
            else{
                memset(receivedFrame, 0, MAX_PAYLOAD_SIZE); //Resets received buffer
                currState = STATE0; //Goes back to initial state
            }
            break;

        case STATE2:
            //Found end flag -> Frame read succesfully -> Exits State Machine
            if(aux_buf[0] == FRAME_FLAG){
                STOP = TRUE;
                receivedFrame[size++] = aux_buf[0];
            }
            //We are still reading the frame content
            else{
                receivedFrame[size++] = aux_buf[0];
            }
            break;
        default:
            perror("LLREAD: Something unexpected happened while reading from pipe\n Aborting Operation\n"); 
            return -1; 
            break;
        }
    }
    
    //Builds Supervision Frame Main structure
    supervisionFrame[0] = FRAME_FLAG;
    supervisionFrame[1] = FRAME_A;
    supervisionFrame[4] = FRAME_FLAG;

    //If Information Frame is NOT DUPLICATE And Information Frame HEADER is Wrong we send a REJ frame as response
    if(receivedFrame[3] != (receivedFrame[1]^receivedFrame[2]) || receivedFrame[2] != ctrlByte){ 
        //Based on RX sequence number we assign the correct Control Field value corresponding to REJ0/REJ1
        switch (rxSequenceNumber)
        {
        case 0:
            supervisionFrame[2] = C_REJ0; 
            break;
        case 1: 
            supervisionFrame[2] = C_REJ1; 
            break; 
        default:
            perror("LLREAD ERROR: Invalid transmiter sequence number\n"); 
            break;
        }
        supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2]; //Calculates Supervision Frame BCC1
        write(conParameters.fd, supervisionFrame, 5); //Sends Reject Frame as Response

        printf(" ----- SENT REJ FRAME CONTENT ----- \n");

        //Prints REJ frame content
        for(int i=0; i<5; i++){
            printf("%x ", supervisionFrame[i]);
        }
        printf("\n"); 

        return -1; 
    }

    int pos = 0; 

    //DeStuffing of the Received Frame content
    for(int i=0; i<size; i++){
        if(receivedFrame[i] == ESC){ //Escape Byte Detected
            if(receivedFrame[i+1]==FLAG_ESC){ //Asserts if the the frame flag has been stuffed
                packet[pos++] = FRAME_FLAG;
                i++;
            }
            if(receivedFrame[i+1]==ESC_ESC){ //Asserts if the escape byte has been stuffed
                packet[pos++] = ESC;
                i++;
            }
        }
        else 
            packet[pos++] = receivedFrame[i];
    }


    //Calculate the correct received frame size by doing some math involving the headers and the data field size
    if(packet[4]==DATA_CTRL){
        //size = 256 * L2(packet[6])+L1(packet[7])->Data Field Size+4(bytes C(control byte), N(sequence number), L2, L1)+6(bytes FLAG, A, C, BCC1, BCC2, FLAG)
        *sizeOfPacket = (256*packet[6])+packet[7]+4+6;
    }
    else{
        *sizeOfPacket += packet[6]+3+4; //L2+3(C, T1, L1)+4(FLAG, A, C, BCC)
        *sizeOfPacket += packet[(*sizeOfPacket)+1]+2+2;//+2(T2,L2)+2(BCC2,FLAG)
    }

    unsigned char BCC2=0x00; 

    //Calculates Expected BCC2 of the data field
    for(int i=4; i<(*sizeOfPacket)-2; i++){
        BCC2 ^= packet[i];
    }

    //Sends Reply Message (SuperVision Frame either RR if everything is OK or REJ if there is an error)
    //Verifies BCC2 in order to verify if the data field is valid
    if(packet[(*sizeOfPacket)-2] == BCC2){
        //BCC2 is Correct 
        //Assigning the correct value of RR(0/1)
        switch (rxSequenceNumber)
        {
        case 0:
            supervisionFrame[2] = C_RR0; 
            break;
        case 1: 
            supervisionFrame[2] = C_RR1; 
            break; 
        default:
            perror("LLREAD ERROR: Invalid transmiter sequence number\n"); 
            break;
        }

        if(packet[4]==DATA_CTRL){
            //Verifify if frame is duplicate
            if(receivedFrame[5] == lastSequenceNumber){
                supervisionFrame[2]; 
                supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2];
                printf("LLREAD: Information Frame Received Correctly. \n Duplicate Frame. \n Replying with RR. \n");
                write(conParameters.fd, supervisionFrame, 5);
                return -1;
            }   
            else{
                lastSequenceNumber = receivedFrame[5]; //Update the sequence number
            }
        }
        supervisionFrame[2]; 
        supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2];
        printf("LLREAD: Information Frame Received Correctly. \n New Frame. \n Replying with RR. \n");
        write(conParameters.fd, supervisionFrame, 5); 
    }
    
    //Invalid Data Field so we discard it and reply with REJ
    else {
        //Assigning the correct value of REJ(0/1)
        switch (rxSequenceNumber)
        {
        case 0:
            supervisionFrame[2] = C_REJ0; 
            break;
        case 1: 
            supervisionFrame[2] = C_REJ1; 
            break; 
        default:
            perror("LLREAD ERROR: Invalid transmiter sequence number\n"); 
            break;
        }
        supervisionFrame[2]; 
        supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2];
        printf("LLREAD: Information Frame Was Not Received Correctly. Invalid Data Field. Replying with REJ. \n");
        write(conParameters.fd, supervisionFrame, 5);

        return -1;
    }

    pos = 0;

    unsigned char data[MAX_PAYLOAD_SIZE] = {0}; 

    //Gets only the Data Field of the received Information Frame
    for(int i=4; i<(*sizeOfPacket)-2; i++){
        data[pos++] = packet[i];
    }

    *packet = data; 

    memset(packet,0,sizeof(packet)); //Resets packet buffer in order to only store the data field

    (*sizeOfPacket) -= 6; //Updates size of the packet removing (FLAG, A, C, BCC1, BCC2, FLAG)

    //Stores only the data field of the information packet inside the packet that will be used to assemble the file in the application layer
    for(int i=0; i<(*sizeOfPacket); i++){
        packet[i] = data[i];
    }

    rxSequenceNumber ? rxSequenceNumber = 0 : rxSequenceNumber++; 

    return 1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(LinkLayer connectionParameters)
{       

    printf(" ------ ATEMPTING INFORMATION TO CLOSE THE CONNECTION ------ \n");

    if(connectionParameters.role == LlRx){

        unsigned char newFrame[5] = {0};

        //Assembles Disc Frame
        unsigned char discFrame[5] = {0};
        discFrame[0] = FRAME_FLAG;
        discFrame[1] = FRAME_A;
        discFrame[2] = DISCFRAME_C;
        discFrame[3] = discFrame[1]^discFrame[2];
        discFrame[4] = FRAME_FLAG;

        STOP = FALSE; 
        alarmCount = 0; //Resets alarm Count

        while(STOP == FALSE){
            if(read(conParameters.fd, newFrame, 5) == -1) //Reads pipe's contents
                continue;

            if(discFrame[1] == newFrame[1] && discFrame[2] == newFrame[2] && discFrame[3] == newFrame[3] && discFrame[4] == newFrame[4]){ //Asserts if Transmitter DISC Frame was Correct
                printf("LLCLOSE (Rx): TRANSMITTER DISC FRAME RECEIVED SUCCESSFULY! \n SENDING REPLY... \n");
                
                //Assembles Reply Disc Frame (Only the Address Byte Changes in comparision with the Transmitter Disc Frame)
                discFrame[1] = llClose_FRAME_A;
                discFrame[3] = discFrame[1]^discFrame[2];

                memset(newFrame, 0, 5); //Resets received Frame Buffer

                //Sends Disc Frames as response until Transmitter Replies with a UA Frame
                while(alarmCount < conParameters.numTransmissions){

                    if(alarmEnabled == FALSE){
                        int bytes = write(conParameters.fd, discFrame, sizeof(discFrame));
                        printf("%d bytes written\n", bytes);
                        startAlarm(conParameters.timeout);
                    }
                    
                    if(read(conParameters.fd, newFrame, 5) == -1)
                        continue; 

                    if(newFrame[0]==FRAME_FLAG){
                        //Assert if UA Frame is valid
                        if(newFrame[2] == UAcloseFRAME_C && (newFrame[3] == newFrame[1]^newFrame[2])){
                            close(conParameters.fd); //Closes the connection between the transmitter and the receiver safely
                            alarmEnabled = FALSE; 
                            printf("LLCLOSE (Rx): UA FRAME RECEIVED SUCCESSFULLY: %x %x %x %x %x. \n CLOSING THE CONNECTION... \n", newFrame[0], newFrame[1], newFrame[2], newFrame[3], newFrame[4]); 
                            return 1; 
                        }
                        else{
                            alarmEnabled = FALSE; 
                            printf("LLCLOSE (Rx): THE RECEIVED UA FRAME WAS NOT CORRECT:  %x %x %x %x %x \n", newFrame[0], newFrame[1], newFrame[2], newFrame[3], newFrame[4]); 
                            printf("LLCLOSE (Rx): PERFORMING ANOTHER ATTEMPT... \n"); 
                            continue; 
                        }
                    }

                }

                if(alarmCount >= conParameters.numTransmissions){
                    printf("LLCLOSE (Rx): MAX ATTEMPTS REACHED ! RETURNING WITH ERROR\n");
                    return -1;
                }
                
            }
        }
    }

    else{
        alarmCount = 0; //Resets alarm Count

        unsigned char newFrame[5] = {0};

        //Assembles Disc Frame
        unsigned char discFrame[5] = {0};
        discFrame[0] = FRAME_FLAG;
        discFrame[1] = FRAME_A;
        discFrame[2] = DISCFRAME_C;
        discFrame[3] = discFrame[1]^discFrame[2];
        discFrame[4] = FRAME_FLAG;

        unsigned char receivedDiscFrame[5] = {0}; 
        receivedDiscFrame[0] = FRAME_FLAG;
        receivedDiscFrame[1] = llClose_FRAME_A;
        receivedDiscFrame[2] = DISCFRAME_C;
        receivedDiscFrame[3] = receivedDiscFrame[1]^receivedDiscFrame[2];
        receivedDiscFrame[4] = FRAME_FLAG;

        while(alarmCount < conParameters.numTransmissions){

            if(!alarmEnabled){
                int bytes = write(conParameters.fd, discFrame, 5);
                printf("%d bytes written\n", bytes);
                startAlarm(conParameters.timeout);
            }
            
            if(read(conParameters.fd, newFrame, 5) == -1)
                continue; 

            if(newFrame[0]==0x7E){
                if(receivedDiscFrame[1] == newFrame[1] && receivedDiscFrame[2] == newFrame[2] && receivedDiscFrame[3] == newFrame[3] && receivedDiscFrame[4] == newFrame[4]){
                    alarmEnabled = FALSE; 
                    memset(newFrame, 0, 5); //Resets received Frame in order to build the response UA Frame
                    newFrame[0] = FRAME_FLAG; 
                    newFrame[1] = llClose_FRAME_A; 
                    newFrame[2] = UAcloseFRAME_C; 
                    newFrame[3] = newFrame[1] ^ newFrame[2]; 
                    newFrame[4] = FRAME_FLAG; 
                    int bytes = write(conParameters.fd, newFrame, 5); 
                    printf("%d bytes written\n", bytes);
                    close(conParameters.fd); //Closes the tranmitter end of the pipe
                    printf("LLCLOSE (Tx): UA FRAME SUCCESSFULLY SENT! \n CLOSING THE CONNECTION... \n", newFrame[0], newFrame[1], newFrame[2], newFrame[3], newFrame[4]);
                    return 1;  
                }
                else{
                    alarmEnabled = FALSE; 
                    printf("LLCLOSE (Tx): THE RECEIVED DISC FRAME WAS NOT CORRECT:  %x %x %x %x %x \n", newFrame[0], newFrame[1], newFrame[2], newFrame[3], newFrame[4]); 
                    printf("LLCLOSE (T): PERFORMING ANOTHER ATTEMPT... \n"); 
                    continue; 
                }
            }
        }
        if(alarmCount >= conParameters.numTransmissions){
            printf("LLCLOSE (Tx): MAX ATTEMPTS REACHED ! RETURNING WITH ERROR\n");
            return -1;
        }
    }

    return -1;
}

