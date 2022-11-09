#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <math.h>
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
int txSequenceNumber = 0;  //Numero de sequencia transmissor
int rxSequenceNumber = 1; //Numero de sequencia recetor


int nTries, timeout, fd, lastFrameNumber = -1;

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
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY | O_NONBLOCK);

    alarmCount = 0;

    if (fd < 0)
    {
        perror(connectionParameters.serialPort);
        exit(-1);
    }

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
    

    printf("LLOPEN: Beggining Connection Estabilishment");

    nTries = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;


    if(connectionParameters.role == LlTx){

        unsigned char buf[5] = {0}, parcels[5] = {0};

        buf[0] = FRAME_FLAG;
        buf[1] = FRAME_A;
        buf[2] = SETFRAME_C;
        buf[3] = SETFRAME_BCC;
        buf[4] = FRAME_FLAG;


        while(alarmCount < nTries){

            if(!alarmEnabled){
                int bytes = write(fd, buf, sizeof(buf));
                printf("\nSET message sent, %d bytes written\n", bytes);
                startAlarm(timeout);
            }
            
            int bytesread = read(fd, parcels, 5);
            if(bytesread != -1 && parcels != 0 && parcels[0]==0x7E){
                //se o UA estiver errado 
                if(parcels[2] != 0x07 || (parcels[3] != (parcels[1]^parcels[2]))){
                    printf("\nUA not correct: 0x%02x%02x%02x%02x%02x\n", parcels[0], parcels[1], parcels[2], parcels[3], parcels[4]);
                    alarmEnabled = FALSE;
                    continue;
                }
                
                else{   
                    printf("\nUA correctly received: 0x%02x%02x%02x%02x%02x\n", parcels[0], parcels[1], parcels[2], parcels[3], parcels[4]);
                    alarmEnabled = FALSE;
                    break;
                }
            }

        }

        if(alarmCount >= nTries){
            printf("\nAlarm limit reached, SET message not sent\n");
            return -1;
        }
    }

    else
    {
        unsigned char buf[1] = {0}, parcels[5] = {0}; // +1: Save space for the final '\0' char

        STATE st = STATE0;
        unsigned char readByte = TRUE;
        
        // Loop for input
        while (STOP == FALSE)
        { 
            if(readByte){
                int bytes = read(fd, buf, 1); //ler byte a byte
                if(bytes==0 || bytes == -1) continue;
            }
        
            
            switch (st)
            {
            case STATE0:
                if(buf[0] == 0x7E){
                    st = STATE1;
                    parcels[0] = buf[0];
                }
                break;

            case STATE1:
                if(buf[0] != 0x7E){
                    st = STATE2;
                    parcels[1] = buf[0];
                }
                else {
                    st = STATE0;
                    memset(parcels, 0, 5);
                }
                break;

            case STATE2:
                if(buf[0] != 0x7E){
                    st = STATE3;
                    parcels[2] = buf[0];
                }
                else {
                    st = STATE0;
                    memset(parcels, 0, 5);
                }
                break;

            case STATE3:
                if(buf[0] != 0x7E){
                    parcels[3] = buf[0];
                    st = STATE4;
                }
                else {
                    st = STATE0;
                    memset(parcels, 0, 5);
                }
                break;

            case STATE4:
                if(buf[0] == 0x7E){
                    parcels[4] = buf[0];
                    st = STATE5;
                    readByte = FALSE;
                }

                else {
                    st = STATE0;
                    memset(parcels, 0, 5);
                }
                break;
            case STATE5:
                if(((parcels[1])^(parcels[2]))==(parcels[3])){
                    printf("\nGreat success! SET message received without errors\n\n");
                    STOP = TRUE;
                }
                else {
                    st = STATE0;
                    memset(parcels, 0, 5);
                    readByte = TRUE;
                }
                break;
            default:
                break;
            }
        }

        parcels[2] = 0x07;
        parcels[3] = parcels[1]^parcels[2];

        //preciso de estar dentro da state machine ate receber um sinal a dizer que o UA foi corretamente recebido

        int bytes = write(fd, parcels, sizeof(parcels));
        printf("UA message sent, %d bytes written\n", bytes);
    }

    alarmEnabled = FALSE;

    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    printf("\n ------ ATEMPTING INFORMATION FRAME TRANSMITION ------ \n\n");

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

    //Stores the data inside data packet into the information frame
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

    //Builds Info frame header & assigns the correct ctr field value to the aux variable
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
        //Sends Frame
        if(alarmEnabled == FALSE){
            write(fd, info, posInfo); //Sends Information Frame
            startAlarm(timeout);
            printf("\nInformation Frame N(s=%d)\n", txSequenceNumber);
        }
        
        int bytesread = read(fd, respFrame, 5);

        //Receives response frame from the receiver
        //If it is a REJ we simply resend the frame
        //If RR BCC1 is correct and Control Field value corresponds to the Tx frame sequence number the RR frame is correct
        //Otherwise we just wait for the timeout 
        if(bytesread != -1 && respFrame != 0){

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
        }

        //Did not receive the expected ACK frame (RR0/RR1)
        if(alarmCount >= nTries){
            perror("\nLLWRITE: Number of attempts exceeded\n");
            STOP = TRUE;
            printf("\n ------ INFORMATION FRAME TRANSMITION FAILED ------ \n\n");
            close(fd);
            return -1;
        }

    }

    printf("\n ------ INFORMATION FRAME TRANSMITION SUCCESSFULLY ------ \n\n");

    //Updates sequence number
    txSequenceNumber ? txSequenceNumber = 0 : txSequenceNumber++; 

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet, int *sizeOfPacket)
{       
    printf("\n------------------------------LLREAD------------------------------\n\n");

    unsigned char receivedFrame[MAX_PAYLOAD_SIZE]={0}; 
    unsigned char supervisionFrame[5]={0}; 
    // ----- AINDA NAO VI ESTA MERDA DESTAS VARIAVEIS ----- //
    unsigned char BCC2=0x00, aux[MAX_PAYLOAD_SIZE] = {0}; 
    int sizeInfo = 0; //Nao entendo esta linha

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

    //Reads all the info on the pipe
    while (STOP == FALSE){ 

        //Reads the content sent by the transmitter byte by byte
        if(read(fd, aux_buf, 1) <= 0) //If an error occurred while reading or didn't read any bytes
            continue; 
    
        switch (currState)
        {
        case STATE0:
            //Found a flag byte, so adds it to the received buffer
            if(aux_buf[0] == 0x7E){
                currState = STATE1; //Goes into stage1 where it detects if it is not a random byte or it is actually a frame
                receivedFrame[sizeInfo++] = aux_buf[0];
            }
            else
                currState = STATE0; //Non relevant byte, so we will keep waiting for the desired flag byte
            break;

        case STATE1:
            //We are reading a frame
            if(aux_buf[0] != FRAME_FLAG){
                currState = STATE2; //Advances to read frame content state
                receivedFrame[sizeInfo++] = aux_buf[0];
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
                receivedFrame[sizeInfo++] = aux_buf[0];
            }
            //We are still reading the frame content
            else{
                receivedFrame[sizeInfo++] = aux_buf[0];
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

    //If Information Frame is Duplicate or Information Frame Header is Wrong we send a REJ frame as response
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
        write(fd, supervisionFrame, 5); //Sends Reject Frame as Response

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
    for(int i=0; i<sizeInfo; i++){
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

    ///VER MELHOR A PARTIR DAQUI -> NAO PRESTEI A DEVIDA ATENÇÃO AINDA

    //3º verificar que os BCCs estao certos
    //4º enviar a mensagem de confirmacao de receçao, positiva se correu tudo bem, negativa se BCC ou assim esta mal
    
    //fazer as contas para confirmar o valor max do buffer

    int size = 0; //tamanho da secçao de dados

    if(packet[4]==0x01){
        size = 256*packet[6]+packet[7]+4 +6; //+4 para contar com os bytes de controlo, numero de seq e tamanho
        for(int i=4; i<size-2; i++){
            BCC2 = BCC2 ^ packet[i];
        }
    }
    
    else{
        size += packet[6]+ 3 + 4; //+3 para contar com os bytes de C, T1 e L1 // +4 para contar com os bytes FLAG, A, C, BCC
        size += packet[size+1] + 2 +2; //+2 para contar com T2 e L2 //+2 para contar com BCC2 e FLAG

        for(int i=4; i<size-2; i++){
            BCC2 = BCC2 ^ packet[i];
        }
    }


    if(packet[size-2] == BCC2){

        if(packet[4]==0x01){
            if(receivedFrame[5] == lastFrameNumber){
                printf("\nInfoFrame received correctly. Repeated Frame. Sending RR.\n");
                supervisionFrame[2] = (rxSequenceNumber << 7) | 0x05;
                supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2];
                write(fd, supervisionFrame, 5);
                return -1;
            }   
            else{
                lastFrameNumber = receivedFrame[5];
            }
        }
        printf("\nInfoFrame received correctly. Sending RR.\n");
        supervisionFrame[2] = (rxSequenceNumber << 7) | 0x05;
        supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2];
        write(fd, supervisionFrame, 5);
    }
    
    else {
        printf("\nInfoFrame not received correctly. Error in data packet. Sending REJ.\n");
        supervisionFrame[2] = (rxSequenceNumber << 7) | 0x01;
        supervisionFrame[3] = supervisionFrame[1] ^ supervisionFrame[2];
        write(fd, supervisionFrame, 5);

        return -1;
    }

    (*sizeOfPacket) = size;

    pos = 0;
    
    for(int i=4; i<(*sizeOfPacket)-2; i++){
        aux[pos++] = packet[i];
    }

    
    (*sizeOfPacket) = size - 6;

    memset(packet,0,sizeof(packet));

    for(int i=0; i<(*sizeOfPacket); i++){
        packet[i] = aux[i];
    }

    //// ---- ///

    rxSequenceNumber ? rxSequenceNumber = 0 : rxSequenceNumber++; 

    return 1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics, LinkLayer connectionParameters, float runTime)
{       
    alarmCount = 0;

    printf("\n------------------------------LLCLOSE------------------------------\n\n");

    if(connectionParameters.role == LlRx){

        unsigned char buf[6] = {0}, parcels[6] = {0};
        unsigned char STOP = 0, UA = 0;

        buf[0] = 0x7E;
        buf[1] = 0x03;
        buf[2] = 0x0B;
        buf[3] = buf[1]^buf[2];
        buf[4] = 0x7E;
        buf[5] = '\0';


        while(!STOP){
            int bytesread = read(fd, parcels, 5);
            
            parcels[5] = '\0';

            if(bytesread==-1){
                continue;
            }


            else if(strcasecmp(buf, parcels) == 0){
                printf("\nDISC message received. Responding now.\n");
                
                buf[1] = 0x01;
                buf[3] = buf[1]^buf[2];

                while(alarmCount < nTries){

                    if(!alarmEnabled){
                        printf("\nDISC message sent, %d bytes written\n", 5);
                        write(fd, buf, 5);
                        startAlarm(timeout);
                    }
                    
                    int bytesread = read(fd, parcels, 5);
                    if(bytesread != -1 && parcels != 0 && parcels[0]==0x7E){
                        //se o UA estiver errado 
                        if(parcels[2] != 0x07 || (parcels[3] != (parcels[1]^parcels[2]))){
                            printf("\nUA not correct: 0x%02x%02x%02x%02x%02x\n", parcels[0], parcels[1], parcels[2], parcels[3], parcels[4]);
                            alarmEnabled = FALSE;
                            continue;
                        }
                        
                        else{   
                            printf("\nUA correctly received: 0x%02x%02x%02x%02x%02x\n", parcels[0], parcels[1], parcels[2], parcels[3], parcels[4]);
                            alarmEnabled = FALSE;
                            close(fd);
                            break;
                        }
                    }

                }

                if(alarmCount >= nTries){
                    printf("\nAlarm limit reached, DISC message not sent\n");
                    return -1;
                }
                
                STOP = TRUE;
            }
        
        }

    }

    else{
        alarmCount = 0;

        unsigned char buf[6] = {0}, parcels[6] = {0};

        buf[0] = 0x7E;
        buf[1] = 0x03;
        buf[2] = 0x0B;
        buf[3] = buf[1]^buf[2];
        buf[4] = 0x7E;
        buf[5] = '\0'; //assim posso usar o strcmp

        while(alarmCount < nTries){

            if(!alarmEnabled){
                
                int bytes = write(fd, buf, 5);
                printf("\nDISC message sent, %d bytes written\n", bytes);
                startAlarm(timeout);
            }

            //sleep(2);
            
            int bytesread = read(fd, parcels, 5);

            buf[1] = 0x01;
            buf[3] = buf[1]^buf[2];
            parcels[5] = '\0';

            if(bytesread != -1 && parcels != 0 && parcels[0]==0x7E){
                //se o DISC estiver errado 
                if(strcasecmp(buf, parcels) != 0){
                    printf("\nDISC not correct: 0x%02x%02x%02x%02x%02x\n", parcels[0], parcels[1], parcels[2], parcels[3], parcels[4]);
                    alarmEnabled = FALSE;
                    continue;
                }
                
                else{   
                    printf("\nDISC correctly received: 0x%02x%02x%02x%02x%02x\n", parcels[0], parcels[1], parcels[2], parcels[3], parcels[4]);
                    alarmEnabled = FALSE;
                    
                    buf[1] = 0x01;
                    buf[2] = 0x07;
                    buf[3] = buf[1]^buf[2];

                    int bytes = write(fd, buf, 5);

                    close(fd);

                    printf("\nUA message sent, %d bytes written.\n\nI'm shutting off now, bye bye!\n", bytes);
                    return 1;

                }
            }

        }

        if(alarmCount >= nTries){
            printf("\nAlarm limit reached, DISC message not sent\n");
            close(fd);
            return -1;
        }


    }


    return 1;

}

