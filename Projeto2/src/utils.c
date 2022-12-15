#include "utils.h"


void display_info(url url){
    printf("Username: %s\n",url.username);
    printf("Password: %s\n",url.password);
    printf("Host: %s\n",url.host);
    printf("Hostname: %s\n",url.hostname);
    printf("path: %s\n",url.path);
    printf("IP: %s\n",url.ip);
}

int get_ip(url *url){
    struct hostent* h;
    if ((h = gethostbyname(url->host)) == NULL) {
        herror("gethostbyname()");
        exit(1);
    }
    
    strcpy(url->hostname,h->h_name);
    strcpy(url->ip,inet_ntoa(*((struct in_addr *) h->h_addr)));
    return 0;
}

int write_socket(int socketFilefd,char* buf){
    write(socketFilefd, buf,strlen(buf));
    return 0;
}

int read_socket_response(FILE* socketFilefd){
    char* buf;
    size_t size = 0;
    int valid_response = 1;
    do{
        getline(&buf,&size,socketFilefd);
        printf("%s",buf);
        if(buf[0] == '5'){
            valid_response = 0;
        }
    } while(buf[3] == '-');
    if(!valid_response){
        return -1;
    }
    return 0;
}

int read_passive_response(FILE* socketFilefd,int* port){
    char buf[256];
    do {
        fgets(buf, 255, socketFilefd);
        printf("%s", buf);
    } while (buf[3] != ' ');
    strtok(buf,"(");//trash
    strtok(NULL,",");
    strtok(NULL,",");
    strtok(NULL,",");
    strtok(NULL,",");
    int port0 = atoi(strtok(NULL,","));
    int port1 = atoi(strtok(NULL,")"));
    (*port) = port0 * 256 + port1;
    
    return 0;
}

int open_conection(int* sockfd,int SERVER_PORT, char* ip){
    struct sockaddr_in server_addr;
    
    bzero((char *) &server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = inet_addr(ip);    /*32 bit Internet address network byte ordered*/
    server_addr.sin_port = htons(SERVER_PORT);        /*server TCP port must be network byte ordered */
    
        /*open a TCP socket*/
    
    if ((*sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("socket()");
        exit(-1);
    }
        /*connect to the server*/
    
    if (connect(*sockfd,
                (struct sockaddr *) &server_addr,
                sizeof(server_addr)) < 0) {
        perror("connect()");
        exit(-1);
    }
    
    
    return 0;
}

int parse_url(char* input,url *url){

    if(strncmp(input,"ftp://",6) != 0){
        printf("Invalid url format \n");
        return -1;
    }
    
    input += 6;
    char* username;
    char* password;
    char* host;
    char* path;
    username = strtok(input,":");
    password = strtok(NULL, "@");
    if(username == NULL || password == NULL){
        username = "anonymous";
        password = "any";
        host = strtok(input,"/");
        path = strtok(NULL,"");
    }
    else{
        host = strtok(NULL,"/");
        path = strtok(NULL,"");
    }
    strcpy(url->username,username);
    strcpy(url->password,password);
    strcpy(url->host, host);
    strcpy(url->path,path);
    get_ip(url);
    return 0;
}

int download_file(int sockDatafd, char* path){
    //getting filename
    char editPath[256];
    strcpy(editPath,path);
    char* token = strtok(editPath,"/");
    char* filename = "";
    while(token != NULL){
        filename = token;
        token = strtok(NULL,"/");
    }
    int fd = open(filename,O_WRONLY | O_CREAT,0777);
    if(fd < 0){
        printf("Couldn't open file");
        return -1;
    }
    int bytes;
    char buf[256];
    while((bytes = read(sockDatafd,buf,256)) > 0){
        write(fd,buf,bytes);
    }
    close(fd);
    return 0;
}
