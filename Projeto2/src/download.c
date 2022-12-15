#include "utils.h"


int main(int argc, char** argv){
    if(argc != 2){
        printf("Invalid usage of the download application");
        return -1;
    }
    url url;
    if(parse_url(argv[1],&url) < 0)
        return -1;
    display_info(url);
    int sockfd;
    int sockDatafd;
    FILE* socketFilefd;
    open_conection(&sockfd,21,url.ip);
    socketFilefd = fdopen(sockfd,"r");
    

    //Mensagem inicial
    if(read_socket_response(socketFilefd) < 0){
        return -1;
    }



    char buf[256];

    //USER
    sprintf(buf,"user %s\r\n",url.username);
    printf("Logging in as %s", buf);
    write_socket(sockfd,buf);
    if(read_socket_response(socketFilefd) < 0){
        return -1;
    }
    

    //Password
    sprintf(buf,"pass %s\n",url.password);
    write_socket(sockfd,buf);
    if(read_socket_response(socketFilefd) < 0){
        return -1;
    }

    //Passive
    sprintf(buf,"pasv\n",5);
    write_socket(sockfd,buf);
    int port;
    read_passive_response(socketFilefd,&port);

    //download
    open_conection(&sockDatafd,port,url.ip);
    sprintf(buf,"retr %s\n",url.path);
    write_socket(sockfd,buf);
    if(read_socket_response(socketFilefd) < 0){
        return -1;
    }
    download_file(sockDatafd,url.path);

    return 0;
}