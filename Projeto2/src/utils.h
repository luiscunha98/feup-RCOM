#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>


typedef struct {
    char username[128];
    char password[128];
    char host[256];
    char hostname[128];
    char path[250];
    char ip[256];
} url;

void display_info(url url);

int get_ip(url *url);

int read_socket_response(FILE* socketFilefd);

int read_passive_response(FILE* socketFilefd,int* port);

int write_socket(int socketFilefd,char* buf);

int open_conection(int *sockfd,int server_port, char* ip);

int download_file(int sockDatafd,char* path);

int parse_url(char* input,url *url);