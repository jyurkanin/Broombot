#include "client.h"


int main(int argc, char *argv[]){
  if(argc < 5){
    printf("Usage: ./client <ip address of broombot> <command_num> <float> <float>\n");
    //see client.h for commands
    return -1;
  }
  
  char buf[9];
  buf[0] = atoi(argv[2]);
  float x = atof(argv[3]);
  float y = atof(argv[4]);
  memcpy(buf+1, &x, sizeof(x)); //sizeof int is 4.
  memcpy(buf+5, &y, sizeof(y));
  printf("%d %f %f\n", buf[0], x, y);
  
  int sockfd = 0;
  struct sockaddr_in address;
  int valread;
  struct sockaddr_in serv_addr;

  if((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0){
    printf("Couldn't open socket\n");
    return -1;
  }

  memset(&serv_addr, '0', sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_port = htons(SERVER_PORT);

  if(inet_pton(AF_INET, argv[1], &serv_addr.sin_addr) <= 0){
    printf("Invalid Server address\n");
    return -1;
  }

  if(connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0){
    printf("Connection Failed\n");
    return -1;
  }

  send(sockfd, buf, 9, 0);
}
