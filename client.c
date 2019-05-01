#include "client.h"


int main(int argc, char *argv[]){
  if(argc < 4){
    printf("Usage: ./client <ip address of broombot> <command_num> <integer> <integer>\n");
    //see client.h for commands
    return -1;
  }
  
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

  char buf[9];
  switch(argv[1]){
  case "1":
    buf[0] = MOVE_REL_CMD;
    break;
  case "2":
    buf[0] = MOVE_ABS_CMD;
    break;
  case "3":
    buf[0] = CALIBRATE_CMD;
    break;
  }
  
  int x = atoi(argv[2]);
  int y = atoi(argv[3]);
  memcpy(buf+1, x, sizeof(x)); //sizeof int is 4.
  memcpy(buf+5, x, sizeof(y));
  send(sockfd, buf, 9, 0);
}
