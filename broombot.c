#include "broombot.h"

/*
 * what needs to be done?
 * 1. Create tcp server to listen for commands.
 * 2. Control motors to execute command.
 * Thats all
 */


float x_set_point; //x motor set point angle
float y_set_point; //x motor set point angle

int x_counter; //counts encoder pulses for each motor
int y_counter;

volatile int is_calibrated = 0;

float count_to_radians(int count){
  //TODO: Figure this one out
}

int parseCommand(char *buf, int size){
  if(size != CMD_LEN) return EXIT_FAILURE;
  float temp_x;
  float temp_y;
  
  
  switch(buf[0]){ //type of command
  case MOVE_ABS_CMD:
    memcpy(&x_set_point, buf+1, 4);
    memcpy(&y_set_point, buf+5, 4);
    break;
  case MOVE_REL_CMD:
    memcpy(&temp_x, buf+1, 4);
    memcpy(&temp_y, buf+5, 4);
    x_set_point += temp_x;
    y_set_point += temp_y;
    break;
  case CALIBRATE_CMD:
    memcpy(buf+1, &x_counter, 4);
    memcpy(buf+5, &y_counter, 4);
    break;
  }
  return EXIT_SUCCESS;
}

void* control_loop(void* ignoreme){
  while(!is_calibrated){
    usleep(10000); //sleep 10 milliseconds until you get a calibration. Otherwise movement is unsafe.
  }
  
  float err_x;
  float err_y;

  while(1){
    if(err_x > SET_POS_THRESHOLD){
      
    }
    if(err_y > SET_POS_THRESHOLD){
      
    }
  }
}

int run_server(){
  struct sockaddr_in address;
  int addrlen = sizeof(address);
  int opt = 1;
  int sockfd, new_socket, valread;
  char buf[CMD_LEN] = {0};
  //Initializes the server socket
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));
  
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(8080);

  bind(sockfd, (struct sockaddr *) &address, sizeof(address));

  //main loop, begins listening for connections
  while(1){
    listen(sockfd, 3);
    new_socket = accept(sockfd, (struct sockaddr*) &address, (socklen_t*)&addrlen);
    //got connection, now wait for a command to be received
    while(valread = read(new_socket, buf, CMD_LEN)){
      if(parseCommand(buf, valread)){
	perror("buffer underrun");
	break;
      }
    }
    
  }
}

int run_control_loop(){
  //got to initialize the control loop thread and the wiringpi library
  wiringPiSetupGPIO();
  pinMode(PIN_X_QUAD_L, INPUT);
  pinMode(PIN_X_QUAD_R, INPUT);

  pinMode(PIN_Y_QUAD_L, INPUT);
  pinMode(PIN_Y_QUAD_R, INPUT);


  pthread_t motor_thread;
  pthread_create(&motor_thread, NULL, &control_loop, NULL);
  
}

int main(int argc, char *argv[]){
  //pthread_t socket_thread;
  //pthread_create(&socket_thread, NULL, &socket_handler, NULL);
  printf("Sizeof int %u\nsizeof float %u\n", sizeof(int), sizeof(float));
  
  run_control_loop();
  run_server();
}
