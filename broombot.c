#include "broombot.h"

/*
 * what needs to be done?
 * 1. Create tcp server to listen for commands.
 * 2. Control motors to execute command.
 * Thats all
 */


float x_set_point; //x motor set point angle
float y_set_point; //x motor set point angle

int x_counter = 0; //counts encoder pulses for each motor
int y_counter = 0;

volatile int is_calibrated = 0;

float count_to_radians(int count){
  return 2*M_PI*count / (10000);
}

float gain(float error){ //gain proportional to the error. Output must be bounded, from 0-100. Input is from 0-2pi
  return 70;
}

int parse_command(char *buf, int size){
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

  //these lines keep the motion bounded to keep from the pendulum from colliding with the base.
  if(x_set_point > X_MAX) x_set_point = X_MAX;
  else if(x_set_point < X_MIN) x_set_point = X_MIN;
  if(y_set_point > Y_MAX) y_set_point = Y_MAX;
  else if(y_set_point < Y_MIN) y_set_point = Y_MIN;
  
  return EXIT_SUCCESS;
}

void* control_loop(void* ignoreme){ //I think this looks good
  while(!is_calibrated){
    usleep(10000); //sleep 10 milliseconds until you get a calibration. Otherwise movement is unsafe.
  }
  
  float err_x;
  float err_y;

  //0 means one direction, 1 means the other.
  int curr_dir_x = 0; //motor can be damaged by rapid direction changes.
  int curr_dir_y = 0; //at least thats what it said on a website.
  
  while(1){
    FILE *file = open(RECORDED_STATE_FILE, "w");
    fprintf(file, "%d\n%d", x_counter, y_counter);
    fclose(file); //jank. opening and closing a file really fast sounds like a bad idea.
    
    usleep(1000); //sleep for a millisecond to reduce load on the cpu
    
    
    err_x = x_set_point - count_to_radians(x_counter);
    if(err_x > SET_POS_THRESHOLD){
      if(curr_dir_x){
	curr_dir_x = 0;
	digitalWrite(PIN_X_H1, LOW);
	digitalWrite(PIN_X_H2, LOW);
      }
      else{
	digitalWrite(PIN_X_H1, LOW);
	digitalWrite(PIN_X_H2, HIGH);
	softPwmWrite(PIN_X_PWM, gain(err_x));
      }
    }
    else if(err_x < -SET_POS_THRESHOLD){
      if(!curr_dir_x){
	curr_dir_x = 1;
	digitalWrite(PIN_X_H1, LOW);
	digitalWrite(PIN_X_H2, LOW);
      }
      else{
	digitalWrite(PIN_X_H1, HIGH);
	digitalWrite(PIN_X_H2, LOW);
	softPwmWrite(PIN_X_PWM, gain(err_x));
      }
    }
    else{
      softPwmWrite(PIN_X_PWM, 0);
      digitalWrite(PIN_X_H1, LOW);
      digitalWrite(PIN_X_H2, LOW);
    }

    err_y = y_set_point - count_to_radians(y_counter);
    if(err_y > SET_POS_THRESHOLD){
      if(curr_dir_y){
	curr_dir_y = 0;
	digitalWrite(PIN_Y_H1, LOW);
	digitalWrite(PIN_Y_H2, LOW);
      }
      else{
	digitalWrite(PIN_Y_H1, LOW);
	digitalWrite(PIN_Y_H2, HIGH);
	softPwmWrite(PIN_Y_PWM, gain(err_y));
      }
    }
    else if(err_y < -SET_POS_THRESHOLD){
      if(!curr_dir_y){
	curr_dir_y = 1;
	digitalWrite(PIN_Y_H1, LOW);
	digitalWrite(PIN_Y_H2, LOW);
      }
      else{
	digitalWrite(PIN_Y_H1, HIGH);
	digitalWrite(PIN_Y_H2, LOW);
	softPwmWrite(PIN_Y_PWM, gain(err_y));
      }
    }
    else{
      softPwmWrite(PIN_Y_PWM, 0);
      digitalWrite(PIN_Y_H1, LOW);
      digitalWrite(PIN_Y_H2, LOW);
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
  address.sin_port = htons(SERVER_PORT);

  bind(sockfd, (struct sockaddr *) &address, sizeof(address));

  //main loop, begins listening for connections
  while(1){
    listen(sockfd, 3);
    new_socket = accept(sockfd, (struct sockaddr*) &address, (socklen_t*)&addrlen);
    //got connection, now wait for a command to be received
    while(valread = read(new_socket, buf, CMD_LEN)){
      if(parse_command(buf, valread)){
	perror("buffer underrun");
	break;
      }
    }
    
  }
}

int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
uint8_t x_state = 0; //4 bits that encode the present and last values of the two encoder wires
uint8_t y_state = 0;
void encoder_x_isr(){
  x_state = x_state << 2;
  if(digitalRead(PIN_X_QUAD_L))
    x_state |= 0b0001; //set high
  else
    x_state &= 0b1110;

  if(digitalRead(PIN_X_QUAD_R))
    x_state |= 0b0010; //set high
  else
    x_state &= 0b1101; //set low
    
  x_counter += lookup_table[x_state & 0b1111];
}

void encoder_y_isr(){
  y_state = y_state << 2;
  if(digitalRead(PIN_Y_QUAD_L))
    y_state |= 0b0001; //set high
  else
    y_state &= 0b1110;

  if(digitalRead(PIN_Y_QUAD_R))
    y_state |= 0b0010; //set high
  else
    y_state &= 0b1101; //set low
    
  y_counter += lookup_table[y_state & 0b1111];
}

void test(){
  digitalWrite(PIN_X_H1, HIGH);
  digitalWrite(PIN_X_H2, LOW);
  
  softPwmWrite(PIN_X_PWM, 10);
  while(1){
    usleep(100000);
    printf("Encoder count %d\n", x_counter);
  }
}

void get_recorded_state(){
  FILE* file = fopen(RECORDED_STATE_FILE, "r");
  fscanf(file, "%d %d", x_counter, y_counter);
  fclose(file);
}

int run_control_loop(){
  //got to initialize the control loop thread and the wiringpi library
  wiringPiSetupGpio();

  pinMode(PIN_X_QUAD_L, INPUT);
  pinMode(PIN_X_QUAD_R, INPUT);
  pullUpDnControl(PIN_X_QUAD_L, PUD_DOWN);
  pullUpDnControl(PIN_X_QUAD_R, PUD_DOWN);
  pinMode(PIN_X_H1, OUTPUT);
  pinMode(PIN_X_H2, OUTPUT);

  pinMode(PIN_Y_QUAD_L, INPUT);
  pinMode(PIN_Y_QUAD_R, INPUT);
  pullUpDnControl(PIN_Y_QUAD_L, PUD_DOWN);
  pullUpDnControl(PIN_Y_QUAD_R, PUD_DOWN);
  pinMode(PIN_Y_H1, OUTPUT);
  pinMode(PIN_Y_H2, OUTPUT);
  
  
  wiringPiISR(PIN_X_QUAD_L, INT_EDGE_BOTH, encoder_x_isr);
  wiringPiISR(PIN_X_QUAD_R, INT_EDGE_BOTH, encoder_x_isr);
  
  wiringPiISR(PIN_Y_QUAD_L, INT_EDGE_BOTH, encoder_y_isr);
  wiringPiISR(PIN_Y_QUAD_R, INT_EDGE_BOTH, encoder_y_isr);

  softPwmCreate(PIN_X_PWM, 0, 100);
  softPwmCreate(PIN_Y_PWM, 0, 100);

  x_set_point = count_to_radians(10000); //try to make it do a half rotation
  //is_calibrated = 1;
  
  get_recorded_state();
  
  pthread_t motor_thread;
  pthread_create(&motor_thread, NULL, &control_loop, NULL);
}

int main(int argc, char *argv[]){
  //pthread_t socket_thread;
  //pthread_create(&socket_thread, NULL, &socket_handler, NULL);
  
  run_control_loop();
  run_server();
}
