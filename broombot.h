#include <wiringPi.h>
#include <softPwm.h>
#include <pthread.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <netinet/ip.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>

#define RECORDED_STATE_FILE "/home/pi/state"
#define SERVER_PORT 8080

#define CMD_LEN 9
#define SET_POS_THRESHOLD .01 //radians
#define GAIN .1 //ignored

#define PIN_X_PWM 14
#define PIN_X_H1 15
#define PIN_X_H2 18
#define PIN_X_QUAD_L 23
#define PIN_X_QUAD_R 24

#define PIN_Y_PWM 2
#define PIN_Y_H1 4
#define PIN_Y_H2 3
#define PIN_Y_QUAD_L 17
#define PIN_Y_QUAD_R 27

#define X_MIN -M_PI/2
#define X_MAX M_PI/2
#define Y_MIN -M_PI/3
#define Y_MAX M_PI/3

//CMD type
#define MOVE_REL_CMD 1 //next two bytes are added to the current angular set point
#define MOVE_ABS_CMD 2 //next two bytes are assigned to the current angular set point
#define CALIBRATE_CMD 3    //Assigns the current position to whatever. Most useful to set it to zero.

void* control_loop(void*);
float count_to_radians(int count); //converts encoder pulses to radians
int parse_command(char *buf, int size);
void get_recorded_state();

int run_server();
int run_control_loop();

void encoder_x_isr(void);
void encoder_y_isr(void);

