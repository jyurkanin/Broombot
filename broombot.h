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

#define CMD_LEN 9
#define SET_POS_THRESHOLD .1 //radians
#define GAIN .1

#define PIN_X_PWM 14
#define PIN_X_H1 15
#define PIN_X_H2 18
#define PIN_X_QUAD_L 23
#define PIN_X_QUAD_R 24

#define PIN_Y_PWM
#define PIN_Y_QUAD_L
#define PIN_Y_QUAD_R

#define X_MIN -PI //TODO: These are guesses, figure out the actual values.
#define X_MAX PI  //
#define Y_MIN -PI
#define Y_MAX PI

//CMD type
#define MOVE_REL_CMD 1 //next two bytes are added to the current angular set point
#define MOVE_ABS_CMD 2 //next two bytes are assigned to the current angular set point
#define CALIBRATE_CMD 3    //Assigns the current position to whatever.

void* control_loop(void*);
float count_to_radians(int count); //converts encoder pulses to radians
int parseCommand(char *buf, int size);

int run_server();
int run_control_loop();

void encoder_x_isr(void);
void encoder_y_isr(void);

