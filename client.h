#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define SERVER_PORT 8080

//CMD type
#define MOVE_REL_CMD 1 //next two bytes are added to the current angular set point
#define MOVE_ABS_CMD 2 //next two bytes are assigned to the current angular set point
#define CALIBRATE_CMD 3    //Assigns the current position to whatever. Most useful to set it to zero.

//typedef char[9] CMD;

void move_rel_cmd(char buf[9], float x, float y);
void move_abs_cmd(char buf[9], float x, float y);
void calibrate_cmd(char buf[9], float x, float y);

