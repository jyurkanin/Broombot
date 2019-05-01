#include <sys/socket.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define SERVER_PORT 8080

//CMD type
#define MOVE_REL_CMD 1 //next two bytes are added to the current angular set point
#define MOVE_ABS_CMD 2 //next two bytes are assigned to the current angular set point
#define CALIBRATE_CMD 3    //Assigns the current position to whatever. Most useful to set it to zero.

