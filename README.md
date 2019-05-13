# Broombot
This repo contains the code that runs the broombot server
as well as the client code that allows you to connect to
the server.

The server works by running two threads, one handles one incoming connection
at a time, and the other thread keeps track of the motor postion and controls 
them to reach a desired position. Because the motor encoders are relative 
encoders, the broombot constantly saves the position of the motors so that when
the robot is turned off/on, the motor positions are preserved.

The client.py file has a Broombot class for talking to the broombot server.
It has a constructor that takes an ip address in a string as a parameter
It provides 3 functions for controlling the motors.
move_rel(dx, dy)
	moves the motors by angles relative to the current position
	dx, dy are numbers that represent the change in angles.
move_abs(x, y)
	moves the motors directly to the specified angle
	x, y are the angles.
calibrate(x, y)
	This tells the server to believe that the angles x and y are the current
	positions of the motors.
