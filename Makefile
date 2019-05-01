all:
	gcc broombot.c -o broombot -g -lwiringPi -lpthread
	gcc client.c -o client -g
