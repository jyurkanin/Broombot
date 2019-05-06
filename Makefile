all:
	gcc broombot.c -o broombot -g -lwiringPi -lpthread
client:
	gcc client.c -o client -g
