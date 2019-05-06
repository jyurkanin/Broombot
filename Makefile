all:
	gcc broombot.c -o broombot -g -lwiringPi -lpthread -lm
client:
	gcc client.c -o client -g
