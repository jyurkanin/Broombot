all:
	g++ broombot.c -o broombot -lwiringPi -lpthread
	g++ client.c -o client
