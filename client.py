import socket
import struct
import sys

class Broombot:
    def __init__(self, ip):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ip, 8080)) #port is 8080

    def __del__(self):
        sock.close()
    
    def move_rel(self, dx, dy): #the change in angle. Floating point
        buf = "\x01" #MOVE relative command
        buf = buf + struct.pack("ff", dx, dy)
        sock.send(buf)

    def move_abs(self, x, y): #the desired in angle. Floating point
        buf = "\x02" #MOVE absolute command
        buf = buf + struct.pack("ff", x, y)
        sock.send(buf)

    def calibrate(self, x, y): #sets the current position. Floating point
        buf = "\x03" #calibrate command
        buf = buf + struct.pack("ff", x, y)
        sock.send(buf)


#this prevents any code from running if this file is imported
if(__name__ == "__main__"):
    if(len(sys.argv) < 5):
        print("Usage: python client.py <ip address of broombot> <command_num> <float> <float>\n")
        sys.exit()
    
    bot = Broombot(argv[1])
    if(argv[2] == "3"):
        calibrate(float(argv[3]), float(argv[4]))
    elif(argv[2] == "2"):
        move_abs(float(argv[3]), float(argv[4]))
    elif(argv[2] == "1"):
        move_rel(float(argv[3]), float(argv[4]))
    
