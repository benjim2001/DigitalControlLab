from click import command
import serial
import getch
serialport=serial.Serial("/dev/tty50")
serial.port.baudrate=115200

while True:
    x = getch.getch()
    if x == "q":
        break
    elif x == "w":
        command = "+100+10015+00"
    elif x == "s":
        command = "-100-10015+00"
    elif x == "d":
        command = "+100-10015+00"
    elif x == "a":
        command = "-100+10015+00"
    elif x == "q":
        command = "+000+00015+00"
        serialport.write(command.encode())
        break
    serialport.write(command.encode())