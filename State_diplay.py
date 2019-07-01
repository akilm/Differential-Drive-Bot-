import serial
speedin=serial.Serial('COM3',9600)
while(1):
    spd=ord(speedin.read(1))
    x=ord(speedin.read(1))
    print("spd: "+spd + " cm/s")
    print("X: " + X + " cm")
    print("Y: " + Y + " cm")

    
