import socket
import math
import time

class plutodrone:

    def __init__(self,TCP_IP = "192.168.4.1",TCP_PORT = 23):
        
        self.TCP_IP = TCP_IP
        self.TCP_PORT = TCP_PORT
        self.buffer=1024
        self.mySocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.mySocket.connect((self.TCP_IP, self.TCP_PORT))

        self.roll=1500
        self.pitch=1500
        self.throttle=1000
        self.yaw=1500
        
        # MSP_SET_RAW_RC Packet (Total Size = 22)
        self.rcvalues=bytearray([])
        self.rcvalues.append(36)              # Header 1 ---------- Byte 1
        self.rcvalues.append(77)              # Header 2 ---------- Byte 2
        self.rcvalues.append(60)              # Direction IN ------ Byte 3
        self.rcvalues.append(16)              # Message Length ---- Byte 4
        self.rcvalues.append(200)             # Type of Payload --- Byte 5
        self.rcvalues.extend([220,5])         # Roll Stick -------- Byte 6 -7 -------- Range 900 - 2100
        self.rcvalues.extend([220,5])         # Pitch Stick ------- Byte 8 -9 -------- Range 900 - 2100
        self.rcvalues.extend([232,3])         # Throttle Stick ---- Byte 10 -11 ------ Range 900 - 2100
        self.rcvalues.extend([220,5])         # Yaw Stick --------- Byte 12 -13 ------ Range 900 - 2100
        self.rcvalues.extend([176,4])         # Aux1 (Mag Mode) --- Byte 14 -15 
        self.rcvalues.extend([176,4])         # Aux2 (Dev Mode) --- Byte 16 -17 
        self.rcvalues.extend([220,5])         # Aux3(Alt Hold) ---- Byte 18 -19 
        self.rcvalues.extend([176,4])         # Aux4(ARM) --------- Byte 20 -21 
        self.rcvalues.append(100)             # CRC---------------- Byte 22

        # MSP_ALTITUDE PACKET (Total Size = 6)
        self.altvalues=bytearray([])
        self.altvalues.append(36)              # Header 1 ---------- Byte 1
        self.altvalues.append(77)              # Header 2 ---------- Byte 2
        self.altvalues.append(60)              # Direction IN ------ Byte 3
        self.altvalues.append(6)               # Message Length ---- Byte 4
        self.altvalues.append(109)             # Type of Payload --- Byte 5
        self.altvalues.append(6^109)           # CRC ---------------- Byte 6  

        # MSP_ATTITUDE PACKET (Total Size = 6)
        self.attvalues=bytearray([])
        self.attvalues.append(36)              # Header 1 ---------- Byte 1
        self.attvalues.append(77)              # Header 2 ---------- Byte 2
        self.attvalues.append(60)              # Direction IN ------ Byte 3
        self.attvalues.append(6)               # Message Length ---- Byte 4
        self.attvalues.append(108)             # Type of Payload --- Byte 5
        self.attvalues.append(6^108)           # CRC --------------- Byte 6

        # MSP_RAW_IMU (Total Size = 6)
        self.imuvalues=bytearray([])
        self.imuvalues.append(36)              # Header 1 ---------- Byte 1
        self.imuvalues.append(77)              # Header 2 ---------- Byte 2
        self.imuvalues.append(60)              # Direction IN ------ Byte 3
        self.imuvalues.append(18)              # Message Length ---- Byte 4
        self.imuvalues.append(102)             # Type of Payload --- Byte 5
        self.imuvalues.append(6^102)           # CRC --------------- Byte 6

        # MSP_SET_COMMAND (Total Size = 8)
        self.cmdvalues = bytearray([])
        self.cmdvalues.append(36)              # Header 1 ---------- Byte 1
        self.cmdvalues.append(77)              # Header 2 ---------- Byte 2
        self.cmdvalues.append(60)              # Direction IN ------ Byte 3
        self.cmdvalues.append(2)               # Message Length ---- Byte 4
        self.cmdvalues.append(217)             # Type of Payload --- Byte 5
        self.cmdvalues.extend([2,0])           # Payload ----------- Byte 6-7 (Takeoff - 1, Land - 2)
        self.cmdvalues.append(217)             # CRC --------------- Byte 8 (Takeoff - 218, Land - 217)

    # Method to send commands to the drone 
    def send(self,commands):
        self.mySocket.send(commands)

    # Method to receive information from the drone
    def receive(self):
        return self.mySocket.recv(self.buffer)

    # Method to calculate the Cyclic Redundancy Check (CRC) value
    def calcCRC(self):
        val=0
        for i in range(3,21):
            val=val^self.rcvalues[i]
        return val

    # Method to takeoff the drone
    def takeoff(self):
        # Sets the throttle to 1000 
        self.throttle=1000
        self.setcmd(0.1)

        # Sets the throttle to 1700 and runs the command for 3.5 seconds
        print("TAKEOFF")
        self.throttle = 1700
        period = 3.5
        self.setcmd(period)
               
        # Hovers at the takeoff altitutde for 0.5 seconds
        print("HOVER")
        self.throttle = 1560
        period = 0.5
        self.setcmd(period)

        # Method to land the drone
    def land(self):
        print("LAND")
        # First resets all the values of the drone
        self.roll = 1500
        self.pitch = 1500
        self.throttle = 1500
        self.yaw = 1500
        period = 0.2
        self.setcmd(period)
        # Gradually decrease the throttle of the drone
        while self.throttle!=900:
            self.throttle=self.throttle-50
            period = 0.25
            self.setcmd(period)

    # Method to arm the drone
    def arm(self):
        self.rcvalues[19]=220  # 1300 < AUX4 <1700 => ARM
        self.rcvalues[20]=5
        self.rcvalues[21]=self.calcCRC()
        self.send(self.rcvalues)
        print("Drone ARMED")

    # Method to disarm the drone
    def disarm(self):
        self.rcvalues[19]=176 # AUX4 outside range 1300 - 1700
        self.rcvalues[20]=4
        self.rcvalues[21]=self.calcCRC()
        self.send(self.rcvalues)
        print("Drone DISARMED")

    # Method to disconnect the drone
    def disconnect(self):
        self.mySocket.close()
        print("Drone DISCONNECTED")
    
    # Method to convert the value into two bytes
    def converttobytes(self,val):
        temp=bytearray([])
        temp.append(val%256)
        temp.append(math.floor(val/256))
        return temp

    # Method to send commands to the drone using the MSP_SET_RAW_RC Packet
    def setcmd(self,period): 
        temp=self.converttobytes(self.roll)
        self.rcvalues[5]=temp[0]
        self.rcvalues[6]=temp[1]
        self.rcvalues[21]=self.calcCRC()
        temp=self.converttobytes(self.pitch)
        self.rcvalues[7]=temp[0]
        self.rcvalues[8]=temp[1]
        self.rcvalues[21]=self.calcCRC()
        temp=self.converttobytes(self.yaw)
        self.rcvalues[11]=temp[0]
        self.rcvalues[12]=temp[1]
        self.rcvalues[21]=self.calcCRC()
        temp=self.converttobytes(self.throttle)
        self.rcvalues[9]=temp[0]
        self.rcvalues[10]=temp[1]
        self.rcvalues[21]=self.calcCRC()
       # print(self.rcvalues)

        # Repeatedly sends the packet to the drone at a frequency of 50 hertz for the specified time duration
        for i in range(int(50*period)):
            self.send(self.rcvalues)
            time.sleep(0.02)

    # Method to get the attitude of the drone
    def getattitude(self):
        self.send(self.attvalues)
        print(self.receive())

    # Method to get the altitude of the drone
    def getaltitude(self):
        self.send(self.altvalues)
        print(self.receive())

    # Method to get the IMU data of the drone
    def getimu(self):
        self.send(self.imuvalues)
        print(self.receive())

    

    

    

