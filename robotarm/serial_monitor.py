import serial
import time
import threading

baud = 115200
port = "/dev/ttyACM0"
serialDevice = None

dataBuffer = [0, 0, 0, 0]
dataSum = 0
samples = 0

bufferLock = threading.Lock()


# result = (effective velocity in rad/s??, absolute rotation, time between data request)

def getData():
    global dataBuffer
    global bufferLock
    global dataSum
    global samples
    scalingFactor = 1.07153 ## 2pi/reported value for a full rotation

    bufferLock.acquire()
    result = (scalingFactor*dataBuffer[2], dataSum*scalingFactor, dataBuffer[3]/1000000 )
    dataBuffer = [0,0,0,0]
    samples = 0
    bufferLock.release()
    return result






"""
    init should be called automatically on import 
"""

def init():
    global serialDevice

    try:

        serialDevice = serial.Serial(port, baud, timeout = 1.0)
    except:
        try:
            serialDevice = serial.Serial("/dev/ttyACM1", baud, timeout = 1.0)
        except:
            serialDevice = None
    thread = threading.Thread(target=threadLoop)
    thread.start()



"""
    reads data off of the microcontroller
    format:
    [0] = X-axis rotational velocity
    [1] = Y-axis rotational velocity
    [2] = Z-axis rotational velocity
    [3] = time between samples in microseconds

    this may report 0's for up to 10 seconds as the IMU controller connects
    to the access point. 
    
"""
def updateData():
    global dataSum
    global serialDevice
    global dataBuffer
    global bufferLock
    global samples
    
    try:
        data = serialDevice.readline().decode('utf-8').rstrip().split(",")
        data[0] = float(data[0][2:])
        data[1] = float(data[1][2:])
        data[2] = float(data[2][2:])
        data[3] = int(data[3][3:])
    except:
        data = [0, 0, 0, 0]

    dataSum +=data[2]*data[3]/1000000

    bufferLock.acquire()
    for x in range(4):
        dataBuffer[x] += data[x]
    samples+=1
    bufferLock.release()

def threadLoop():
    # print("starting loop")
    while True:
        updateData()


def main():
    while True:
        input("")
        data = getData()
        print(data)

    



init()
if __name__ == "__main__":
    main()