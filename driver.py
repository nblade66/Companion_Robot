import numpy as np
import serial
import time
from threading import Thread, Event
from binascii import hexlify
import camera



# TODO function to convert string to Serial Code for better user experience

isWindowsOS = False

if isWindowsOS:
    port = 'COM5'
else:
    port = '/dev/ttyUSB0'

ser = serial.Serial(port=port, baudrate=9600, timeout=1)

# Wait for Arduino to connect
while True:
    if ser.in_waiting > 0:
        msg = ser.read(1)
        if msg == b'\xFF':
            print("Arduino connected")
            break
time.sleep(5)

ser.flushInput()

shared_arr = [0]
event = Event()


def thread_modify():
    while True:
        if event.is_set():
            break
        shared_arr[0] += 1
        time.sleep(1)


def thread_read():
    while True:
        if event.is_set():
            break
        print(f"shared data: {shared_arr}")
        time.sleep(1)


def forward():
    print("Enter following mode")
    ser.write(b'\x02')
    time.sleep(2)

    print("Go Foward...")
    ser.write(b'\xC0')
    time.sleep(5)
    print("Stopping")
    ser.write(b'\x10')
    time.sleep(1)

def right():
    ser.write(b'\x40')
    ser.write(b'\x02')
    ser.write(b'\x40')

def left():
    ser.write(b'\x80')
    ser.write(b'\x02')
    ser.write(b'\x80')


def nav_test():
    ser.write(b'\x02')  # Enter Following Mode
    time.sleep(2)
    for i in range(2):
        ser.write(b'\x40')
        time.sleep(5)
        ser.write(b'\x80')
        time.sleep(5)
        ser.write(b'\xC0')
        time.sleep(5)

    # ser.write(b'\x03')  # Enter Calibration Mode
    # time.sleep(2)
    #
    # while True:
    #     if ser.in_waiting > 0:
    #         msg = ser.read(1)
    #         print("Message from Arduino:")
    #         print(hexlify(msg))


def calibrate():
    ser.write(b'\x03')
    while True:
        if ser.in_waiting > 0:
            msg = ser.read(1)
            print("Message from Arduino")
            print(hexlify(msg))
            break


# TODO Modify the code to account for target being on the outer edges of the camera (variable speeds)
# TODO make the code more robust so that if the target disappears for a few frames,
#   the robot doesn't just stop and start searching immediately.
def follow_thread():
    video_width = 720
    margin = 30
    left_threshold = video_width / 3 + margin
    right_threshold = video_width / 3 * 2 - margin
    event.clear()
    ser.write(b'\x02')          # Set robot to Following Mode
    while True:
        if ser.in_waiting > 0:  # At the moment, Arduino does not send message to Nvidia
            if ser.read(1) == b'\x7f':
                print("Robot arrived")
                break
        if event.is_set():
            break

        if camera.currX == -1:    # No target to follow is detected
            ser.write(b'\x46')      # Turn left until target is found
            while camera.currX == -1 or camera.currX < left_threshold or camera.currX > right_threshold:
                print(camera.currX)
                time.sleep(1)
                # keep turning left until target is centered
        elif camera.currX > right_threshold:    # Color is to the right of robot
            ser.write(b'\xEE')                  # Drive Forward-right
        elif camera.currX < left_threshold:     # Target is to the left of robot
            ser.write(b'\xCE')                  # Drive Forward-left
        else:
            ser.write(b'\xC0')                  # Drive straight

        print(camera.currX)
        time.sleep(1)


# TODO This sequence of commands doesn't respond. I'll work on it later
#   I think I might need to have the Arduino send back the commands it receives to make sure the correct
#   commands are being received and processed.
if __name__ == "__main__":
    print("starting...")
    time.sleep(10)
    print("Going Forward")
    forward()
    print("Entering Nav Test")
    nav_test()
    print("Calibrating")
    calibrate()
    # color = Thread(target=camera.camera_thread)
    # color.start()
    # while True:
    #     try:
    #         if camera.currX is not None:
    #             print(camera.currX)
    #         time.sleep(1)
    #     except (KeyboardInterrupt, Exception):
    #         camera.event.set()
    #         color.join()
    #         break

    # COMMENTED OUT BELOW: USED FOR REFERENCE WHEN STARTING THREADS
    # read = Thread(target=thread_read)
    # write = Thread(target=thread_modify)
    # read.start()
    # write.start()
    # try:
    #     while True:
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     event.set()
    #     read.join()
    #     write.join()
    #     print("Keyboard Interrupt")



