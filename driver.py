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

class Node:
    def __init__(self):
        self.is_visited = False
        self.is_obstacle = False

startNode = Node()
startNode.isVisited = True

class AVMap:
    def __init__(self):
        self.map = [[startNode]]    # This should be a 2D list of Node objects; each Node represents a 5x5cm square
        self.start = [0, 0]
        self.currPosition = [0, 0]

    def clear_map(self):
        self.map = [[startNode]]
        self.start = [0, 0]
        self.currPosition = [0, 0]

    # Accepts coordinate values
    def update_map(self, end, obstacle=False):
        # Only implemented for Manhattan Movement for now
        a = self.currPosition[0]
        b = self.currPosition[1]
        x = end[0]
        y = end[1]
        if a < x and b == y:
            for i in range(1, x - a + 1):
                if a + i >= len(self.map[1]):
                    self.append_col()
                self.currPosition[0] += 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        elif a > x and b == y:  # when robot is moving left
            for i in range(1, a - x + 1):
                if a - i < 0:   # when robot gets to negative coordinates, we shift the whole array right
                    self.prepend_col()
                    a += 1
                    self.currPosition[0] += 1
                    self.start[0] += 1
                self.currPosition[0] -= 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        elif a == x and b < y:
            for i in range(1, y - b + 1):
                if a + i >= len(self.map):
                    self.append_row()
                self.currPosition[1] += 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        elif a == x and b > y:  # when the robot is moving up
            for i in range(1, b - y + 1):
                if b - i < 0:   # when robot gets to negative coordinates, we shift the whole array down
                    self.prepend_row()
                    b += 1
                    self.currPosition[1] += 1
                    self.start[1] += 1
                self.currPosition[0] -= 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        if obstacle:
            self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = True

    def append_col(self):
        for row in self.map:
            row.append(Node())

    def prepend_col(self):
        for row in self.map:
            row.insert(0, Node())

    def append_row(self):
        new_row = []
        for _ in self.map[0]:
            new_row.append(Node())
        self.map.append(new_row)

    def prepend_row(self):
        new_row = []
        for _ in self.map[0]:
            new_row.append(Node())
        self.map.insert(0, new_row)

    def print_map(self):
        for row in self.map:
            for element in row:
                if element.is_obstacle:
                    print('x', end=' ')
                elif element.is_visited:
                    print('#', end=' ')
                else:
                    print('o', end=' ')
            print()


avMap = AVMap() # Stores the grid of the room

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
follow_event = Event()
roam_event = Event()


def forward_test():
    print("Enter following mode")
    ser.write(b'\x02')
    time.sleep(2)

    print("Go Forward...")
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


# TODO  1) Sets Arduino mode to 'distance'
#       2) Sends Arduino a distance command based on roaming algorithm
#       3) Waits for Arduino to send a serial message back indicating distance and whether obstacle is detected
#       4) Update the path map
#       5) Plot an image of the map for debugging purposes
#       6) Repeat 2-5 based on the roaming algorithm
#       How do I turn saved paths (which are lines) into a "known area"?
def roam_thread():
    roam_event.clear()


# TODO Create navigate_thread() function that makes the robot navigate to a point in the room based on the obstacle map
#   This should use a pathfinding algorithm (see pathfinding tutorial)
#   Note: This function should still update the path map when obstacles are encountered

# TODO Create an obstacle avoidance function that navigates around an obstacle based on the path map
#   Note: This function should still update the path map when obstacles are encountered


# TODO Add system commands
def change_mode(command):
    if command == 'distance':
        ser.write(b'\x01')
    elif command == 'following':
        ser.write(b'\x02')
    elif command == 'calibration':
        ser.write(b'\x03')
    elif command == 'waiting':
        ser.write(b'\x00')

# TODO add backwards
# TODO Add following commands
# value argument is an integer the represents units of movement
# For distance, 1 unit = 5 cm. For turning, 1 unit = 3 degrees
# We directly send the unit value over to the Arduino
# Max unit value is 63 (for backward command, it's 31)
def go(command, value=None):
    if command == 'forward':
        ser.write(bytes([b'\xC0'[0] | value]))
    elif command == 'right':
        ser.write(bytes([b'\x80'[0] | value]))
    elif command == 'left':
        ser.write(bytes([b'\x40'[0] | value]))


# TODO Modify the code to account for target being on the outer edges of the camera (variable speeds)
# TODO make the code more robust so that if the target disappears for a few frames,
#   the robot doesn't just stop and start searching immediately.
def follow_thread():
    video_width = 720
    margin = 30
    left_threshold = video_width / 3 + margin
    right_threshold = video_width / 3 * 2 - margin
    follow_event.clear()
    ser.write(b'\x02')          # Set robot to Following Mode
    while True:
        if ser.in_waiting > 0:  # At the moment, Arduino does not send message to Nvidia
            if ser.read(1) == b'\x7f':
                print("Robot arrived")
                break
        if follow_event.is_set():
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
    forward_test()
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



