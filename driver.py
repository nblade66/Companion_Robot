import numpy as np
import serial
import time
from threading import Thread, Event
from binascii import hexlify
import camera


# TODO function to convert string to Serial Code for better user experience

isWindowsOS = False
serialConnected = False

if isWindowsOS:
    port = 'COM5'
else:
    port = '/dev/ttyUSB0'

if serialConnected:
    ser = serial.Serial(port=port, baudrate=9600, timeout=1)

class Node:
    def __init__(self):
        self.is_visited = False
        self.is_obstacle = False


startNode = Node()
startNode.is_visited = True


class AVMap:
    def __init__(self):
        # Map should be a 2D list of Node objects; each Node represents a 5x5cm square
        # The nested lists are the rows; the coordinates are (x,y), it means self.map[y][x]
        self.map = [[startNode]]
        self.start = [0, 0]
        self.currPosition = [0, 0]
        self.direction = 0      # Angle in degrees, 0 is right, 90 is up, 180 is left, 270 is down

    def clear_map(self):
        self.map = [[startNode]]
        self.start = [0, 0]
        self.currPosition = [0, 0]
        self.direction = 0

    # Accepts the serial message received from Arduino
    # The endpoint it uses in the code is the cartesian coordinate endpoint. The currPosition is in array coordinates.
    # In the function, the endpoint is adjusted to be in array coordinates by adding the array coordinates of the
    # Start point (the origin)
    def update_map(self, serial_byte):
        # Only implemented for Manhattan Movement for now
        end, obstacle = self.serial2endpoint(serial_byte)
        a = self.currPosition[0]
        b = self.currPosition[1]
        x = end[0] + self.start[0]
        y = end[1] + self.start[1]
        if a < x and b == y:
            self.direction = 0
            for i in range(1, x - a + 1):
                if a + i >= len(self.map[0]):
                    self.append_col()
                self.currPosition[0] += 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        elif a > x and b == y:  # when robot is moving left
            self.direction = 180
            for i in range(1, a - x + 1):
                if a - i < 0:   # when robot gets to negative coordinates, we shift the whole array right
                    self.prepend_col()
                    a += 1
                self.currPosition[0] -= 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        elif a == x and b < y:
            self.direction = 270
            for i in range(1, y - b + 1):
                if a + i >= len(self.map):
                    self.append_row()
                self.currPosition[1] += 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False

        elif a == x and b > y:  # when the robot is moving up
            self.direction = 90
            for i in range(1, b - y + 1):
                if b - i < 0:   # when robot gets to negative coordinates, we shift the whole array down
                    self.prepend_row()
                    b += 1
                self.currPosition[1] -= 1
                self.map[self.currPosition[1]][self.currPosition[0]].is_visited = True
                self.map[self.currPosition[1]][self.currPosition[0]].is_obstacle = False
        self.add_obstacle(obstacle)

    def append_col(self):
        for row in self.map:
            row.append(Node())

    def prepend_col(self):
        self.currPosition[0] += 1
        self.start[0] += 1
        for row in self.map:
            row.insert(0, Node())

    def append_row(self):
        new_row = []
        for _ in self.map[0]:
            new_row.append(Node())
        self.map.append(new_row)

    def prepend_row(self):
        self.currPosition[1] += 1
        self.start[1] += 1
        new_row = []
        for _ in self.map[0]:
            new_row.append(Node())
        self.map.insert(0, new_row)

    def add_obstacle(self, obstacle):
        if obstacle:
            if self.direction == 0:
                if self.currPosition[0] + 1 >= len(self.map[0]):
                    self.append_col()
                self.map[self.currPosition[1]][self.currPosition[0] + 1].is_obstacle = True

            elif self.direction == 180:
                if self.currPosition[0] - 1 < 0:
                    self.prepend_col()
                self.map[self.currPosition[1]][self.currPosition[0] - 1].is_obstacle = True

            elif self.direction == 270:
                if self.currPosition[1] + 1 >= len(self.map):
                    self.append_row()
                self.map[self.currPosition[1] + 1][self.currPosition[0]].is_obstacle = True

            elif self.direction == 90:
                if self.currPosition[1] - 1 < 0:
                    self.prepend_row()
                self.map[self.currPosition[1] - 1][self.currPosition[0]].is_obstacle = True

    # All endpoints are relative to the origin coordinates (start position), so start is always in the (0,0) position
    # even if the actual array coordinates are not
    def serial2endpoint(self, serial_byte):
        _, has_obstacle, value = parse_serial(serial_byte)
        # I could use trig functions here, but eh, maybe later
        endpoint = (self.currPosition[0] - self.start[0], self.currPosition[1] - self.start[1])
        if self.direction == 0:     # Facing to the right
            endpoint = (endpoint[0] + value, endpoint[1])
        elif self.direction == 90:
            endpoint = (endpoint[0], endpoint[1] - value)
        elif self.direction == 180:
            endpoint = (endpoint[0] - value, endpoint[1])
        elif self.direction == 270:
            endpoint = (endpoint[0], endpoint[1] + value)

        return endpoint, has_obstacle

    # Positive angle means turning counter-clockwise (like a unit circle in trig)
    def turn(self, angle):
        self.direction += angle
        self.direction %= 360

    # direction is found by looking at the command that was sent to the Arduino
    # TODO make determining turn direction less prone to errors. It's a bit roundabout right now
    def serial_turn(self, serial_byte, direction):
        _, has_obstacle, value = parse_serial(serial_byte)
        if direction == 'left':
            angle = value * 3       # Arduino sends turn angle in units. 1 unit = 3 degrees
        else:
            angle = value * -3
        self.turn(angle)
        self.add_obstacle(has_obstacle)

    # Sends serial message to turn the robot to face a certain direction based on the avMap
    def set_direction(self, direction):
        angle_needed = direction - self.direction
        # TODO this way of dealing with the 0 -> 270 jump is horrible. At some point, I need to improve this
        if angle_needed > 180:
            angle_needed = -90
        elif angle_needed < -180:
            angle_needed = 90
        elif angle_needed == -180:
            angle_needed = 180
        # if angle_needed == 90:      # turn left
        #     go('left', value=30)
        # elif angle_needed == -90:   # turn right
        #     go('right', value=30)
        # elif angle_needed == 180:   # turn right
        #     go('right', value=60)

        self.direction = direction

    def print_map(self):
        for y, row in enumerate(self.map):
            for x, element in enumerate(row):
                if self.currPosition[0] == x and self.currPosition[1] == y:
                    if self.direction == 0:
                        dir_symbol = '>'
                    elif self.direction == 90:
                        dir_symbol = '^'
                    elif self.direction == 180:
                        dir_symbol = '<'
                    else:
                        dir_symbol = 'v'
                    print(dir_symbol, end=' ')
                elif self.start[0] == x and self.start[1] == y:
                    print('+', end=' ')
                elif element.is_obstacle:
                    print('x', end=' ')
                elif element.is_visited:
                    print('o', end=' ')
                else:
                    print('.', end=' ')
            print()


avMap = AVMap()     # Stores the grid of the room
mode = 'waiting'
# Allowable modes are "waiting", "following", "distance", and "calibration"

# Wait for Arduino to connect
if serialConnected:
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
#       2) Sends Arduino a distance command based on roaming algorithm (let's say... move to closest non-visited spot)
#       3) Waits for Arduino to send a serial message back indicating distance and whether obstacle is detected
#       4) Update the path map
#       5) Plot an image of the map for debugging purposes
#       6) Repeat 2-5 based on the roaming algorithm
#       How do I turn saved paths (which are lines) into a "known area"?
# TODO  Test that the Serial Messages from the Arduino are correct
def roam_thread():
    roam_event.clear()
    # send turn message
    avMap.set_direction(90)
    print(f"direction: {avMap.direction}")
    # go('forward', 1)
    print(avMap.serial2endpoint(b'\x01'))
    avMap.update_map(b'\x01')
    avMap.print_map()
    print(f"direction after move: {avMap.direction}")
    print(f"Position after move: {avMap.currPosition}")
    print(f"Start after move: {avMap.start}")
    print()
    avMap.set_direction(0)
    print(f"direction: {avMap.direction}")
    print(avMap.serial2endpoint(b'\x0A'))
    avMap.update_map(b'\x0A')
    avMap.print_map()
    print(f"direction after move: {avMap.direction}")
    print(f"Position after move: {avMap.currPosition}")
    print(f"Start after move: {avMap.start}")
    print()
    avMap.clear_map()



# TODO Create navigate_thread() function that makes the robot navigate to a point in the room based on the obstacle map
#   This should use a pathfinding algorithm (see pathfinding tutorial)
#   So basically, it finds the shortest path, then if it encounters an obstacle, it refinds the shortest path
#   Note: This function should still update the path map when obstacles are encountered

# TODO Create an obstacle avoidance function that navigates around an obstacle based on the path map
#   Note: This function should still update the path map when obstacles are encountered


# TODO Add system commands
def change_mode(command):
    global mode
    if command == 'distance':
        ser.write(b'\x01')
        mode = 'distance'
    elif command == 'following':
        ser.write(b'\x02')
        mode = 'following'
    elif command == 'calibration':
        ser.write(b'\x03')
        mode = 'calibration'
    elif command == 'waiting':
        ser.write(b'\x00')
        mode = 'waiting'


# TODO add backwards
# TODO Add following commands
# value argument is an integer the represents units of movement
# For distance, 1 unit = 5 cm. For turning, 1 unit = 3 degrees
# We directly send the unit value over to the Arduino
# Max unit value is 63 (for backward command, it's 31)
def go(command, value=0):
    if command == 'forward':
        ser.write(bytes([b'\xC0'[0] | value]))
    elif command == 'right':
        ser.write(bytes([b'\x80'[0] | value]))
    elif command == 'left':
        ser.write(bytes([b'\x40'[0] | value]))


# TODO Not implemented for state messages yet
def parse_serial(serial_byte):
    message_type = 'distance' if serial_byte[0] & b'\x80'[0] == 0 else 'state'
    has_obstacle = False if serial_byte[0] & b'\x40'[0] == 0 else True
    value = serial_byte[0] & b'\x1F'[0]

    return message_type, has_obstacle, value


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



