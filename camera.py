import numpy as np
import imutils
import cv2
import time
from collections import deque
from threading import Event

# TODO add face recognition code to this. If it can follow a face, that would be more accurate that colors

# define the lower and upper boundaries of the "green"
# ball in the HSV color space, then initialize the
# list of tracked points
color = None

# TODO adjust these thresholds
if color == "green":
    colorLower = (29, 86, 6)
    colorUpper = (64, 255, 255)
elif color == "red":
    colorLower = (0, 114, 6)
    colorUpper = (225, 255, 255)
else:
    colorLower = (30, 50, 10)
    colorUpper = (70, 255, 255)
buffer = 64
pts = deque(maxlen=buffer)
currX = None

isNvidia = True
debug = False
event = Event()

# Get a reference to webcam #0 (the default one)
def gstreamer_pipeline(
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=30,
        flip_method=2,
):
    return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
    )


# To flip the image, modify the flip_method parameter (0 and 2 are the most common)
if isNvidia:
    camera = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
else:
    camera = cv2.VideoCapture(0)


def camera_thread():
    # keep looping
    while True:
        if event.is_set():
            break
        # grab the current frame
        (grabbed, frame) = camera.read()

        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=720)
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colorLower, colorUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if (radius < 300) & (radius > 10):
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                if debug:
                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, center, 5, (0, 0, 255), -1)

        # update the points queue
        pts.appendleft(center)
        global currX
        if center is not None:
            currX = center[0]
        else:
            currX = -1

        if debug:
            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue

                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

            # show the frame to our screen
            cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    # cleanup the camera and close any open windows
    camera.release()
    cv2.destroyAllWindows()