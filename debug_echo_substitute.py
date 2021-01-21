# This script is used in place of the Amazon Echo for quicker testing
import driver
import camera
from threading import Thread
# import facerec_from_webcam_faster as face_rec
import time

# TODO Make it easier to keep track of threads and their events

threads = []
newCommand = False


def forward():
    forward_thread = Thread(target=driver.forward)
    forward_thread.start()
    threads.append(forward_thread)


def left():
    None


def right():
    None


# TODO Before starting following thread (or distance thread), set events for all other Arduino threads
#   and join the threads. This prevents command conflicts to the Arduino.
#   How can I set all the events associated with a thread, though?
def follow_me():
    follow_thread = Thread(target=driver.follow_thread, name="following_thread")
    follow_thread.start()
    threads.append(follow_thread)


def roam():
    roam_thread = Thread(target=driver.roam_thread(), name="roam_thread")
    driver.follow_event.set()
    for i in range(len(threads)):
        if threads[i].name == "following_thread":
            threads.pop(i).join()


def calibrate():
    driver.calibrate()


def dance():
    None


if __name__ == '__main__':
    print("starting...")
    cam_thread = Thread(target=camera.camera_thread)
    # face_rec_thread = Thread(target=face_rec.facerec_thread)
    # face_rec_thread.start()
    # threads.append(face_rec_thread)
    cam_thread.start()
    threads.append(cam_thread)
    # forward()
    # driver.forward()

    try:
        while True:
            command = input()
            time.sleep(1)
            if command == 'follow':
                follow_me()
            elif command == 'calibrate':
                calibrate()
            elif command == 'roam':
                roam()
            elif command == 'stop following':
                driver.follow_event.set()
                for i in range(len(threads)):
                    if threads[i].name == "following_thread":
                        threads.pop(i).join()
            elif command == 'stop all':
                driver.follow_event.set()
                camera.event.set()
                for i in range(len(threads)):
                    threads.pop().join()
                break

    except (KeyboardInterrupt, Exception):
        driver.follow_event.set()
        # face_rec.event.set()
        camera.event.set()
        for i in range(len(threads)):
            threads.pop().join()



