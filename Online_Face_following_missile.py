# -*- coding: utf-8 -*-
"""
Created on Mon Jan 22 08:39:21 2024

@author: haal0002
"""

#!/usr/bin/env python3
# encoding: utf8

import time
import threading
import usb.core
import usb.util
### USB MISSILE  THUNDER
## https://devicehunt.com/search/type/usb/vendor/2123/device/any
## USB	2123	Cheeky Dream	1010	Rocket Launcher
VENDOR = 0x2123
PRODUCT = 0x1010
DOWN = 1
UP = 2
LEFT = 4
UP_LEFT = 6
DOWN_LEFT = 5
SLOW_LEFT = 7
RIGHT = 8
UP_RIGHT = 10
DOWN_RIGHT = 9
SLOW_RIGHT = 11
SLOW_UP = 14
SLOW_DOWN = 13
FIRE = 16
STOP = 32



#// | 0 | 0 | 0 | 0 | 1 | 1 – Up
#// | 0 | 0 | 0 | 1 | 0 | 2 – Down
#// | 0 | 0 | 0 | 1 | 1 | 3 – nothing
#// | 0 | 0 | 1 | 0 | 0 | 4 – Left
#// | 0 | 0 | 1 | 0 | 1 | 5 – Up / Left
#// | 0 | 0 | 1 | 1 | 0 | 6 – Down / left
#// | 0 | 0 | 1 | 1 | 1 | 7 – Slow left
#// | 0 | 1 | 0 | 0 | 0 | 8 – Right
#// | 0 | 1 | 0 | 0 | 1 | 9 – Up / Right
#// | 0 | 1 | 0 | 1 | 0 | 10 – Down / Right
#// | 0 | 1 | 0 | 1 | 1 | 11 – Slow Right
#// | 0 | 1 | 1 | 0 | 0 | 12 – nothing
#// | 0 | 1 | 1 | 0 | 1 | 13 – Slow Up
#// | 0 | 1 | 1 | 1 | 0 | 14 – Slow Down
#// | 0 | 1 | 1 | 1 | 1 | 15 – Stop
#// | 1 | 0 | 0 | 0 | 0 | 16 – Fire

#Finetuning
Threshold = 30           #Minimale Distanz für Bewegung

SlowThreshold = 500         #Maximale Distanz für langsame Bewegung

ImageDelay = 0.05            #Delay nach Verarbeiten des Bildes
DifferenceThreshold = 30   #Maximaler Unterschied zwischen x und y Distanz für Kombinierte Bewegung




dev = usb.core.find(idVendor=VENDOR, idProduct=PRODUCT)

if dev is None:
    raise Exception('Could not find USB device')

for cfg in dev:
    print(str(cfg.bConfigurationValue),'\n')

try:
    dev.detach_kernel_driver(0)
    print("Device unregistered")
except Exception:
    print("Already unregistered")
    pass # already unregistered
dev.reset()

class Launcher(object):
    def __init__(self, dev):
        self.dev = dev
        self.dev.set_configuration()
        self.cfg = dev.get_active_configuration()
        self.intf = self.cfg[(0,0)]

        usb.util.claim_interface(self.dev, self.intf)

        self.ep = usb.util.find_descriptor(self.intf, custom_match=\
lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

        self.send_command(0)

        self.t = threading.Thread(target=self.read_process)
        self.running = True
        self.firing = False

        self.state = {
            'up' : False,
            'down' : False,
            'left' : False,
            'right' : False,
            'fire' : False,
        }

        self.t.start()

#        try:
#            self.dev.reset()
#        except usb.core.USBError, e:
#            print("RESET ERROR", e)

    def read_process(self):
        abort_fire = False
        fire_complete_time = time.time()
        while self.running:
            time.sleep(0.1)
            if self.firing and abort_fire:
                if time.time() - fire_complete_time > 1.0:
                    print("Aborting fire")
                    self.send_command(0)
                    self.firing = False
                    abort_fire = False

            data = self.read(8)
            #print(data)
            if data:
                a,b = data[:2]
                RIGHT_LIMIT = (b & 0x08) != 0
                LEFT_LIMIT  = (b & 0x04) != 0
                FIRE_COMPLETED = (b & 0x80) != 0
                UP_LIMIT = (a & 0x80) != 0
                DOWN_LIMIT = (a & 0x40) != 0

                self.state['up'] = UP_LIMIT
                self.state['down'] = DOWN_LIMIT
                self.state['left'] = LEFT_LIMIT
                self.state['right'] = RIGHT_LIMIT
                self.state['fire'] = FIRE_COMPLETED

                if LEFT_LIMIT:
                    pass
                    print("All the way left")
                elif RIGHT_LIMIT:
                    pass
                    print("All the way right")
                elif UP_LIMIT:
                    pass
                    print("All the way up")
                elif DOWN_LIMIT:
                    pass
                    print("All the way down")

                if FIRE_COMPLETED and self.firing and not abort_fire:
                    print("Fire aborted")
                    fire_complete_time = time.time()
                    abort_fire = True
        print("THREAD STOPPED")


    def read(self, length):
        try:
            return self.ep.read(length)
        except usb.core.USBError:
            return None

    def send_command(self, command):
        try:
            self.dev.ctrl_transfer(0x21, 0x09, 0x200, 0, [0x02, command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]) ## DEVICE_THUNDER
        except usb.core.USBError as e:
            print("SEND ERROR", e)

launcher = Launcher(dev)
calibrated = 0

def Calibrate():
    calibrated = 1
    print ("Calibrate the Launcher")


    delay = 5
    launcher.send_command(RIGHT)
    #if launcher.state['right']:.-
    #    delay = 0
    #else:
    #    launcher.send_command(RIGHT)
    time.sleep(delay)
    launcher.send_command(STOP)

    delay = 3
    launcher.send_command(LEFT)
    #if launcher.state['right']:
    #    delay = 0
    #else:
    #    launcher.send_command(UP)
    time.sleep(delay)
    launcher.send_command(STOP)


XPos = 0
#Calibrate()



print('starting Automatic Aim')

# import the opencv library
import cv2
from PIL import Image
import face_recognition
import random


write_faces = False
show_faces = True
XPos = 0

# Versuche verschiedene Indizes und prüfe, ob eine Kamera geöffnet werden kann
def find_camera():
    for i in range(5):  # Teste Kamera-Indizes von 0 bis 4
        print(f"Versuche Kamera mit Index {i}...")
        vid = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # Versuche, mit DirectShow (Windows) zu öffnen
        if vid.isOpened():
            print(f"Kamera mit Index {i} erfolgreich geöffnet.")
            return vid
        else:
            print(f"Kamera mit Index {i} konnte nicht geöffnet werden.")
        vid.release()  # Kamera-Objekt nach dem Test wieder freigeben
    return None

# Finde die Kamera
vid = find_camera()

# Wenn keine Kamera gefunden wurde, beende das Programm
if vid is None:
    print("Fehler: Keine Kamera gefunden.")
    exit()

# Weiter mit der Bildaufnahme
print("Video Capture in progress...")
cv2.namedWindow("preview")



def Raketenregelung(x,y):
    print(x,y)
    calibrated = 0
    launcher.send_command(STOP)
    if abs(x) > Threshold or abs(y) > Threshold:
        if ((abs(x) > SlowThreshold) and (abs(y) > SlowThreshold)):
            #if (abs(abs(x)-abs(y)) < DifferenceThreshold):
                try:
                    if ((x < 0) and y < 0):
                        launcher.send_command(DOWN_LEFT)
                        print('ul')
                    if ((x > 0) and y < 0):
                        launcher.send_command(DOWN_RIGHT)
                        print('dr')
                    if ((x > 0) and y > 0):
                        launcher.send_command(UP_RIGHT)
                        print('ur')
                    else:
                        launcher.send_command(UP_LEFT)
                        print('ul')
                    pass

                except Exception as e:
                    print(f"Fehler beim Senden des Befehls: {e}")
        else:
            if abs(x) > Threshold:
                try:
                    if x < 0:
                            launcher.send_command(LEFT)
                    else:
                            launcher.send_command(RIGHT)
                except Exception as e:
                    print(f"Fehler beim Senden des Befehls: {e}")
            else:
                if abs(y) > Threshold:
                        try:
                            if y > 0:
                                launcher.send_command(UP)
                            else:
                                launcher.send_command(DOWN)
                        except Exception as e:
                            print(f"Fehler beim Senden des Befehls: {e}")

    else:
        launcher.send_command(STOP)

import cv2
import dlib
import numpy as np

# Initialize video capture
cap = cv2.VideoCapture(0)  # Use 0 for webcam, adjust accordingly for other cameras

# Initialize dlib's face detector and tracker
detector = dlib.get_frontal_face_detector()
tracker = dlib.correlation_tracker()

face_detected = False

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if not face_detected:
        # Detect faces if none are tracked
        faces = detector(gray)
        if faces:
            # Assuming the largest face is the closest
            largest_face = max(faces, key=lambda r: r.width() * r.height())
            tracker.start_track(frame, largest_face)
            face_detected = True
    else:
        # Update the tracker
        tracker.update(frame)
        pos = tracker.get_position()

        # Calculate center of the tracked face
        face_x = (pos.left() + pos.right()) // 2
        face_y = (pos.top() + pos.bottom()) // 2

        # Calculate frame center
        frame_center_x = frame.shape[1] // 2
        frame_center_y = frame.shape[0] // 2

        diff_x = face_x - frame_center_x
        diff_y = face_y - frame_center_y

        print(diff_x, " ", diff_y)

        Raketenregelung(diff_x, diff_y)

        # Draw bounding box for visualization
        cv2.rectangle(frame, (int(pos.left()), int(pos.top())),
                      (int(pos.right()), int(pos.bottom())), (0, 255, 0), 2)

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()