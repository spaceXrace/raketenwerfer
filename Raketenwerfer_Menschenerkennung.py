# -*- coding: utf-8 -*-
"""
Created on Mon Jan 22 08:39:21 2024

@author: haal0002
"""

# !/usr/bin/env python3
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

# // | 0 | 0 | 0 | 0 | 1 | 1 – Up
# // | 0 | 0 | 0 | 1 | 0 | 2 – Down
# // | 0 | 0 | 0 | 1 | 1 | 3 – nothing
# // | 0 | 0 | 1 | 0 | 0 | 4 – Left
# // | 0 | 0 | 1 | 0 | 1 | 5 – Up / Left
# // | 0 | 0 | 1 | 1 | 0 | 6 – Down / left
# // | 0 | 0 | 1 | 1 | 1 | 7 – Slow left
# // | 0 | 1 | 0 | 0 | 0 | 8 – Right
# // | 0 | 1 | 0 | 0 | 1 | 9 – Up / Right
# // | 0 | 1 | 0 | 1 | 0 | 10 – Down / Right
# // | 0 | 1 | 0 | 1 | 1 | 11 – Slow Right
# // | 0 | 1 | 1 | 0 | 0 | 12 – nothing
# // | 0 | 1 | 1 | 0 | 1 | 13 – Slow Up
# // | 0 | 1 | 1 | 1 | 0 | 14 – Slow Down
# // | 0 | 1 | 1 | 1 | 1 | 15 – Stop
# // | 1 | 0 | 0 | 0 | 0 | 16 – Fire

# Finetuning
Threshold = 0.08  # Minimale Distanz für Bewegung
DifferenceThreshold = 0.3  # Maximaler Unterschied zwischen x und y Distanz für Kombinierte Bewegung
Empfindlichkeit = 2  # Neigt ab etwa 4 zum Überschwingen

dev = usb.core.find(idVendor=VENDOR, idProduct=PRODUCT)

if dev is None:
    raise Exception('Could not find USB device')

for cfg in dev:
    print(str(cfg.bConfigurationValue), '\n')

try:
    dev.detach_kernel_driver(0)
    print("Device unregistered")
except Exception:
    print("Already unregistered")
    pass  # already unregistered
dev.reset()


class Launcher(object):
    def __init__(self, dev):
        self.dev = dev
        self.dev.set_configuration()
        self.cfg = dev.get_active_configuration()
        self.intf = self.cfg[(0, 0)]

        usb.util.claim_interface(self.dev, self.intf)

        self.ep = usb.util.find_descriptor(self.intf, custom_match= \
            lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN)

        self.send_command(0)

        self.t = threading.Thread(target=self.read_process)
        self.running = True
        self.firing = False

        self.state = {
            'up': False,
            'down': False,
            'left': False,
            'right': False,
            'fire': False,
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
            # print(data)
            if data:
                a, b = data[:2]
                RIGHT_LIMIT = (b & 0x08) != 0
                LEFT_LIMIT = (b & 0x04) != 0
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
            self.dev.ctrl_transfer(0x21, 0x09, 0x200, 0,
                                   [0x02, command, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])  ## DEVICE_THUNDER
        except usb.core.USBError as e:
            print("SEND ERROR", e)


launcher = Launcher(dev)
calibrated = 0


def Calibrate():
    calibrated = 1
    print("Calibrate the Launcher")

    delay = 5
    launcher.send_command(RIGHT)
    # if launcher.state['right']:.-
    #    delay = 0
    # else:
    #    launcher.send_command(RIGHT)
    time.sleep(delay)
    launcher.send_command(STOP)

    delay = 3
    launcher.send_command(LEFT)
    # if launcher.state['right']:
    #    delay = 0
    # else:
    #    launcher.send_command(UP)
    time.sleep(delay)
    launcher.send_command(STOP)


# Calibrate()


print('starting Automatic Aim')

# import the opencv library
import cv2
import mediapipe as mp


write_faces = False
show_faces = True

XPos, YPos = 0,0
BEFEHL = STOP
PWM = 0.5

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
cap = find_camera()

# Wenn keine Kamera gefunden wurde, beende das Programm
if cap is None:
    print("Fehler: Keine Kamera gefunden.")
    exit()

# Weiter mit der Bildaufnahme
print("Video Capture in progress...")


# PWM-Thread-Funktion
def pwm_thread():
    Stepsize = 0.05
    while True:
        duty = PWM
        if (duty) > 1:
            duty = 1
        if duty < 0:
            duty = -1*duty
        #print(BEFEHL, duty)
        try:
            launcher.send_command(BEFEHL)
            time.sleep(Stepsize * duty)
            launcher.send_command(STOP)
            time.sleep(Stepsize * (1 - duty))
        except Exception as e:
            print(f"[Raketenregelung] Fehler: {e}")


# Raketenregelungs-Thread-Funktion
def raketenregelung_thread():
    while True:
        x = XPos
        y = YPos    
        
        global BEFEHL
        global PWM
        try:
            if abs(x) > Threshold or abs(y) > Threshold:
                if abs(x) > DifferenceThreshold and abs(y) > DifferenceThreshold:
                    PWM = (abs(x) + abs(y))**2 * Empfindlichkeit
                    print('diagonal')
                    if x < 0 and y < 0:
                        BEFEHL = DOWN_LEFT
                        # print("Befehl: DOWN_LEFT")
                    elif x > 0 and y < 0:
                        BEFEHL = DOWN_RIGHT
                        # print("Befehl: DOWN_RIGHT")
                    elif x > 0 and y > 0:
                        BEFEHL = UP_RIGHT
                        # print("Befehl: UP_RIGHT")
                    else:
                        BEFEHL = UP_LEFT
                        # print("Befehl: UP_LEFT")
                else:
                    print('gerade')
                    if abs(x) > abs(y):
                        if abs(x) > Threshold:
                            PWM = x**2 * Empfindlichkeit
                            if x < 0:
                                BEFEHL = LEFT
                                # print("Befehl: LEFT")
                            else:
                               BEFEHL = RIGHT
                                # print("Befehl: RIGHT")
                    elif abs(y) > Threshold:
                        PWM = y**2 * Empfindlichkeit
                        if y > 0:
                            BEFEHL = UP
                            # print("Befehl: UP")
                        else:
                            BEFEHL = DOWN
                            # print("Befehl: DOWN")
            else:
                befehl = STOP
                #print("Befehl: STOP")

        except Exception as e:
            print(f"[Raketenregelung] Fehler: {e}")


# Warteschlangen für die Threads

# Threads starten
pwm = threading.Thread(target=pwm_thread,  name="pwm", daemon=True)
raketenregelung = threading.Thread(target=raketenregelung_thread, name="raketenregelung",daemon=True)

pwm.start()
raketenregelung.start()

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.75, min_tracking_confidence=0.75, model_complexity=1)


frame_center_x = None
frame_center_y = None

# Hauptschleife zur kontinuierlichen Datenübergabe
try:
        while cap.isOpened():
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to capture frame.")
                break

            # Get image dimensions
            h, w, _ = frame.shape

            # Convert the frame to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Human detection using MediaPipe Pose
            results = pose.process(rgb_frame)

            XPos, YPos = 0, 0

            if results.pose_landmarks:
                # Get head landmarks
                nose = results.pose_landmarks.landmark[mp_pose.PoseLandmark.NOSE]
                left_eye = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_EYE]
                right_eye = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_EYE]

                # Calculate bounding box around the head based on landmarks
                x_min = int(min(nose.x, left_eye.x, right_eye.x) * w)
                x_max = int(max(nose.x, left_eye.x, right_eye.x) * w)
                y_min = int(min(nose.y, left_eye.y, right_eye.y) * h)
                y_max = int(max(nose.y, left_eye.y, right_eye.y) * h)

                closest_x = int(nose.x * w)
                closest_y = int(nose.y * h)

                XPos = (nose.x - 0.5) * 2
                YPos = -1 * (nose.y - 0.5) * 2

                # Draw landmarks and bounding box
                mp_drawing.draw_landmarks(
                    frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                    mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=2),
                )

                cv2.rectangle(
                    frame,
                    (x_min, y_min),
                    (x_max, y_max),
                    (0, 255, 0), 2
                )

            # Display the frame
            cv2.imshow('Human Detection', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:
    print("Programm beendet.")
    launcher.send_command(STOP)

cap.release()
launcher.send_command(STOP)
cv2.destroyAllWindows()

