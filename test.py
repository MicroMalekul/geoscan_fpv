import pioneer_sdk
import cv2.aruco as aruco
import cv2
import time
import math
import numpy
import keyboard

cap = cv2.VideoCapture(1)
drone = pioneer_sdk.Pioneer()
dic = aruco.Dictionary_get(aruco.DICT_4X4_50) # aruco markers dictionary
par = aruco.DetectorParameters_create()
out = cv2.VideoWriter("output.avi", cv2.VideoWriter_fourcc(*"MJPG"), 25, (640, 480)) # drone records video in output.avi


# functions fu, fur and fup return value to set speeds to centre_oao
# high stands for upper bound of absolute value of speed
# low stands for lower bound of absolute value of speed
def fu(x, a=0, high=0.2, low=0.1):
    ans = max(min((x - a) / 300, high), -high)
    if 0 < ans < low:
        ans = low
    elif 0 > ans > -low:
        ans = -low
    return ans


def fur(x, a=0, high=0.15, low=0.1):
    x -= 320 - a
    ans = max(min(x / 640, high), -high)
    if 0 < ans < low:
        ans = low
    elif 0 > ans > -low:
        ans = -low
    return ans


def fup(x, a=0, high=0.15, low=0.1):
    x = 240 - x + a
    ans = max(min(x / 240, high), -high)
    if 0 < ans < low:
        ans = low
    elif 0 > ans > -low:
        ans = -low
    return ans


# centre_oao function makes drone to turn into direction aruco marker is turned and go to the centre of the marker
# size -- side of the square in centre of the screen in which centre of marker shoud go
# a -- number of aruco marker (a < 0 -- any marker)
# kx and ky -- make square stretch square in x, and y direction respectively
def centre_oao(size, a=-1, kx=0, ky=0):
    size //= 2
    pitch = roll = yaw = 0
    while 1:
        b1 = 0
        image = cap.read()[1]
        (markers, ids, fake) = aruco.detectMarkers(image, dic, parameters=par)
        if ids is None:
            ids = [-1]
        if len(markers) > 0:
            ids = ids.flatten()
        cv2.rectangle(image, (320 - size - kx, 240 - size - ky), (320 + size + kx, 240 + size + ky), (0, 0, 0), 1)
        for (i, sides) in zip(ids, markers):
            if i == a or a < 0:
                b1 = 1
                cv2.polylines(image, numpy.int32([sides]), 1, (255, 0, 0))
                markers = sides.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = markers
                k = 0
                kol = 0
                if topLeft[0] != topRight[0]:
                    kol += 1
                    k = (topLeft[1] - topRight[1]) / (topLeft[0] - topRight[0])
                if bottomLeft[0] != bottomRight[0]:
                    kol += 1
                    k += (bottomLeft[1] - bottomRight[1]) / (bottomLeft[0] - bottomRight[0])
                if topLeft[1] != bottomLeft[1]:
                    kol += 1
                    k -= (topLeft[0] - bottomLeft[0]) / (topLeft[1] - bottomLeft[1])
                if topRight[1] != bottomRight[1]:
                    kol += 1
                    k -= (topRight[0] - bottomRight[0]) / (topRight[1] - bottomRight[1])
                dif = round(640 * k / kol)
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = (topLeft[0] + bottomRight[0]) // 2
                cY = (topLeft[1] + bottomRight[1]) // 2
                cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                if abs(dif) < 11 and (cX in range(320 - size - kx, 321 + size + kx)) and (
                        cY in range(240 - size - ky, 241 + size + ky)):
                    drone.set_manual_speed_body_fixed(0, 0, 0, 0)
                    time.sleep(0.1)
                    return
                if abs(dif) > 10:
                    if topLeft[1] > bottomLeft[1]:
                        image = cv2.arrowedLine(image, (320, 240),
                                                (320 - round(320 * fu(dif)), 240 - round(240 * fu(dif))),
                                                (0, 0, 255), 2)
                        yaw = -fu(dif)
                    else:
                        image = cv2.arrowedLine(image, (320, 240),
                                                (320 + round(320 * fu(dif)), 240 - round(240 * fu(dif))),
                                                (0, 0, 255), 2)
                        yaw = fu(dif)
                else:
                    yaw = 0
                if not (cY in range(240 - size - ky, 241 + size + ky)):
                    image = cv2.arrowedLine(image, (320, 240), (320, 240 - round(240 * fup(cY))),
                                            (0, 0, 255), 2)
                    pitch = fup(cY)
                else:
                    pitch = 0
                if not (cX in range(320 - size - kx, 321 + size + kx)):
                    roll = fur(cX)
                    image = cv2.arrowedLine(image, (320, 240), (320 + round(320 * fur(cX)), 240),
                                            (0, 0, 255), 2)
                else:
                    roll = 0
        cv2.imshow('a', image)
        cv2.waitKey(1)
        out.write(image)
        if not b1:
            yaw = 0
        drone.set_manual_speed_body_fixed(roll, pitch, 0, yaw)


# direction functions make drone go in direction function is named until drone sees aruco marker with number a (if a less than zero if drone sees any marker)
def forward(a=-1, speed=0.2, x=0):
    while 1:
        drone.set_manual_speed_body_fixed(0, speed, 0, 0)
        image = cap.read()[1]
        (markers, ids, fake) = aruco.detectMarkers(image, dic, parameters=par)
        if ids is None:
            ids = [-2]
        if len(markers) > 0:
            ids = ids.flatten()
        for (sides, i) in zip(markers, ids):
            if i == a or a < 0:
                markers = sides.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = markers
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topRight = (int(topRight[0]), int(topRight[1]))
                side = math.sqrt((topLeft[0] - topRight[0]) ** 2 + (topLeft[1] - topRight[1]) ** 2)
                if side > 45:
                    drone.set_manual_speed_body_fixed(0, -1, 0, 0)
                    time.sleep(x)
                    drone.set_manual_speed_body_fixed(0, 0, 0, 0)
                    time.sleep(0.1)
                    return
        cv2.imshow('a', image)
        out.write(image)
        cv2.waitKey(1)


def backward(a=-1, speed=0.2, x=0):
    while 1:
        drone.set_manual_speed_body_fixed(0, -speed, 0, 0)
        image = cap.read()[1]
        (markers, ids, fake) = aruco.detectMarkers(image, dic, parameters=par)
        if ids is None:
            ids = [-2]
        if len(markers) > 0:
            ids = ids.flatten()
        for (sides, i) in zip(markers, ids):
            if i == a or a < 0:
                markers = sides.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = markers
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topRight = (int(topRight[0]), int(topRight[1]))
                side = math.sqrt((topLeft[0] - topRight[0]) ** 2 + (topLeft[1] - topRight[1]) ** 2)
                if side > 45:
                    drone.set_manual_speed_body_fixed(0, -1, 0, 0)
                    time.sleep(x)
                    drone.set_manual_speed_body_fixed(0, 0, 0, 0)
                    time.sleep(0.1)
                    return
        cv2.imshow('a', image)
        out.write(image)
        cv2.waitKey(1)


def left(a=-1, speed=0.2, x=0):
    while 1:
        drone.set_manual_speed_body_fixed(-speed, 0, 0, 0)
        image = cap.read()[1]
        (markers, ids, fake) = aruco.detectMarkers(image, dic, parameters=par)
        if ids is None:
            ids = [-2]
        if len(markers) > 0:
            ids = ids.flatten()
        for (sides, i) in zip(markers, ids):
            if i == a or a < 0:
                markers = sides.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = markers
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topRight = (int(topRight[0]), int(topRight[1]))
                side = math.sqrt((topLeft[0] - topRight[0]) ** 2 + (topLeft[1] - topRight[1]) ** 2)
                if side > 45:
                    drone.set_manual_speed_body_fixed(0, -1, 0, 0)
                    time.sleep(x)
                    drone.set_manual_speed_body_fixed(0, 0, 0, 0)
                    time.sleep(0.1)
                    return
        cv2.imshow('a', image)
        out.write(image)
        cv2.waitKey(1)


def right(a=-1, speed=0.2, x=0):
    while 1:
        drone.set_manual_speed_body_fixed(speed, 0, 0, 0)
        image = cap.read()[1]
        (markers, ids, fake) = aruco.detectMarkers(image, dic, parameters=par)
        if ids is None:
            ids = [-2]
        if len(markers) > 0:
            ids = ids.flatten()
        for (sides, i) in zip(markers, ids):
            if i == a or a < 0:
                markers = sides.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = markers
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                topRight = (int(topRight[0]), int(topRight[1]))
                side = math.sqrt((topLeft[0] - topRight[0]) ** 2 + (topLeft[1] - topRight[1]) ** 2)
                if side > 45:
                    drone.set_manual_speed_body_fixed(0, -1, 0, 0)
                    time.sleep(x)
                    drone.set_manual_speed_body_fixed(0, 0, 0, 0)
                    time.sleep(0.1)
                    return
        cv2.imshow('a', image)
        out.write(image)
        cv2.waitKey(1)


if __name__ == "__main__": # test script, drone flies triangle of three markers 2 times
    keyboard.add_hotkey('Ctrl', lambda: drone.land())  # adding hotkeys so we could land or disarm drone in case of emergency
    keyboard.add_hotkey('Ctrl+d', lambda: drone.disarm())
    drone.arm()
    drone.takeoff()
    time.sleep(3)
    drone.go_to_local_point(0, 0, 1, 0)  # drone goes to 1 meter height
    while not drone.point_reached():
        pass
    for i in range(2):
        f(34)  # drone goes forward to marker 34
        centre_oao(50, 34)  # drone turns in direction marker is turned and goes to the centre of marker
        f(41)  # same for marker 41
        centre_oao(50, 41)
        f(21)  # same for marker 21
        centre_oao(50, 21)
    time.sleep(3)  # waiting to make it look cool
    drone.land()
