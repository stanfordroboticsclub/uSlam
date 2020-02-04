import msgpack
import UDPComms
import tkinter as tk
import math

import cv2
import numpy as np

sub = UDPComms.Subscriber(8110, timeout = 1)

r = tk.Tk()
canvas = tk.Canvas(r,width=500,height=500)
canvas.pack()


def create_point(x,y):
    canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

def update():
    try:
        data = sub.get()

        canvas.delete('all')
        canvas.create_oval(255, 255, 245, 245, fill = '#FF0000')
        for _, angle, dist in data:
            # print(angle,dist)
            a = math.radians(angle)
            create_point(250 + math.sin(a) * dist/50, 250 - math.cos(a) * dist/50)
        find_landmarks(data)
        print()
    finally:
        r.after(100,update)

def find_landmarks(scan):
        xs = []
        ys = []

        img = np.zeros((600,600), dtype=np.uint8)

        for _, angle, dist in scan:
            a = math.radians(angle)
            x = int(350 + math.sin(a) * dist/10)
            y = int(350 - math.cos(a) * dist/10)
            try:
                img[x,y] = 255
            except IndexError:
                pass


        # blur = 255 - img #cv2.GaussianBlur(img,(3,3),0)
        blur = 255 - 10*cv2.GaussianBlur(img,(7,7),0)
        blur[blur>255] = 255
        blur[blur<0] = 0

        # lines = cv2.HoughLinesP(blur,1,np.pi/180,100,100,100)
        lines = cv2.HoughLines(blur,1,np.pi/180,10)
        # print("lines",lines)

        if lines is None:
            cv2.imshow("frame",blur)
            return 

        print(len(lines[0]))
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            blur = cv2.line(blur,(x1,y1),(x2,y2),(0),4)

        cv2.imshow("frame",blur)


r.after(100,update)
r.mainloop()


