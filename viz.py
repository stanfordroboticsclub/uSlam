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

        img = np.zeros((500,500), dtype=np.uint8)

        for _, angle, dist in scan:
            a = math.radians(angle)
            x = int(250 + math.sin(a) * dist/10)
            y = int(250 - math.cos(a) * dist/10)
            img[x,y] = 255


        blur = 255 - img #cv2.GaussianBlur(img,(3,3),0)

        lines = cv2.HoughLinesP(blur,1,np.pi/180,100,20,10)
        print("lines",lines)

        if lines is None:
            # cv2.imshow("frame",blur)
            return 

        for x1,y1,x2,y2 in lines[0]:
            blur = cv2.line(blur,(x1,y1),(x2,y2),(0),4)

        cv2.imshow("frame",blur)


r.after(100,update)
r.mainloop()


