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


def create_point(x,y, c = '#000000', w= 1):
    canvas.create_oval(x, y, x, y, width = w, fill = c)

def update():
    try:
        data = sub.get()

        canvas.delete('all')
        canvas.create_oval(255, 255, 245, 245, fill = '#FF0000')
        for _, angle, dist in data:
            # print(angle,dist)
            a = math.radians(angle)
            create_point(250 + math.sin(a) * dist/50, 250 - math.cos(a) * dist/50)
        land = find_landmarks(data)
        for x,y in land:
            create_point(x,y,c="#00ff00", w =5)
            print(x,y)
        print()
    finally:
        r.after(100,update)

def find_landmarks(scan):
        xs = []
        ys = []

        points = np.zeros((len(scan),2))

        for i,(_, angle, dist) in enumerate(scan):
            a = math.radians(angle)
            x = 250 + math.sin(a) * dist/50
            y = 250 - math.cos(a) * dist/50

            # xs.append(x)
            # ys.append(y)
            points[i,0] = x
            points[i,1] = y


        out = []
        for p1, p2,p3 in zip(np.roll(points,-1, axis=0),
                             np.roll(points, 0, axis=0),
                             np.roll(points, 1, axis=0)):

            a = - grad(p1,p2) * grad(p2,p3)

            if ( a > 0.5 and a<1.5):
                # print(a)
                out.append( (p2[0], p2[1]) )

        return out




def grad(p1, p2):
    dx ,dy = p1 - p2
    return dy/dx


r.after(100,update)
r.mainloop()


