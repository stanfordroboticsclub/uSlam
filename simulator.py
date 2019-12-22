#!/usr/bin/env python3


import tkinter as tk
import numpy as np
from UDPComms import Publisher, Subscriber
from time import monotonic as time


def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

class Robot:
    def __init__(self, x, y, canvas):
        self.canvas = canvas

        self.x = x
        self.y = y
        self.a = 0

        self.prev_pos = [self.x,self.y,self.a]
        self.prev_time = time()

        self.objects = []
        self.redraw()

    def get_odom(self):
        dx = self.x - self.prev_pos[0]
        dy = self.y - self.prev_pos[1]
        da = self.a - self.prev_pos[2]

        dr = (dx**2 + dy**2)**0.5
        dt = time() - self.prev_time

        self.prev_pos = [self.x,self.y,self.a]
        self.prev_time = time()

        return (dr/dt , da/dt)

    def move(self, f, theta):
        self.x += f * np.sin(self.a)
        self.y += f * np.cos(self.a)
        self.a += theta
        self.redraw()

    def redraw(self):
        for obj in self.objects:
            self.canvas.delete(obj)

        self.objects.append(self.canvas.create_circle(self.x, self.y, 10))

        points = [self.x + 10*np.sin(self.a), self.y + 10*np.cos(self.a),
                  self.x +  5*np.cos(self.a), self.y -  5*np.sin(self.a),
                  self.x -  5*np.cos(self.a), self.y +  5*np.sin(self.a)]

        self.objects.append(self.canvas.create_polygon(points, fill='red'))

            


class Simualtor:

    def __init__(self):
        self.master = tk.Tk()

        self.PIX_per_M = 50
        self.canvas = tk.Canvas(self.master, width=400, height=400)
        self.canvas.pack()

        self.robot = Robot(200, 200, self.canvas)

        self.master.bind('<Left>',  lambda e: self.robot.move(0, 0.1) )
        self.master.bind('<Right>', lambda e: self.robot.move(0,-0.1) )
        self.master.bind('<Up>',    lambda e: self.robot.move( 1,-0) )
        self.master.bind('<Down>',  lambda e: self.robot.move(-1,-0) )

        self.canvas.bind('<Button-1>',        self.mouse_draw_down)
        self.canvas.bind('<B1-Motion>',       self.mouse_draw_move)
        self.canvas.bind('<ButtonRelease-1>', self.mouse_draw_up)
        self.canvas.bind('<Button-2>',        self.mouse_delete)

        self.obstacles = []
        self.mouse_mode = None

        self.scan_points = []
        self.lidar = Publisher(8110)
        self.odom = Publisher(8821) # publishes twist not wheel rotations

        self.master.after(1000, self.scan)
        self.master.after(100, self.odom_pub)
        tk.mainloop()

    def mouse_draw_down(self, event):
        self.mouse_mode = self.canvas.create_line(event.x, event.y, event.x, event.y)

    def mouse_draw_move(self, event):
        if self.mouse_mode != None:
            coords = self.canvas.coords(self.mouse_mode)
            self.canvas.coords(self.mouse_mode, coords[0], coords[1], event.x, event.y)

    def mouse_draw_up(self, event):
        if self.mouse_mode != None:
            coords = self.canvas.coords(self.mouse_mode)
            self.canvas.coords(self.mouse_mode, coords[0], coords[1], event.x, event.y)
            self.obstacles.append(self.mouse_mode)
        self.mouse_mode = None

    def mouse_delete(self, event):
        obj, = self.canvas.find_closest(event.x, event.y)
        if obj in self.obstacles:
            self.canvas.delete(obj)

    def odom_pub(self):
        out = self.robot.get_odom()
        self.odom.send( (out[0]/self.PIX_per_M, out[1]) )
        self.master.after(100, self.odom_pub)

    def scan(self):
        coords = [self.canvas.coords(obj) for obj in self.obstacles]
        output = []

        for s in self.scan_points:
            self.canvas.delete(s)

        start = 20
        end = 300

        step = 50

        for angle in np.linspace(0, 2*np.pi, step):
            s = np.sin(angle)
            c = np.cos(angle)

            x = self.robot.x - s*start
            y = self.robot.y + c*start

            for dist in range(start,end):
                x -= s
                y += c
                items = self.canvas.find_overlapping(x,y, x+2, y+2)
                if len(items) != 0:
                    output.append( (None, np.degrees(angle + self.robot.a), 1000*dist/self.PIX_per_M) )
                    self.scan_points.append(self.canvas.create_line(x, y, x+1, y, fill='red'))
                    break

        print(output)
        self.lidar.send(output)
        self.master.after(500, self.scan)


if __name__ == "__main__":
    s = Simualtor()
