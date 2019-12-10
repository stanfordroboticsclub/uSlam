#!/usr/bin/env python3


import tkinter as tk
import numpy as np

def _create_circle(self, x, y, r, **kwargs):
    return self.create_oval(x-r, y-r, x+r, y+r, **kwargs)
tk.Canvas.create_circle = _create_circle

class Robot:
    def __init__(self, x, y, canvas):
        self.canvas = canvas

        self.x = x
        self.y = y
        self.a = 0

        self.objects = []
        self.redraw()

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

    def get_scan(self, obstacles):
        for angle in np.linspace(0, 2*np.pi, 50):
            pass


class Simualtor:

    def __init__(self):
        self.master = tk.Tk()

        self.PIX_per_M = 50
        self.canvas = tk.Canvas(self.master, width=200, height=100)
        self.canvas.pack()

        self.robot = Robot(50,50, self.canvas)

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

    def scan(self):
        coords = [self.canvas.coords(obj) for obj in self.obstacles]
        self.robot.get_scan(coords)


if __name__ == "__main__":
    s = Simualtor()
