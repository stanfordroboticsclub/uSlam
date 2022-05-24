

import tkinter as tk
import time
from collections import defaultdict

def schedule(func):
    def replacement(self, *args, **kwargs):
        self.after_idle(lambda : func(self, *args, **kwargs))
    return replacement

class Vizualizer(tk.Tk):
    def __init__(self, size = 1000, mm_per_pix = 15):
        super().__init__()
        self.SIZE = size
        self.MM_PER_PIX = mm_per_pix

        self.canvas = tk.Canvas(self,width=self.SIZE,height=self.SIZE)
        self.canvas.pack()

        self.tags = defaultdict(list)

    def clear(self):
        self.canvas.delete("all")
        
    def delete(self, tag):
        for obj in self.tags[tag]:
            self.canvas.delete(obj)

        del self.tags[tag]

    @schedule
    def plot_line(self, p1, p2, tag = None):
        if tag is not None:
            self.delete(tag)

        line = self.canvas.create_line(self.SIZE/2 + p1[0]/self.MM_PER_PIX,
                           self.SIZE/2 - p1[1]/self.MM_PER_PIX,
                           self.SIZE/2 + p2[0]/self.MM_PER_PIX,
                           self.SIZE/2 - p2[1]/self.MM_PER_PIX)

        if tag is not None:
            self.tags[tag].append(line)

    @schedule
    def plot_PointCloud(self, pc, c='systemTextColor', tag = None):
        # systemTextColor is white on a darkmode system and black otherwise
        if tag is not None:
            self.delete(tag)

        pc = pc.global_frame()

        objs = []
        for x, y,_ in pc.points:
            point = self.create_point(x, y, c=c)
            objs.append(point)

        if tag is not None:
            self.tags[tag].extend(objs)

    @schedule
    def plot_Robot(self, robot, c="#FF0000", tag = None):
        if tag is not None:
            self.delete(tag)

        pos, head = robot.get_pose()
        head *= 20

        arrow = self.canvas.create_line(self.SIZE/2 + pos[0]/self.MM_PER_PIX,
                           self.SIZE/2 - pos[1]/self.MM_PER_PIX,
                           self.SIZE/2 + pos[0]/self.MM_PER_PIX + head[0],
                           self.SIZE/2 - pos[1]/self.MM_PER_PIX - head[1],
                           arrow=tk.LAST)

        oval = self.canvas.create_oval(self.SIZE/2+5 + pos[0]/self.MM_PER_PIX, 
                                self.SIZE/2+5 - pos[1]/self.MM_PER_PIX,
                                self.SIZE/2-5 + pos[0]/self.MM_PER_PIX,
                                self.SIZE/2-5 - pos[1]/self.MM_PER_PIX,
                                fill = c)

        if tag is not None:
            self.tags[tag].extend([arrow, oval])

    @schedule
    def plot_Pose(self, pose, c="#FF0000", tag = None):
        if tag is not None:
            self.delete(tag)

        pos, head = pose.get_arrow()
        head *= 20

        arrow = self.canvas.create_line(self.SIZE/2 + pos[0]/self.MM_PER_PIX,
                           self.SIZE/2 - pos[1]/self.MM_PER_PIX,
                           self.SIZE/2 + pos[0]/self.MM_PER_PIX + head[0],
                           self.SIZE/2 - pos[1]/self.MM_PER_PIX - head[1],
                           arrow=tk.LAST)

        oval = self.canvas.create_oval(self.SIZE/2+5 + pos[0]/self.MM_PER_PIX, 
                                self.SIZE/2+5 - pos[1]/self.MM_PER_PIX,
                                self.SIZE/2-5 + pos[0]/self.MM_PER_PIX,
                                self.SIZE/2-5 - pos[1]/self.MM_PER_PIX,
                                fill = c)

        if tag is not None:
            self.tags[tag].extend([arrow, oval])

    def create_point(self,x,y, c = 'systemTextColor', w= 1):
        return self.canvas.create_oval(self.SIZE/2 + x/self.MM_PER_PIX,
                                self.SIZE/2 - y/self.MM_PER_PIX,
                                self.SIZE/2 + x/self.MM_PER_PIX,
                                self.SIZE/2 - y/self.MM_PER_PIX, width = w, fill = c, outline = c)


