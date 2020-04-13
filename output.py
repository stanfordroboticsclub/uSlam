

import tkinter as tk
import time

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
        
    def delete(self, item):
        if hasattr(item, "tkiner_canvas_ids"):
            for obj in item.tkiner_canvas_ids:
                self.canvas.delete(obj)
        item.tkiner_canvas_ids = []

    @schedule
    def plot_PointCloud(self, pc, c='#000000'):
        self.delete(pc)

        for x, y,_ in pc.points:
            point = self.create_point(x, y, c=c)
            pc.tkiner_canvas_ids.append(point)

    @schedule
    def plot_Robot(self, robot, c="#FF0000"):
        self.delete(robot)

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

        robot.tkiner_canvas_ids = [oval, arrow]

    def create_point(self,x,y, c = '#000000', w= 1):
        return self.canvas.create_oval(self.SIZE/2 + x/self.MM_PER_PIX,
                                self.SIZE/2 - y/self.MM_PER_PIX,
                                self.SIZE/2 + x/self.MM_PER_PIX,
                                self.SIZE/2 - y/self.MM_PER_PIX, width = w, fill = c, outline = c)


