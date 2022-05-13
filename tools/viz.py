import msgpack
import UDPComms
import tkinter as tk
import math


sub = UDPComms.Subscriber(8110)

r = tk.Tk()
r.configure()

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
            print(angle,dist)
            a = math.radians(angle)
            create_point(250 + math.sin(a) * dist/10, 250 - math.cos(a) * dist/10)
        print()
    finally:
        r.after(100,update)


r.after(100,update)
r.mainloop()


