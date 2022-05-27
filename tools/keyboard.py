import time
import tkinter as tk
from UDPComms import Publisher


class CommandPannel:

    port = 8830

    def __init__(self):
        self.root = tk.Tk()

        self.fd=tk.Button(self.root,text='Forwards',command=self.ha)
        self.bk=tk.Button(self.root,text='Back',command=self.ha)
        self.rt=tk.Button(self.root,text='Right',command=self.ha)
        self.lt=tk.Button(self.root,text='Left',command=self.ha)

        self.fd.pack()
        self.bk.pack()
        self.rt.pack()
        self.lt.pack()

        self.root.bind('<Left>',  lambda x: self.leftKey(x))
        self.root.bind('<Right>', lambda x: self.rightKey(x))
        self.root.bind('<Up>',    lambda x: self.upKey(x))
        self.root.bind('<Down>',  lambda x: self.downKey(x))
        self.root.bind('<space>',  lambda x: self.spaceKey(x))

        self.speed = 10
        self.forwards = 0
        self.twist = 0
        self.time = time.time()

        self.pub = Publisher(self.port)

        self.root.after(100, self.publish)
        self.root.mainloop()

    @staticmethod
    def ha():
        print("ha")

    def spaceKey(self, event):
        print("space")
        self.forwards = 0
        self.twist = 0
        self.time = time.time()

    def leftKey(self, event):
        print("left")
        self.forwards = 0
        self.twist = self.speed
        self.time = time.time()
        
    def rightKey(self, event):
        print("right")
        self.forwards = 0
        self.twist = -self.speed
        self.time = time.time()

    def upKey(self, event):
        print("up")
        self.forwards = self.speed
        self.twist = 0
        self.time = time.time()

    def downKey(self, event):
        print("down")
        self.forwards = -self.speed
        self.twist = 0
        self.time = time.time()

    def publish(self):
        if (time.time() - self.time) > 1:
            self.forwards = 0
            self.twist = 0
        self.pub.send(self.forwards,self.twist)
        print("magic")
        self.root.after(100, self.publish)


if __name__ == "__main__":
    a = CommandPannel()
