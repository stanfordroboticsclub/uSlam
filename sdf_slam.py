#!/usr/bin/env python3

import math
import time
import tkinter as tk

import numpy as np

import msgpack
import UDPComms


MM_PER_INCH =2.54

WINDOW_SIDE = 1000
MM_PER_PIX = 4

RESOLUTION_MM = 400
MAP_SIZE_M = 4
RATE = 100


class Line:
    def __init__(self, a, b, c):
        # ax + by + c = 0
        self.a = a
        self.b = b
        self.c = c

    def get_y(self,x):
        return (- self.a * x - self.c)/self.b

    @classmethod
    def from_mc(cls, m, c):
        # y = mx + c
        return cls(m, -1, c)

    def get_distance(self,point):
        # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
        x, y = point
        return math.fabs(self.a * x + self.b * y + self.c) \
               /math.sqrt(self.a**2 + self.b**2)

    @classmethod
    def from_fit(cls, points):
        A = np.array(points)
        y = A[:,1].copy()
        A[:,1] = 1

        # mx + c = y
        m,c = np.linalg.pinv(A) @ y
        return cls.from_mc(m,c)

    @classmethod
    def from_points(cls, p1, p2):
        x1, y1 = p1
        x2, y2 = p2

        a = y2 - y1
        b = x1 - x2

        c = x2 * y1 - y2 * x1
        return cls(a,b,c)

class SDFMap:


    def __init__(self):
        self.size_mm = MAP_SIZE_M * 1000
        self.resolution = RESOLUTION_MM

        self.size_g = int(self.size_mm/self.resolution)

        # TODO: figure out what defaut works
        self.map = [ [None]*self.size_g for _ in range(self.size_g)] 

        # self.map_pub = UDPComms.Publisher(8888)

        self.modes = [[ self.dw, self.dw ],
                        [ self.dw, self.up ],
                        [ self.up, self.dw ],
                        [ self.up, self.up ]]

    def real(self, idx):
        "coords from index"
        return idx * self.resolution - self.size_mm/2

    def up(self, coords):
        "ceiling of index from coord"
        return int((coords + self.size_mm/2)/self.resolution) + 1

    def dw(self, coords):
        "floor of index from coord"
        return int((coords + self.size_mm/2)/self.resolution)

    def fc(self, coords):
        "fractional part of index from coord"
        # out = (coords + self.size_mm/2) % self.resolution
        out = ((coords + self.size_mm/2)/self.resolution) - self.dw(coords)
        if 0<= out and out <= 1:
            return out
        else:
            raise IndexError

    def round(self, mode, key):
        "roudned coords from coords"
        # 32
        # 01
        if mode == 0:
            return (self.real(self.dw(key[0])), self.real(self.dw(key[1])) )
        elif mode == 1:
            return (self.real(self.up(key[0])), self.real(self.dw(key[1])) )
        elif mode == 2:
            return (self.real(self.up(key[0])), self.real(self.up(key[1])) )
        elif mode == 3:
            return (self.real(self.dw(key[0])), self.real(self.up(key[1])) )
        else:
            assert False

    def gen_pattern(self, point):
        pass

    def __getitem__(self, key):
        mode, x, y = key
        # 32
        # 01
        if mode == 0:
            return self.map[ self.dw(x) ][ self.dw(y) ]
        elif mode == 1:
            return self.map[ self.up(x) ][ self.dw(y) ]
        elif mode == 2:
            return self.map[ self.up(x) ][ self.up(y) ]
        elif mode == 3:
            return self.map[ self.dw(x) ][ self.up(y) ]
        else:
            assert False

    def __setitem__(self, key, value):
        mode, x, y = key
        # 32
        # 01
        if mode == 0:
            self.map[ self.dw(x) ][ self.dw(y) ] = value
        elif mode == 1:
            self.map[ self.up(x) ][ self.dw(y) ] = value
        elif mode == 2:
            self.map[ self.up(x) ][ self.up(y) ] = value
        elif mode == 3:
            self.map[ self.dw(x) ][ self.up(y) ] = value
        else:
            assert False

    def publish_map(self):
        pass

    def interpolate(self, x, y):
        bl = self.map[self.dw(x)][self.dw(y)]
        br = self.map[self.up(x)][self.dw(y)]

        tl = self.map[self.dw(x)][self.up(y)]
        tr = self.map[self.up(x)][self.up(y)]

        M =      self.fc(y)  * (self.fc(x)* tr + (1 - self.fc(x))* tl ) +  \
            (1 - self.fc(y)) * (self.fc(x)* br + (1 - self.fc(x))* bl )
        
        return M

    def interpolate_derivative(self, x, y):
        bl = self.map[self.dw(x)][self.dw(y)]
        br = self.map[self.up(x)][self.dw(y)]

        tl = self.map[self.dw(x)][self.up(y)]
        tr = self.map[self.up(x)][self.up(y)]

        # if np.sign(bl) == np.sign(br) == np.sign(tl) == np.sign(tr):
        # dx = self.fc(y)* (br - bl) + (1 - self.fc(y))* (tr - tl)  
        # dy = self.fc(x)* (tl - bl) + (1 - self.fc(x))* (tr - br)  

        dx = self.fc(y)* (tr - tl) + (1 - self.fc(y))* (br - bl)
        dy = self.fc(x)* (tr - br) + (1 - self.fc(x))* (tl - bl)

        return (dx/RESOLUTION_MM, dy/RESOLUTION_MM)

class Robot:
    def __init__(self):

        self.odom = UDPComms.Subscriber(8821, timeout = 2)

        # self.th measured CCW from X axis
        self.x = 0
        self.y = 0
        self.th = math.pi/2

        self.WHEEL_RAD = MM_PER_INCH * 6.5/2 #mm
        self.WHEEL_BASE = MM_PER_INCH * 18  #mm
        # self.wheel_l, self.wheel_r = self.odom.recv() #rotations

        self.lidar_offset_forward = MM_PER_INCH * 18

    def get_pose(self):
        return (self.x, self.y, self.th)

    def set_pose(self, x, y, th):
        self.x  = x
        self.y  = y
        self.th = th

    def lidar_to_map(self, angle, dist):
        return (self.x + dist * math.cos(angle + self.th) + self.lidar_offset_forward * math.cos(self.th), 
                self.y + dist * math.sin(angle + self.th) + self.lidar_offset_forward * math.sin(self.th))

    def update_odom(self):
        # import pdb; pdb.set_trace()
        linear, rot = self.odom.get()

        v_right   = 0
        v_forward = linear * RATE
        v_th      = rot * RATE/1000

        print(v_forward, v_th)

        delta_x = (v_forward * math.cos(self.th + v_th/2) ) # - v_right * math.sin(th)) # needed fro omni robots
        delta_y = (v_forward * math.sin(self.th + v_th/2) ) # + v_right * math.cos(th))

        self.x += delta_x;
        self.y += delta_y;
        self.th += v_th;

        print(self.x, self.y, self.th)

class SLAM:
    def __init__(self, window):
        self.lidar = UDPComms.Subscriber(8110, 0.2)
        self.robot = Robot()
        self.sdf = SDFMap()

        self.window = window

        self.mapped = 0

    def update(self):

        print("HERE")
        self.robot.update_odom()


        # orig = self.robot.get_pose()

        try:
            scan = self.lidar.get()
        except UDPComms.timeout:
            return

        if self.mapped < 5:
            self.update_sdf(scan)
            self.mapped += 1
            return

        for _ in range(10):
            delta_pose = self.scan_match(scan)
            # sum_pose += delta_pose

            
            pose = self.robot.get_pose()
            self.robot.set_pose(pose[0] + delta_pose[0],
                               pose[1] + delta_pose[1],
                               pose[2] + delta_pose[2])
        self.update_sdf(scan)

        # self.robot.set_pose(*orig)


    def get_map(self):
        return self.sdf.map

    def get_pose(self):
        return self.robot.get_pose()

    def get_lidar(self):
        # TODO: change to stored scan?
        for _, angle, dist in self.lidar.get():
            a = math.radians(angle)
            yield self.robot.lidar_to_map(a, dist)


    def update_sdf(self, scan):
        """ first janky update rule """

        points = {}
        for x,y in self.get_lidar():
            ind_x = self.sdf.dw(x)
            ind_y = self.sdf.dw(y)
            points[ (ind_x,ind_y) ] = points.get( (ind_x,ind_y) , []) + [(x,y)]


        for key, values in points.items():
            if len(values) < 2:
                continue

            line = Line.from_fit(values)


            point = values[0]

            # key is in index space !1
            # it should be in coord space!
            for mode in range(4):
                corner = self.sdf.round(mode, point)

                s = 1
                if np.sign( line.get_y(corner[0]) - corner[1] ) == \
                   np.sign( line.get_y(self.robot.get_pose()[0]) - self.robot.get_pose()[0] ):
                    s = -1

                if mode == 0:
                    self.window.canvas.create_line(*self.window.to_canvas( \
                corner[0], line.get_y(corner[0])), \
                                                  *self.window.to_canvas( \
                corner[0] + self.sdf.resolution , line.get_y(corner[0] + self.sdf.resolution)) )

                try:
                    dist = line.get_distance( corner )
                    # if math.fabs( dist ) < math.fabs(self.sdf[mode,point[0],point[1]]) or \
                    #                     self.sdf[mode,point[0],point[1]] == 0:
                    self.sdf[mode,point[0],point[1]] = s * dist


                except IndexError:
                    pass


    def scan_match(self, scan):
        Map_derivate = np.zeros((3))
        current_M = 0
        for _,angle,dist in scan:
            a = math.radians(angle)
            th = a + self.robot.th

            # d (x,y)/ d(rob_x, rob_y, rob_th)
            dPointdPose = np.array( [[1, 0, -dist* math.sin(th)], 
                                     [0, 1, dist* math.cos(th)]] )

            # (x, y)
            point = self.robot.lidar_to_map(a,dist)

            # if self.sdf.dw(point[0]) != 4 or \
            #    self.sdf.dw(point[1]) != 4:
            #     continue

            try:
                # current_M
                M = self.sdf.interpolate(*point)
            except IndexError:
                continue
            except TypeError:
                continue
            else:
                current_M += M**2

                # dM/ d(x,y)
                dMdPoint = 2 * M * np.array(self.sdf.interpolate_derivative(*point))
                # not handling sqare!!! ?
                # (x,y,th) = [2, _] @ [2 ,3]
                Map_derivate += dMdPoint @ dPointdPose

        # aprox = current_M + Map_derivate * dPose = 0 
        #  dPose = Map_derivate ^-1 @  (-current_M)

        # regualrised newton-gauss
        A = np.block( [ [Map_derivate], [ 10000 * np.diag([1,1,1000] )] ] )
        b = np.block( [ [-current_M], [np.zeros((3, 1))] ] )

        dPose = np.linalg.pinv(A) @ (b)

        # gradient desent
        # learning_rate = np.array([ -0.001, -0.001, -0.00000001])
        # print("current M", current_M)
        # print("map derivative", learning_rate * Map_derivate)
        # return  learning_rate* Map_derivate

        # straight netwton-gauss
        # dPose = np.linalg.pinv( Map_derivate[np.newaxis, :] ) * (-current_M)

        print("current M", current_M)
        print("map derivative", Map_derivate)
        print("delta pose = \n", dPose)
        return dPose[:,0]

class LidarWindow:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root,width=WINDOW_SIDE,height=WINDOW_SIDE)
        self.canvas.pack()

        self.canvas.bind("<Motion>", self.on_click)

        self.slam = SLAM(self)

        self.arrow = self.canvas.create_line(0, 0, 1, 1, arrow=tk.LAST)

        self.root.after(3000,self.update)
        self.root.mainloop()

    def create_point(self,x,y):
        return self.canvas.create_oval(x, y, x, y, width = 1, fill = '#000000')

    def create_pose(self,x,y, th):
        return self.canvas.create_line(x, y, x + 10*math.sin(th),
                                        y - 10*math.cos(th), arrow=tk.LAST)

    def create_map(self, sdf):
        pass

    def to_canvas(self,x, y):
        y_new= WINDOW_SIDE/2 - x / MM_PER_PIX
        x_new= WINDOW_SIDE/2 + y / MM_PER_PIX
        return (x_new,y_new)

    def from_canvas(self,x_new, y_new):
        x = MM_PER_PIX * (WINDOW_SIDE/2  - y_new)
        y = MM_PER_PIX * ( x_new - WINDOW_SIDE/2)
        return (x,y)

    def on_click(self,event):
        point = self.from_canvas(event.x, event.y)
        print(point, self.slam.sdf.interpolate(*point),
              self.slam.sdf.interpolate_derivative(*point))

    def update(self):
        update_start = time.time()
        try:
            self.canvas.delete('all')
            self.slam.update()

            for x in range(self.slam.sdf.size_g):
                for y in range(self.slam.sdf.size_g):
                    px,py = self.to_canvas( self.slam.sdf.real(x), self.slam.sdf.real(y) )

                    value = self.slam.sdf.map[x][y]
                    if value == None:
                        value = 0

                    if value == 0:
                        color = (0, 0, 255)
                    elif value > 0:
                        color  = ( int(255*value)//1000, 0,0 )
                    else:
                        color = ( 0,255, -int(255*value)//1000  )

                    self.canvas.create_rectangle(px, py, px + 10, py + 10, fill = \
                                                 "#%02x%02x%02x" % color    )
            self.canvas.delete(self.arrow)
            x, y, th = self.slam.get_pose()
            # print(self.slam.get_pose())
            self.arrow = self.create_pose(*self.to_canvas(x, y), th )
            
            for x,y in self.slam.get_lidar():
                self.create_point( *self.to_canvas(x, y ) )

        finally:
            self.root.after(RATE,self.update)
            # print("update time", (time.time() - update_start ) *1000 )

if __name__ == "__main__":
    window = LidarWindow()



