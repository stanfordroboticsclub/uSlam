

class point_cloud:
    def get_nearest(self,points):
        # points is [N] array
        pass

        # return [N, 2] array
        return nearest_point, distance

    def add_points(self,points):
        # points is [N] array
        pass


    def remove_points(self,points):
        # points is [N] array
        pass


class KDTree(point_cloud):
    pass

class SLAM:

    def __init__(self):
        pass



    def update_odometry(self,odom):
        pass

    def shift_position(self, 

    def scan_match(self, scan):
        
        lidar_points = to_global_frame(scan)
        matching_points = pc.get_nearest(lidar_points)
        transform = svd_match(lidar_points, matching_points)
        self.shift_position



