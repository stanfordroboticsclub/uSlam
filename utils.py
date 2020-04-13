import numpy as np
from sklearn.neighbors import NearestNeighbors

class Transform:
    def __init__(self, matrix):
        self.matrix = matrix

    @classmethod
    def fromOdometry(cls, angle, xy):
        matrix = np.eye(3)
        matrix[0,0] = np.cos(angle); matrix[0,1] =-np.sin(angle)
        matrix[1,0] = np.sin(angle); matrix[1,1] = np.cos(angle)
        matrix[:2,2] = xy

        return cls(matrix)

    @classmethod
    def fromComponents(cls, angle, xy = None):
        if xy == None:
            xy = np.zeros((2))
        else:
            xy = np.array(xy)
        return cls.fromOdometry(np.radians(angle), xy)

    def combine(self, other):
        return Transform(self.matrix @ other.matrix)

    def inv(self):
        R = self.matrix[:2, :2]
        matrix = np.eye(3)
        matrix[:2,:2] = np.linalg.inv(R)
        matrix[:2,2]  = np.linalg.inv(R) @ self.matrix[:2, 2]
        return Transform(matrix)

    def get_components(self):
        x,y = self.matrix[:2,:2] @ np.array([1,0])
        angle = np.arctan2(y,x)
        return (angle, self.matrix[:2, 2])

    def copy(self):
        return Transform(self.matrix)


class Robot:
    def __init__(self, xy = (0,0), angle = 0):
        self.tranform = Transform.fromComponents(angle, xy)

    def drive(self, tranform):
        #local move
        self.tranform = self.tranform.combine(tranform)

    def move(self, tranform):
        #global move
        self.tranform = tranform.combine(self.tranform)

    def get_transform(self):
        return self.tranform

    def get_pose(self):
        pos = np.array([0,0,1])
        head = np.array([0,1,1])

        pos  = self.tranform.matrix @ pos
        head = self.tranform.matrix @ head - pos
        return (pos[:2], head[:2])

    def copy(self):
        return Robot(self.tranform.copy())

    def replace(self, other):
        self.transform = other.tranform

class PointCloud:
    def __init__(self, array):
        self.points = array

    def copy(self):
        return PointCloud(self.points.copy())

    def replace(self, other):
        self.points = other.points

    @classmethod
    def fromScan(cls, scan):
        # from y axis clockwise
        scan = np.array(scan)
        angles = np.radians(scan[:,1])
        dists = scan[:,2]
        array = np.stack([dists*np.sin(angles), dists*np.cos(angles), np.ones(angles.shape)], axis=-1)
        return cls( array )

    def move(self, tranform):
        # print("matrix", tranform.matrix.shape)
        # print("self", self.points.shape)
        return PointCloud( (tranform.matrix @ self.points.T).T )

    def extend(self, other):
        MIN_DIST = 100

        nbrs = NearestNeighbors(n_neighbors=2).fit(self.points)

        # only middle (high resolution) points are valid to add
        print("other", other.points.shape)
        ranges = (other.points - np.mean(other.points, axis=0))[:, :2]
        ranges = np.sum(ranges**2, axis=-1)**0.5
        # print(ranges)
        points = other.points[ ranges < 2500, :]

        if points.shape[0] == 0:
            return

        distances, indices = nbrs.kneighbors(points)
        
        # print("distances", distances.shape)
        distances = np.mean(distances, axis=-1)
        matched_other = points[distances > MIN_DIST, :]

        self.points = np.vstack( (self.points, matched_other) )

    def fitICP(self, other):
        # TODO: better way of terminating
        transform = Transform.fromComponents(0)
        for itereation in range(15):

            aligment = self.AlignSVD(other)
            if aligment is None:
                return None, transform

            angle, xy = aligment.get_components()
            dist = np.sum(xy**2)**0.5

            if( np.abs(angle) > 0.3 or dist > 300 ):
                print("sketchy", itereation, angle, dist)
                return None, transform

            transform = aligment.combine(transform)
            other = other.move(aligment)

            if( angle < 0.001 and dist < 1 ):

                print("done", itereation)
                angle, xy = transform.get_components()
                dist = np.sum(xy**2)**0.5
                print("angle", angle, "Xy", xy)

                if( np.abs(angle) > 0.3 or dist > 300):
                    print("sketchy", itereation, angle)
                    return None, Transform(np.eye(3))
                return other, transform
        else:
            print("convergence failure!")
            return None, transform


    def AlignSVD(self, other):
        # other is the one moving
        MAX_DIST = 300

        # keep around
        nbrs = NearestNeighbors(n_neighbors=1).fit(self.points)
        distances, indices = nbrs.kneighbors(other.points)

        distances = np.squeeze(distances)
        indices = np.squeeze(indices)

        matched_indes = indices[distances <= MAX_DIST]
        matched_other = other.points[distances <= MAX_DIST, :]
        matched_self  = self.points[matched_indes, :]

        if matched_self.shape[0] < 10:
            print("not enough matches")
            return None

        self_mean = np.mean(matched_self, axis=0)
        other_mean = np.mean(matched_other, axis=0)

        matched_self = matched_self- self_mean
        matched_other = matched_other - other_mean

        M = np.dot(matched_other.T,matched_self)
        U,W,V = np.linalg.svd(M)

        R = np.dot(V.T,U.T)

        #consequence of homogeneous coordinates
        assert R[0,2] == 0
        assert R[1,2] == 0
        assert R[2,2] == 1
        assert R[2,0] == 0
        assert R[2,1] == 0
        
        t = self_mean - other_mean
        R[:2,2] = t[:2]
        
        return Transform(R)

