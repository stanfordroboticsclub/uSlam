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
        self.transform = Transform.fromComponents(angle, xy)

    def drive(self, transform):
        #local move
        self.transform = self.transform.combine(transform)

    def move(self, transform):
        #global move
        self.transform = transform.combine(self.transform)

    def get_transform(self):
        return self.transform

    def get_pose(self):
        pos = np.array([0,0,1])
        head = np.array([0,1,1])

        pos  = self.transform.matrix @ pos
        head = self.transform.matrix @ head - pos
        return (pos[:2], head[:2])

    def copy(self):
        return Robot(self.transform.copy())

    def replace(self, other):
        self.transform = other.transform.copy()

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
        for itereation in range(50):

            aligment = self.AlignSVD(other)
            if aligment is None:
                return None, transform

            angle, xy = aligment.get_components()
            dist = np.sum(xy**2)**0.5

            # if( np.abs(angle) > 0.4 or dist > 500 ):
            #     print("early sketchy", itereation, angle, dist)
            #     return None, transform

            transform = aligment.combine(transform)
            other = other.move(aligment)

            if( angle < 0.001 and dist < 1 ):

                # self.last_matched_distances

                print("done at itereation", itereation)
                angle, xy = transform.get_components()
                dist = np.sum(xy**2)**0.5
                print("angle", angle, "dist", dist)

                if( np.abs(angle) > 0.8 or dist > 500):
                    print("sketchy")
                    return None, Transform(np.eye(3))

                mean = np.mean(self.last_matched_distances)
                d = self.last_matched_distances - mean
                std = np.sqrt(np.mean(d**2))
                # if std != 0:
                skew = np.mean(d**3)/std**3
                # else:
                #     skew = -99998765

                print("skew", skew)
                if( skew < 1.5):
                    print("bad skew")
                    return None, Transform(np.eye(3))

                return other, transform
        else:
            print("convergence failure!")
            return None, transform


    def AlignSVD(self, other):
        # other is the one moving
        MAX_DIST = 500

        # keep around
        nbrs = NearestNeighbors(n_neighbors=1).fit(self.points)
        distances, indices = nbrs.kneighbors(other.points)

        distances = np.squeeze(distances)
        indices = np.squeeze(indices)

        matched_indes = indices[distances <= MAX_DIST]
        matched_other = other.points[distances <= MAX_DIST, :-1] # -1 removes homogeneous coord
        matched_self  = self.points[matched_indes,          :-1]

        if matched_self.shape[0] < 10:
            print("not enough matches")
            self.last_matched_distances = np.array([float("inf")])
            return None

        self_mean = np.mean(matched_self, axis=0)
        other_mean = np.mean(matched_other, axis=0)

        matched_self = matched_self- self_mean
        matched_other = matched_other - other_mean

        M = np.dot(matched_other.T,matched_self)
        U,W,V = np.linalg.svd(M)

        R = V.T @ U.T

        # is reflection is optimal fix it
        if np.linalg.det(R) < 0:
            R = V.T @ np.diag([1,-1]) @ U.T

        if not np.isclose(np.linalg.det(R), 1):
            print("determinant", np.linalg.det(R))
            raise ValueError
        
        t = self_mean - other_mean

        T = np.eye(3)
        T[:2,:2] = R
        T[:2,2] = t
        
        self.last_matched_distances = distances[distances <= MAX_DIST]
        return Transform(T)

