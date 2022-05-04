from geometry_msgs.msg import Pose


class Position(Pose):
    EQ_THRESHOLD = 0.000001

    def __init__(self, x=0, y=0, z=0):
        super(Position, self).__init__()
        self.position.x = x
        self.position.y = y
        self.position.z = z

        self.orientation.x = 0
        self.orientation.y = 0
        self.orientation.z = 0
        self.orientation.w = 0

    def __str__(self):
        return str([self.position.x, self.position.y])

    def __eq__(self, other):
        return self.dist_to(other) < Position.EQ_THRESHOLD

    def __sub__(self, other):
        new = Position()
        new.position.x = self.position.x - other.position.x
        new.position.y = self.position.y - other.position.y
        return new

    def __truediv__(self, scalar: float):
        """Position(x1/x2, y1/y2)"""
        new = Position()
        new.position.x = self.position.x / scalar
        new.position.y = self.position.y / scalar
        return new

    def __mul__(self, other):
        new = Position()
        new.position.x = self.position.x * other.position.x
        new.position.y = self.position.y * other.position.y
        return new

    def __add__(self, other):
        new = Position()
        new.position.x = self.position.x + other.position.x
        new.position.y = self.position.y + other.position.y
        return new

    def dist_to(self, other):
        return ((self.position.x - other.position.x) ** 2 + (self.position.y - other.position.y) ** 2) ** 0.5

    def to_pose(self):
        pose = Pose()
        pose.position.x = self.position.x
        pose.position.y = self.position.y
        pose.position.z = self.position.z

        pose.orientation.x = self.orientation.x
        pose.orientation.y = self.orientation.y
        pose.orientation.z = self.orientation.z
        pose.orientation.w = self.orientation.w

        return pose
