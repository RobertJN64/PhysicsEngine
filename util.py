def get_prev_point(l, n):
    if n > 0:
        return l[n - 1]
    else:
        return l[n + 1] #Previous point is the same as next point at peak

def get_next_point(l, n):
    if n < len(l) - 1:
        return l[n + 1]
    else:
        return l[n - 1] #Previous point is the same as next point at peak

class Vector:
    def __init__(self, mag, angle):
        """
        Angle in rad
        """
        self.mag = mag
        self.angle = angle
