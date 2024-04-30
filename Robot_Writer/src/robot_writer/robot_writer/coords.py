from math import sin, cos, pi, atan2

def abs2rel(x, y, angle, origin_x=0.0, origin_y=0.0, rotation=0.0):
    """!
    Returns x, y, angle in a system with origin in coordinates
    (origin_x, origin_y) rotated by rotation in radian.
    """
    max_rotation = 2 * pi
    new_qw = cos(((angle - rotation) % max_rotation)/2)
    new_qz = sin(((angle - rotation) % max_rotation)/2)
    new_x = x*cos(rotation)-y*sin(rotation)-(origin_x*cos(rotation)-origin_y*sin(rotation))
    new_y = x*sin(rotation)+y*cos(rotation)-(origin_x*sin(rotation)+origin_y*cos(rotation))
    if round(sin(rotation), 2) != 0.0:
        new_x = -new_x
        new_y = -new_y
    return round(new_x, 2), round(new_y, 2), round(new_qz,2), round(new_qw, 2)
