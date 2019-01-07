from math import degrees, cos, sin, pi


def spherical_to_cartesian(spherical):
    '''Transform from spherical to cartesian coordinates'''
    try:
        dist = spherical.dist
    except AttributeError:
        try:
            dist = spherical.volume
        except AttributeError:
            return 0, 0
    x = dist * cos(spherical.angle)
    y = dist * sin(spherical.angle)
    return x, y


def add_spherical_to_cart(cartesian, spherical):
    '''
    Calculate the new position.
    Adds a spherical coordinate to a cartesian coordinate.
    '''
    x, y = spherical_to_cartesian(spherical)
    new_x = cartesian.x + x
    new_y = cartesian.y + y
    return new_x, new_y

