import math
from urllib.parse import urlparse
import numpy as np
import logging
from haversine import inverse_haversine, haversine, Direction, Unit

common_formatter = logging.Formatter('%(asctime)s.%(msecs)03d %(levelname)s %(name)s - %(message)s', datefmt="%H:%M:%S")


def dist_ned(pos1, pos2):
    return np.sqrt(np.sum((pos1 - pos2) ** 2, axis=0))


def dist_gps(lat1, long1, alt1, lat2, long2, alt2):
    dist_horiz = haversine((lat1, long1), (lat2, long2), unit=Unit.METERS)
    dist_alt = alt1 - alt2
    return math.sqrt(dist_horiz*dist_horiz + dist_alt*dist_alt)


def relative_gps(north, east, up, lat, long, alt):
    """ Given a NEU offset and a GPS position, calculate the GPS coordinates of the offset position."""
    target_alt = alt + up
    coords = (lat, long)
    coords = inverse_haversine(coords, north, Direction.NORTH, unit=Unit.METERS)
    target_lat, target_long = inverse_haversine(coords, east, Direction.EAST, unit=Unit.METERS)
    return target_lat, target_long, target_alt


def parse_address(string):
    """ Used to ensure that udp://:14540, udp://localhost:14540 and udp://127.0.0.1:14540 are recognized as equivalent.

    Missing elements from the string or the other entries are replaced with defaults. These are udp, empty host and
    50051 for the scheme, host and port, respectively.
    """
    scheme, rest = string.split("://")
    if scheme == "serial":
        loc, append = rest.split(":")
    else:
        parse_drone_addr = urlparse(string)
        scheme = parse_drone_addr.scheme
        loc = parse_drone_addr.hostname
        append = parse_drone_addr.port
        if scheme is None:
            scheme = "udp"
        if loc is None:
            loc = ""
        if loc == "localhost":
            loc = ""
        elif loc == "127.0.0.1":
            loc = ""
        if append is None:
            append = 50051
    return scheme, loc, append
