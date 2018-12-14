import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys
import math


def getDistance(coord1, coord2):
    lat1 = math.radians(coord1[0])
    lat2 = math.radians(coord2[0])
    lon1 = math.radians(coord1[1])
    lon2 = math.radians(coord2[1])

    delta_lat = lat2 - lat1
    delta_lon = lon2 - lon1

    a = (math.pow(math.sin(delta_lat / 2), 2) +
         math.cos(lat1) * math.cos(lat2) *
         math.pow(math.sin(delta_lon / 2), 2))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    R = 6371 * (10 ** 3)
    return R * c


def getVelocity(coord1, coord2):
    distance = getDistance(coord1, coord2)
    delta_t = abs(coord1[2] - coord2[2])
    return distance / delta_t


class Coordinate():
    def __init__(self, lat, lon, time):
        self.lat = lat
        self.lon = lon
        self.time = time

    def __str__(self):
        return '{},{},{}'.format(self.lat, self.lon, self.time)


data_file = Path() / 'logs' / '10-25_20:00.log'
if not data_file.is_file:
    print('Error importing data')
    sys.exit()

data = np.genfromtxt(data_file, delimiter=',')

coord_list = []

for l in data:
    coord = (l[0], l[1], l[2])
    coord_list.append(coord)

velocity_list = []
distance_list = []

for x in range(1, len(coord_list)):
    coord1 = coord_list[x - 1]
    coord2 = coord_list[x]
    distance = getDistance(coord1, coord2)
    velocity = getVelocity(coord1, coord2)
    velocity_list.append(velocity)
    distance_list.append(distance)

np_velocity = np.array(velocity_list)
np_distance = np.array(distance_list)

time = coord_list[-1][2] - coord_list[0][2]
total_distance = np.sum(np_distance)

print('Time Elapsed: {:.3f} s'.format(time))
print('Distance covered: {:.3f} m'.format(total_distance))
print('Average speed: {:.3f} m/s'.format(total_distance / time))

plt.plot(np_velocity * 3.6)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/h)')
plt.title('GPS test drive velocity')

plt.figure()
plt.xlabel('Latitude')
plt.ylabel('Lonitude')
plt.title('GPS test drive coordinates')
plt.plot(data[:, 0], data[:, 1], linestyle='-', color='grey')
plt.plot(data[:, 0], data[:, 1], 'rx', markersize=3)

plt.show()
