import RPi.GPIO as gpio
import time
import serial
import adafruit_gps
import math
from pathlib import Path


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


def checkValidity(coord1, coord2):
    for i in coord1:
        if i is None:
            return False

    for i in coord2:
        if i is None:
            return False

    return True


def main():
    # Set up file logging
    base_path = Path() / 'log'
    if not base_path.is_dir():
        base_path.mkdir()
    log_path = base_path / '{}.log'.format(time.strftime('%m-%d_%H:%M'))
    if not log_path.is_file():
        log_path.touch()
    log = log_path.open('a')

    # Set up RPi board for signaling
    bottom_pin = 22
    middle_pin = 18
    top_pin = 16
    gpio.setmode(gpio.BOARD)
    gpio.setup(bottom_pin, gpio.OUT)
    gpio.setup(middle_pin, gpio.OUT)
    gpio.setup(top_pin, gpio.OUT)

    # PWM stuff
    bottom = gpio.PWM(bottom_pin, 1)
    middle = gpio.PWM(middle_pin, 1)
    top = gpio.PWM(top_pin, 1)

    # Establish GPS connection
    uart = serial.Serial('/dev/ttyS0', timeout=3000)
    gps = adafruit_gps.GPS(uart)

    # Turn on GPS functionality
    gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
    gps.send_command(b'PMTK220,1000')

    prev_t = time.time()
    prev_coords = None
    total_distance = 0
    start_time = time.time()
    track_distance = 9600
    max_time = 24 * 60
    v_margin = 0.5

    while True:
        gps.update()
        try:
            current_t = time.time()

            # If 1 second has passed, read data from gps
            if current_t - prev_t >= 1:
                prev_t = current_t
                # Make sure we have a fix
                if not gps.has_fix:
                    print('Waiting for fix...')
                    continue

                # Print data
                print('=' * 20)
                print('Latitude:  {}'.format(gps.latitude))
                print('Longitude: {}'.format(gps.longitude))
                print('Fix quality: {}'.format(gps.fix_quality))
                if gps.satellites is not None:
                    print('# of satellites: {}\n'.format(gps.satellites))

                # Calculate distance and speed
                if prev_coords is not None:
                    # We have previous coordinates. Calculate
                    current_coords = (gps.latitude, gps.longitude, current_t)

                    if not checkValidity(current_coords, prev_coords):
                        continue
                    distance = getDistance(current_coords, prev_coords)
                    total_distance += distance
                    velocity = getVelocity(current_coords, prev_coords)

                    # Get ideal speed
                    ideal_velocity = (
                        (track_distance - total_distance) /
                        (max_time - (current_t - start_time))
                    )

                    # Output GPIO
                    if velocity < ideal_velocity - v_margin:
                        # Go faster
                        top.start(20)
                        top.ChangeFrequency(1)
                        middle.stop()
                        bottom.stop()
                    if velocity > ideal_velocity + v_margin:
                        # Go slower
                        top.stop()
                        middle.stop()
                        bottom.start(20)
                        bottom.ChangeFrequency(1)
                    if (
                        velocity > ideal_velocity - v_margin and
                        velocity < ideal_velocity + v_margin
                    ):
                        # Keep steady
                        top.stop()
                        middle.start(20)
                        middle.ChangeFrequency(1)
                        bottom.stop()
                    print('Distance: {:.3f}m'.format(distance))
                    print('Velocity: {:.3f} m/s'.format(velocity))
                    print('Ideal Velocity: {:.3f} m/s'.format(ideal_velocity))
                else:
                    # We're just starting up. Assign initial coordinates
                    current_coords = (gps.latitude, gps.longitude, current_t)
                log.write(
                    '{},{},{}\n'.format(
                        current_coords[0],
                        current_coords[1],
                        current_coords[2]
                    )
                )
                prev_coords = current_coords
        except Exception as e:
            raise e
            continue


if __name__ == '__main__':
    try:
        main()
    finally:
        gpio.cleanup()
