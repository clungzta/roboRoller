#!/usr/bin/env python
import sys
import ssl
import json
import time
import math
import collections
import ThunderBorg
import numpy as np
from pprint import pprint
import paho.mqtt.client as mqtt
from uuid import getnode as get_mac
from diff_drive import RobotDiffDriveController

# Setup MQTT
start = time.time()
host = "192.168.0.101"
port = 1883

POSE_UNCERTAINTY_THRESH = 10
DRIVE = True

# This will be updated when readings come in from the IMU and Pozyx
imu_pozyx_heading_offset = 0.0

robot_controller_settings = {
    'kP_lin' : 1.0,
    'kP_ang': 0.006,
    'max_lin_vel' : 0.7,
    'max_ang_vel' : 0.15,
    'max_driver_power': 0.65,
    'min_driver_power': 0.3,
    'reached_target_threshold': 1.2,
    'verbose' : False
}

robot_controller = RobotDiffDriveController(**robot_controller_settings)

# Circular Buffer of the last 500 pozyx and imu readings respectively
pozyx_readings = collections.deque(maxlen=500)
pozyx_readings_target = collections.deque(maxlen=500)
imu_readings = collections.deque(maxlen=500)

def on_connect(client, userdata, flags, rc):
    print(mqtt.connack_string(rc))

def on_message(client, userdata, msg):
    if msg.topic == 'tags':
        tag_data = json.loads(msg.payload.decode())
        if tag_data[0]['tagId'] == '1':
            pozyx_readings.append(tag_data[0])
        elif tag_data[0]['tagId'] == '2':
            pozyx_readings_target.append(tag_data[0])

    elif 'imu' in msg.topic:
        imu_data = json.loads(msg.payload.decode())
        imu_readings.append(imu_data)

def on_subscribe(client, userdata, mid, granted_qos):
    print("Subscribed to topic!")
    print(userdata)

def constrainAngle(x):
    x = (x + 180) % 360
    if (x < 0):
        x += 360
    return x - 180

imu_topic = 'imu/{}'.format(get_mac())

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_subscribe = on_subscribe
client.connect(host, port=port)
client.subscribe('tags')
client.subscribe(imu_topic)

# works blocking, other, non-blocking, clients are available too.
client.loop_start()

# Setup the ThunderBorg
TB = ThunderBorg.ThunderBorg()     # Create a new ThunderBorg object
#TB.i2cAddress = 0x15              # Uncomment and change the value if you have changed the board address
TB.Init()                          # Set the board up (checks the board is connected)
if not TB.foundChip:
    boards = ThunderBorg.ScanForThunderBorg()
    if len(boards) == 0:
        print('No ThunderBorg found, check you are attached :)')
    else:
        print('No ThunderBorg at address %02X, but we did find boards:' % (TB.i2cAddress))
        for board in boards:
            print('    %02X (%d)' % (board, board))
        print('If you need to change the I2C address change the setup line so it is correct, e.g.')
        print('TB.i2cAddress = 0x%02X' % (boards[0]))
    sys.exit()

pozyx_heading_ewma_alpha = 0.7
# filtered_heading = None
filtered_heading = -90.0

# TB.SetMotor1(1.0)
# TB.SetMotor2(0.0)
# time.sleep(1)
# exit()

print('Press CTRL+C to finish')
try:
    while True:
        try:
            latest_pozyx_reading_target = pozyx_readings_target[-1]
            latest_pozyx_reading = pozyx_readings[-1]
            prev_pozyx_reading = pozyx_readings[-2]
        except IndexError:
            print('Waiting for two pozyx positions to arrive before continuing...')
            time.sleep(0.05)
            continue

        if not 'coordinates' in latest_pozyx_reading['data']:
            print('No coordinates in latest Pozyx reading')
            TB.SetMotor1(0.0)
            TB.SetMotor2(0.0)
            continue

        if not 'coordinates' in latest_pozyx_reading_target['data']:
            print('No coordinates in latest Pozyx target reading')
            TB.SetMotor1(0.0)
            TB.SetMotor2(0.0)
            continue

        if 'coordinates' in pozyx_readings[-1]['data'] and 'coordinates' in pozyx_readings[-2]['data']:
            prev_coords = pozyx_readings[-2]['data']['coordinates']
            coords = pozyx_readings[-1]['data']['coordinates']

            measured_dx = (prev_coords['x'] - coords['x']) / 1000.0
            measured_dy = (prev_coords['y'] - coords['y']) / 1000.0
            measured_d = math.sqrt(math.pow(measured_dx, 2) + math.pow(measured_dy, 2))
            dt = pozyx_readings[-1]['timestamp'] - pozyx_readings[-2]['timestamp']
            pozyx_heading = math.atan2(measured_dy, measured_dx) * (180.0 / math.pi)
            robot_velocity = measured_d / dt
            x, y, z = imu_readings[-1]

            # Convert imu heading from IMU space to Pozyx space...
            imu_heading = -constrainAngle(z + 180) + imu_pozyx_heading_offset

            # Ensure that we are moving before trying to capture a heading...
            if robot_velocity > 0.025: # 0.075
                if filtered_heading is None:
                    filtered_heading = imu_heading
                    # filtered_heading = pozyx_heading
                else:
                    filtered_heading = pozyx_heading_ewma_alpha * pozyx_heading + (1.0 - pozyx_heading_ewma_alpha) * filtered_heading
                
                if robot_controller.verbose:
                    print('Pozyx heading: {:.4f}, robot_velocity: {:.4f}'.format(pozyx_heading, robot_velocity))


                # print()

        # pprint(latest_pozyx_reading)
        # uncertainty = latest_pozyx_reading['data']['metrics']['reliability']['uncertainty']
        # if uncertainty > POSE_UNCERTAINTY_THRESH:
        #     print('Pozyx Tag position is uncertain ({:.2f}), stopping the robot...'.format(uncertainty))
        #     TB.SetMotor1(0.0)
        #     TB.SetMotor2(0.0)
        #     continue
        
        # print(uncertainty)

        coords_target = latest_pozyx_reading_target['data']['coordinates']
        pose_target = (coords_target['x'] / 1000.0, coords_target['y'] / 1000.0)

        # Update robot_controller Position with the latest reading from Pozyx
        coords = latest_pozyx_reading['data']['coordinates']
        pose = (coords['x'] / 1000.0, coords['y'] / 1000.0)
        robot_controller.position_update(pose[0], pose[1])
        
        robot_controller.heading_update(filtered_heading)
        
        # target_x and target_y are the coordinates that we want the robot to drive to
        target_x = pose_target[0]
        target_y = pose_target[1]

        try:
            left_speed, right_speed = robot_controller.get_wheel_velocities(target_x, target_y)
            
            if DRIVE:
                # TB wired backwards, i.e. motor1 is right, motor2 is left...
                TB.SetMotor1(right_speed)
                TB.SetMotor2(left_speed)
        
        except Exception as e:
            print(e)

        if robot_controller.verbose:
            print('Motor speeds - Left: {:.4f}, Right: {:.4f}'.format(left_speed, right_speed))
        time.sleep(0.001)                   # Wait between steps

except KeyboardInterrupt:
    # User has pressed CTRL+C
    TB.MotorsOff()                 # Turn both motors off
    print('Done')
