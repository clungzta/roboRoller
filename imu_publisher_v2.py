import sys
import time
import json
import math
import numpy as np
from pprint import pprint
import paho.mqtt.client as mqtt
from uuid import getnode as get_mac

topic = 'imu'
host = "192.168.0.101"
port = 1883

def on_connect(mqttc, obj, flags, rc):
    print("Connected: " + str(rc))

def on_publish(mqttc, obj, mid):
    print("Published: " + str(mid))

def isRotationMatrix(R) :
    '''
    Checks if a matrix is a valid rotation matrix.
    '''
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    print(n)
    return n < 1e-4

def rotationMatrixToEulerAngles(R):
    # assert(isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.connect(host, port, 60)

mqttc.loop_start()

topic = 'imu/{}'.format(get_mac())
print("Publishing to '{}'".format(topic))
time.sleep(0.5)

while True:
    data = np.asarray(sys.stdin.readline().strip().split(), dtype=np.float64)
    R = data[:9].reshape(3,3)
    x, y, z = rotationMatrixToEulerAngles(R) * 180 / np.pi
    print(z)

    json_str = json.dumps(tuple([x, y, z]))
    infot = mqttc.publish(topic, json_str)
    infot.wait_for_publish()
    # time.sleep(0.5)

'''
raw (no longer needed):
`minimu9-ahrs --mode raw`
m[0], m[1], m[2], a[0], a[1], a[2], g[0], g[1], g[2]

g[2] is most useful
'''
