import sys
import time
import json
import paho.mqtt.client as mqtt
from uuid import getnode as get_mac

topic = 'imu'
host = "192.168.0.101"
port = 1883

def on_connect(mqttc, obj, flags, rc):
    print("Connected: " + str(rc))

def on_publish(mqttc, obj, mid):
    print("Published: " + str(mid))

mqttc = mqtt.Client()
mqttc.on_connect = on_connect
mqttc.on_publish = on_publish
mqttc.connect(host, port, 60)

mqttc.loop_start()

topic = 'imu/{}'.format(get_mac())
print("Publishing to '{}'".format(topic))
time.sleep(0.5)

while True:
    data = sys.stdin.readline().strip().split()
    # print(data)
    data_dict = {
        'time': '{:.2f}'.format(time.time()),
        'euler_angles': data[:3],
        'acc': data[3:6],
        'mag': data[6:9]
    }

    json_str = json.dumps(data_dict)
    print(data[0])
    infot = mqttc.publish(topic, json_str)
    infot.wait_for_publish()
    # time.sleep(0.5)

'''
raw (no longer needed):
`minimu9-ahrs --mode raw`
m[0], m[1], m[2], a[0], a[1], a[2], g[0], g[1], g[2]

g[2] is most useful
'''
