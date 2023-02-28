import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import time
from dronekit import connect
import threading

broker_address = 'localhost'
broker_port = 1883


def videoStream():
    global video
    cap = cv.VideoCapture(0)
    while video:
        start = time.time()
        # Read Frame
        _, frame = cap.read()
        # Encoding the Frame
        _, buffer = cv.imencode('.jpg', frame)
        # Converting into encoded bytes
        jpg_as_text = base64.b64encode(buffer)
        # Publishig the Frame on the Topic home/server
        client.publish('Video', jpg_as_text)
        end = time.time()
        t = end - start
        fps = int(1/t)
        client.publish('fps', fps)
        #print(fps)


def on_message(client, userdata, message):
    global takingPictures
    global video

    if message.topic == 'getHeading':
        print ('Get heading')
        connection_string = "udpin:0.0.0.0:14551"
        vehicle = connect(connection_string, wait_ready=True, baud=115200)
        client.publish('heading', str(vehicle.heading))
        vehicle.close()

    if message.topic == 'takePicture':
        print ('take picture') 
        cap = cv.VideoCapture(0) 
        ret, frame = cap.read() 
        _, buffer = cv.imencode('.jpg', frame) 
        # Converting into encoded bytes 
        jpg_as_text = base64.b64encode(buffer) 
        client.publish('picture', jpg_as_text)

    if message.topic == 'startVideo':
        print('Start video')
        video = True
        w = threading.Thread(target=videoStream)
        w.start()

    if message.topic == 'stopVideo':
        print('Stop video')
        video = False


client = mqtt.Client('On board controller')
client.on_message = on_message
client.connect(broker_address, broker_port)
print('waiting commands . . .')
client.subscribe('getHeading')
client.subscribe('takePicture') 
client.subscribe('startVideo')
client.subscribe('stopVideo') 
client.loop_forever()
