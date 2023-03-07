import base64 
import cv2 as cv 
import numpy as np 
from PIL import Image as Img 
from PIL import ImageTk 
import paho.mqtt.client as mqtt 
import tkinter as tk 
from tkinter.simpledialog import askstring 

def on_message(client, userdata, message): 

    if message.topic == 'heading':
        heading =str(message.payload.decode("utf-8"))
        headingLabel.heading = heading
        headingLabel.configure(text=heading)

    if message.topic == 'Video':
        #Decoding the message
        img = base64.b64decode(message.payload)
        # converting into numpy array from buffer
        npimg = np.frombuffer(img, dtype=np.uint8)
        # Decode to Original frame
        image_buffer = cv.imdecode(npimg, 1)
        cv2image = cv.cvtColor(image_buffer, cv.COLOR_BGR2RGBA) 
        img = Img.fromarray(cv2image) 
        imgtk = ImageTk.PhotoImage(image=img) 
        pictureLabel.imgtk = imgtk 
        pictureLabel.configure(image=imgtk)

    if message.topic == 'detection':
        detection = message.payload.decode('utf-8')
        detectionLabel.heading = detection
        detectionLabel.configure(text=detection)

    if message.topic == 'fps':
        fps = 'fps: ' + str(message.payload.decode("utf-8"))
        fpsLabel.heading = fps
        fpsLabel.configure(text=fps)

    if message.topic == 'protocol':
        protocol = 'Detection status: ' + str(message.payload.decode("utf-8"))
        protocolLabel.heading = protocol
        protocolLabel.configure(text=protocol)

    
def getHeading():
    global client
    client.publish('getHeading')


def startVideo():
    global client
    client.publish('startVideo')


def stopVideo():
    global client
    client.publish('stopVideo')


def getDetection():
    global client
    client.publish('getDetection')

def getConnection():
    global client
    client.publish('getConnection')
    

master = tk.Tk()

autoPilotFrame = tk.LabelFrame (text = "Autopilot control") 
autoPilotFrame.grid (row = 0, column = 0, padx = 5, pady = 5)

getConnectionButton = tk.Button(autoPilotFrame, text="Connect", bg='red', fg="white", command=getConnection) 
getConnectionButton.grid (row =0, column = 0, padx = 5, pady = 5)

getHeadingButton = tk.Button(autoPilotFrame, text="Get heading", bg='red', fg="white", command=getHeading) 
getHeadingButton.grid (row =1, column = 0, padx = 5, pady = 5) 
headingLabel = tk.Label (autoPilotFrame, text = "heading will be shown here") 
headingLabel.grid ( row = 1, column = 1, padx = 5, pady = 5)

protocolLabel = tk.Label (autoPilotFrame, text = "Detecting . . .") 
protocolLabel.grid ( row = 2, column = 1, padx = 5, pady = 5)

cameraFrame = tk.LabelFrame (text = "Camera control") 
cameraFrame.grid (row = 0, column = 1, padx = 5, pady = 5) 
startVideoButton = tk.Button(cameraFrame, text="Start Video", bg='red', fg="white", command=startVideo) 
startVideoButton.grid (row =0, column = 0, padx = 5, pady = 5)
stopVideoButton = tk.Button(cameraFrame, text="Stop Video", bg='red', fg="white", command=stopVideo) 
stopVideoButton.grid (row =0, column = 1, padx = 5, pady = 5)

pictureLabel = tk.Label(cameraFrame, text='Picture will be shown here') 
pictureLabel.grid(row=1, column=0, columnspan = 3)
fpsLabel = tk.Label(cameraFrame, text='fps will be shown here') 
fpsLabel.grid(row=2, column=0, columnspan = 3)

detectionButton = tk.Button(cameraFrame, text="Detection", bg='red', fg="white", command=getDetection) 
detectionButton.grid (row =0, column = 2, padx = 5, pady = 5)
detectionLabel = tk.Label(cameraFrame, text='detection will be shown here') 
detectionLabel.grid(row=2, column=1, columnspan = 3)

broker_address ="10.10.10.1" 
broker_port = 1883 

client = mqtt.Client("Little ground station") 
client.on_message = on_message 
client.connect(broker_address, broker_port) 
client.loop_start() 
client.subscribe('heading') 
client.subscribe('Video')
client.subscribe('fps')
client.subscribe('detection')
client.subscribe('protocol')
master.mainloop()