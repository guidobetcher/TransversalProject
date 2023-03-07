import cv2 as cv
import paho.mqtt.client as mqtt
import base64
import time
from dronekit import connect, VehicleMode
import threading
import pytesseract
import numpy as np
from pymavlink import mavutil

broker_address = 'localhost'
broker_port = 1883


def set_camera_parameters(focal_length, pixel_size):
    return  {'focal_length': focal_length, 'pixel_size': pixel_size}

def set_landing_pad_properties(radius, upper_hsv_color, lower_hsv_color):
    return {'radius': radius, 'hsv_color': {'upper': upper_hsv_color, 'lower': lower_hsv_color}}

def get_GSD(altitude):
    GSD = altitude * camera['pixel_size'] / camera['focal_length']
    return GSD

def get_scaled_kernel(altitude):
    GSD = altitude * camera['pixel_size'] / camera['focal_length']
    kernel_size = int(0.002 / GSD) # kernel is 2cm at all altitudes
    return np.ones((kernel_size, kernel_size), np.uint8)

def get_filtered_image(frame, kernel):
    filtered_image = apply_threshold(frame, landing_pad['hsv_color']['upper'], landing_pad['hsv_color']['lower'], kernel)
    return filtered_image

def get_matching_color_objects_contours(frame):
    # find the contours of the object
    return cv.findContours(frame, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

def apply_threshold(image, upper_theshold, lower_theshold, kernel):
    # change image to HSV
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    # filter the image by color
    mask = cv.inRange(image,lower_theshold, upper_theshold)  # returns a binary image
    # clean the image
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    closing = cv.morphologyEx(opening, cv.MORPH_CLOSE, kernel)
    return closing

def connectVehicle():
    connection_string = "/dev/ttyS0"
    vehicle = connect(connection_string, wait_ready=True, baud=115200)
    return vehicle

def getHeading(vehicle):
    client.publish('heading', str(vehicle.heading))
        
def rotate_image(img, angle):
    ancho = img.shape[1] #columnas
    alto = img.shape[0] # filas
    M = cv.getRotationMatrix2D((ancho//2,alto//2),angle,1)
    return cv.warpAffine(img,M,(ancho,alto))

def char_finder(img, char):
    ocr_config = r'--psm 10 -c tessedit_char_whitelist=' + char + ' -c min_orientation_margin=180'
   # print(ocr_config)
    res = pytesseract.image_to_string(img, config=ocr_config)
    print(res)    
    if 'H' in res:
       # print(res)
        return True
    else:
        return False

def doDetectionProtocol(vehicle, cap):
    client.publish('protocol', 'Performing Detection Protocol')
    vehicle.mode = VehicleMode('LOITER')
    time.sleep(3)        
    # define and send a command to the vehicle via mavlink
    msg = vehicle.message_factory.command_long_encode(0, 0,  # target system, target component
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
    0,  # confirmation
    6,  # servo number (parameter 1)
    2006,  # PWM value (parameter 2)
    0, 0, 0, 0, 0)  # parameters 3 to 7 (not used)
    vehicle.send_mavlink(msg)
    client.publish('protocol', 'Servo opened')
    time.sleep(1)
    msg2 = vehicle.message_factory.command_long_encode(0, 0,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    6,
    1000,
    0, 0, 0, 0, 0)
    vehicle.send_mavlink(msg2)
    client.publish('protocol', 'Servo closed')
    time.sleep(2)
    # set mode to return to launch
    vehicle.mode = VehicleMode('RTL')
    client.publish('protocol', 'returning to launch')
    cap.release()
    vehicle.close()

def videoStream():
    global video
    cap = cv.VideoCapture(0)
    while video:
        roi_fframe=[]
        start = time.time()
        # Read Frame
        _, frame = cap.read()
       
        altitude = vehicle.rangefinder.distance
        GSD = get_GSD(altitude)
        target_radius_pixel = int(landing_pad["radius"] / (GSD)) # we divide the GSD by 2 to adjuste the value. Maybe due to the calibration the calculled values didn't fit the measurments
        print('r target',target_radius_pixel)
        print(altitude)
        
        fframe = apply_threshold(frame, landing_pad["hsv_color"]["upper"], landing_pad["hsv_color"]["lower"], get_scaled_kernel(altitude))
        interest_region = fframe[target_radius_pixel:len(fframe)-target_radius_pixel][target_radius_pixel:len(fframe[0])-target_radius_pixel]
        contours, hierarchy = get_matching_color_objects_contours(fframe)
        if len(contours) > 0:
            # find the biggest contour and show it in blue
            c = max(contours, key=cv.contourArea)
            (x_max, y_max), radius_max = cv.minEnclosingCircle(c)
            center = (int(x_max), int(y_max))
            radius_max = int(radius_max)
            cv.circle(frame, center, radius_max, (255, 0, 0), 2)
            print('r max', radius_max)
            
            # compute the size in pixels of the target at certain altitude
            
            # compare target radius size in pixels to the measured radius size from the contours
            #measurements.pop(0)
            image_center = (len(frame[0])/2, len(frame)/2)
            if target_radius_pixel * 0.95 < radius_max < target_radius_pixel * 1.05 and radius_max < center[0] < len(frame[0])-radius_max and radius_max < center[1] < len(frame)-radius_max:
                
                # setting the path of pytesseract exe
                # you have to write the location of
                # on which your tesseract was installed
                pytesseract.pytesseract.tesseract_cmd = '/usr/bin/tesseract'

                # Now we will read the image in our program
                # you have to put your image path in place of photo.jpg
                #img = cv2.imread('IMG_7602.jpg')
                #img = cv2.imread('Captura de pantalla_2023-02-26_15-51-29.png')

                # Our image will read as BGR format,
                # So we will convert in RGB format because
                # tesseract can only read in RGB format

                # Rotación
                rotation_angle = 15
                roi_fframe = fframe[int(center[1]-radius_max/2):int(center[1]+radius_max/2), int(center[0]-radius_max/2):int(center[0]+radius_max/2)]
                for i in range(int(90/rotation_angle)):
                    # img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
                    # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,2))
                    # img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel,iterations=1)
                    # img = 255 - img
                    # For getting the text and number from image
                    print('x1:',int(center[1]-radius_max/2) )
                    print('x2:',int(center[1]+radius_max/2) )
                    print('y', int(center[0]+radius_max/2) )
                    print('y', int(center[0]-radius_max/2))
                    print(len(fframe))
                    print(len(fframe[0]))
                    if char_finder(roi_fframe, "H"):
                        print("H detected")
                        client.publish('detection','H detected')
                        doDetectionProtocol(vehicle, cap)
                        # For displaying the original image
                    else:
                        roi_fframe = rotate_image(roi_fframe, rotation_angle)
                        print("no H detected")
                        client.publish('detection','no H detected')
        if len(roi_fframe)>0:
            fframe =roi_fframe
        else:
            pass
        _, buffer = cv.imencode('.jpg', fframe)
        # Converting into encoded bytes
        jpg_as_text = base64.b64encode(buffer)
        # Publishig the Frame on the Topic home/server
        client.publish('Video', jpg_as_text)
        end = time.time()
        t = end - start
        fps = int(1/t)
        client.publish('fps', fps)


def on_message(client, userdata, message):
    global video
    global vehicle

    if message.topic == 'getHeading':
        print ('Get heading')
        w = threading.Thread(target=getHeading(vehicle))
        w.start()

    if message.topic == 'startVideo':
        print('Start video')
        video = True
        w = threading.Thread(target=videoStream)
        w.start()

    if message.topic == 'stopVideo':
        print('Stop video')
        video = False

    if message.topic == 'getConnection':
        vehicle=connectVehicle()
        print ('connecting')
        


camera = set_camera_parameters(focal_length=6.048e-04, pixel_size=1.12e-06)
'''
 To set the landing pad's upper and lower HSV colors we recomend to take a picture with the camera you will
use and with some tool take 10 color measurments of the landing pad and select the maximum and
minimum values of all three dimensions (Hue, Saturation and Value)
'''
landing_pad = set_landing_pad_properties(radius=0.54,
                                         upper_hsv_color=np.array([120, 255, 255]),
                                         lower_hsv_color=np.array([85, 70, 50]))

client = mqtt.Client('On board controller')
client.on_message = on_message
client.connect(broker_address, broker_port)
print('waiting commands . . .')
client.subscribe('getHeading') 
client.subscribe('startVideo')
client.subscribe('stopVideo') 
client.subscribe('getConnection')
client.loop_forever()
