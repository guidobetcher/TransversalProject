import cv2
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
from vidgear.gears import NetGear

def set_video_server(ip, port, protocol):
    options = {"flag": 0, "copy": False, "track": False} # Define Netgear server at given IP address and define parameters
    return NetGear( address=ip, port=port, protocol=protocol, pattern=0, logging=True, **options )

def set_camera_parameters(focal_length, pixel_size):
    return  {'focal_length': focal_length, 'pixel_size': pixel_size}

def set_landing_pad_properties(radius, upper_hsv_color, lower_hsv_color):
    return {'radius': radius, 'hsv_color': {'upper': upper_hsv_color, 'lower': lower_hsv_color}}

def get_scaled_kernel(altitude):
    GSD = altitude * camera['pixel_size'] / camera['focal_length']
    kernel_size = int(0.02 / GSD) # kernel is 2cm at all altitudes
    return np.ones((kernel_size, kernel_size), np.uint8)

def get_matching_color_objects_contours(frame):
    # change each frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    altitude = vehicle.rangefinder.distance
    kernel = get_scaled_kernel(altitude)
    # apply a threshold
    filtered_image = apply_threshold(hsv, landing_pad['color']['upper'], landing_pad['color']['lower'], kernel)

    # find edge
    # find the contours of the object
    return cv2.findContours(filtered_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

def apply_threshold(image, upper_theshold, lower_theshold, kernel):
    mask = cv2.inRange(hsv, lower, upper)  # returns a binary image
    # clean the image
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    return closing

server = set_video_server("10.10.10.240", "5454", "tcp")
camera = set_camera_parameters(focal_length=3.04e-03, pixel_size=1.12e-06)
'''
 To set the landing pad's upper and lower HSV colors we recomend to take a picture with the camera you will
use and with some tool take 10 color measurments of the landing pad and select the maximum and
minimum values of all three dimensions (Hue, Saturation and Value)
'''
landing_pad = set_landing_pad_properties(radius=0.54,
                                         upper_hsv_color=np.array([120, 255, 255]),
                                         lower_hsv_color=np.array([95, 50, 20]))

# define a connection string and connect to the vehicle
cs = '/dev/ttyS0'  # for Raspi UART
#cs = "tcp:127.0.0.1:5763"  # for mission planner sitl
file = open('/home/ubuntu/detection.log', 'a')
measurements = [0,0,0,0,0,0,0,0]

while True:
    if 'vehicle' in locals():
        vehicle_mode = vehicle.mode.name
        print(vehicle_mode)
        if vehicle_mode == 'LOITER':
            # inizialize video capture
            video_capture = cv2.VideoCapture(0)
            positive_count = 0
            
            # maake sure came is correctly inizialized
            if not video_capture.isOpened():
                print('Error')
                file.write(str(time.time()) + ' Error')
            while video_capture.isOpened():
                # Read the video frame by frame
                ret, frame = video_capture.read()
            
                if ret:
                    #print('Capturing video...')
                    contours, hierarchy = get_matching_color_objects_contours(frame)
                    if len(contours) > 0:
                        # find the biggest contour and show it in blue
                        c = max(contours, key=cv2.contourArea)
                        (x_max, y_max), radius_max = cv2.minEnclosingCircle(c)
                        center = (int(x_max), int(y_max))
                        radius_max = int(radius_max)
                        cv2.circle(frame, center, radius_max, (255, 0, 0), 2)
                        print('r max', radius_max)
            
                        # compute the size in pixels of the target at certain altitude
                        altitude = vehicle.rangefinder.distance  # altitude
                        target_radius_pixel = target_radius / (GSD/2) # we divide the GSD by 2 to adjuste the value. Maybe due to the calibration the calculled values didn't fit the measurments
                        print('r target',target_radius_pixel)
                        # compare target radius size in pixels to the measured radius size from the contours
                        measurements.pop(0)
                        if target_radius_pixel * 0.95 < radius_max < target_radius_pixel * 1.05:
                            server.send(frame)
                            measurements.append(1)
                            if sum(measurements) >= 4:
                                # set vehicle mode to loiter and stop detecting objects
                                vehicle.mode = VehicleMode('LOITER')
                                video_capture.release()
                                time.sleep(3)
            
                                # define and send a command to the vehicle via mavlink
                                msg = vehicle.message_factory.command_long_encode(0, 0,  # target system, target component
                                                                              mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
                                                                              0,  # confirmation
                                                                              6,  # servo number (parameter 1)
                                                                              2006,  # PWM value (parameter 2)
                                                                              0, 0, 0, 0, 0)  # parameters 3 to 7 (not used)
                                vehicle.send_mavlink(msg)
                                time.sleep(1)
                                msg2 = vehicle.message_factory.command_long_encode(0, 0,
                                                                               mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                                               0,
                                                                               6,
                                                                               1000,
                                                                               0, 0, 0, 0, 0)
                                vehicle.send_mavlink(msg2)
                                time.sleep(2)
                                # set mode to return to launch
                                vehicle.mode = VehicleMode('RTL')
                            file.write(str(time.time()) + ' +')
                            print('+')
                        else:
                            measurements.append(0)
                            print('-')
                            file.write(str(time.time()) + ' -')
    else:
        try:
            vehicle = connect(cs, wait_ready=True, baud=115200)
        except:
            time.sleep(3)

# close communication with the vehicle
vehicle.close()
video_capture.release()
server.close()
