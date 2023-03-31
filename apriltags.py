from pupil_apriltags import Detector
import cv2
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import numpy as np


def connectVehicle():
    connection_string = "tcp:127.0.0.1:5763"
    vehicle = connect(connection_string, wait_ready=True, baud=115200)
    return vehicle

def set_camera_parameters(focal_length, pixel_size):
    return  {'focal_length': focal_length, 'pixel_size': pixel_size}

def get_GSD(altitude):
    GSD = altitude * camera['pixel_size'] / camera['focal_length']
    return GSD

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    print(msg)

    #send command to vehicle on 1 Hz cycle


camera = set_camera_parameters(focal_length=6.048e-04, pixel_size=1.12e-06)
GSD=get_GSD(5)
at_detector = Detector(families="tag36h11",nthreads=10,quad_decimate=1.0,quad_sigma=0.0,refine_edges=1,decode_sharpening=0.25,debug=0)
vehicle=connectVehicle()
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    # Our operations on the frame come here
    
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow('frame', frame)
    tags=at_detector.detect(img)
    imcx,imcy=(320,240)
    #print(tags)
    if len(tags) > 0:
        vehicle.mode = VehicleMode('GUIDED')
        for r in tags:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))
            # draw the bounding box of the AprilTag detection
            cv2.line(frame, ptA, ptB, (0, 255, 0), 2)
            cv2.line(frame, ptB, ptC, (0, 255, 0), 2)
            cv2.line(frame, ptC, ptD, (0, 255, 0), 2)
            cv2.line(frame, ptD, ptA, (0, 255, 0), 2)
            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(frame, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.imshow('frame', frame)
            if imcx*0.8<cX<imcx*1.2 and imcy*0.9<cY<imcy*1.1:
                print('image centered')
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
                cap.release()
                cv2.destroyAllWindows
            else:
                v=np.array([[(cX-imcx)*GSD/5],[(-cY+imcy)*GSD/5],[0]])               
                msg = vehicle.message_factory.set_position_target_local_ned_encode(
                        0,       # time_boot_ms (not used)
                        0, 0,    # target system, target component
                        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
                        0b0000111111000111, # type_mask (only speeds enabled)
                        0, 0, 0, # x, y, z positions (not used)
                        v[0], -v[1], 0, # x, y, z velocity in m/s
                        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
                vehicle.send_mavlink(msg)
                

            
    if cv2.waitKey(1) == ord('q'):
        break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()


    