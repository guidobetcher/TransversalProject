# import required libraries
from vidgear.gears import NetGear
import cv2
import numpy as np
#Comentario 2
def findpoint(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        (B, G, R) = frame[y, x]
        #print('B = ', B, 'G = ', G, 'R = ', R)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        (H, S, V) = hsv[y, x]
        print('H = ', H, 'S = ', S, 'V = ',V)
        print('(',x,',',y,')')

# define tweak flags
options = {"flag": 0, "copy": False, "track": False}
# Define Netgear Client at given IP address and define parameters
client = NetGear( address="10.10.10.240", port="5454", protocol="tcp", pattern=0, receive_mode=True, logging=True, **options )
# loop over
while True:
    # receive frames from network
    frame = client.recv()  # check for received frame
    if frame is None:
        break
    cv2.namedWindow('Output Frame')
    cv2.setMouseCallback('Output Frame', findpoint)
    # Show output window
    cv2.imshow("Output Frame", frame)

    # check for 'q' key if pressed
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
'''
# change each frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of color in HSV
    lower = np.array([95, 50, 50])
    upper = np.array([120, 255, 255])

    # apply a threshold
    mask = cv2.inRange(hsv, lower, upper)  # returns a binary image
    altitude = 2  # vehicle.location.global_frame.alt  # altitude
    target_radius = 0.5  # target radius in meters
    p = 1.12e-06  # camera pixel size in meters
    f = 3.04e-03  # camera focal length in meters
    GSD = altitude * p / f
    ks = int(0.02/GSD)
    kernel = np.ones((ks, ks), np.uint8)
    # clean the image
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)

        #find the contours of the object
    contours, hierarchy = cv2.findContours(closing, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
            # draw all found contours in green
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)
        cv2.circle(frame, center, radius, (0, 255, 0), 2)

        # show the circles in the image
        #cv2.imshow('Circle', frame)

        # returns binary image but with the desired color (i.e. black and red image)
    color = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('Detector', color)

    if len(contours) > 0:
        # find the biggest contour and show it in blue
        c = max(contours, key=cv2.contourArea)
        (x_max, y_max), radius_max = cv2.minEnclosingCircle(c)
        center = (int(x_max), int(y_max))
        radius_max = int(radius_max)
        print('Radius max', radius_max)
        cv2.circle(frame, center, radius_max, (255, 0, 0), 2)
        cv2.imshow('Circle', frame)

            # compute the size in pixels of the target at certain altitude
        altitude = 2  # vehicle.location.global_frame.alt  # altitude
        target_radius = 0.5  # target radius in meters
        p = 1.12e-06  # camera pixel size in meters
        f = 6.048e-04  # camera focal length in meters
        GSD = altitude * p / f
        target_radius_pixel = target_radius / GSD
        print('target radius', target_radius_pixel)
# close output window'''
cv2.destroyAllWindows()
# safely close client
client.close()
