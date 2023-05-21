import matplotlib.pyplot as plt
import matplotlib.patches as mpp
import matplotlib.lines as mpl
import numpy as np
import os
import tkinter as tk
import subprocess
from pymavlink import mavutil
#from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from tkinter import ttk
# Represent a non-vertical line segment from start_pt to end_pt
# as y = mx + b and minv <= x <= maxv.
# For vertical lines x = b, m = None and minv <= y <= maxv
class LineSeg():

    def __init__(self, start_pt, end_pt):

        self.x, self.y = start_pt
        self.x2, self.y2 = end_pt

        if self.x != self.x2:
            self.m = (self.y2 - self.y) / (self.x2 - self.x)
            self.b = self.y - self.m*self.x

            self.minv = min(self.x, self.x2)
            self.maxv = max(self.x, self.x2)

        else:
            self.m = None
            self.b = self.x
            self.minv = min(self.y, self.y2)
            self.maxv = max(self.y, self.y2)

    def length(self):
        return np.linalg.norm([self.x2-self.x, self.y2-self.y])


    # Find intersection (x, y) with line y = mx + b
    def intersect_w_line(self, m, b):
        # Parallel lines
        if m == self.m:
            return (None, None)

        # Line is vertical but line segment is not
        elif m == None:
            if self.minv <= b <= self.maxv:
                return (b, self.m*b + self.b)
            else:
                return (None, None)

        # Line segment is vertical, but line is not
        elif self.m == None:
            y = m*self.b + b

            if self.minv <= y <= self.maxv:
                return (self.b, y)
            else:
                return (None, None)

        else:

            x = (b - self.b) / (self.m - m)
            y = self.m*x + self.b

            if self.minv <= x <= self.maxv:
                return (x, y)
            else:
                return (None, None)

    # Find intercept range with line y = mx + b
    def intercept_range(self, m):

        if self.m == m:
            return (self.b, self.b)

        # Line is vertical, but segment is not
        elif m == None:
            return sorted([self.x, self.x2])

        # Line is not vertical
        else:
            b = self.y - m*self.x
            b2 = self.y2 - m*self.x2

            return sorted([b, b2])

def sortPoints(points):
    # compute centroid
    coords= np.array(points)
    cx, cy = coords.mean(0)
    x, y = coords.T
    angles = np.arctan2(x - cx, y - cy)
    indices = np.argsort(angles)
    return coords[indices]

def get_distance(point1, point2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    return np.sqrt((dx*dx) + (dy*dy))

def closest_point(point, points):
    points = np.asarray(points)
    deltas = points - point
    dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    return np.argmin(dist_2)

def setEdgePoints():
    f = open("mission_region.txt", "w")
    f.truncate
    f.close()
    subprocess.Popen(["streamlit","run","mapDemo.py"])
    #os.system("streamlit run mapDemo.py &")

def getPoints():
    f = open("mission_region.txt", "r")
    points = f.read()
    f.close()
    return points

def getFoV():
    fov={}
    fov["vertical"] = float(v_fov_entry.get())
    fov["horizontal"] = float(h_fov_entry.get())
    return fov

def getFlightAltitude():
    return float(flight_altitude_entry.get())

def getOverlapping():
    return float(overlap_entry.get())

def getEdgePoints():
    points = []
    lines = getPoints().split(",\n")
    for line in lines:
        if line:
            point = line[1:-1].split(",")
            points.append([float(point[1]),float(point[0])])
    return points

def updatePoints():
    points_label.config(text="Actual Waypoints:\n" + getPoints())

def getClosestPoint(points, home):
    distance = []
    for point in points:
        distance.append(get_distance(home, point))
    return np.array(points[distance.index(min(distance))])

def getWaypoints(edge_points, grid_spacing, home):
    points = sortPoints(edge_points)
    closest_point = getClosestPoint(points, home)
    while points[0,0] != closest_point[0] and points[0,1] != closest_point[1]:
        points=np.roll(points,-1,axis=0)
    linesegs = [LineSeg(points[i], points[i+1]) if i+1 < len(points) else LineSeg(points[i], points[0]) for i in range(len(points))] #create objects
    closest_seg = linesegs[0]
    m = closest_seg.m
    b = closest_seg.b

    intercept_ranges = [lineseg.intercept_range(m) for lineseg in linesegs]

    max_intercept = np.max(intercept_ranges)
    min_intercept = np.min(intercept_ranges)
    intercepts = np.arange(min_intercept + grid_spacing, max_intercept, grid_spacing)
    line_pts = []
    for intercept in intercepts:
        linesegs = linesegs[::-1]
        for lineseg in linesegs:
            if lineseg.intersect_w_line(m, intercept)[0] is not None:
                line_pts.append(lineseg.intersect_w_line(m, intercept))
    waypoints = np.array(line_pts, dtype=np.dtype(object))
    return waypoints

def sendFlightPlan(cmds, waypoints, altitude, speed):
    cmds.clear()
    wp = []
    a = False
    for waypoint in waypoints:
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
                                 waypoint[0],waypoint[1],2))
    cmds.upload()
    vehicle.close()
    plt.close('all')

    fig, ax = plt.subplots(1, 1)

    polygon = mpp.Polygon(waypoints, closed = True, fill = False)

    ax.add_artist(polygon)

    ax.set_xlim(0, 13)
    ax.set_ylim(0, 12)
    '''ax.set_xlim(1.988, 1.989)
    ax.set_ylim(41.276, 41.277)'''
    plt.show()

def getSpeed():
    # Falta definir esta función
    return 1

def submitFlightParams():
    fov = getFoV()
    altitude = getFlightAltitude()
    overlapping = getOverlapping()
    edge_points = getEdgePoints()
    cs='tcp:127.0.0.1:5763'
    vehicle = connect(cs, wait_ready=True, baud=115200)
    cmds=vehicle.commands
    cmds.download()
    cmds.wait_ready()
    lon=vehicle.location.global_relative_frame.lon
    lat=vehicle.location.global_relative_frame.lat
    home = np.array([lon, lat])
    # Hay que corregir el spacing. Tiene que ser la distancia a desplazar en el eje de las y
    grid_spacing = flight_altitude*np.tan(fov["horizontal"]/2)*(2-ovelapping)
    waypoints = getWaypoints(edge_points, grid_spacing, home)
    speed = getSpeed()
    sendFlightPlan(cmds, waypoints, altitude, speed)

root = tk.Tk()
# Crear caja de texto.
v_fov_label = ttk.Label(root, text ="Enter Horizontal FoV")
h_fov_label = ttk.Label(root, text ="Enter Vertical FoV")
flight_altitude_label = ttk.Label(root, text ="Enter Flight Altitude")
overlap_label = ttk.Label(root, text ="Enter Overlapping")
points_label = ttk.Label(root, justify='center', text = "Actual Waypoints:\n" + getPoints())

v_fov_entry = ttk.Entry(root, width=10)
h_fov_entry = ttk.Entry(root, width=10)
flight_altitude_entry = ttk.Entry(root, width=10)
overlap_entry = ttk.Entry(root, width=10)

map_button = tk.Button( root, text ="Set new Waypoints", command = setEdgePoints)
update_points_button = tk.Button( root, text ="Update Points", command = updatePoints)
submit_button = tk.Button( root, text ="Submit Flight Params", command = submitFlightParams)

# Posicionarla en la ventana.
v_fov_label.grid(row = 0, column = 0, pady = 2)
h_fov_label.grid(row = 0, column = 1, pady = 2)
flight_altitude_label.grid(row = 2, column = 0, columnspan = 2, pady = 2)
overlap_label.grid(row = 4, column = 0, columnspan = 2, pady = 2)
points_label.grid(row=8,column = 0, columnspan=2, pady=10)

v_fov_entry.grid(row = 1, column = 0, pady = 2)
h_fov_entry.grid(row = 1, column = 1, pady = 2)
flight_altitude_entry.grid(row = 3, column = 0, columnspan = 2, pady = 2)
overlap_entry.grid(row = 5, column = 0, columnspan = 2, pady = 2)
map_button.grid(row = 6, column = 0, columnspan=2, pady = (10,2))
update_points_button.grid(row = 7, column = 0, columnspan=2, pady = 2)
submit_button.grid(row = 9, column = 0, columnspan=2, pady = 2)

root.mainloop()
