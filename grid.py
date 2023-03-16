import matplotlib.pyplot as plt
import matplotlib.patches as mpp
import matplotlib.lines as mpl
import numpy as np
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command

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

cs='tcp:127.0.0.1:5763'
vehicle = connect(cs, wait_ready=True, baud=115200)   
cmds=vehicle.commands
cmds.download()
cmds.wait_ready()


#points = [[12, 1], [2, 10],[10, 0], [10, 4], [0, 8]]
points = [[1.9884409, 41.2764156], 
          [1.9886743, 41.2762453],
          [1.9888969, 41.2762775], 
          [1.9889881, 41.2763894], 
          [1.9887936, 41.2764650],
          [1.9886152, 41.2764630]]
points = sortPoints(points)
print('points1', points)
lon=vehicle.location.global_relative_frame.lon
lat=vehicle.location.global_relative_frame.lat
home = np.array([lon, lat])
print(home)
distance = []
for point in points:
    distance.append(get_distance(home, point))

closest_point = points[distance.index(min(distance))]
closest_point=np.array(closest_point)
while points[0,0] != closest_point[0] and points[0,1] != closest_point[1]:
    points=np.roll(points,-1,axis=0)

print('points', points)

linesegs = [LineSeg(points[i], points[i+1]) if i+1 < len(points) else LineSeg(points[i], points[0]) for i in range(len(points))] #create objects
lengths = [lineseg.length() for lineseg in linesegs]
print('lengths: ', lengths)
longest_seg = [lineseg for lineseg in linesegs if lineseg.length() == max(lengths)]
m = longest_seg[0].m
print(m)
b = longest_seg[0].b
print(b)
closest_seg = linesegs[0]
m = closest_seg.m
print(m)
b = closest_seg.b
print(b)

intercept_ranges = [lineseg.intercept_range(m) for lineseg in linesegs]
print('intercept ranges', intercept_ranges)

max_intercept = np.max(intercept_ranges)
min_intercept = np.min(intercept_ranges)
print('max intercept', max_intercept)
print('min intercept', min_intercept)

num_lines = 10
#spacing =  0.00005
spacing=(max_intercept - min_intercept) / (num_lines+1)

intercepts = np.arange(min_intercept + spacing, max_intercept, spacing)
print('intercepts', intercepts)

line_pts = [[lineseg.intersect_w_line(m, intercept) for lineseg in linesegs if lineseg.intersect_w_line(m, intercept)[0] is not None] for intercept in intercepts]
print(line_pts)
line_pts=np.array(line_pts, dtype=np.dtype(object))

print('line points', line_pts)
cmds.clear()
wp = []
a = False
for i in line_pts:
    if a == True:
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
                                 i[0,1],i[0,0],2))
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
                                 i[1,1],i[1,0],2))
        # wp.append([i[0,1],i[0,0]])
        # wp.append([i[1,1],i[1,0]])
        a = False
    elif a ==False:
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
                                 i[1,1],i[1,0],2))
        cmds.add(Command(0,0,0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,0,0,0,0,0,0,
                                 i[0,1],i[0,0],2))
        # wp.append([i[1,1],i[1,0]])
        # wp.append([i[0,1],i[0,0]])
        a = True
cmds.upload()
vehicle.close()
plt.close('all')

fig, ax = plt.subplots(1, 1)

polygon = mpp.Polygon(points, closed = True, fill = False)

ax.add_artist(polygon)

for start, end in line_pts:
    line = mpl.Line2D([start[0], end[0]], [start[1], end[1]])
    ax.add_artist(line)

ax.set_xlim(0, 13)
ax.set_ylim(0, 12)
'''ax.set_xlim(1.988, 1.989)
ax.set_ylim(41.276, 41.277)'''

print('wp', wp)
plt.show()

# waypoints = []
# waypoints.append([closest_seg.x, closest_seg.y])
# waypoints.append([closest_seg.x2, closest_seg.y2])
# line_pts = line_pts[::-1]
# for line_pt in line_pts:
#     wp_options = points
#     wp_options = np.append(wp_options, line_pt)
#     next_wp = closest_point(waypoints[-1], wp_options)
#     print(wp_options)
#     print(waypoints[-1])
#     break