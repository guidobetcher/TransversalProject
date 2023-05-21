import os
import tkinter as tk
from tkinter import ttk
import subprocess

def getRegionPoints():
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
def setFoV():
    print("vertical fov: " + v_fov_entry.get())
    print("horizontal fov: " + h_fov_entry.get())
def setFlightAltitude():
    print("flight altitude: " + flight_altitude_entry.get())
def setOverlapping():
    print("Overlapping: " + overlap_entry.get())
def setWaypoints():
    print("Waipoints: \n" + getPoints())
def updatePoints():
    points_label.config(text="Actual Waypoints:\n" + getPoints())
def submitFlightParams():
    setFoV()
    setFlightAltitude()
    setOverlapping()
    setWaypoints()

root = tk.Tk()
# root.config(width=500, height=200)
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

fov_button = tk.Button( root, text ="Submit Camera FoV", command = setFoV)
flight_altitude_button = tk.Button( root, text ="Submit Flight Altitude", command = setFlightAltitude)
overlap_button = tk.Button( root, text ="Submit Overlapping", command = setOverlapping)
map_button = tk.Button( root, text ="Set new Waypoints", command = getRegionPoints)
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

# fov_button.place(x=250, y=50)
# flight_altitude_button.place(x=250, y=100)
# overlap_button.place(x=250, y=150)
# map_button.place(x=150, y=200)
# update_points_butt.place(x=150, y=250)

root.mainloop()
