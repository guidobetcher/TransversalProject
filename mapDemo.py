import folium as fl
from streamlit_folium import st_folium
import streamlit as st
import os

def get_pos(lat,lng):
    return lat,lng

#m = fl.Map(location=[41.27563909993681, 1.9875612290901894], zoom_start=15)
m = fl.Map()
#m.save("map.html")
m.add_child(fl.LatLngPopup())

map = st_folium(m, height=350, width=700)
data = get_pos(map['last_clicked']['lat'],map['last_clicked']['lng'])

if data is not None:
    f = open("mission_region.txt", "a")
    f.write(str(data) + ",\n")
    f.close()

if st.button('Close'):
    exit()
else:
    st.write('Select at least one point')