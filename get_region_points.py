import os

def getRegionPoints():
    f = open("mission_region.txt", "w")
    f.truncate
    f.close()
    os.system("streamlit run mapDemo.py")
    f = open("mission_region.txt", "r")
    print(f.read())

getRegionPoints()

