# -*- coding: utf-8 -*-
"""
plot lines on open street map
"""

# Make beautiful maps with Leaflet.js and Python
# conda install -c conda-forge folium
import folium
import csv
import numpy as np
import matplotlib.pyplot as plt

# points=[(31.949515,118.697229),
# (31.950135,118.696985),
# (31.950556,118.696913),
# (31.951091,118.697034),
# (31.951475,118.697531),
# (31.951647,118.698275),
# (31.951669,118.698371)]


lat =[37.496712,37.496686,37.496597,37.496132,37.496085,37.496143,37.496244,37.496566,37.496709,37.496691]
lon = [126.957522,126.957866,126.957951,126.957891,126.957773,126.957023,126.956948,126.956998,126.957280,126.957489]

new_lat = []
new_lon = []

for i in range(len(lat)):
    if i != (len(lat)-1):
        new_lat += list(np.linspace(lat[i],lat[i+1],100))
        new_lon += list(np.linspace(lon[i],lon[i+1],100))


gps_x =[]
gps_y = []
ekf_x = []
ekf_y = []
ndt_x = []
ndt_y = []

points1 = []
points2 = []
points3 = []
points4 = []
f1 = open("/home/taehokim/csv_file/gps_local3.csv",'r')
f2 = open("/home/taehokim/csv_file/ekf_local3.csv",'r')
f3 = open("/home/taehokim/csv_file/ndt_local3.csv",'r')

for line in f1:
    data = line.split(',')
    gps_x.append(float(data[0])) 
    gps_y.append(float(data[1]))
    # points1.append(tuple([lat, lon]))

for line in f2:
    data = line.split(',')
    ekf_x.append(float(data[0]))
    ekf_y.append(float(data[1]))
    # points2.append(tuple([lat, lon]))
    
for line in f3:
    data = line.split(',')
    ndt_x.append(float(data[0]))
    ndt_y.append(float(data[1]))
    # points4.append(tuple([lat, lon]))    


gps_x = np.array(gps_x)
gps_y = np.array(gps_y)
ekf_x = np.array(ekf_x)
ekf_y = np.array(ekf_y)
ndt_x = np.array(ndt_x)
ndt_y = np.array(ndt_y)

error_gps_x = (gps_x-gps_x)
error_gps_y = (gps_y-gps_y)
error_ekf_x = (gps_x- ekf_x)
error_ekf_y = gps_y - ekf_y
error_ndt_x = (gps_x - ndt_x)
error_ndt_y = gps_y - ndt_y

def rmse(error):
    return np.sqrt((error ** 2).mean())


print(np.max(error_ekf_x))
print(np.max(error_ekf_y))
print(np.max(error_ndt_x))
print(np.max(error_ndt_y))

# plt.figure(figsize=(8,3))
# plt.plot(list(range(len(error_gps_x))),error_gps_y,"r-",label="rtk-gps")
# plt.plot(list(range(len(error_gps_x))),error_ekf_y,"g-", label="EKF" )
# plt.plot(list(range(len(error_gps_x))),error_ndt_y,"b-", label="ndt-matching")
# plt.xlabel("count")
# plt.ylabel("error y [m]")


# plt.legend(loc=(0.02,0.85))
# plt.show()


# for i in range(len(new_lat)):
#     points3.append(tuple([new_lat[i],new_lon[i]]))

# # initiate to plotting area
# my_map = folium.Map(
#     location=[points1[0][0],points1[-1][1]], 
#     zoom_start=15, 
#     tiles='OpenStreetMap',
#     png_enabled=True, 
#     prefer_canvas=True)

# folium.PolyLine(points1, color="red", weight=2.0, opacity=1).add_to(my_map)

# folium.PolyLine(points2, color="blue", weight=2.0, opacity=1).add_to(my_map)

# # folium.PolyLine(points3, color="green", weight=3.0, opacity=1).add_to(my_map)

# folium.PolyLine(points4, color="pink", weight=2.0, opacity=1).add_to(my_map)

# # for point in points:
# #       folium.Marker(point).add_to(my_map)
      
# my_map.save("/home/taehokim/map_with_paths4.html")
