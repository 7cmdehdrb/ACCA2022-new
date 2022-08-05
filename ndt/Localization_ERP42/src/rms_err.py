import csv
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as ip
import pandas as pd


# lat =[37.496730,37.496688,37.496644,37.496617,37.496110,37.496077,37.496152,37.496574,37.496644,37.496745,37.496720]
# lon = [126.957392,126.957948,126.958010,126.958010,126.957896,126.957781,126.956842,126.956941,126.957150,126.957191,126.957475]

# new_lat = []
# new_lon = []

# for i in range(len(lat)):
#     if i != (len(lat)-1):
#         new_lat += list(np.linspace(lat[i],lat[i+1],100))
#         new_lon += list(np.linspace(lon[i],lon[i+1],100))


# f1 = open("/home/taehokim/csv_file/ekf_local2.csv","r")

# f2 = open("/home/taehokim/csv_file/gps_local2.csv","r")
# data_x = []
# data_y = []
# Data_x = []
# Data_y = []
# data = []
# for line in f1:
#     data_x.append(float(line.split(",")[0])) 
#     data_y.append(float(line.split(",")[1]))

# for line2 in f2:
#     Data_x.append(float(line2.split(",")[0]))
#     Data_y.append(float(line2.split(",")[1]))
    
    

# for i in range(len(data_x)):
#     tmp = 0
#     for j in range(len(Data_x)):    
#         if tmp == 0:
#             tmp = ((Data_x[j]-data_x[i])**2+(Data_y[j]-data_y[i])**2)**2
           
#         if tmp > ((Data_x[j]-data_x[i])**2+(Data_y[j]-data_y[i])**2):
#             tmp = ((Data_x[j]-data_x[i])**2+(Data_y[j]-data_y[i])**2)**2
            
#     data.append(tmp)
   
# print(max(data))
  
#print((sum(data)/len(data))**0.5)
  
  
# err_gps = pd.DataFrame(data=None, columns={"error"})
# for input_data in data:
#     err_gps.loc[len(err_gps)]=input_data
#     err_gps.to_csv("/home/taehokim/csv_file/rms_ekf_local3.csv", index = False)


f1 = open("/home/taehokim/csv_file/rms_ekf_local3.csv", "r")
data = []
for line in f1:
    data.append(float(line))
index = list(range(len(data)))
print(max(data))


# f2 = open("/home/taehokim/csv_file/rms_gps_local2.csv", "r")
# data2 = []0
# for line in f2:
#     data2.append(float(line))
# index2 = list(range(len(data2)))

f3 = open("/home/taehokim/csv_file/rms_ndt_local3.csv","r")
data3 = []
for line in f3:
    data3.append(float(line))
index3 =list(range(len(data3)))
print(max(data3))
# plt.plot(index,data,'r',label="ekf")
# plt.plot(index2,data2,'g',label="gps")
# plt.plot(index3,data3,'b',linewidth="2", label="ndt-matching")
# plt.xlabel("index")
# plt.ylabel("Error")
# plt.rc('legend', fontsize = 10)
# plt.legend()
# plt.show()
