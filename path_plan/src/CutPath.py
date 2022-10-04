import rospy
import sqlite3
import csv
from DataBase import*
from LoadPath import LoadPath
from SavePath import SavePath
from path_plan.msg import PathRequest, PathResponse
import pandas as pd

def loadCSV(path):
    cx = []
    cy = []
    cyaw = []
    
    with open(path, "r") as csvFile:
        reader = csv.reader(csvFile, delimiter=",")
        print(type(reader))
        for row in reader:
            cx.append(row[0])        
            cy.append(row[1])
            cyaw.append(row[2])
                
    return cx, cy, cyaw

def save_to_DB(cx, cy, cyaw):
    path = PathResponse()
    
    path.start = "P2"
    path.end = "A2"
    path.path_id = "P2A2"
    
    
    for i in range(1275, len(cx)):
        
        path.cx.append(cx[i])
        path.cy.append(cy[i])
        path.cyaw.append(cyaw[i])

    return path        

if __name__ == "__main__":
    rospy.init_node("cut_path")
    db = DB(db_name="/0924_bs.db")
    load_path = LoadPath(db)
    save_path = SavePath()
    
    rospy.Subscriber("/path_request", PathRequest, callback = save_path.pathRequestCallback)
    rospy.Subscriber("/path_response", PathResponse, callback=load_path.RequestCallback)
    
    cx, cy, cyaw = loadCSV('/home/enbang/catkin_ws/src/ACCA2022-new/path_plan/path/cut_path.csv')
    path = save_to_DB(cx, cy, cyaw)
    db.savePath(path)