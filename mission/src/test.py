import pandas as pd

file_path="/home/acca/catkin_ws/src/ACCA2022-new/mission/data/center.csv"
path_data = pd.read_csv(file_path)

path_cx = path_data.cx.tolist()
path_cy = path_data.cy.tolist()
path_cyaw = path_data.cyaw.tolist()

print(path_cx)
