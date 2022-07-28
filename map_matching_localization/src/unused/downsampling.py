import numpy as np
from point_cloud import*
import open3d as o3d
import pcl


pcd = o3d.io.read_point_cloud("/home/enbang/k_citi_0722_02_down.pcd")
# print(type(pcd))
# print(np.asarray(pcd.points))
print(len(np.asarray(pcd.points)))
o3d.visualization.draw_geometries([pcd], window_name="pcd", width=1920, height=1080, left=50, top=50)

downpcd = pcd.voxel_down_sample(voxel_size=0.05)
print(len(np.asarray(downpcd.points)))
# o3d.visualization.draw_geometries([downpcd], window_name="downpcd", width=1920, height=1080, left=50, top=50)
o3d.io.write_point_cloud("/home/enbang/k_citi_0722_02_down.pcd", downpcd)