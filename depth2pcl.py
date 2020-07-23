import argparse
import os
import sys
import glob
import numpy as np
import open3d as o3d
from PIL import Image
from pathlib import Path
from plyfile import PlyData, PlyElement

focalLength = 525.0
centerX = 319.5
centerY = 239.5
scalingFactor = 5000.0

for i in range(4):
    directory = f"/home/kate/catkin_ws/src/DATASETS/Dataset/{i}"

    for j in range (2):
        rgb = Image.open(f"{directory}/rgb{j}.png")
        depth = Image.open(f"{directory}/depth{j}.png")
        points = []    
        for v in range(rgb.size[1]):
            for u in range(rgb.size[0]):
                color = rgb.getpixel((u,v))
                Z = depth.getpixel((u,v)) / scalingFactor
                if Z==0: continue
                X = (u - centerX) * Z / focalLength
                Y = (v - centerY) * Z / focalLength
                points.append("%f %f %f %d %d %d 0\n"%(X,Y,Z,color[0],color[1],color[2]))
        file = open(f"{directory}/pcl{j}.ply","w")
        file.write('''ply format ascii 1.0
    element vertex %d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    property uchar alpha
    end_header
    %s
    '''%(len(points),"".join(points)))
        file.close()
        # print("Testing IO for pointcloud...")
        pcd = o3d.io.read_point_cloud(f"{directory}/pcl{j}.ply")
        o3d.io.write_point_cloud(f"{directory}/pcl{j}.ply",pcd)