import os
import cv2 as cv
import numpy as np
import open3d as o3d
import yaml

file_dir = "/home/ysc/lite_cog/yinsai_ws/map/"
file_name = "map.pcd"
png_name = file_name.replace(".pcd",".png")
pgm_name = file_name.replace(".pcd",".pgm")
dilate_name = file_name.replace(".pcd","_erode.pgm")
yaml_name = file_name.replace(".pcd",".yaml")
file_path = os.path.join(file_dir, file_name)
resolution = 0.1
thresh_min_z = 0.2
thresh_max_z = 1.0

def pcl2pgm():
       point_cloud = o3d.io.read_point_cloud(file_path)
       point_array = np.array(point_cloud.points)
       min_x,min_y,min_z = np.min(point_array,axis=0)
       max_x,max_y,max_z = np.max(point_array,axis=0)
       x_l = int((max_x - min_x)/resolution)+1
       y_l = int((max_y - min_y)/resolution)+1
       valid_mask = np.logical_and(point_array[:,2]>thresh_min_z,point_array[:,2]<thresh_max_z)
       valid_point = point_array[valid_mask]
       img = np.ones(shape=(y_l, x_l),dtype=np.uint8) * 255
       xy_idx = np.floor((valid_point[:,:2] - np.array([min_x, min_y]))/resolution).astype(int)

       xy_idx[:,1] = y_l - xy_idx[:,1] - 1
       img[xy_idx[:,1], xy_idx[:,0]] = 0
       cv.imwrite(os.path.join(file_dir, png_name), img)

       config_dict = dict()
       config_dict['image'] = pgm_name
       config_dict['resolution'] = resolution
       config_dict['origin'] =[float(min_x), float(min_y), 0.0]
       config_dict['negate'] = 0
       config_dict['occupied_thresh'] = 0.65
       config_dict['free_thresh'] = 0.196

       with open(os.path.join(file_dir, yaml_name), 'w') as f:
              yaml.dump(config_dict, f)


def erode():
       image = cv.imread(os.path.join(file_dir, pgm_name), cv.IMREAD_GRAYSCALE)
       kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (3, 3))
       eroded = cv.erode(image, kernel)
       dilated = cv.dilate(eroded, kernel)
       cv.imwrite(os.path.join(file_dir, dilate_name), dilated)

if __name__ == "__main__":
    pcl2pgm()
