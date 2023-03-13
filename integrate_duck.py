# ----------------------------------------------------------------------------
# -                        Open3D: www.open3d.org                            -
# ----------------------------------------------------------------------------
# The MIT License (MIT)
#
# Copyright (c) 2018-2021 www.open3d.org
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
# ----------------------------------------------------------------------------

# examples/python/t_reconstruction_system/integrate.py

# P.S. This example is used in documentation, so, please ensure the changes are
# synchronized.

import os
import numpy as np
import open3d as o3d
import open3d.core as o3c
import time
import matplotlib.pyplot as plt
from dt_apriltags import Detector
import cv2

from common import load_rgbd_file_names, load_depth_file_names, load_intrinsic, load_extrinsics, get_default_dataset


img_width = 720
img_height = 960

# use apriltag pose estimation for cam_extrinsic

def draw_camera(cam_extrinsic, cam_intrinsic):

    cam = o3d.geometry.LineSet.create_camera_visualization(img_width, img_height, cam_intrinsic, cam_extrinsic)
    return cam

def integrate(depth_file_names, color_file_names, depth_intrinsic,
              color_intrinsic, depth_intrinsic_tensor,
              color_intrinsic_tensor, extrinsics, integrateColor,
			  camera_pose):

	# camera intrinsic is the same as color intrinsic
	cam_intrinsic = color_intrinsic
    
	n_files = 100
	device = o3d.core.Device("CPU:0")

	if integrateColor:
		vbg = o3d.t.geometry.VoxelBlockGrid(
		attr_names=('tsdf', 'weight', 'color'),
		attr_dtypes=(o3c.float32, o3c.float32, o3c.float32),
		attr_channels=((1), (1), (3)),
		voxel_size=3.0 / 512,
		block_resolution=16,
		block_count=50000,
		device=device)
	else:
		vbg = o3d.t.geometry.VoxelBlockGrid(attr_names=('tsdf', 'weight'),
			                            attr_dtypes=(o3c.float32,
			                                         o3c.float32),
			                            attr_channels=((1), (1)),
			                            voxel_size=3.0 / 512,
			                            block_resolution=16,
			                            block_count=50000,
			                            device=device)

	start = time.time()
	for i in range(n_files):
		print('Integrating frame {}/{}'.format(i, n_files))
                
		img = cv2.imread(color_file_names[i], cv2.IMREAD_GRAYSCALE)
		fx = depth_intrinsic[0][0]
		fy = depth_intrinsic[1][1]
		cx = depth_intrinsic[0][2]
		cy = depth_intrinsic[1][2]
		print(type(fx))

		at_detector = Detector(families='tag36h11',
							nthreads=8,
							quad_decimate=1.0,
							quad_sigma=0.0,
							refine_edges=1,
							decode_sharpening=0.25,
							debug=0)

		
		tags = at_detector.detect(img, estimate_tag_pose=True, camera_params=[fx, fy, cx, cy], tag_size=0.15)
		print(f'tags : {tags}')

		extrinsic = np.hstack((tags[0].pose_R, tags[0].pose_t))
		extrinsic = np.vstack((extrinsic, np.array([0, 0, 0, 1])))

		pose = draw_camera(extrinsic, cam_intrinsic)
		camera_pose.append(pose)
	
		# depth = o3d.t.io.read_image(depth_file_names[i]).to(device).to(dtype=o3c.Dtype.UInt16)
		depth = o3d.t.io.read_image(depth_file_names[i])
		nparr = np.asarray(depth)
		
		frustum_block_coords = vbg.compute_unique_block_coordinates(depth, depth_intrinsic_tensor, extrinsic, 1000.0, 5.0)
		
		print(color_intrinsic)
		print(depth_intrinsic)
		

		if integrateColor:
			color = o3d.t.io.read_image(color_file_names[i]).to(device)
			vbg.integrate(frustum_block_coords, depth, color,
					      depth_intrinsic_tensor, color_intrinsic_tensor, extrinsic,
					      1000.0, 5.0)
		else:
			vbg.integrate(frustum_block_coords, depth, depth_intrinsic_tensor,
					      extrinsic, 1000.0, 5.0)
			dt = time.time() - start

	return vbg


if __name__ == '__main__':
    

	pos = np.load("frames/pos.npy")
	
	depth_intrinsic = np.array([[760.79602051,   0.      ,   358.30508423],
				 [  0.     ,    760.79602051, 481.65368652],
				 [  0.       ,    0.       ,    1.        ]])

	color_intrinsic = np.array([[760.79602051,   0.      ,   358.30508423],
				 [  0.     ,    760.79602051, 481.65368652],
				 [  0.       ,    0.       ,    1.        ]])



	depth_intrinsic_tensor = o3c.Tensor(
                      np.array([[760.79602051,   0.      ,   358.30508423],
				 [  0.     ,    760.79602051, 481.65368652],
				 [  0.       ,    0.       ,    1.        ]]))

	color_intrinsic_tensor = o3c.Tensor(
                      np.array([[760.79602051,   0.      ,   358.30508423],
				 [  0.     ,    760.79602051, 481.65368652],
				 [  0.       ,    0.       ,    1.        ]]))


	extrinsics = o3c.Tensor(pos).to(o3c.float64) # given by record 3d

	print (extrinsics)

	

	depthFiles = []
	colorFiles = []
	camera_pose = []

	for i in range(200):
		depthFiles.append(f"frames/depth/{i}.png")
		colorFiles.append(f"frames/color/{i}.jpg")


	vbg = integrate(depthFiles, colorFiles, depth_intrinsic, color_intrinsic,
                    color_intrinsic_tensor, depth_intrinsic_tensor, extrinsics, True,
					camera_pose)

	pcd = vbg.extract_point_cloud(weight_threshold = -1.0)
	o3d.visualization.draw([pcd])
	o3d.visualization.draw(camera_pose, show_ui=True)

	mesh = vbg.extract_triangle_mesh(weight_threshold = -1.0)
	o3d.visualization.draw([mesh.to_legacy()])

