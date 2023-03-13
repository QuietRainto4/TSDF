import open3d as o3d
import numpy as np
import cv2

img_width = 1920
img_height = 1080

cam_intrinsic = np.array([[fx, 0, cx],
                            [0, fy, cy],
                            [0, 0, 1]])

# use apriltag pose estimation for cam_extrinsic
def draw_camera(cam_extrinsic):
    cam = o3d.geometry.LineSet.create_camera_visualization(img_width, img_height, cam_intrinsic, cam_extrinsic)

    return cam

def main():
    camera_pose []
    for i in range(cam_poses):
        pose = draw_camera()
        camera_pose.append(pose)

    o3d.visualization.draw(camera_pose, show_ui=True)

if __name__ == "__main__":
    main()
