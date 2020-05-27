import open3d as o3d
import cv2
import matplotlib
import scipy.linalg as lin
import numpy as np

def homogenous_image_coordinates(h, w) :
    """
    image coordinates in homogenous coordinate
    :param h:
    :param w:
    :return: image coordinate: [3, w * h]
    """
    x = np.linspace(0, w - 1, w).astype(np.int)
    y = np.linspace(0, h - 1, h).astype(np.int)
    [x, y] = np.meshgrid(x, y)
    return np.vstack((x.flatten(), y.flatten(), np.ones_like(x.flatten())))

def intrinsic_from_fov(h, w, fov=90) :
    px, py = w / 2.0, h / 2.0
    fov_h = fov / 360.0 * 2.0 * np.pi
    focal_length_x = w / (2.0 * np.tan (fov_h / 2.0))

    fov_v = 2.0 * np.arctan(np.tan(fov_h / 2.0) * (h / w))
    focal_length_y = h / (2.0 * np.tan(fov_v / 2.0))

    return np.array([[focal_length_x, 0, px],
                     [0, focal_length_y, py],
                     [0, 0, 1]])


rgb = cv2.cvtColor(cv2.imread('rgb.png'), cv2.COLOR_BGR2RGB)
depth = cv2.imread('depth.exr', cv2.IMREAD_ANYDEPTH)
h, w, _ = rgb.shape
K = intrinsic_from_fov(h, w, 90)
K_inverse = np.linalg.inv(K)
image_coordinates = homogenous_image_coordinates(h, w)
camera_coordinates = K_inverse[:3, :3] @ image_coordinates * depth.flatten()
camera_coordinates = camera_coordinates[:, np.where(camera_coordinates[2] < 150)[0]]

pointcloud_in_camera_coordinates = o3d.geometry.PointCloud()
pointcloud_in_camera_coordinates.points = o3d.utility.Vector3dVector(camera_coordinates.T[:, :3])
o3d.visualization.draw_geometries([pointcloud_in_camera_coordinates])




