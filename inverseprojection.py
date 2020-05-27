import open3d as o3d
import cv2
import matplotlib.pyplot as plt
import scipy.linalg as lin
import numpy as np

def orthogonal_view(camera_points):
    longitunal_max = 70
    x_window = (-50, 50)
    y_window = (-3, longitunal_max)

    x, y, z = camera_points
    y = -1 * y

    # In an ego vehicle, the observing camera is placed 1 meter top of the roof top.
    points_with_70meters = np.where((z < longitunal_max) & (y > -1.2))
    z_based_view = camera_points[:3, points_with_70meters]

    # color points by radial distance
    distances = np.sqrt(np.sum(z_based_view[0:2:2, :] ** 2, axis=0))
    axex_limit = 10
    color_array = np.minimum(1, distances / axex_limit / np.sqrt(2))

    fig, axes = plt.subplots(figsize=(12, 12))
    axes.scatter(z_based_view[0, :], z_based_view[2, :], c=color_array, s=0.1)
    axes.set_xlim(x_window)
    axes.set_ylim(y_window)

    plt.axis('off')
    plt.gca().set_aspect('equal')
    plt.show()


def homogenous_image_coordinates(h, w):
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

def intrinsic_from_fov(h, w, fov=90):
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

orthogonal_view(camera_coordinates)




