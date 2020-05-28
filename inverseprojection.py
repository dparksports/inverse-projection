import open3d as o3d
import cv2
import matplotlib.pyplot as plt
import scipy.linalg as lin
import numpy as np

def orthogonal_view(points_in_camera_coordinates):
    longitunal_max = 70
    x_window = (-50, 50)
    y_window = (-3, longitunal_max)

    x, y, z = points_in_camera_coordinates
    y = -1 * y

    # In an ego vehicle, the observing camera is placed 1 meter of the roof top.
    points_in_70meters = np.where((y > -1.2))
#    points_in_70meters = np.where(z < longitunal_max)
#    points_in_70meters = np.where((z < longitunal_max) & (y > -1.2))
    z_based_view = points_in_camera_coordinates[:3, points_in_70meters]
#    z_based_view = points_in_camera_coordinates[:3]

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


a = np.arange(9).reshape((3, 3))
print(a)
#print(np.where(a < 4, -1, 100))

d = a[:3, np.where(a[0] < 4)]
print(d)

b = a[:, np.where(a[1] < 4)]
print(b)

c = a[:, np.where(a[1] < 4)[0]]
print(c)

a = np.arange(10)
print(a)
a = np.where(a < 5, a, 10*a)
print(a)

a = np.where([[True, False], [True, True]],
         [[1, 2], [3, 4]],
         [[9, 8], [7, 6]])
print(a)

x, y = np.ogrid[:3, :4]
print(x)
print(y)
print(np.where(x < y, x + 1000, 100 + y)) # both x and 10+y are broadcast

a = np.array([[0, 1, 2],
              [0, 2, 4],
              [0, 3, 6]])
print(a)
print(np.where(a < 4, a, -1))  # -1 is broadcast

a = np.arange(30).reshape((3, 10))
b = np.arange(10)
c = a * b  # element wise multiplication or dot
print(a)
print(b)
print(c)

m = np.arange(9).reshape((3,3))
d = m @ a  # matrix multiplication
print(m)
print(d)

rgb = cv2.cvtColor(cv2.imread('rgb.png'), cv2.COLOR_BGR2RGB)
depth = cv2.imread('depth.exr', cv2.IMREAD_ANYDEPTH)

print("rgb:", rgb.shape, rgb)
print("depth:", depth.shape, depth)

h, w, _ = rgb.shape
K = intrinsic_from_fov(h, w, 90)
K_inverse = np.linalg.inv(K)

print("K:", K.shape, K)
print("K_inverse:", K_inverse.shape, K_inverse)

pixels_in_image_coordinates = homogenous_image_coordinates(h, w)
c = pixels_in_image_coordinates[1]
c = c.reshape(768, 1024)
print("c:", c)

c = np.asarray(rgb)
print("c.shape:", c.shape)
c1 = c[:,:,0].reshape(768 * 1024)
print("c1.shape:", c1.shape)
print("c1:", c1)

c = np.asarray(depth)
print("c.shape:", c.shape)
c1 = c[0].reshape(32,32)
np.set_printoptions(precision=0, suppress=True)
np. set_printoptions(threshold=np. inf)
print("c1.shape:", c1.shape)
print("c1:", c1)

print("pixels_in_image_coordinates:", pixels_in_image_coordinates.shape, pixels_in_image_coordinates)
print("depth.flatten():", depth.flatten().shape, depth.flatten())

# @ - matrix multiplication
points_in_camera_coordinates = K_inverse[:, :] @ pixels_in_image_coordinates
print("points_in_camera_coordinates:", points_in_camera_coordinates.shape, points_in_camera_coordinates)

# * - elementwise multiplication
points_in_camera_coordinates = points_in_camera_coordinates * depth.flatten()
print("points_in_camera_coordinates:", points_in_camera_coordinates.shape, points_in_camera_coordinates)

# @ - matrix multiplication
points_in_camera_coordinates = K_inverse[:3, :3] @ pixels_in_image_coordinates * depth.flatten()
print("points_in_camera_coordinates:", points_in_camera_coordinates.shape, points_in_camera_coordinates)

print("np.where(points_in_camera_coordinates[2] < 150)[0]:", np.where(points_in_camera_coordinates[2] < 150)[0].shape, np.where(points_in_camera_coordinates[2] < 150)[0])

points_in_camera_coordinates = points_in_camera_coordinates[:, np.where(points_in_camera_coordinates[2] < 1000)[0]]
print("points_in_camera_coordinates:", points_in_camera_coordinates.shape, points_in_camera_coordinates)

pointcloud_in_camera_coordinates = o3d.geometry.PointCloud()
pointcloud_in_camera_coordinates.points = o3d.utility.Vector3dVector(points_in_camera_coordinates.T[:, :3])
pointcloud_in_camera_coordinates.transform([[1,0,0,0],
                                            [0,-1,0,0],
                                            [0,0,-1,0],
                                            [0,0,0,1]])
o3d.visualization.draw_geometries([pointcloud_in_camera_coordinates])

orthogonal_view(points_in_camera_coordinates)




