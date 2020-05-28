### inverse projection

## With FOV Horizontal and Vertical

# Camera Spec
Given: 
- Primesense Carmine 1.09 Depth Camera
- VGA : 640 x 480
- Horizontal FOV: 57.5 degrees
- Vertical FOV: 45 degrees

# Focal Length
- Focal_Length_x = (width / 2) / (tan (a_x / 2.0)  // a_x = FOV_H
- Focal_length_y = (height / 2) / (tan (a_y / 2.0)  // a_y = FOV_V

[fx, s, a_x]
[0, fy, a_y]
[0, 0, 1   ]

|fx, s, a_x|                            (1)
|0, fy, a_y|
|0, 0, 1   |


# Sample K
[583 0 320]
[0 579 240]
[0 0   1  ]


## With a single FOV value:

- hFov = fov / 360 * 2 * PI
- fx = w / (2 * tan (hFov / 2))

- vFov = 2 * arctan( tan(hFov / 2) * (h/w) )
- fy = h / (2 * tan (vFov / 2))

## Results

PointCloud 1 | PointCloud 2
------------ | -------------
![cell 1](https://github.com/dparksports/inverse-projection/blob/master/pointcloud.png) | ![cell 2](https://github.com/dparksports/inverse-projection/blob/master/cloudpoint2.png)


Orthogonal (> -1m ego) | Orthogonal (70m z)
------------ | -------------
![cell 1](https://github.com/dparksports/inverse-projection/blob/master/orthogonal2.png) | ![cell 2](https://github.com/dparksports/inverse-projection/blob/master/orthogonal-no-filter.png)


CARLA 
------------ 
![cell 2](https://github.com/dparksports/inverse-projection/blob/master/rgb.png)



