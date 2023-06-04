import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import time


pointcloud = o3d.io.read_point_cloud("/home/mrac/libish/06_03_16_24.ply")
pcd = pointcloud.voxel_down_sample(voxel_size=0.01)
pcd.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
o3d.visualization.draw_geometries([pcd])

# Segment plane
plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f'Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')
inlier_cloud = pcd.select_by_index(inliers)
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud = pcd.select_by_index(inliers, invert=True)
o3d.visualization.draw_geometries([pcd])


# Load a point cloud
pcd = o3d.io.read_point_cloud("path/to/point_cloud.pcd")

# Create a visualization window
vis = o3d.visualization.Visualizer()
vis.create_window()

# Add the point cloud to the visualization window
vis.add_geometry(pcd)

# Animation loop
num_frames = 100
for i in range(num_frames):
    # Rotate the point cloud around the vertical axis (Y-axis)
    rotation = pcd.get_rotation_matrix_from_axis_angle(
        (0, 1, 0), np.pi / num_frames)
    pcd.rotate(rotation, center=pcd.get_center())

    # Update the visualization
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

    # Add a time delay for each frame
    time.sleep(0.02)

# Close the visualization window
vis.destroy_window()


# # Segment cylinder
# cylinder_model, inliers = pcd.segment_cylinder(ransac_n=50,
#                                                 distance_threshold=0.05,
#                                                 radius=0.1)
#                                                 [a, b, c, d, e, f] = cylinder_model
# print(f'Cylinder equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0')

# inlier_cloud = pcd.select_by_index(inliers)
# inlier_cloud.paint_uniform_color([1.0, 0, 0])
# outlier_cloud = pcd.select_by_index(inliers, invert=True)
# o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],
#                                     zoom=0.8,
#                                     front=[-0.4999, -0.1659, -0.8499],
#                                     lookat=[2.1813, 2.0619, 2.0999],
#                                     up=[0.1204, -0.9852, 0.1215])
