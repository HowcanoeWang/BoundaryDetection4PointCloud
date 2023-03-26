import numpy as np
import open3d as o3d
from tqdm import tqdm

"""
@Params : points : numpy array
          boundary : numpy array
          raidus : float
          min_z  : float
          high_z : float
@Return  : pointcloud data
"""
def remove_cylinder(points, boundary, radius, min_z, high_z) :
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    idx = np.arange(len(pc.points))

    qbar = tqdm(range(boundary.shape[0]), "Remove Cylinder")
    for i in qbar:
        z = np.arange(min_z, high_z, radius * 0.7)
        pillar_points = np.tile(boundary[i], (z.shape[0], 1))

        pillar_points = np.hstack((pillar_points, z.reshape(z.shape[0], 1)))
        pillar_pc = o3d.geometry.PointCloud()
        pillar_pc.points = o3d.utility.Vector3dVector(pillar_points)

        dists = np.asarray(pc.compute_point_cloud_distance(pillar_pc))
        idx = idx[dists > radius]
        ind = np.where(dists > radius)[0]
        pc = pc.select_by_index(ind)
    return pc, idx

"""
@Params : points : numpy array
          boundary : numpy array
@Return  : numpy array
"""
def remove_wall(points, boundary, method="edge", radius=0.5, box_size=0.5, min_z=None, high_z=None) :
    if method == "edge" :
        #use edge algorithm
        pass
    if method == "box" :
        #use box algorithm
        pass
    if method == "cylinder" :
        result_wall, result_bound = remove_cylinder(points, boundary, radius=radius, min_z=min_z, high_z=high_z)

    return result_wall, result_bound