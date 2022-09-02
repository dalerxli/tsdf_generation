from mesh_to_sdf import sample_sdf_near_surface, get_surface_point_cloud, mesh_to_voxels
# import sys
# sys.path.insert(1, '/home/nleuze/vgn_grasp/src/vgn/src/vgn')
# import perception



import trimesh
import skimage
import pyrender
import numpy as np
import open3d as o3d

'''mesh = o3d.io.read_triangle_mesh('/home/nleuze/generate_tsdf/data/hope_meshes_eval/Ketchup.obj')
print(mesh)
print("Try to render a mesh with normals (exist: " +
          str(mesh.has_vertex_normals()) + ") and colors (exist: " +
          str(mesh.has_vertex_colors()) + ")")
# o3d.visualization.draw_geometries([mesh])
# print("A mesh with no normals and no colors does not seem good.")

print("Computing normal and rendering it.")
mesh.compute_vertex_normals()
triangle_normals = np.asarray(mesh.triangle_normals)
print(triangle_normals, triangle_normals.shape)
o3d.visualization.draw_geometries([mesh])'''



mesh = trimesh.load('/home/nleuze/generate_tsdf/data/hope_meshes_eval/Ketchup.obj')
print(mesh)

# points, sdf = sample_sdf_near_surface(mesh, number_of_points=64000, surface_point_method='scan')

# print('Points: ', points.shape, 'SDF: ', sdf.shape)
# print(points[1,:])

# pc = get_surface_point_cloud(mesh)
# print('pointcloud: ', pc)

# colors = np.zeros(points.shape)
# colors[sdf < 0, 2] = 1
# colors[sdf > 0, 0] = 1
# cloud = pyrender.Mesh.from_points(points, colors=colors)
# scene = pyrender.Scene()
# scene.add(cloud)
# viewer = pyrender.Viewer(scene, use_raymond_lighting=True, point_size=2)


################
# Voxelize Mesh:
################

voxels = mesh_to_voxels(mesh, 40, pad=False, surface_point_method='scan', check_result=True, scan_count=100)
voxels = np.asarray(voxels)
np.savez('voxels.npz', voxels)

voxels = (voxels + 1) * 0.5
np.expand_dims(voxels, axis=0)
print(voxels.shape)


###############################
# Initialize RVIZ Visualization
###############################
'''def draw_workspace(size):
    scale = size * 0.005
    pose = Transform.identity()
    scale = [scale, 0.0, 0.0]
    color = [0.5, 0.5, 0.5]
    msg = _create_marker_msg(Marker.LINE_LIST, "task", pose, scale, color)
    msg.points = [ros_utils.to_point_msg(point) for point in workspace_lines(size)]
    pubs["workspace"].publish(msg)'''





