import numpy as np
import open3d as o3d
import trimesh
# FUNCTION:
#    Converts open3d TriangleMesh to trimesh Trimesh and vise versa
# INPUT:
#    mesh (open3d TriangleMesh/trimesh Trimesh)
# OUTPUT:
#    return (trimesh Trimesh/open3d TriangleMesh)

def convert(mesh):
    if isinstance(mesh, o3d.cpu.pybind.geometry.TriangleMesh):
        return trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))
    else:
        mesh = tmesh.as_open3d
        mesh.compute_vertex_normals()
        return mesh
    
def convert2open3d(tmesh):
    if isinstance(tmesh, o3d.cpu.pybind.geometry.TriangleMesh):
        return tmesh
    else:
        mesh = tmesh.as_open3d
        mesh.compute_vertex_normals()
        return mesh
    
def convert2trimesh(mesh):
    if isinstance(mesh, o3d.cpu.pybind.geometry.TriangleMesh):
        return trimesh.Trimesh(np.asarray(mesh.vertices), np.asarray(mesh.triangles), vertex_normals=np.asarray(mesh.vertex_normals))
    else:
        return tmesh

def make_mesh(cloud, alpha = 0.2, log = False, asTrimesh = False):
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(cloud, alpha)       
    mesh.compute_vertex_normals()    
    tmesh = convert2trimesh(mesh)
    trimesh.repair.fix_normals(tmesh)
    trimesh.repair.fill_holes(tmesh)
    if asTrimesh:
        return tmesh
    else:
        return tmesh.as_open3d
    
def smooth(mesh, smooth_lamb = 0.5, smooth_iterations = 10, log = False, asTrimesh = False):
    tmesh = convert2trimesh(mesh)
    tmesh = trimesh.smoothing.filter_laplacian(tmesh, lamb=smooth_lamb, iterations=smooth_iterations, implicit_time_integration=False, volume_constraint=True, laplacian_operator=None)
    
    if asTrimesh:
        return tmesh
    else:
        return tmesh.as_open3d
    
# FUNCTION converts an open3d PointCloud into an open3d mese
# INPUT:
#   - cloud (open3d PointCloud): point cloud
#   - smoothing (boolean): whether or not to aplly laplacian smoothing to the mesh
#   - alpha (float): alpha parameter of the meshing algorithm
#   - smooth_lamb (float): lambda parameter of the laplacian smoothing algorithm
#   - smooth_iterations (float): iterations parameter of the laplacian smoothing algorithm    
def make_smooth_mesh(cloud, alpha = 0.2, smooth_lamb = 0.5, smooth_iterations = 10, log = False, asTrimesh = False):    
    return smooth(make_mesh(cloud, alpha, log = log, asTrimesh = False), smooth_lamb, smooth_iterations, log = log, asTrimesh = asTrimesh)