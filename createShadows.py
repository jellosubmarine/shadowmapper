import cloudComPy as cc                                               
import cloudComPy.PCV
import numpy as np
import open3d as o3d
print("Started")
cloud = cc.loadPointCloud("moon_DS.ply")           
print("cloud name: %s"%cloud.getName())

# cloud = cc.CloudSamplingTools.noiseFilter(cloud, 1,1,removeIsolatedPoints=True).getAssociatedCloud()
# print("Filtered")
mesh = cc.ccMesh.triangulate(cloud,cc.TRIANGULATION_TYPES.DELAUNAY_2D_AXIS_ALIGNED,updateNormals=True)
print("Created mesh")
mesh.laplacianSmooth()
print("Smoothened")

# cc.SaveMesh(mesh, "mesh.ply")

initialAngle = (0, 0.85, -0.5)

rotSmall = cc.ccGLMatrix.FromToRotation((0,1,0), (-0.31622776601683794,0.9486832980505138,0))

transMat = cc.ccGLMatrix.FromToRotation((0,0,1), initialAngle) #must not come from under plane
plane = cloudComPy.ccPlane(5,5, transMat).samplePoints(False, 2)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cc.ccPointCloud.toNpArray(cloud))
o3d.io.write_point_cloud("points.ply", pcd)
print("Wrote out points")
for i in range(18):
  cc.PCV.computeShadeVIS([cloud], cloudWithNormals=plane)
  scalfield = cc.ccPointCloud.getScalarField(cloud, cloud.getNumberOfScalarFields()-1)
  np.save("shadows/shadows"+str(i), cc.ScalarField.toNpArray(scalfield))
  cc.ccPointCloud.applyRigidTransformation(plane, rotSmall)
  print("Wrote ", i)
