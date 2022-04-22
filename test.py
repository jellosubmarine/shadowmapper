import cloudComPy as cc                                               
import cloudComPy.PCV
import numpy as np
import open3d as o3d

cloud = cc.loadPointCloud("/home/madis/Downloads/tykk.las")           
print("cloud name: %s"%cloud.getName())

filteredCloud = cc.CloudSamplingTools.noiseFilter(cloud, 1,1,removeIsolatedPoints=True).getAssociatedCloud()

mesh = cc.ccMesh.triangulate(filteredCloud,cc.TRIANGULATION_TYPES.DELAUNAY_2D_AXIS_ALIGNED,updateNormals=True)

mesh.laplacianSmooth()

cc.SaveMesh(mesh, "mesh.ply")

vec1 = (0, 0.85, -0.5)

trans90 = cc.ccGLMatrix.FromToRotation((0,1,0), (-1,0,0))

rotSmall = cc.ccGLMatrix.FromToRotation((0,1,0), (-0.31622776601683794,0.9486832980505138,0))


transMat = cc.ccGLMatrix.FromToRotation((0,0,1), vec1) #must not come from under plane
plane = cloudComPy.ccPlane(5,5, transMat).samplePoints(False, 10)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(cc.ccPointCloud.toNpArray(filteredCloud))
o3d.io.write_point_cloud("points.ply", pcd)

for i in range(18):
  cc.PCV.computeShadeVIS([mesh], cloudWithNormals=plane)
  scalfield = cc.ccPointCloud.getScalarField(filteredCloud, filteredCloud.getNumberOfScalarFields()-1)
  np.save("shadows"+str(i), cc.ScalarField.toNpArray(scalfield))
  cc.ccPointCloud.applyRigidTransformation(plane, rotSmall)


# # for i in range(filteredCloud.getNumberOfScalarFields()):
# #   print(filteredCloud.getScalarFieldName(i))
  
# scalfield = cc.ccPointCloud.getScalarField(filteredCloud, filteredCloud.getNumberOfScalarFields()-1)
# # print(cc.ScalarField.computeMeanAndVariance(scalfield))
# # cc.SavePointCloud(filteredCloud,"illu.las")
# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(cc.ccPointCloud.toNpArray(filteredCloud))
# o3d.io.write_point_cloud("test.ply", pcd)

# np.save("illu", cc.ScalarField.toNpArray(scalfield))






# cc.ccPointCloud.applyRigidTransformation(plane, trans90)
# cc.PCV.computeShadeVIS([mesh], cloudWithNormals=plane)
# # cc.SavePointCloud(filteredCloud,"illu1.las")
# np.save("illu1", cc.ScalarField.toNpArray(scalfield))
