#!/usr/bin/env python3
# 2021/03/17 hato #!/usr/bin/env python -> !/usr/bin/env python3

# Python includes
import numpy

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Vector3,Transform
from tf import transformations
from rovi_utils import tflib
from rviz_tools_py import rviz_tools

# Initialize the ROS Node
rospy.init_node('marker_ycam', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    # 2021/03/16 hato ------------------------------ start ------------------------------
    # print "Shutting down node"
    print("Shutting down node")
    # 2021/03/16 hato ------------------------------  end  ------------------------------
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('camera', 'ycam_marker')
T1 = transformations.translation_matrix((0,0,0))
sc1 = Vector3(1,1,1)
tr2=Transform()
tr2.translation.z=530
tr2.rotation.w=1.
T2=tflib.toRT(tr2)
sc2 = Vector3(400,300,300)

while not rospy.is_shutdown():
  mesh_file1 = "package://rovi_industrial/mesh/YCAM3.stl"
  markers.publishMesh(T1,mesh_file1,'white', sc1, 0.5)
  markers.publishCube(T2,(0.1,0.1,0.1,0.05), sc2, 0.5)
  rospy.Rate(5).sleep() #5Hz
