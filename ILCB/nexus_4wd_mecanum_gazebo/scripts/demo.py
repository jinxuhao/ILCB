#!/usr/bin/env python

# Copyright (c) 2015, Carnegie Mellon University
# All rights reserved.
# Authors: David Butterworth <dbworth@cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
This is a demo of Rviz Tools for python which tests all of the
available functions by publishing lots of Markers in Rviz.
"""

# Python includes
import numpy
import random

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Polygon
from tf import transformations # rotation_matrix(), concatenate_matrices()

import rviz_tools_py as rviz_tools
from cbf_test import x_center_0, y_center_0


# Initialize the ROS Node
rospy.init_node('test', anonymous=False, log_level=rospy.INFO, disable_signals=False)

# Define exit handler
def cleanup_node():
    print "Shutting down node"
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)


markers = rviz_tools.RvizMarkers('/odom', 'visualization_marker')


while not rospy.is_shutdown():

    # Line:

    # Publish a line between two ROS Point Msgs
    point1 = Point(-2,1,0)
    point2 = Point(2,1,0) 
    width = 0.05
    markers.publishLine(point1, point2, 'green', width, 5.0) # point1, point2, color, width, lifetime

    # Publish a line between two ROS Poses
    P1 = Pose(Point(-2,1.1,0),Quaternion(0,0,0,1))
    P2 = Pose(Point(2,1.1,0),Quaternion(0,0,0,1))
    width = 0.02
    markers.publishLine(P1, P2, 'red', width, 5.0) # point1, point2, color, width, lifetime

    # Publish a line between two numpy transform matrices
    T1 = transformations.translation_matrix((-2,1.2,0))
    T2 = transformations.translation_matrix((2,1.2,0))
    width = 0.02
    markers.publishLine(T1, T2, 'blue', width, 5.0) # point1, point2, color, width, lifetime


    # Path:

    # Publish a path using a list of ROS Point Msgs
    path = []
    path.append( Point(0,-0.5,0) )
    path.append( Point(1,-0.5,0) )
    path.append( Point(1.5,-0.2,0) )
    path.append( Point(2,-0.5,0) )
    path.append( Point(2.5,-0.2,0) )
    path.append( Point(3,-0.5,0) )
    path.append( Point(4,-0.5,0) )
    width = 0.02
    markers.publishPath(path, 'orange', width, 5.0) # path, color, width, lifetime



    # Sphere:

    # Publish a sphere using a numpy transform matrix
    T = transformations.translation_matrix((-3,3.2,0))
    scale = Vector3(0.5,0.5,0.5) # diameter
    color = [0,1,0] # list of RGB values (green)
    markers.publishSphere(T, color, scale, 5.0) # pose, color, scale, lifetime

    # Publish a sphere using a ROS Pose
    P = Pose(Point(-2,3.2,0),Quaternion(0,0,0,1))
    scale = Vector3(0.6,0.6,0.6) # diameter
    color = (0,0,1) # tuple of RGB values (blue)
    markers.publishSphere(P, color, scale, 5.0) # pose, color, scale, lifetime

    # Publish a sphere using a ROS Point
    point = Point(x_center_0,y_center_0,0)
    scale = Vector3(0.7,0.7,0.7) # diameter
    color = 'orange'
    markers.publishSphere(point, color, scale, 5.0) # pose, color, scale, lifetime

    # Publish a sphere by passing diameter as a float
    point = Point(0,3.2,0)
    diameter = 0.8
    markers.publishSphere(point, 'yellow', diameter, 5.0) # pose, color, diameter, lifetime

    # Publish a sphere with higher render quality (this is one sphere in a SPHERE_LIST)
    point = Point(1,3.2,0)
    diameter = 0.9
    markers.publishSphere2(point, 'brown', diameter, 5.0) # pose, color, scale, lifetime



    # Model mesh:

    # Publish STL mesh of box, colored green
    T = transformations.translation_matrix((3,1,0))
    scale = Vector3(1.5,1.5,1.5)
    mesh_file1 = "package://rviz_tools_py/meshes/box_mesh.stl"
    markers.publishMesh(T, mesh_file1, 'lime_green', scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime

    # Display STL mesh of bottle, re-scaled to smaller size
    P = Pose(Point(4,1,0),Quaternion(0,0,0,1))
    scale = Vector3(0.6,0.6,0.6)
    mesh_file2 = "package://rviz_tools_py/meshes/fuze_bottle_collision.stl"
    markers.publishMesh(P, mesh_file2, 'blue', scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime

    # Display collada model with original texture (no coloring)
    P = Pose(Point(5,1,0),Quaternion(0,0,0,1))
    mesh_file3 = "package://rviz_tools_py/meshes/fuze_bottle_visual.dae"
    mesh_scale = 4.0
    markers.publishMesh(P, mesh_file3, None, mesh_scale, 5.0) # pose, mesh_file_name, color, mesh_scale, lifetime


    rospy.Rate(1).sleep() #1 Hz
