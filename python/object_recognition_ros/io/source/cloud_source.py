# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above
# copyright notice, this list of conditions and the following
# disclaimer in the documentation and/or other materials provided
# with the distribution.
# * Neither the name of Willow Garage, Inc. nor the names of its
# contributors may be used to endorse or promote products derivedo
# from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Module defining several outputs for the object recognition pipeline
"""

from ecto_image_pipeline.io.source import create_source
from object_recognition_core.io.source import SourceBase
from object_recognition_ros import init_ros
import ecto
import ecto_pcl_ros
import ecto_pcl
import object_recognition_ros
from object_recognition_ros.ecto_cells.io_ros  import EctoPointCloudOrganizedToMatXYZ
from ecto_pcl_ros import Message2PointCloud

from ecto_image_pipeline.io.source.camera_base import CameraType
from ecto import BlackBoxCellInfo as CellInfo, BlackBoxForward as Forward
import ecto_ros
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs

PointSub = ecto_sensor_msgs.Subscriber_PointCloud2
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo

# similar to SourceBase and Similar to OpenNISubscriber
#
# OpenNISubscriber in ecto_image_pipeline from BaseSource: connects to topics and forwards them
# BaseSource (base of OpenNISubscriber) makes the 3d using DepthTo3D but we do not this: DepthTo3D is sent to crop_box emitted as points3d (CropBox in ecto_opencv)
#
# We need the converter from PointCloud2 to points and then simply subscribe
#
# Conversions in reconstruction: MatToPointCloud XYZ/XYZRGB Organized
# Conversions in ecto_pcl: Message2PointCloud and PointCloud2Message
class CloudSource(ecto.BlackBox):
    CAMERA_TYPE = CameraType.RGBD
    def __init__(self, *args, **kwargs):
        init_ros()
        ecto.BlackBox.__init__(self, *args, **kwargs)

    @staticmethod
    def declare_cells(_p):
        qsize = 1
        cells = ecto.BlackBox.declare_cells(_p)
        subs = dict(Ccamera_points=PointSub(topic_name='/bogus_topic_image', queue_size=qsize),
                    Ccamera_info=CameraInfoSub(topic_name='/bogus_topic_image', queue_size=qsize),
                 )
        cells.update(subs)
        cells['Csource'] = ecto_ros.Synchronizer('Synchronizator', subs=subs)
        cells['Ccamera_info_image'] = CellInfo(ecto_ros.CameraInfo2Cv) 
        cells['Cmsg2points'] = Message2PointCloud("msg2cloud",format=ecto_pcl.XYZ)
        cells['Cpoints2mat'] = EctoPointCloudOrganizedToMatXYZ("points2mat")
        return cells

    @staticmethod
    def declare_forwards(_p):
        p = {}
        p['Ccamera_info'] = [Forward('topic_name', 'camera_info', 'The ROS topic for the RGB camera info.','/kinect2/sd/camera_info')]
        p['Ccamera_points']  = [Forward('topic_name', 'camera_points', 'The ROS topic for the RGB camera info.','/kinect2/sd/points')]
        i = {}
        o = {'Ccamera_info_image': [Forward('K', 'K_image', 'The camera intrinsics matrix of the image camera.')],
            "Cpoints2mat" : [Forward('points',"points3d", "The output points for OpenCV")] }
        return (p,i,o)
    def configure(self, _p, _i, _o):
        pass
    def connections(self, p):
        #ros message converters
        graph = [
          self.Csource["Ccamera_info"] >> self.Ccamera_info_image['camera_info'],
          self.Csource["Ccamera_points"] >> self.Cmsg2points["input"],
          self.Cmsg2points["output"] >> self.Cpoints2mat["point_cloud"]
        ]

        return graph