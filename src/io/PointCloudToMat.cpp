/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <fstream>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>
#include <ecto/ecto.hpp>
#include <ecto_pcl/ecto_pcl.hpp>
#include <ecto_pcl/pcl_cell.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "conversions.hpp"

using ecto::tendrils;

namespace object_recognition_core
{
  namespace conversion
  {
    #if 0
    /** Ecto implementation of a module that takes a point cloud as
        an input and stacks it in a matrix of floats:
        - if the point cloud is organized, the return
          a matrix is width by height with 3 channels (for x, y and z)
        - if the point cloud is unorganized, the
          return a matrix is n_point by 1 with 3 channels (for x, y and z)
     */
    struct PointCloudToMatXYZ
    {
      // Get the original keypoints and point cloud
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> CloudType;
      typedef CloudType::ConstPtr CloudOutT;


      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare(&PointCloudToMatXYZ::points3d_out, "points",
                       "The width by height by 3 channels (x, y and z)");
        inputs.declare(&PointCloudToMatXYZ::cloud_in, "point_cloud",
                        "The XYZ point cloud");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        CloudType::Ptr point_cloud(new CloudType);
        cvToCloud(*points3d, *point_cloud);
        *cloud_out = point_cloud;
        return ecto::OK;
      }
      ecto::spore<cv::Mat> points3d_out;
      ecto::spore<CloudOutT> cloud_in;
    };
    #endif
    struct PointCloudOrganizedToMatXYZ
    {
      // Get the original keypoints and point cloud
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> CloudType;
      typedef CloudType::ConstPtr CloudInT;

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare(&PointCloudOrganizedToMatXYZ::points3d_out, "points", "The width by height by 3 channels (x, y and z)");
        inputs.declare(&PointCloudOrganizedToMatXYZ::cloud_in, "point_cloud", "The XYZ point cloud");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        image_pipeline::cvFromCloudOrganized(**cloud_in,*points3d_out);
        return ecto::OK;
      }
      ecto::spore<cv::Mat> points3d_out;
      ecto::spore<CloudInT> cloud_in;
    };

    struct EctoPointCloudOrganizedToMatXYZ
    {
      typedef pcl::PointXYZ PointType;
      typedef pcl::PointCloud<PointType> CloudType;

      static void
      declare_io(const tendrils& params, tendrils& inputs, tendrils& outputs)
      {
        outputs.declare(&EctoPointCloudOrganizedToMatXYZ::points3d_out, "points", "The width by height by 3 channels (x, y and z)");
        inputs.declare(&EctoPointCloudOrganizedToMatXYZ::cloud_in, "point_cloud", "The XYZ point cloud");
      }

      int
      process(const tendrils& inputs, const tendrils& outputs)
      {
        image_pipeline::cvFromCloudOrganized(*(*cloud_in).cast<CloudType>(),*points3d_out);
        return ecto::OK;
      }
      ecto::spore<cv::Mat> points3d_out;
      ecto::spore<ecto::pcl::PointCloud> cloud_in;
    };    
}
}
ECTO_CELL(io_ros, object_recognition_core::conversion::PointCloudOrganizedToMatXYZ, "PointCloudOrganizedToMatXYZ", "Given a pcl::PointCloud<pcl::PointXYZ> produces");

ECTO_CELL(io_ros, object_recognition_core::conversion::EctoPointCloudOrganizedToMatXYZ, "EctoPointCloudOrganizedToMatXYZ", "Given a ecto::pcl::PointCloud produces");
//ECTO_CELL(io_ros, image_pipeline::conversion::PointCloudToMatXYZ, "PointCloudToMatXYZ",
//        "Given a pcl::PointCloud<pcl::PointXYZ> produces");
