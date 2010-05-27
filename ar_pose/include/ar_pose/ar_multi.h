/*
 *  Multi Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  William Morris <morris@ee.ccny.cuny.edu>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AR_SINGLE_H
#define AR_SINGLE_H

#include <AR/gsub.h>
#include <AR/video.h>
#include <AR/param.h>
#include <AR/ar.h>
#include <AR/arMulti.h>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>

#include "ar_pose/object.h"

#define DEFAULT_CAMERA_PARAM "Data/camera_para.dat"
#define DEFAULT_PATTERN "Data/patt.hiro"

namespace ar_pose
{
class ARSinglePublisher
{
public:
   ARSinglePublisher(ros::NodeHandle &n);
   ~ARSinglePublisher(void);

private:
   void arInit ();
   void getTransformationCallback(const sensor_msgs::ImageConstPtr&);
   void camInfoCallback(const sensor_msgs::CameraInfoConstPtr&);
   	
   ros::NodeHandle n_;
   tf::TransformBroadcaster broadcaster_;
   ros::Subscriber sub_;
   image_transport::Subscriber cam_sub_;
   
   //OBJECT_MAX define in object.h
   geometry_msgs::TransformStamped transform_[OBJECT_MAX]; 
   image_transport::ImageTransport it_;
   sensor_msgs::CvBridge bridge_;
   sensor_msgs::CameraInfo cam_info_;
   
   // Camera Calibration Parameters
   ARParam cam_param_;
   
   // AR Marker Info
   ARMultiMarkerInfoT * config;
   ar_object::ObjectData_T * object;
   int objectnum;
   
   char camera_image_topic_[FILENAME_MAX];
   char camera_info_topic_[FILENAME_MAX];
   char cam_param_filename_[FILENAME_MAX];
   char pattern_filename_[FILENAME_MAX];
   // Size of the AR Marker in mm
   double marker_width_;
   // Physical Center of the Marker
   double marker_center_[2];
   // Marker Transform
   double marker_trans_[3][4];
   int xsize_, ysize_;
   int threshold_;
   int mode,contF;
   bool getCamInfo_;  
   bool knownPattern_;
   CvSize sz;
   IplImage* capture;
  
}; // end class ARSinglePublisher
} //end namespace ar_pose

#endif
