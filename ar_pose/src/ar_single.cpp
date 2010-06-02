/*
 *  Single Marker Pose Estimation using ARToolkit
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

#include "ar_pose/ar_single.h"

namespace ar_pose
{
  ARSinglePublisher::ARSinglePublisher (ros::NodeHandle & n):n_ (n), it_ (n_)
  {
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

    if (n_param.hasParam ("camera_image_topic"))
    {
      n_param.getParam ("camera_image_topic", local_path);
    }
    else
    {
      local_path = "/camera/image";
    }
    sprintf (camera_image_topic_, "%s", local_path.c_str ());
    ROS_INFO ("Camera Image Topic: %s", camera_image_topic_);

    if (n_param.hasParam ("camera_info_topic"))
    {
      n_param.getParam ("camera_info_topic", local_path);
    }
    else
    {
      local_path = "/camera/camera_info";
    }
    sprintf (camera_info_topic_, "%s", local_path.c_str ());
    ROS_INFO ("Camera Info Topic: %s", camera_info_topic_);

    if (n_param.hasParam ("cam_param"))
    {
      n_param.getParam ("cam_param", local_path);
    }
    else
    {
      local_path = "data/camera_para.dat";
    }
    sprintf (cam_param_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
    ROS_INFO ("Camera Parameter Filename: %s", cam_param_filename_);

    n_param.param ("marker_pattern", local_path, std::string ("data/patt.hiro"));
    sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
    ROS_INFO ("Marker Pattern Filename: %s", pattern_filename_);

    n_param.param ("marker_width", marker_width_, 80.0);
    ROS_INFO ("Marker Width: %.1f", marker_width_);

/*
   if (n_param.hasParam("marker_center"))
   {
      n_param.getParam("marker_center", xml_marker_center);
      for (int i = 0; i < 2; i++) {
         ROS_DEBUG("XML Parameter %s", xml_marker_center.toXml().c_str());
         marker_center_[i] = (double)(xml_marker_center[i]);
      }
   }
   else {
      marker_center_[0] = 0.0;
      marker_center_[1] = 0.0;
   }
*/
    n_param.param ("marker_center_x", marker_center_[0], 0.0);
    n_param.param ("marker_center_y", marker_center_[1], 0.0);
    ROS_INFO ("Marker Center: (%.1f,%.1f)", marker_center_[0], marker_center_[1]);

    n_param.param ("threshold", threshold_, 100);
    ROS_INFO ("Threshold %d", threshold_);

    // If mode=0, we use arGetTransMat instead of arGetTransMatCont
    // The arGetTransMatCont function uses information from the previous image
    // frame to reduce the jittering of the marker
    n_param.param ("history_mode", history_mode, 1);
    ROS_INFO ("History_mode %d", history_mode);

    ROS_INFO ("Subscribing to info topic");
    sub_ = n_.subscribe (camera_info_topic_, 1, &ARSinglePublisher::camInfoCallback, this);
    getCamInfo_ = false;
  }

  ARSinglePublisher::~ARSinglePublisher (void)
  {
    //cvReleaseImage(&capture); //Don't know why but crash when release the image
    arVideoCapStop ();
    arVideoClose ();
  }

  void ARSinglePublisher::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    if (!getCamInfo_)
    {
      cam_info_ = (*cam_info);
      xsize_ = cam_info_.width;
      ysize_ = cam_info_.height;
      ROS_INFO ("Image size (x,y) = (%d,%d)", xsize_, ysize_);
      arInit ();

      ROS_INFO ("Subscribing to image topic");
      cam_sub_ = it_.subscribe (camera_image_topic_, 1, &ARSinglePublisher::getTransformationCallback, this);
      getCamInfo_ = true;
    }
  }

  void ARSinglePublisher::arInit ()
  {
    ARParam wparam;

    // Setup the initial camera parameters
    ROS_INFO ("Loading Camera Parameters");
    if (arParamLoad (cam_param_filename_, 1, &wparam) < 0)
    {
      ROS_ERROR ("Camera parameter load error: %s", cam_param_filename_);
      ROS_BREAK ();
    }
    arParamChangeSize (&wparam, xsize_, ysize_, &cam_param_);
    arInitCparam (&cam_param_);
    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load pattern file
    ROS_INFO ("Loading pattern");
    patt_id_ = arLoadPatt (pattern_filename_);
    if (patt_id_ < 0)
    {
      ROS_ERROR ("Pattern file load error: %s", pattern_filename_);
      ROS_BREAK ();
    }

    sz_ = cvSize (xsize_, ysize_);
    capture_ = cvCreateImage (sz_, IPL_DEPTH_8U, 4);
  }

  void ARSinglePublisher::getTransformationCallback (const sensor_msgs::ImageConstPtr & image_msg)
  {
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k;

    /* Get the image from ROSTOPIC
     * NOTE: the dataPtr format is BGR because the ARToolKit library was
     * build with V4L, dataPtr format change according to the 
     * ARToolKit configure option (see config.h).*/
    try
    {
      capture_ = bridge_.imgMsgToCv (image_msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException & e)
    {
      ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str ());
    }
    //cvConvertImage(capture_,capture_,CV_CVTIMG_FLIP); //flip image
    dataPtr = (ARUint8 *) capture_->imageData;

    // detect the markers in the video frame 
    if (arDetectMarker (dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
      ROS_FATAL ("arDetectMarker failed");
      ROS_BREAK ();             // FIX: I don't think this should be fatal... -Bill
    }

    // check for known patterns
    k = -1;
    for (i = 0; i < marker_num; i++)
    {
      if (marker_info[i].id == patt_id_)
      {
        ROS_DEBUG ("Found pattern: %d ", patt_id_);

        // make sure you have the best pattern (highest confidence factor)
        if (k == -1)
          k = i;
        else if (marker_info[k].cf < marker_info[i].cf)
          k = i;
      }
    }
    if (k == -1)
      contF = 0;
    else if (k != -1)
    {
      // get the transformation between the marker and the real camera
      //double cam_trans[3][4];
      double quat[4], pos[3];

      if (history_mode == 0 || contF == 0)
      {
        arGetTransMat (&marker_info[k], marker_center_, marker_width_, marker_trans_);
      }
      else
      {
        arGetTransMatCont (&marker_info[k], marker_trans_, marker_center_, marker_width_, marker_trans_);
      }
      contF = 1;

      //arUtilMatInv (marker_trans_, cam_trans);
      arUtilMat2QuatPos (marker_trans_, quat, pos);

      ROS_DEBUG (" QUAT: Pos x: %3.1f  y: %3.1f  z: %3.1f", pos[0], pos[1], pos[2]);
      ROS_DEBUG ("     Quat qx: %3.2f qy: %3.2f qz: %3.2f qw: %3.2f", quat[0], quat[1], quat[2], quat[3]);

      transform_.header.frame_id = "usb_cam";
      transform_.child_frame_id = "ar_single";
      transform_.header.stamp = ros::Time::now ();
      transform_.transform.translation.x = pos[0] / 1000;
      transform_.transform.translation.y = pos[1] / 1000;
      transform_.transform.translation.z = pos[2] / 1000;

      transform_.transform.rotation.x = quat[0];
      transform_.transform.rotation.y = quat[1];
      transform_.transform.rotation.z = quat[2];
      transform_.transform.rotation.w = quat[3];

      broadcaster_.sendTransform (transform_);
      ROS_DEBUG ("ar_single tf published");
    }
    else
    {
      ROS_DEBUG ("Failed to locate marker");
    }
  }
}                               // end namespace ar_pose

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_single");
  ros::NodeHandle n;
  ar_pose::ARSinglePublisher ar_single (n);
  ros::spin ();
  return 0;
}
