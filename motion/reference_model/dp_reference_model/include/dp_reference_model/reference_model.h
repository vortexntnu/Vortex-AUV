/*   Written by Jae Hyeong Hwang
     Copyright (c) 2021 Manta AUV, Vortex NTNU.
     All rights reserved. */
#ifndef DP_REFERENCE_MODEL_H
#define DP_REFERENCE_MODEL_H

#include <Eigen/Dense>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "eigen_conversions/eigen_msg.h"
#include "tf/tf.h"
#include "tf_conversions/tf_eigen.h"
using namespace Eigen;

class ReferenceModel {   
public:
     ReferenceModel(ros::NodeHandle nh);

     ros::Subscriber setpoint_sub;
     ros::Publisher reference_pub;

private:
     void setpoint_cb(const geometry_msgs::Pose&msg);

     Eigen::Vector3d calculate_smooth(const Eigen::Vector3d &x_ref);
     void reset(Eigen::Vector3d pos);

     Eigen::Vector3d x_d_prev;         /** Previous desired body position            */
     Eigen::Vector3d x_d_prev_prev;    /** Previous previous desired body position   */
     Eigen::Vector3d x_ref_prev;       /** Previous reference body position          */
     Eigen::Vector3d x_ref_prev_prev;  /** Previous previous reference body position */

     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // DP_REFERENCE_MODEL_H