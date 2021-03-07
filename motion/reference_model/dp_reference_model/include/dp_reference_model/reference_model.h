/**
 * @file
 * 
 * Written by Jae Hyeong Hwang
 * Modified by Ø. Solbø
 * Copyright (c) 2021 Manta AUV, Vortex NTNU.
 * All rights reserved.
 * 
 * 
 * Module connecting the DP-controller to the joystick_guidance and 
 * the dp_guidance
 */

/*    */
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
private:
     Eigen::Vector3d x_d_prev;         /** Previous desired body position            */
     Eigen::Vector3d x_d_prev_prev;    /** Previous previous desired body position   */
     Eigen::Vector3d x_ref_prev;       /** Previous reference body position          */
     Eigen::Vector3d x_ref_prev_prev;  /** Previous previous reference body position */

     /**
      * @brief Callback-function calculating desired position and orientation 
      * for publishing to the DP-controller
      * 
      * @param msg Reference Pose
      */
     void setpoint_cb(const geometry_msgs::Pose& msg);


     /**
      * @brief Utility function that calculates a smooth trajectory from current
      * position to the desired position @p x_ref
      * 
      * @param x_ref Reference used 
      */
     Eigen::Vector3d calculate_smooth(const Eigen::Vector3d &x_ref);

     /**
      * @brief Function that resets the private variables to @p pos
      */
     void reset(Eigen::Vector3d pos);


     EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
     ReferenceModel(ros::NodeHandle nh);

     ros::Subscriber setpoint_sub;
     ros::Publisher reference_pub;

};

#endif  // DP_REFERENCE_MODEL_H