/*!********************************************************************************
 * \brief     self_localize_with_ekf_sensor_fusion implementation
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/
 
#ifndef REFINE_H
#define REFINE_H

// System
#include <iostream>
#include <string>
#include <math.h>
#include <mutex>
#include <binders.h>
#include <thread>
// ROS
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PointStamped.h"
#include "droneMsgsROS/dronePose.h"
#include "droneMsgsROS/droneSpeeds.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Float32.h"
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include "eigen3/Eigen/Eigen"
#include "eigen_conversions/eigen_msg.h"
//tf
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//ros time synchronizer
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//dynamic reconfigure
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <pluginlib/class_list_macros.h>

// Aerostack libraries
#include <behavior_execution_controller.h>

namespace multi_sensor_fusion
{
class BehaviorSelfLocalizeWithEKFSensorFusion : public BehaviorExecutionController
{
  // Constructor
public:
  BehaviorSelfLocalizeWithEKFSensorFusion();
  ~BehaviorSelfLocalizeWithEKFSensorFusion();

private:
  // BehaviorExecutionController
  void onConfigure();
  void onActivate();
  void onDeactivate();
  void onExecute();
  bool checkSituation();
  void checkGoal();
  void checkProgress();
  void checkProcesses();

private:
    ros::NodeHandle n;
    std::string nspace; 
    //Subscibers
    ros::Subscriber speed_sub;
    void speedCallback(const geometry_msgs::TwistStamped& msg);
    ros::Subscriber imu_sub;
    void imuCallback(const sensor_msgs::Imu& msg);
    ros::Subscriber altitude_sub;
    void altitudeCallback(const geometry_msgs::PointStamped& msg);
    ros::Subscriber msf_odom_sub_;
    void msfOdomCallback(const nav_msgs::Odometry& msg);
    ros::Subscriber position_sub;
    void positionCallback(const geometry_msgs::PointStamped& msg);

    //Publishers
    ros::Publisher velocity_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher altitude_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher msf_odom_pub_;
    ros::Publisher aerostack_pose_pub_;
    ros::Publisher aerostack_speed_pub_;

    ros::Publisher pose_deprecated;
    ros::Publisher speed_deprecated;
    tf::TransformListener listener_;

    inline void publishMSFOdom();
    inline void setMSFOdom(nav_msgs::Odometry odom);
    inline void getMSFOdom(nav_msgs::Odometry& odom);

    sensor_msgs::Imu imu_;
    nav_msgs::Odometry msf_odom_;
    double vo_position_x_, vo_position_y_, vo_position_z_, altitude_;

    //publisher thread
private:
    std::thread *msf_odom_pub_thread;
    std::mutex msf_odom_lock_;

private:
    bool tello_first_yaw_measurement_;
    double tello_first_roll_, tello_first_pitch_, tello_first_yaw_;

    bool dji_first_yaw_measurement_;
    double dji_first_roll_, dji_first_pitch_, dji_first_yaw_;
    double prev_time_; double current_time_;
    bool start_calcualating_dt_;

private:
    bool use_vo_data_, use_global_positioning_data_;
    bool use_rtk_position_, use_rtk_pose_, use_gps_pose_;
    bool use_optitrack_data_;
    bool is_first_vo_z_msg_;
    float first_vo_z_msg_;



void publishSpeedAndPose(nav_msgs::Odometry msg);
Eigen::Vector3f ConvertToENU(geometry_msgs::Vector3Stamped msg);
};
}

#endif
