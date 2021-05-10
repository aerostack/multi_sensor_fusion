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
#include "../include/behavior_self_localize_with_ekf_sensor_fusion.h"

namespace multi_sensor_fusion
{
BehaviorSelfLocalizeWithEKFSensorFusion::BehaviorSelfLocalizeWithEKFSensorFusion() : BehaviorExecutionController() {
 setName("self_localize_with_ekf_sensor_fusion"); 
 setExecutionGoal(ExecutionGoals::KEEP_RUNNING);
}

BehaviorSelfLocalizeWithEKFSensorFusion::~BehaviorSelfLocalizeWithEKFSensorFusion() {}

bool BehaviorSelfLocalizeWithEKFSensorFusion::checkSituation() 
{ 
 return true; 
}

void BehaviorSelfLocalizeWithEKFSensorFusion::checkGoal() {}

void BehaviorSelfLocalizeWithEKFSensorFusion::checkProgress() {}

void BehaviorSelfLocalizeWithEKFSensorFusion::checkProcesses() {}
void BehaviorSelfLocalizeWithEKFSensorFusion::onConfigure()
{
  n = getNodeHandle();
  nspace = getNamespace(); 
  
  altitude_=0;

  msf_odom_.pose.pose.position.x = 0; msf_odom_.pose.pose.position.y = 0; msf_odom_.pose.pose.position.z = 0;
  msf_odom_.pose.pose.orientation.x = 0; msf_odom_.pose.pose.orientation.y = 0; msf_odom_.pose.pose.orientation.z = 0; msf_odom_.pose.pose.orientation.w = 1;
  msf_odom_.twist.twist.linear.x = 0; msf_odom_.twist.twist.linear.y = 0; msf_odom_.twist.twist.linear.z = 0;
  msf_odom_.twist.twist.angular.x = 0; msf_odom_.twist.twist.angular.y = 0; msf_odom_.twist.twist.angular.z = 0;
}

void BehaviorSelfLocalizeWithEKFSensorFusion::onActivate()
{
    std::cout<< "Self localize with EKF Sensor fusion activated" << std::endl;
    //Subscribers
    speed_sub = n.subscribe("/" + nspace + "/" +"sensor_measurement/linear_speed",1, &BehaviorSelfLocalizeWithEKFSensorFusion::speedCallback, this);
    imu_sub = n.subscribe("/" + nspace + "/" +"sensor_measurement/imu",1,&BehaviorSelfLocalizeWithEKFSensorFusion::imuCallback, this);
    altitude_sub = n.subscribe("/" + nspace + "/" +"sensor_measurement/altitude",1,&BehaviorSelfLocalizeWithEKFSensorFusion::altitudeCallback, this);
    msf_odom_sub_  = n.subscribe("/" + nspace +"/msf_core/odometry", 1, &BehaviorSelfLocalizeWithEKFSensorFusion::msfOdomCallback, this);
    position_sub  = n.subscribe("/" + nspace + "/" +"self_localization/position", 1, &BehaviorSelfLocalizeWithEKFSensorFusion::positionCallback, this);


    pose_pub_      = n.advertise<geometry_msgs::PoseStamped>("/" + nspace +"/msf_updates/pose_input",1);
    velocity_pub_  = n.advertise<geometry_msgs::PointStamped>( "/" + nspace +"/msf_updates/velocity", 1);
    altitude_pub_  = n.advertise<geometry_msgs::PointStamped>( "/" + nspace +"/msf_updates/altitude_above_takeoff",1);
    imu_pub_  = n.advertise<sensor_msgs::Imu>("/" + nspace +"/msf_core/imu_state_input",1);    
    msf_odom_pub_  = n.advertise<nav_msgs::Odometry>("/" + nspace +"/msf_interface/odometry",1);

    aerostack_pose_pub_  = n.advertise<geometry_msgs::PoseStamped>("/" + nspace + "/" +"self_localization/pose",1);
    aerostack_speed_pub_ = n.advertise<geometry_msgs::TwistStamped>("/" + nspace + "/" +"self_localization/speed",1);

    pose_deprecated          = n.advertise<droneMsgsROS::dronePose>("/" + nspace + "/" +"gazebo_estimated_pose",1, true);
    speed_deprecated         = n.advertise<droneMsgsROS::droneSpeeds>("/" + nspace + "/" +"gazebo_estimated_speed",1, true);

    //thread for publishing msf at given hz
    msf_odom_pub_thread = new std::thread(&BehaviorSelfLocalizeWithEKFSensorFusion::publishMSFOdom, this);

    //waiting for the msf to be launched first
    usleep(5000000);
    //starting the ethz msf
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::Config conf;

    bool_param.name = "core_init_filter";
    bool_param.value = true;
    conf.bools.push_back(bool_param);

    srv_req.config = conf;
    ros::service::call("/" + nspace +"/msf_pose_position_velocity_altitude_sensor/pose_velocity_altitude_sensor/set_parameters", srv_req, srv_resp);
}

void BehaviorSelfLocalizeWithEKFSensorFusion::onDeactivate()
{
}

void BehaviorSelfLocalizeWithEKFSensorFusion::publishSpeedAndPose(nav_msgs::Odometry msg)
{
    geometry_msgs::PoseStamped estimated_pose;
    geometry_msgs::TwistStamped estimated_speed;

    //   /* Calculating Roll, Pitch, Yaw */
    tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);

    //convert quaternion to euler angels
    double y, p, r;
    m.getEulerYPR(y, p, r);

    /* Rotation from Global frame to body frame */
    Eigen::Vector3f BodyFrame;
    Eigen::Vector3f GlobalFrame;
    Eigen::Matrix3f RotationMat;

    BodyFrame(0,0) = (+1)*msg.twist.twist.linear.x;
    BodyFrame(1,0) = (+1)*msg.twist.twist.linear.y;
    BodyFrame(2,0) = 0;

    RotationMat(0,0) = cos(y);
    RotationMat(1,0) = -sin(y);
    RotationMat(2,0) = 0;

    RotationMat(0,1) = sin(y);
    RotationMat(1,1) = cos(y);
    RotationMat(2,1) = 0;

    RotationMat(0,2) = 0;
    RotationMat(1,2) = 0;
    RotationMat(2,2) = 1;

    GlobalFrame = RotationMat.transpose()*BodyFrame;

    //Self_localization/pose
    estimated_pose.header.stamp = ros::Time::now(); 
    estimated_pose.pose.orientation = msg.pose.pose.orientation;
    estimated_pose.pose.position = msg.pose.pose.position;
    aerostack_pose_pub_.publish(estimated_pose);

    //Self_localization/speed
    estimated_speed.header.stamp = ros::Time::now();
    estimated_speed.twist.linear.x = GlobalFrame(0);
    estimated_speed.twist.linear.y = GlobalFrame(1);
    estimated_speed.twist.linear.z = msg.twist.twist.linear.z;
    estimated_speed.twist.angular = msg.twist.twist.angular;      
    aerostack_speed_pub_.publish(estimated_speed);

    //Deprecated topics
    droneMsgsROS::dronePose deprecated_pose;
    droneMsgsROS::droneSpeeds deprecated_speed;

    deprecated_pose.x = msg.pose.pose.position.x;
    deprecated_pose.y = msg.pose.pose.position.y;
    deprecated_pose.z = msg.pose.pose.position.z;
    deprecated_pose.roll = r;
    deprecated_pose.pitch = p;
    deprecated_pose.yaw = y;

    deprecated_speed.dx = GlobalFrame(0);
    deprecated_speed.dy = GlobalFrame(1);
    deprecated_speed.dz = msg.twist.twist.linear.z;

    pose_deprecated.publish(deprecated_pose);
    speed_deprecated.publish(deprecated_speed);
}

void BehaviorSelfLocalizeWithEKFSensorFusion::onExecute()
{
  
}

void BehaviorSelfLocalizeWithEKFSensorFusion::positionCallback(const geometry_msgs::PointStamped& msg){
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg.header;
    pose_msg.pose.position.x = msg.point.x;
    pose_msg.pose.position.y = msg.point.y;
    pose_msg.pose.position.z = altitude_;
    pose_msg.pose.orientation = imu_.orientation;
    
    pose_pub_.publish(pose_msg);
}
// Custom topic Callbacks
void BehaviorSelfLocalizeWithEKFSensorFusion::speedCallback(const geometry_msgs::TwistStamped &msg)
{
    geometry_msgs::PointStamped velocity_world;
    velocity_world.header = msg.header;
    velocity_world.point.x   = msg.twist.linear.x;
    velocity_world.point.y   = msg.twist.linear.y;
    velocity_world.point.z   = msg.twist.linear.z;

    velocity_pub_.publish(velocity_world);
}

void BehaviorSelfLocalizeWithEKFSensorFusion::imuCallback(const sensor_msgs::Imu &msg)
{
    imu_ = msg;
    imu_pub_.publish(msg);
}

void BehaviorSelfLocalizeWithEKFSensorFusion::altitudeCallback(const geometry_msgs::PointStamped &msg)
{
    geometry_msgs::PointStamped altitude;
    altitude.header.stamp = ros::Time::now();
    altitude.point.x      = 0;
    altitude.point.y      = 0;
    altitude.point.z      = -msg.point.z;

    altitude_ = -altitude.point.z;

    altitude_pub_.publish(altitude);
}

void BehaviorSelfLocalizeWithEKFSensorFusion::msfOdomCallback(const nav_msgs::Odometry &msg)
{
    this->setMSFOdom(msg);
    publishSpeedAndPose(msg);
}

void BehaviorSelfLocalizeWithEKFSensorFusion::setMSFOdom(nav_msgs::Odometry odom)
{
    msf_odom_lock_.lock();
    msf_odom_ = odom;
    msf_odom_lock_.unlock();
}

void BehaviorSelfLocalizeWithEKFSensorFusion::getMSFOdom(nav_msgs::Odometry &odom)
{
    msf_odom_lock_.lock();
    odom = msf_odom_;
    msf_odom_lock_.unlock();
}


void BehaviorSelfLocalizeWithEKFSensorFusion::publishMSFOdom()
{
    ros::Rate loop_rate(200);
    while(ros::ok())
    {
        nav_msgs::Odometry odom;
        this->getMSFOdom(odom);
        msf_odom_pub_.publish(odom);
        loop_rate.sleep();
    }
}

}
PLUGINLIB_EXPORT_CLASS(multi_sensor_fusion::BehaviorSelfLocalizeWithEKFSensorFusion, nodelet::Nodelet)
