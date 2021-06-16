#pragma once
#include "ros/ros.h"

#include <vector>
#include <iostream>

#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"

#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "aerostack_msgs/TrajectoryWaypoints.h"
#include "nav_msgs/Path.h"

#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"
#include "robot_process.h"
#include "Eigen/Dense"

#define DEBUG 1
#define N_GATES 2

class GatesPathGenerator :public RobotProcess{
public:
    GatesPathGenerator(){};
    void ownSetUp();
    void ownStart(){};
    void ownStop(){};
    void ownRun();
    float speed_ = 0.2;
    float refresh_rate_ = 2; // in Hz

private:

    std::string n_space_;
    std::string gates_topic_ ;
    std::string traj_waypoints_topic_ ;
    std::string self_localization_pose_topic_;

    ros::Publisher gate2_pub_;
    // tf2_ros::TransformBroadcaster tf2_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_ptr_;


    ros::NodeHandle nh_;
    ros::Subscriber gates_sub_; 
    ros::Subscriber pose_sub_;   
    ros::Publisher  traj_waypoints_pub_; 

    aerostack_msgs::TrajectoryWaypoints waypoints_msgs_;
    std::vector<geometry_msgs::PoseStamped> gates_vector_;

    void gatesCallback(const nav_msgs::Path& _msg);
    void publishWaypoints();
    void generateWaypoints(const std::vector<geometry_msgs::PoseStamped>& vector);

    ros::Subscriber gen_traj_sub_;
    void genManualTraj(const std_msgs::Float32& speed);


    void poseCallback(const geometry_msgs::PoseStamped& _msg);
    geometry_msgs::PoseStamped pose_msg_;

    void genTraj();
    void publishTraj();
    bool generating_traj_ = false;
    bool gates_has_changes = false;
    
};
