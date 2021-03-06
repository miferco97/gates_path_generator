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

#define SPHERE_RADIOUS 1.0f
#define GATE_POSITION_CHANGE_THRESHOLD 0.20f
#define DIST_BETWEEN_POINTS 0.5f

typedef std::vector<Eigen::Vector3f> segmentType; 

struct GateInfo{

    GateInfo(int id)
        :gate_id(id){};
    
    GateInfo(int id , bool generate_three_points)
        :gate_id(id), generate_three_points_(generate_three_points){};
    
    GateInfo(int id ,float x, float y, float z , bool generate_three_points, bool is_final_point=false)
        :gate_id(id), generate_three_points_(generate_three_points), is_final_point_(is_final_point){

            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.w = 1.0f;

            has_pose = true;
            first_update = false;
        };

    

    int gate_id;
    geometry_msgs::PoseStamped pose;

    segmentType gate_segment;
    bool has_passed = true;
    bool is_inside = false;
    bool has_pose = false;
    bool is_final_point_ = false;
    bool generate_three_points_ = false;

    private:
        const float alpha = 0.05;
        bool first_update = true;
        void calculateGateSegment(const Eigen::Vector3f&  _estimated_pose){
            gate_segment.clear();
	        if (!has_pose) return;
            Eigen::Vector3f center_point_vec;
            center_point_vec << pose.pose.position.x,
                                pose.pose.position.y,
                                pose.pose.position.z;
                
            if (generate_three_points_ && ((_estimated_pose - center_point_vec)).norm()< 3.0f){

                Eigen::Vector3f same_z_estimated_pose;
                same_z_estimated_pose << _estimated_pose(0),
                                         _estimated_pose(1),
                                         center_point_vec(2);

                Eigen::Vector3f v_director = (center_point_vec - same_z_estimated_pose).normalized(); 
                Eigen::Vector3f before_point_vec = center_point_vec - DIST_BETWEEN_POINTS * (v_director); 
                Eigen::Vector3f after_point_vec = center_point_vec + DIST_BETWEEN_POINTS * (v_director); 
                gate_segment.emplace_back(before_point_vec);
                gate_segment.emplace_back(center_point_vec);
                gate_segment.emplace_back(after_point_vec);
                
            }else{
                gate_segment.emplace_back(center_point_vec);
            }
        };
    public:

    void filterPose(const geometry_msgs::PoseStamped& _new_pose){
        Eigen::Vector3f new_pose(_new_pose.pose.position.x,_new_pose.pose.position.y,_new_pose.pose.position.z);
        auto last_pose = getVectorPose();
        if ((new_pose-last_pose).norm() > GATE_POSITION_CHANGE_THRESHOLD/2){
            new_pose = last_pose + (new_pose-last_pose).normalized()*GATE_POSITION_CHANGE_THRESHOLD/2;
        }
        
        pose.header= _new_pose.header;
        pose.pose.position.x = pose.pose.position.x * (1-alpha) + new_pose(0)* (alpha);
        pose.pose.position.y = pose.pose.position.y * (1-alpha) + new_pose(1)* (alpha);
        pose.pose.position.z = pose.pose.position.z * (1-alpha) + new_pose(2)* (alpha);
        
    }

    void updatePose(const geometry_msgs::PoseStamped& new_pose){
        has_pose=true;
        if (first_update){
            pose = new_pose;
            first_update = false;
        }else{
            filterPose(new_pose);
        }
    }

    Eigen::Vector3f getVectorPose(){
        return Eigen::Vector3f(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
    }

    bool checkIsInside(const Eigen::Vector3f& estimated_pose){
        if (is_final_point_) return false;
        bool is_inside_now = false;
        if (gate_segment.size()==0){
                is_inside_now = false;
            }
            else if(gate_segment.size()==3){
                is_inside_now= (gate_segment.at(1)-estimated_pose).norm()< 1*(SPHERE_RADIOUS);

            }
            else if(gate_segment.size()==1){
                is_inside_now= (gate_segment.at(0)-estimated_pose).norm()<SPHERE_RADIOUS;
            }
        if (!is_inside_now && is_inside && !has_passed){
            has_passed = true;
            std::cout << "GATE " <<gate_id <<" HAS BEEN PASED THROUGH"<< std::endl;
        }

        is_inside = is_inside_now;
        return is_inside;
    }

    bool recalculateSegment(const Eigen::Vector3f& estimated_pose){
            checkIsInside(estimated_pose);
            if (is_inside && !is_final_point_) return false;
            if (gate_segment.size()==0){
                calculateGateSegment(estimated_pose);
                return true;
            }
            else if(gate_segment.size()==3){                
                auto pose = getVectorPose();
                if ((gate_segment.at(1)-pose).norm()>GATE_POSITION_CHANGE_THRESHOLD){
                    calculateGateSegment(estimated_pose);
                    return true;
                }
            }
            else if(gate_segment.size()==1){
                auto pose = getVectorPose();
                if ((gate_segment.at(0)-pose).norm()>GATE_POSITION_CHANGE_THRESHOLD){
                    calculateGateSegment(estimated_pose);
                    return true;
                }
                
            }else{
                std::cout<<" CHECK GATE SEGMENTS SIZE " << std::endl;
            }
            return false;   
        }

        
};


class GatesPathGenerator :public RobotProcess{
public:
    GatesPathGenerator(){};
    void ownSetUp();
    void ownStart(){};
    void ownStop(){};
    void ownRun();
    float speed_ = 0.2;
    float refresh_rate_ = 2; // in Hz
    // std::vector<int> gates_id_; 




private:

    std::string n_space_;
    std::string gates_topic_ ;
    std::string traj_waypoints_topic_ ;
    std::string self_localization_pose_topic_;

    ros::Publisher gate2_pub_;
    ros::Publisher gate1_pub_;
    // tf2_ros::TransformBroadcaster tf2_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_ptr_;


    ros::NodeHandle nh_;
    ros::Subscriber gates_sub_; 
    ros::Subscriber pose_sub_;   
    ros::Publisher  traj_waypoints_pub_; 

    aerostack_msgs::TrajectoryWaypoints waypoints_msgs_;
    // std::vector<geometry_msgs::PoseStamped> gates_vector_;

    void gatesCallback(const nav_msgs::Path& _msg);
    void publishWaypoints();
    void generateWaypoints(const std::vector<geometry_msgs::PoseStamped>& vector);

    ros::Subscriber gen_traj_sub_;
    void getManualTrajCallback(const std_msgs::Float32& speed);


    void poseCallback(const geometry_msgs::PoseStamped& _msg);
    Eigen::Vector3f estimated_pose_;


    void genTraj();
    void publishTraj();
    bool generating_traj_ = false;
    bool gates_has_changes = false;
    
    // vector of pairs <gate_id(int), gate_passed(bool)>
    std::vector<GateInfo> gates_info_;
    std::vector<segmentType> gates_path_;
    
    
};
