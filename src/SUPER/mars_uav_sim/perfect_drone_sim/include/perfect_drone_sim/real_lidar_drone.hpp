#ifndef _REAL_LIDAR_DRONE_HPP_
#define _REAL_LIDAR_DRONE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <livox_ros_driver2/CustomMsg.h>
#include <pcl/filters/voxel_grid.h> 
#include "perfect_drone_sim/config.hpp"

class RealLidarDrone {
public:
    RealLidarDrone(ros::NodeHandle& nh) : nh_(nh) {
        // 加载配置文件
        std::string cfg_path, cfg_name;
        if (nh_.param("config_path", cfg_path, std::string(""))) {
            ROS_INFO("Load config from: %s", cfg_path.c_str());
        } else if (nh_.param("config_name", cfg_name, std::string("click.yaml"))) {
            cfg_path = std::string(ROOT_DIR) + "config/" + cfg_name;
            ROS_INFO("Load config by file name: %s", cfg_path.c_str());
        }
        cfg_ = perfect_drone::Config(cfg_path);

        // 初始化状态
        position_ = cfg_.init_pos;
        velocity_.setZero();
        yaw_ = cfg_.init_yaw;
        q_ = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());

        // 初始化发布器
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odom", 100);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lidar_slam/pose", 100);
        local_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100);
        global_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_pc", 100, true); // latch=true

        // 订阅控制指令和Livox点云
        cmd_sub_ = nh_.subscribe("/planning/pos_cmd", 100, &RealLidarDrone::cmdCallback, this);
        livox_sub_ = nh_.subscribe("/livox/lidar", 100, &RealLidarDrone::livoxCallback, this);

        // 加载全局地图 (如果配置了pcd_name)
        if (!cfg_.pcd_name.empty()) {
            loadGlobalMap();
        }

        // 定时器
        odom_pub_timer_ = nh_.createTimer(ros::Duration(0.01), &RealLidarDrone::publishOdomPose, this);
    }

    double getSensingRate() const {
        return cfg_.sensing_rate;
    }

private:
    ros::NodeHandle nh_;
    perfect_drone::Config cfg_;
    ros::Publisher odom_pub_, pose_pub_, local_pc_pub_, global_pc_pub_;
    ros::Subscriber cmd_sub_, livox_sub_;
    ros::Timer odom_pub_timer_;
    
    Eigen::Vector3d position_, velocity_;
    double yaw_;
    Eigen::Quaterniond q_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;

    void loadGlobalMap() {
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        std::string pcd_path = std::string(ROOT_DIR) + "pcd/" + cfg_.pcd_name;
        
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *global_map_) == -1) {
            ROS_ERROR("Couldn't read global map file: %s", pcd_path.c_str());
            return;
        }
        
        sensor_msgs::PointCloud2 global_pc_msg;
        pcl::toROSMsg(*global_map_, global_pc_msg);
        global_pc_msg.header.frame_id = "world";
        global_pc_msg.header.stamp = ros::Time::now();
        global_pc_pub_.publish(global_pc_msg);
        ROS_INFO("Published global map with %ld points", global_map_->size());
    }

    void livoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        
        // 坐标转换 (Livox坐标系到世界坐标系)
        for (const auto& point : msg->points) {
            Eigen::Vector3f p(point.x, point.y, point.z);
            p = q_.cast<float>() * p;  // 旋转到世界系
            p += position_.cast<float>();
            raw_cloud->points.emplace_back(p.x(), p.y(), p.z());
        }
        
        // 下采样 (使用配置参数)
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(cfg_.downsample_res, cfg_.downsample_res, cfg_.downsample_res);
        voxel.setInputCloud(raw_cloud);
        voxel.filter(*filtered_cloud);
        
        sensor_msgs::PointCloud2 pc_msg;
        pcl::toROSMsg(*filtered_cloud, pc_msg);
        pc_msg.header.frame_id = "world";
        pc_msg.header.stamp = ros::Time::now();
        local_pc_pub_.publish(pc_msg);
    }

    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
        Eigen::Vector3d pos(msg->position.x, msg->position.y, msg->position.z);
        Eigen::Vector3d vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        Eigen::Vector3d acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
        double yaw = msg->yaw;
        
        position_ = pos;
        velocity_ = vel;
        yaw_ = yaw;
        q_ = Eigen::AngleAxisd(yaw_, Eigen::Vector3d::UnitZ());
    }

    void publishOdomPose(const ros::TimerEvent& e) {
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world";
        odom.child_frame_id = "perfect_drone";
        
        odom.pose.pose.position.x = position_.x();
        odom.pose.pose.position.y = position_.y();
        odom.pose.pose.position.z = position_.z();
        odom.pose.pose.orientation.x = q_.x();
        odom.pose.pose.orientation.y = q_.y();
        odom.pose.pose.orientation.z = q_.z();
        odom.pose.pose.orientation.w = q_.w();
        
        odom.twist.twist.linear.x = velocity_.x();
        odom.twist.twist.linear.y = velocity_.y();
        odom.twist.twist.linear.z = velocity_.z();
        
        odom_pub_.publish(odom);
        
        geometry_msgs::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        pose_pub_.publish(pose);
        
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform;
        transform.header = odom.header;
        transform.child_frame_id = "perfect_drone";
        transform.transform.translation.x = position_.x();
        transform.transform.translation.y = position_.y();
        transform.transform.translation.z = position_.z();
        transform.transform.rotation.x = q_.x();
        transform.transform.rotation.y = q_.y();
        transform.transform.rotation.z = q_.z();
        transform.transform.rotation.w = q_.w();
        br.sendTransform(transform);
    }
};

#endif