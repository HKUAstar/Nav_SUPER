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
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

class RealLidarDrone {
public:
    RealLidarDrone(ros::NodeHandle& nh) 
        : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_), first_transform_received_(false) {
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
        odom_.header.frame_id = "world";
        path_.poses.clear();
        path_.header.frame_id = "world";
        path_.header.stamp = ros::Time::now();

        // 初始化发布器
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/lidar_slam/odom", 100);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/lidar_slam/pose", 100);
        local_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered1", 100);
        global_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_pc", 100, true); // latch=true
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 100);

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
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    geometry_msgs::TransformStamped last_transform_;
    ros::Time last_transform_time_;
    bool first_transform_received_;

    ros::NodeHandle nh_;
    nav_msgs::Path path_;
    perfect_drone::Config cfg_;
    ros::Publisher odom_pub_, pose_pub_, local_pc_pub_, global_pc_pub_, path_pub_;
    ros::Subscriber cmd_sub_, livox_sub_;
    ros::Timer odom_pub_timer_;
    
    Eigen::Vector3d position_, velocity_;
    double yaw_;
    Eigen::Quaterniond q_;
    nav_msgs::Odometry odom_;
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
    
    bool getCurrentTransform(geometry_msgs::TransformStamped& transform) {
        try {
            // 假设pointlio发布的是odom到base_link的变换
            transform = tf_buffer_.lookupTransform("camera_init", "aft_mapped", ros::Time(0), ros::Duration(0.1));
            return true;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }

    void livoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
        // 1. 获取当前位姿（从TF或odom_）
        geometry_msgs::TransformStamped current_tf;
        if (!getCurrentTransform(current_tf)) {
            ROS_WARN_THROTTLE(1.0, "Cannot transform point cloud without TF data");
            return;
        }
    
        // 2. 准备点云容器
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        raw_cloud->reserve(msg->points.size());
    
        // 3. 坐标转换参数
        Eigen::Vector3f current_position(
            current_tf.transform.translation.x,
            current_tf.transform.translation.y,
            current_tf.transform.translation.z
        );
        
        Eigen::Quaternionf current_orientation(
            current_tf.transform.rotation.w,
            current_tf.transform.rotation.x,
            current_tf.transform.rotation.y,
            current_tf.transform.rotation.z
        );
    
        // 4. 点云转换
        for (const auto& point : msg->points) {
            // 距离过滤
            float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
            if (distance < 0.5) continue;
    
            // 坐标系转换：Livox->机体->世界
            Eigen::Vector3f p(point.x, point.y, point.z);
            p = current_orientation * p;  // 旋转到世界系
            p += current_position;        // 平移到世界系
            
            raw_cloud->points.emplace_back(p.x(), p.y(), p.z());
        }
    
        // 5. 下采样处理
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!raw_cloud->empty()) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setLeafSize(cfg_.downsample_res, cfg_.downsample_res, cfg_.downsample_res);
            voxel.setInputCloud(raw_cloud);
            voxel.filter(*filtered_cloud);
        }
    
        // 6. 发布点云
        if (!filtered_cloud->empty()) {
            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*filtered_cloud, pc_msg);
            pc_msg.header.frame_id = "world";
            pc_msg.header.stamp = current_tf.header.stamp; // 使用TF时间戳保持同步
            local_pc_pub_.publish(pc_msg);
        }
    }

    void calculateVelocity(const geometry_msgs::TransformStamped& current_tf) {
        if (!first_transform_received_) {
            first_transform_received_ = true;
            last_transform_ = current_tf;
            last_transform_time_ = current_tf.header.stamp;
            return;
        }
        
        // 计算时间差
        double dt = (current_tf.header.stamp - last_transform_time_).toSec();
        if (dt <= 0) return;  // 无效时间差
        
        // 计算线速度
        velocity_.x() = (current_tf.transform.translation.x - last_transform_.transform.translation.x) / dt;
        velocity_.y() = (current_tf.transform.translation.y - last_transform_.transform.translation.y) / dt;
        velocity_.z() = (current_tf.transform.translation.z - last_transform_.transform.translation.z) / dt;
        
        // 更新缓存
        last_transform_ = current_tf;
        last_transform_time_ = current_tf.header.stamp;
    }

    void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
        Eigen::Vector3d pos(msg->position.x, msg->position.y, msg->position.z);
        Eigen::Vector3d vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        Eigen::Vector3d acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
        double yaw = msg->yaw;
        
        // position_ = pos;
        // velocity_ = vel;
        // yaw_ = yaw;
        Eigen::Vector3d gravity_(0, 0, 9.80);
        Eigen::Vector3d xC(cos(yaw_), sin(yaw_), 0);
        Eigen::Vector3d zB = (gravity_ + acc).normalized();
        Eigen::Vector3d yB = (zB.cross(xC)).normalized();
        Eigen::Vector3d xB = yB.cross(zB);
        Eigen::Matrix3d R;
        R << xB, yB, zB;
        // q_ = Eigen::Quaterniond(R);
    }

    void publishOdomPose(const ros::TimerEvent& e) {
        geometry_msgs::TransformStamped current_tf;
        if (!getCurrentTransform(current_tf)) {
            ROS_WARN_THROTTLE(1.0, "Failed to get transform from TF");
            return;
        }
        
        calculateVelocity(current_tf);
        
        // 填充odom消息
        odom_.header.stamp = current_tf.header.stamp;
        odom_.header.frame_id = "world";
        odom_.child_frame_id = "perfect_drone";
        
        // 位置和姿态
        odom_.pose.pose.position.x = current_tf.transform.translation.x;
        odom_.pose.pose.position.y = current_tf.transform.translation.y;
        odom_.pose.pose.position.z = current_tf.transform.translation.z;
        odom_.pose.pose.orientation = current_tf.transform.rotation;
        
        // 速度
        odom_.twist.twist.linear.x = velocity_.x();
        odom_.twist.twist.linear.y = velocity_.y();
        odom_.twist.twist.linear.z = velocity_.z();
        
        // 发布odom和pose
        odom_pub_.publish(odom_);
        
        geometry_msgs::PoseStamped pose;
        pose.header = odom_.header;
        pose.pose = odom_.pose.pose;
        pose_pub_.publish(pose);
        
        // 发布TF
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transform;
        transform.header = odom_.header;
        transform.child_frame_id = "perfect_drone";
        transform.transform.translation.x = odom_.pose.pose.position.x;
        transform.transform.translation.y = odom_.pose.pose.position.y;
        transform.transform.translation.z = odom_.pose.pose.position.z;
        transform.transform.rotation = odom_.pose.pose.orientation;
        br.sendTransform(transform);
        
        // 路径发布逻辑保持不变
        static int slow_down = 0;
        if (slow_down++ % 10 == 0) {
            path_.poses.push_back(pose);
            path_.header = odom_.header;
            path_pub_.publish(path_);
        }
    }
};

#endif