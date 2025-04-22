void publishOdomPose(const ros::TimerEvent& e) {
    // 获取 camera_init→aft_mapped 的变换
    geometry_msgs::TransformStamped aft_mapped_tf;
    if (!getCurrentTransform(aft_mapped_tf)) {
        ROS_WARN_THROTTLE(1.0, "Waiting for Pointlio TF...");
        return;
    }

    // 转换为 world→perfect_drone 的变换
    geometry_msgs::TransformStamped world_to_drone;
    world_to_drone.header.stamp = aft_mapped_tf.header.stamp;
    world_to_drone.header.frame_id = "world";
    world_to_drone.child_frame_id = "perfect_drone";
    
    // 假设 world 和 camera_init 坐标系重合（否则需要坐标变换）
    world_to_drone.transform = aft_mapped_tf.transform;

    // 计算速度（使用原有逻辑）
    calculateVelocity(aft_mapped_tf);

    // 填充odom消息
    odom_.header = world_to_drone.header;
    odom_.child_frame_id = "perfect_drone";
    odom_.pose.pose.position.x = world_to_drone.transform.translation.x;
    odom_.pose.pose.position.y = world_to_drone.transform.translation.y;
    odom_.pose.pose.position.z = world_to_drone.transform.translation.z;
    odom_.pose.pose.orientation = world_to_drone.transform.rotation;
    odom_.twist.twist.linear.x = velocity_.x();
    odom_.twist.twist.linear.y = velocity_.y();
    odom_.twist.twist.linear.z = velocity_.z();

    // 发布odom和pose
    odom_pub_.publish(odom_);
    geometry_msgs::PoseStamped pose;
    pose.header = odom_.header;
    pose.pose = odom_.pose.pose;
    pose_pub_.publish(pose);

    // 发布 world→perfect_drone 的TF
    static tf2_ros::TransformBroadcaster br;
    br.sendTransform(world_to_drone);

    // 路径发布（保持原有逻辑）
    static int slow_down = 0;
    if (slow_down++ % 10 == 0) {
        path_.poses.push_back(pose);
        path_.header = odom_.header;
        path_pub_.publish(path_);
    }
}
