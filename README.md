void cmdCallback(const quadrotor_msgs::PositionCommandConstPtr& msg) {
    Eigen::Vector3d pos(msg->position.x, msg->position.y, msg->position.z);
    Eigen::Vector3d vel(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    Eigen::Vector3d acc(msg->acceleration.x, msg->acceleration.y, msg->acceleration.z);
    yaw_ = msg->yaw;

    position_ = pos;
    velocity_ = vel;

    // 修复1：完整更新姿态（微分平坦性）
    Eigen::Vector3d gravity_(0, 0, 9.80);
    Eigen::Vector3d xC(cos(yaw_), sin(yaw_), 0);
    Eigen::Vector3d zB = (gravity_ + acc).normalized();
    Eigen::Vector3d yB = (zB.cross(xC)).normalized();
    Eigen::Vector3d xB = yB.cross(zB);
    Eigen::Matrix3d R;
    R << xB, yB, zB;
    q_ = Eigen::Quaterniond(R);

    // 修复2：清空旧路径（示例条件）
    if ((pos - last_goal_).norm() > 1.0) {  // 当新目标距离旧目标超过1米时清空
        path_.poses.clear();
        last_goal_ = pos;
    }
}
