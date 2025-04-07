#include "perfect_drone_sim/real_lidar_drone.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_drone_node");
    ros::NodeHandle nh("~");
    
    RealLidarDrone drone(nh);
    
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::waitForShutdown();
    return 0;
}