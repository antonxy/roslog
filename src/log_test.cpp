#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "log_test");
    ros::NodeHandle nh;
    ros::Rate rate(1.0);
    int i = 0;
    while(true) {
        ROS_WARN("LOG MESSAGE %d", i);
        ros::spinOnce();
        rate.sleep();
        ++i;
    }
    return 0;
}
