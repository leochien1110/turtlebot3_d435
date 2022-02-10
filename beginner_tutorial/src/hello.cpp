#include <ros/ros.h>                            // ros.h header

int main(int argc, char** argv){
    ros::init(argc, argv, "hello_cpp_node");    // initialize hello_cpp_node
    ros::NodeHandle handler;                    // node handler (different from rospy)

    int indx;
    handler.getParam("/happy_indx", indx);

    ROS_INFO("Hello World!");                   // print Hello World
    while (ros::ok()){
        ROS_INFO("Hello World in loop!");       
        ROS_INFO("Happy Index: %d",indx);
        ros::Duration(1).sleep();               // pause  1 second
    }
}