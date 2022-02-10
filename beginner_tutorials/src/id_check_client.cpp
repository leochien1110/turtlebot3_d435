#include "ros/ros.h"
#include "beginner_tutorials/my_srv.h"
#include <cstdlib>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "check_id_client");
    if (argc != 2)
    {
        ROS_INFO("usage: Please enter your 3 digits id.");
        return 1;
    }
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<beginner_tutorials::my_srv>("id_check");
    beginner_tutorials::my_srv srv;
    
    srv.request.id = std::stoi(argv[1]); // atoll: convert string to long long int
    
    if (client.call(srv))   // call the service
    {
        ROS_INFO("Name: %s, Gender: %s, Age: %d", srv.response.name.c_str(), \
                srv.response.gender.c_str(), (int)srv.response.age);
        ROS_INFO("Hello from id check client");
    }
    else
    {
        {
            ROS_ERROR("Failed to call service id_check");
            return 1;
        }
    }

    return 0;    
}