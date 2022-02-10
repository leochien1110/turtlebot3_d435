#include "ros/ros.h"
#include "beginner_tutorials/my_srv.h"

bool check(beginner_tutorials::my_srv::Request &req,
	 beginner_tutorials::my_srv::Response &res)
{
  if (req.id == 100)
  {
      res.name = "Gawr Gura";
      res.gender = "Old female shark";
      res.age = 9000;
  }
  
  ROS_INFO("request: id=%ld", (long int)req.id);
  ROS_INFO("Name, Gender, Age: [%s,%s,%d]", res.name.c_str(), \
            res.gender.c_str(), (int) res.age);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "id_check_server"); // initialize node
  ros::NodeHandle n; // node handler

  ros::ServiceServer service = n.advertiseService("id_check",check); //def service server and callback function
  ROS_INFO("Ready to check id information.");
  ros::spin();	// keep running this node

  return 0;
}
