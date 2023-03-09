#include "ros/ros.h"
#include "full_coverage/Srv.h"

bool calculation(full_coverage::Srv::Request &req, full_coverage::Srv::Response &res)
{
    res.result = req.a + req.b;


ROS_INFO("request: x=%ld, y=%ld",(long int)req.a, (long int)req.b);
ROS_INFO("sending back response:%ld",(long int)res.result);

return true;

}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"service_server");
    ros::NodeHandle nh;

    ros::ServiceServer full_coverage_sevice_server 
    = nh.advertiseService("full_coverage_server",calculation);

    ROS_INFO("ready srv server!");

    ros::spin();

    return 0;

}