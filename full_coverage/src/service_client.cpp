#include "ros/ros.h"
#include "full_coverage/Srv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc,argv, "service_client");

    if(argc != 3)
    {
        ROS_INFO("cmd : rosrun full_coverage service_client arg0 arg1");
        ROS_INFO("arg0: double number, arg1: double number");

        return 1;
    }


ros::NodeHandle nh;

ros::ServiceClient full_coverage_service_client 
= nh.serviceClient<full_coverage::Srv>("full_coverage_server");

full_coverage::Srv srv;

srv.request.a = atoll(argv[1]);
srv.request.b = atoll(argv[2]);

if(full_coverage_service_client.call(srv))
{
    ROS_INFO("send srv,srv.Request.a and b: %ld,%ld", 
    (long int)srv.request.a,(long int)srv.request.b);
    ROS_INFO("receive srv, srv.Response.result: %ld", (long int)srv.response.result);

}
else
{
    ROS_ERROR("FAILED TO CALL SERVICE full_coverage_SRV");
    return 1;
}
return 0;


}