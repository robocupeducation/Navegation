
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "geometry_msgs/Twist.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mover_publisher");
  ros::NodeHandle n("~");
  ros::Publisher num_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  ros::Rate loop_rate(10);
  while(ros::ok()){
    
    geometry_msgs::Twist vel;
    vel.angular.z = 0.2;
    num_pub.publish(vel);
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
