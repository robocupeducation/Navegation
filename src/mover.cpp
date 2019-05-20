
#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "geometry_msgs/Twist.h"

const float Pi = 3.141595;

class Turner
{
private:
  ros::NodeHandle n;
  ros::Publisher num_pub;

public:

  Turner()
  {
    num_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  }

  void step()
  {
    geometry_msgs::Twist vel;
    vel.angular.z = Pi / 4.0;
    num_pub.publish(vel);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "mover_publisher");
  Turner turner;

  ros::Rate loop_rate(10);

  while(ros::ok()){

    turner.step();
    ros::spinOnce();
    loop_rate.sleep();

  }
  return 0;
}
