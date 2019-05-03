/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "geometry_msgs/Point.h"

namespace csuro_navigation
{
class Navigator
{
  public:
    Navigator() : nh_("~"), action_client_("move_base", true), goal_sended_(false)
    {
      while(!action_client_.waitForServer(ros::Duration(5.0)))
      {
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      //GrannieAnnie MAIN POSSITION POINTS
      addNewPoint(1.24, 5.52);  //BEDROOM
      addNewPoint(4.74, 3.77);  //KITCHEN
      addNewPoint(0.837, 2.31); //ROOM
      addNewPoint(7.43, 3.47);  //DINNING ROOM
      addNewPoint(4.38, 1.73);  //LIVING ROOM
      goal_cont = 0;
    }

    void addNewPoint(float x, float y) //ADD A POINT TO THE VECTOR
    {
      geometry_msgs::Point point;
      point.x = x;
      point.y= y;
      goals.push_back(point); //INSERT POINTS INTO VECTOR
    }

    void navigate(geometry_msgs::Point point)
    {
      ROS_INFO("[navigate_to_wp] Commanding to (%f %f)", point.x, point.y); //PAINT A POINT
      move_base_msgs::MoveBaseGoal goal;
      goal.target_pose.pose.position = point;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.header.frame_id = "/map";
      goal.target_pose.header.stamp = ros::Time::now();
      action_client_.sendGoal(goal);
      goal_sended_ = true;
    }

    void step()
    {
      if (goal_sended_)
      {
        bool finished_before_timeout = action_client_.waitForResult(ros::Duration(0.5));
        if (finished_before_timeout)
        {
          actionlib::SimpleClientGoalState state = action_client_.getState();
          if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
          {
            ROS_INFO("[navigate_to_wp] Goal Reached!");
            goal_cont++;
          }
          else
            ROS_ERROR("[navigate_to_wp] Something bad happened!");
          goal_sended_ = false;
        }
      }
      else
      {
        if (goal_cont < goals.size())
          navigate(goals[goal_cont]);
      }
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber wp_sub_;
    bool goal_sended_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client_;
    std::vector<geometry_msgs::Point> goals; //CREATE A VECTOR
    int goal_cont;
};
}  // namespace bica_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visit_my_home_node");
  csuro_navigation::Navigator navigator;
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    navigator.step();
    ros::spinOnce();
  }
  return 0;
}
