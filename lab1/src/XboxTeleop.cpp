/* Created by Tony Schneider on 9/12/2013
 * 
 * XboxTeleop.cpp
 *
 * This node is used to receive messages from the
 * joystick sensor node and publish them to the
 * hovercraft's thrusters.
 */

#include <ros/ros.h>
#include <hovercraft/Thruster.h>
#include <sensor_msgs/Joy.h>
#include <iostream>
#include <ros/console.h>
class XboxTeleop
{
public:
  XboxTeleop();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

private:
  ros::NodeHandle nh_;
  ros::Publisher thruster_pub;
  ros::Subscriber joy_sub;
  bool thrusterOn;
  bool startButtonDepressed;
};

XboxTeleop::XboxTeleop()
{
  thruster_pub = nh_.advertise<hovercraft::Thruster>("hovercraft/thruster", 1);
  joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxTeleop::joyCallback, this);
  thrusterOn = false;
  startButtonDepressed = false;
}

void XboxTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  hovercraft::Thruster thrust;
  double lift;
  double thruster1;
  double thruster2;
  double thruster3;
  double thruster4;
  double thruster5;

  if (joy->buttons[7] == 1)
    {
      startButtonDepressed = true;
    }
  else if (joy->buttons[7] == 0 and startButtonDepressed)
    {
      startButtonDepressed = false;
      if (thrusterOn)
	{
	  lift = 0;
	  thruster1 = 0;
	  thruster2 = 0;
	  thruster3 = 0;
	  thruster4 = 0;
	  thruster5 = 0;
	  thrusterOn = false;
	}
      else
	{
	  lift = .4;
	  thrusterOn = true;
	}
    }

  thrust.lift = lift;
  thrust.thruster1 = thruster1;
  thrust.thruster2 = thruster2;
  thrust.thruster3 = thruster3;
  thrust.thruster4 = thruster4;
  thrust.thruster5 = thruster5;

  thruster_pub.publish(thrust);
  // vel.angular = a_scale_*joy->axes[angular_];
  // vel.linear = l_scale_*joy->axes[linear_];
  //   vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_teleop");
  XboxTeleop xbox_teleop;
  ros::spin();
}
