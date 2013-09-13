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
    thruster_pub = nh_.advertise<hovercraft::Thruster>("hovercraft/Thruster", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxTeleop::joyCallback, this);
    thrusterOn = false;
    startButtonDepressed = false;
}

void XboxTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    hovercraft::Thruster thrust;
    double lift=0;
    double thruster1=0;
    double thruster2=0;
    double thruster3=0;
    double thruster4=0;
    double thruster5=0;


    ROS_DEBUG("joyCallback executed");
    printf("joyCallback executed\n");
    if (joy->buttons[7] == 1)
    {
        startButtonDepressed = true;
        ROS_DEBUG("start depressed");
        printf("start depressed\n");
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
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "XboxTeleop");
    XboxTeleop xbox_teleop;
    ROS_DEBUG("Xbox Teleop spinning");
    printf("Xbox Teleop spinning\n");
    ros::spin();
}
