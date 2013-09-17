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
    bool liftOn;
    bool startButtonDepressed;
    const double liftPower;
    const double thrustModifier;
};

XboxTeleop::XboxTeleop() : liftPower (.4), thrustModifier(.5)
{
    thruster_pub = nh_.advertise<hovercraft::Thruster>("hovercraft/Thruster", 1);
    joy_sub = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxTeleop::joyCallback, this);
    liftOn = false;
    startButtonDepressed = false;
}

void XboxTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    hovercraft::Thruster thrust;
    ROS_DEBUG("joyCallback executed");
    printf("joyCallback executed\n");

    //Lift logic
    if (joy->buttons[7] == 1 and not startButtonDepressed)
    {
        startButtonDepressed = true;
        ROS_DEBUG("Start Button Pressed\n");
        printf("Start Button Pressed\n");
        thrust.thruster1 = 0;
        thrust.thruster2 = 0;
        thrust.thruster3 = 0;
        thrust.thruster4 = 0;
        thrust.thruster5 = 0;
        if (liftOn)
        {
            printf("Turning Thruster Off\n");
            ROS_DEBUG("Turning Thruster Off\n");
            liftOn = false;
            thrust.lift = 0;
        }
        else
        {
            printf("Turning Thruster On\n");
            ROS_DEBUG("Turning Thruster On\n");
            thrust.lift = liftPower;
            liftOn = true;
        }
        thruster_pub.publish(thrust);
    }
    else if (joy->buttons[7] == 0 and startButtonDepressed)
    {
        startButtonDepressed = false;
    }

    //Ensures all thrusters are zeroed before liftoff
    if (liftOn)
    {
        //Initialize all the thrusters to 0.  Values are replaced if necessary.
        thrust.lift = liftPower;
        thrust.thruster1 = 0;
        thrust.thruster2 = 0;
        thrust.thruster3 = 0;
        thrust.thruster4 = 0;
        thrust.thruster5 = 0;

        //Rotational Controls
        if (joy->axes[0] < 0) //Counterclockwise?
        {
            thrust.thruster4 = 0;
            thrust.thruster5 = fabs(joy->axes[0]) * thrustModifier;
        }
        else if(joy->axes[0] > 0) //Clockwise?
        {
            thrust.thruster5 = 0;
            thrust.thruster4 = joy->axes[0] * thrustModifier;
        }
        else
        {
            thrust.thruster5 = 0;
            thrust.thruster4 = 0;
        }

        //Translation Controls
        if (joy->axes[4] > 0) //Forward
        {
            thrust.thruster3 = joy->axes[4] * thrustModifier; //Is this the correct thruster number?
        }

        if (joy->axes[3] > 0) //Translate left
        {

        }
        else if (joy->axes[3] < 0) //Translate right
        {

        }
        thruster_pub.publish(thrust);


    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "XboxTeleop");
    XboxTeleop xbox_teleop;
    ROS_DEBUG("Xbox Teleop spinning");
    printf("Xbox Teleop spinning\n");
    ros::spin();
}
