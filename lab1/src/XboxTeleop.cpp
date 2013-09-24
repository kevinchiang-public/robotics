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
        if (joy->axes[0] < -.1) //Counterclockwise?
        {
            thrust.thruster4 = 0;
            thrust.thruster5 = fabs(joy->axes[0]) * thrustModifier;
	    //printf("Firing thruster5 with power: %f\n", thrust.thruster5);
        }
        else if(joy->axes[0] > .1) //Clockwise?
        {
            thrust.thruster5 = 0;
            thrust.thruster4 = joy->axes[0] * thrustModifier;
	    //printf("Firing thruster4 with power: %f\n", thrust.thruster4);
        }

        //Translation Controls
	double yAxis = joy->axes[4];
	double xAxis = joy->axes[3];
	double angle;
	//angle is in degrees
	angle=(atan2(yAxis,xAxis)+3.141593)*(180/3.141593); 
	//pythagorean
	double magnitude = sqrt(xAxis*xAxis+yAxis*yAxis);
	magnitude = magnitude>1?1:magnitude;
	printf("xAxis: %f, yAxis: %f, Angle: %f, Magnitude:, %f\n",xAxis,yAxis, angle, magnitude);
	//Using math to calculate thruster power

	/*
	The theory behind this:
	Orientation:  Right is 0 degrees.  Increasing in clockwise direction. 360 is 0
	There are three critical points:  90, 210, and 330 degrees.  These are the points where only
	one thruster is needed.

	So the thruster1 range should increase from -30 degrees to 90 degrees, and decrease from
	90 to 210 degrees. I'm assuming it should be continuous and equally distributed.
	If the power for thruster1 is 1 at 90 degrees, then it should be 1-(1/120) at 89 degrees,
	or 91 degrees. Etc...

	The angles are normalized so I don't have to worry about negative numbers.
	*/	

	//Thruster 1
	double t1angle = angle<330?angle+360:angle;
	if (t1angle < 450){
		thrust.thruster1 = 1.0- ((450.0 - t1angle)*(1.0/120.0));
	}else if (t1angle == 450){
		thrust.thruster1 = 1.0;
	}else if (t1angle >450){
		thrust.thruster1 = 1.0-((t1angle - 450.0)*(1.0/120.0));
	}
	//Thruster 2
	double t2angle = angle<210?angle+360:angle;
	if (t2angle < 330){
		thrust.thruster2 = 1.0-((330.0 - t2angle)*(1.0/120.0));
	}else if(t2angle == 330){
		thrust.thruster2 = 1.0;
	}else if(t2angle > 330){
		thrust.thruster2 = 1.0-((t2angle - 330.0)*(1.0/120.0));
	}

	//Thruster 3
	double t3angle = angle;
	if (t3angle < 210){
		thrust.thruster3 = 1.0-((210.0 - t3angle)*(1.0/120.0));
	}else if(t3angle == 210){
		thrust.thruster3 = 1.0;
	}else if(t3angle > 210){
		thrust.thruster3 = 1.0-((t3angle - 210.0)*(1.0/120.0));
	}	
	thrust.thruster1 = thrust.thruster1<0?0:thrust.thruster1;
	thrust.thruster2 = thrust.thruster2<0?0:thrust.thruster2;
	thrust.thruster3 = thrust.thruster3<0?0:thrust.thruster3;

	thrust.thruster1 = thrust.thruster1*thrustModifier*magnitude;
	thrust.thruster2 = thrust.thruster2*thrustModifier*magnitude*.7; //.8 is compensating for something..
	thrust.thruster3 = thrust.thruster3*thrustModifier*magnitude;
	printf("t1angle   : %f, t2angle   : %f, t3angle   : %f\n",t1angle,t2angle,t3angle);
	printf("Thruster 1: %f, Thruster 2: %f, Thruster 3: %f\n",thrust.thruster1,thrust.thruster2,thrust.thruster3);
	printf("Thruster 4: %f, Thruster 5: %f\n\n", thrust.thruster4, thrust.thruster5);
	/*	//Don't delete this.

	if (joy->axes[4] > .1) //Foward
	{
	    thrust.thruster2=joy->axes[4]*thrustModifier;
	    thrust.thruster3=joy->axes[4]*thrustModifier;
	    printf("Firing thrusters 2 and 3 with power: %f\n",thrust.thruster2); 
	}
        else if (joy->axes[4] < -.1) //Reverse
        {
            thrust.thruster1 = joy->axes[4] * thrustModifier; //Is this the correct thruster number?
	    printf("Firing thruster1 with power: %f\n", thrust.thruster1);
        }
	
        if (joy->axes[3] < -.1) //Translate left
        {
	    double temp2 = thrust.thruster2;
	    double temp1 = thrust.thruster1;
	    thrust.thruster2 = fabs(joy->axes[3])*thrustModifier;
	    thrust.thruster1 = thrust.thruster2*.273;
	    thrust.thruster2 += temp2;
	    thrust.thruster1 += temp1;
	    printf("Firing thruster2 with power: %f and thruster3 with power: %f\n", thrust.thruster2, thrust.thruster3);
        }
        else if (joy->axes[3] > .1) //Translate right
        {	    
	    double temp3 = thrust.thruster3;
	    double temp1 = thrust.thruster1;
	    thrust.thruster3 = fabs(joy->axes[3])*thrustModifier;
	    thrust.thruster1 = thrust.thruster3*.273;
	    thrust.thruster3 += temp3;
	    thrust.thruster1 += temp1;
	    printf("Firing thruster2 with power: %f and thruster3 with power: %f\n", thrust.thruster2, thrust.thruster3);	
        }
	*/
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
