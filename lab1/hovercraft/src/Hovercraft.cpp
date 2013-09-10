/****************************************************************************
*
*   Copyright (c) 2011 Carrick Detweiler
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation; either version 2 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*
* Code based on Tyler Lemburg's ROS code for interfacing with Tim
* Echtenkamp's code that runs on the hoverboard.
* 
******************************************************************************/

#include <ros/ros.h>
#include <hoverboard/ADCRaw.h>
#include <hoverboard/GPIORaw.h>
#include <hoverboard/GyroRaw.h>
#include <hoverboard/LEDRaw.h>
#include <hoverboard/PWMRaw.h>
#include <hoverboard/ThrusterRaw.h>
#include <hovercraft/Thruster.h>
#include <hovercraft/Gyro.h>
#include <hovercraft/Current.h>
#include <hovercraft/BatteryVoltage.h>
#include <hovercraft/LED.h>
#include <vector>

class Hovercraft {
public:
  Hovercraft();
private:
  /**
   * Converts the passed thrust (0.0 to 1.0) to an integer value
   * linearly mapped between min and max.
   **/
  int thrustToInteger(double thrust, int min, int max);

  /**
   * Converts the passed thrust (0.0 to 1.0) to an integer value to
   * pass on to the HoverboardLL (which takes 0 to 255, although this
   * function may limit it to a smaller value).
   **/
  int thrustToInteger(double thrust);

  //Node handle
  ros::NodeHandle nh;

  /**
   * ADC subscriber and things we drive from the ADC message since we
   * mainly just translate and pass on what we receive from the
   * HoverboardLL layer.
   **/
  ros::Subscriber adc_sub;
  void adcCallback(const hoverboard::ADCRaw::ConstPtr& data);
  ros::Publisher batteryVoltage_pub;
  ros::Publisher thrusterCurrent_pub;
  ros::Publisher processorCurrent_pub;

  /**
   * Gyro publisher and subscriber since we mainly just translate and
   * then pass it on to the HoverboardLL layer.
   **/
  ros::Publisher gyro_pub;
  ros::Subscriber gyro_sub;
  void gyroCallback(const hoverboard::GyroRaw::ConstPtr& data);

  /**
   * Subscriber, callback and publisher LED messages since we mainly
   * just translate and pass on to the HoverboardLL layer.
   **/
  ros::Subscriber led_sub;
  ros::Publisher led_pub;
  void ledCallback(const hovercraft::LED::ConstPtr&);

  /**
   * Subscriber, callback and publisher Thruster messages since we mainly
   * just translate and pass on to the HoverboardLL layer.
   **/
  ros::Subscriber thruster_sub;
  ros::Publisher thruster_pub;
  void thrusterCallback(const hovercraft::Thruster::ConstPtr&);
  ros::Publisher pwm_pub;


};


/**
 * Constructor to set things up and subscribe/advertise
 **/
Hovercraft::Hovercraft(void){

  //Subscribe to ADC message and advertise the things we derive from it
  adc_sub = nh.subscribe("hoverboard/ADCRaw",1,&Hovercraft::adcCallback,this);
  batteryVoltage_pub = nh.advertise<hovercraft::BatteryVoltage>("hovercraft/batteryVoltage",1);
  thrusterCurrent_pub = nh.advertise<hovercraft::Current>("hovercraft/thrusterCurrent",1);
  processorCurrent_pub = nh.advertise<hovercraft::Current>("hovercraft/processorCurrent",1);

  //Subscribe and republish gyro data
  gyro_sub = nh.subscribe("hoverboard/GyroRaw",1,&Hovercraft::gyroCallback,this);
  gyro_pub = nh.advertise<hovercraft::Gyro>("hovercraft/Gyro",1);
  
  //Subscribe to messages that we will send on to hoverboard
  thruster_sub = nh.subscribe("hovercraft/Thruster",1,&Hovercraft::thrusterCallback,this);
  thruster_pub = nh.advertise<hoverboard::ThrusterRaw>("hoverboard/ThrusterRaw",1);
  pwm_pub = nh.advertise<hoverboard::PWMRaw>("hoverboard/PWMRaw",1);
  led_sub = nh.subscribe("hovercraft/LED",1,&Hovercraft::ledCallback,this);
  led_pub = nh.advertise<hoverboard::LEDRaw>("hoverboard/LEDRaw",1);

  //TBD servos
  //TBD set gyro rates
  //TBD GPIO
}


void Hovercraft::adcCallback(const hoverboard::ADCRaw::ConstPtr& adcRaw){
  //Battery voltage is 3x
  hovercraft::BatteryVoltage bv;
  bv.header.stamp = adcRaw->header.stamp;
  bv.voltage = adcRaw->adc5_5 * 3.0;
  if(adcRaw->adc5_5 >= 0) batteryVoltage_pub.publish(bv);

  //For thruster current the 0.01ohm 20x gain, vout_max 5V
  //--> 5amps/Volt
  hovercraft::Current tc;
  tc.header.stamp = adcRaw->header.stamp;
  tc.current = 5.0 * adcRaw->adc5_6;
  if(adcRaw->adc5_6 >= 0) thrusterCurrent_pub.publish(tc);

  //For processor current the 0.2ohm 20x gain, vout_max 5V
  //--> 0.25amps/Volt
  hovercraft::Current pc;
  pc.header.stamp = adcRaw->header.stamp;
  pc.current = 0.25 * adcRaw->adc5_7;
  if(adcRaw->adc5_7 >= 0) processorCurrent_pub.publish(pc);
}

void Hovercraft::gyroCallback(const hoverboard::GyroRaw::ConstPtr& gyroRaw){
  static int lastReading = 0;
  static double angle = 0.0;
  hovercraft::Gyro gyro;


  //Assuming we don't jump too much, then this should compenstate for
  //the discontinuity and allow us to accumulate the angle.
  if((lastReading > 300) && (gyroRaw->angle < 60)) lastReading -= 360;
  if((lastReading < 60) && (gyroRaw->angle > 300)) lastReading += 360;
  angle += (gyroRaw->angle - lastReading) * 1.0 ;

  //Package it up
  gyro.header.stamp = gyroRaw->header.stamp;
  gyro.angle = angle;
  gyro.rate = gyroRaw->rate * 1.0;

  gyro_pub.publish(gyro);

  lastReading = gyroRaw->angle;
}

/**
 * Converts the passed thrust (0.0 to 1.0) to an integer value
 * linearly mapped between min and max.
 **/
int Hovercraft::thrustToInteger(double thrust, int min, int max){
  if(thrust < 0.01) return 0;
  if(thrust > 1.0) return max;

  //Map 0.0->1.0 to MIN->MAX
  return (thrust * (max - min)) + min;
}

/**
 * Converts the passed thrust (0.0 to 1.0) to an integer value to
 * pass on to the HoverboardLL (which takes 0 to 255, although this
 * function may limit it to a smaller value).
 **/
int Hovercraft::thrustToInteger(double thrust){
  return thrustToInteger(thrust,30,150);
}

void Hovercraft::thrusterCallback(const hovercraft::Thruster::ConstPtr& thrusters){
  //Special one for lift as it is brushless on a pwm output
  hoverboard::PWMRaw pw;
  pw.header.stamp = thrusters->header.stamp;
  pw.channel = 0;
  pw.pwm = thrustToInteger(thrusters->lift,25,40);
  //0 messes it up (below normal range), so make it a bit higher
  if(pw.pwm == 0) pw.pwm = 20;
  pwm_pub.publish(pw);

  hoverboard::ThrusterRaw tr;
  tr.header.stamp = thrusters->header.stamp;

  //Scale to somewhere between 0 and 255ish (although possibly capped)
  tr.thruster1 = thrustToInteger(thrusters->thruster1);
  tr.thruster2 = thrustToInteger(thrusters->thruster2);
  tr.thruster3 = thrustToInteger(thrusters->thruster3);
  tr.thruster4 = thrustToInteger(thrusters->thruster4);
  tr.thruster5 = thrustToInteger(thrusters->thruster5);
  tr.thruster6 = thrustToInteger(thrusters->thruster6);

  thruster_pub.publish(tr);
}

void Hovercraft::ledCallback(const hovercraft::LED::ConstPtr& led){
  hoverboard::LEDRaw lr;

  lr.header.stamp = led->header.stamp;

  //Direct translation
  lr.led33_red = led->led33_red;
  lr.led33_green = led->led33_green;

  led_pub.publish(lr);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "Hovercraft");
	Hovercraft hc;
  ros::spin();
  return 0;
}
