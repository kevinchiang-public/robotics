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
#include <rxtx/rxtxData.h>
#include <hoverboard/ADCRaw.h>
#include <hoverboard/GPIORaw.h>
#include <hoverboard/GyroRaw.h>
#include <hoverboard/LEDRaw.h>
#include <hoverboard/ThrusterRaw.h>
#include <hoverboard/PWMRaw.h>
#include <vector>

/**
 * Class that is the low level interface to the hoverboard.
 * Basically, this takes as input the raw values that should be sent
 * to the hoverboard.  This does the direct translation into the
 * packets that need to be sent and does not do any other conversions.
 * It also receives and parses the data from the hoverboard and does a
 * direct translation to ROS messages.
 **/
class HoverboardLL {
public:
  HoverboardLL();

  /**
   * Republish our most current messages
   **/
  void republish();

private:

  //Node handle
  ros::NodeHandle nh;

  /**
   * Parses a completed packet (that has a valid checksum).  The packet
   * should not contain the start bytes or the checksum.
   **/
  void parsePacket(unsigned char *packet, int length);

  /**
   * Process/parse a received byte.
   **/
  void processByte(unsigned char c);

  /**
   * Calculate the checksum of a packet
   **/
  char calculateCheckSum(std::vector<unsigned char> data);

  /**
   * Publisher for data we will send out over the serial
   **/
  ros::Publisher serial_send_pub;

  /**
   * ADC publisher
   **/
  ros::Publisher adc_pub;

  /**
   * Gyro publisher
   **/
  ros::Publisher gyro_pub;

  /**
   * GPIO publisher
   **/
  ros::Publisher gpio_pub;

  /**
   * Subscriber for data we will receive
   **/
  ros::Subscriber serial_receive_sub;
  void receiveData(const rxtx::rxtxData::ConstPtr& data);

  /**
   * Subscriber for receiving LED messages
   **/
  ros::Subscriber led_sub;
  void ledCallback(const hoverboard::LEDRaw::ConstPtr& data);
  rxtx::rxtxData ledPacket;

  /**
   * Subscriber for receiving thurster messages
   **/
  ros::Subscriber thruster_sub;
  void thrusterCallback(const hoverboard::ThrusterRaw::ConstPtr& data);
  rxtx::rxtxData thrusterPacket;

  /**
   * Subscriber for receiving pwm messages;
   **/
  ros::Subscriber pwm_sub;
  void pwmCallback(const hoverboard::PWMRaw::ConstPtr& pwm);
  rxtx::rxtxData pwmPacket;
  rxtx::rxtxData pwmPacket1;

};


/**
 * Constructor to set things up and subscribe/advertise
 **/
HoverboardLL::HoverboardLL(void){

  serial_send_pub = nh.advertise<rxtx::rxtxData>("rxtx/send",100);
  serial_receive_sub = nh.subscribe("rxtx/receive",100,&HoverboardLL::receiveData,this);

  //Advertise messages we publish
  adc_pub = nh.advertise<hoverboard::ADCRaw>("hoverboard/ADCRaw",1);
  gpio_pub = nh.advertise<hoverboard::GPIORaw>("hoverboard/GPIORaw",1);
  gyro_pub = nh.advertise<hoverboard::GyroRaw>("hoverboard/GyroRaw",1);
  
  //Subscribe to messages that we will send on to hoverboard
  thruster_sub = nh.subscribe("hoverboard/ThrusterRaw",1,&HoverboardLL::thrusterCallback,this);
  led_sub = nh.subscribe("hoverboard/LEDRaw",10,&HoverboardLL::ledCallback,this);
  pwm_sub = nh.subscribe("hoverboard/PWMRaw",1,&HoverboardLL::pwmCallback,this);

  //TBD set gyro rates


  /////Default Messages
  char checksum;
  //Set all thrusters to 0
  thrusterPacket.header.stamp = ros::Time::now();
  thrusterPacket.data.clear();
  thrusterPacket.data.push_back(0x55);
  thrusterPacket.data.push_back(0x10);
  thrusterPacket.data.push_back(0);
  thrusterPacket.data.push_back(0);
  thrusterPacket.data.push_back(0);
  thrusterPacket.data.push_back(0);
  thrusterPacket.data.push_back(0);
  thrusterPacket.data.push_back(0);
  checksum = calculateCheckSum(thrusterPacket.data);
  thrusterPacket.data.push_back(checksum);

  //Set default to green LED on
  ledPacket.header.stamp = ros::Time::now();
  ledPacket.data.clear();
  ledPacket.data.push_back(0x55);
  ledPacket.data.push_back(0x41);
  checksum = calculateCheckSum(ledPacket.data);
  ledPacket.data.push_back(checksum);

  //Set pwm channel zero (lift thruster) to zero
  pwmPacket.header.stamp = ros::Time::now();
  pwmPacket.data.push_back(0x55);
  pwmPacket.data.push_back(0x80 | 0);
  //THIS IS A BAD HARD CODED DEFAULT THAT THE BRUSHLESS NEEDS
  pwmPacket.data.push_back(20);
  checksum = calculateCheckSum(pwmPacket.data);
  pwmPacket.data.push_back(checksum);

  //Set pwm channel one (lift thruster) to zero
  pwmPacket1.header.stamp = ros::Time::now();
  pwmPacket1.data.push_back(0x55);
  pwmPacket1.data.push_back(0x80 | 1);
  //THIS IS A BAD HARD CODED DEFAULT THAT THE BRUSHLESS NEEDS
  pwmPacket1.data.push_back(20);
  checksum = calculateCheckSum(pwmPacket1.data);
  pwmPacket1.data.push_back(checksum);

}

  /**
   * Republish our most current messages
   **/
void HoverboardLL::republish(){
  static unsigned long cnt = 0;
  cnt++;

  //Only send one per call (to slow data flow to board)
  switch(cnt % 3){
  case 0:
    //serial_send_pub.publish(thrusterPacket);
    break;
  case 1:
    //N.B. this will just resend the last PWM message (which is only a
    //single channel)
    serial_send_pub.publish(pwmPacket);
    //HORRIBLE HARD CODING TBD FIX
    if(cnt < 100){
      serial_send_pub.publish(pwmPacket1);
    }
    break;
  case 2:
    serial_send_pub.publish(ledPacket);
    break;
  }
}

char HoverboardLL::calculateCheckSum(std::vector<unsigned char> data){
	char sum = 0x00;
  //Skip the first byte as this is the header
	for (unsigned int i = 1; i < data.size(); i++)
	{
		sum = sum ^ data.at(i);
	}
	return sum;
}

void HoverboardLL::thrusterCallback(const hoverboard::ThrusterRaw::ConstPtr& thrusters){
  thrusterPacket.header.stamp = ros::Time::now();
  thrusterPacket.data.clear();
  thrusterPacket.data.push_back(0x55);
  thrusterPacket.data.push_back(0x10);
  thrusterPacket.data.push_back(thrusters->thruster1);
  thrusterPacket.data.push_back(thrusters->thruster2);
  thrusterPacket.data.push_back(thrusters->thruster3);
  thrusterPacket.data.push_back(thrusters->thruster4);
  thrusterPacket.data.push_back(thrusters->thruster5);
  thrusterPacket.data.push_back(thrusters->thruster6);
  char checksum = calculateCheckSum(thrusterPacket.data);
  thrusterPacket.data.push_back(checksum);
  //Publish it
  serial_send_pub.publish(thrusterPacket);
}

void HoverboardLL::pwmCallback(const hoverboard::PWMRaw::ConstPtr& pwm){
  ROS_DEBUG("pwm channel %d set to value %d", pwm->channel, pwm->pwm);
  if(pwm->channel < 0 || pwm->channel > 5){
    ROS_ERROR("Requested oob pwm channel %d",pwm->channel);
    return;
  }
  int val = pwm->pwm;
  //if it is less than zero turn off
  if(val < 0){
    val = 255;
  }else if(val > 100){
    val = 100;
    ROS_WARN("Capping pwm%d to 100 from %d",pwm->channel,pwm->pwm);
  }

  pwmPacket.header.stamp = ros::Time::now();
  pwmPacket.data.clear();
  pwmPacket.data.push_back(0x55);
  pwmPacket.data.push_back(0x80 | pwm->channel);
  pwmPacket.data.push_back(val);
  char checksum = calculateCheckSum(pwmPacket.data);
  pwmPacket.data.push_back(checksum);
  //Publish it
  serial_send_pub.publish(pwmPacket);
}

void HoverboardLL::ledCallback(const hoverboard::LEDRaw::ConstPtr& led){
  //Don't know the initial state...so assume false
  static bool red = false;
  static bool green = false;

  //If it is zero make it false, one true, otherwise leave the same
  if(led->led33_red == 0) red = false;
  else if(led->led33_red == 1) red = true;
  else if(led->led33_red == 2) red = !red;

  //If it is zero make it false, one true, otherwise leave the same
  if(led->led33_green == 0) green = false;
  else if(led->led33_green == 1) green = true;
  else if(led->led33_green == 2) green = !green;

  ledPacket.header.stamp = ros::Time::now();
  ledPacket.data.clear();
  ledPacket.data.push_back(0x55);

  if (red && green){
    ledPacket.data.push_back(0x43);
  }else if (red && !green){
    ledPacket.data.push_back(0x42);
  }else if (!red && green){
    ledPacket.data.push_back(0x41);
  }else if (!red && !green){
    ledPacket.data.push_back(0x40);
  }
  char checksum = calculateCheckSum(ledPacket.data);
  ledPacket.data.push_back(checksum);
  //Publish it
  serial_send_pub.publish(ledPacket);
}


/**
 * Parses a completed packet (that has a valid checksum).  The packet
 * should not contain the start bytes or the checksum.
 **/
void HoverboardLL::parsePacket(unsigned char *packet, int length){
  //Start at 2 for the first bytes that indicate what comes next
  int computedLength = 2;
  bool publishADC = false;
  bool publishGPIO = false;
  bool publishGyro = false;

//First byte has these bits set if we want to read these
#define READ_GYRO_FLAG 0x80
#define READ_GPIO_33_FLAG 0x40
#define READ_ADC0_33_FLAG 0x20
#define READ_ADC1_33_FLAG 0x10
#define READ_ADC2_33_FLAG 0x08
#define READ_ADC3_33_FLAG 0x04
#define READ_ADC4_33_FLAG 0x02
#define READ_ADC5_33_FLAG 0x01
  for(int i = 0; i < 8; i++){
    if(((packet[0] >> i) & 0x1) == 1){
      computedLength += 2;
    }
  }
  //add an exception for the gyro bit
  if ((packet[0] & READ_GYRO_FLAG) != 0){
    computedLength += 2;
  }

  //add an exception for the GPIO pins 
  if ((packet[0] & READ_GPIO_33_FLAG) != 0){
    computedLength -= 1;
  }


//Second byte has these bits set if we want to read these
#define READ_VOLTS_FLAG 0x80
#define READ_GPIO_5_FLAG 0x40
#define READ_ADC0_5_FLAG 0x20
#define READ_ADC1_5_FLAG 0x10
#define READ_ADC2_5_FLAG 0x08
#define READ_ADC3_5_FLAG 0x04
#define READ_ADC4_5_FLAG 0x02
#define READ_CURRENT_FLAG 0x01
  //#define READ_ADC5_5_FLAG 0x01
  for (int i = 0; i < 8; i++){
    if (((packet[1] >> i) & 0x1) == 1){
      computedLength += 2;
    }
  }
  //now the special cases
  if ((packet[1] & READ_GPIO_5_FLAG) != 0){
    computedLength -= 1;
  }
  //now the special cases
  if ((packet[1] & READ_CURRENT_FLAG) != 0){
    computedLength += 2;
  }



  if(computedLength != length){
    ROS_ERROR("ERROR: parsePacket computed packet length %d doesn't match given %d",computedLength,length);
    return;
  }

  //Start at 2 because the first two bytes are the flag bytes
  int idx = 2;

  //The messages we will receive and send off
  hoverboard::ADCRaw adc;
  hoverboard::GPIORaw gpio;
  hoverboard::GyroRaw gyro;  
  //Init default values (just so if in the future we change the code
  //to be selective in what is received we will still be ok).
  adc.header.stamp = ros::Time::now();
  adc.adc5_0 = -1;adc.adc5_1 = -1;adc.adc5_2 = -1;
  adc.adc5_3 = -1;adc.adc5_4 = -1;adc.adc5_5 = -1;
  adc.adc5_6 = -1;adc.adc5_7 = -1;
  adc.adc33_0 = -1;adc.adc33_1 = -1;adc.adc33_2 = -1;
  adc.adc33_3 = -1;adc.adc33_4 = -1;adc.adc33_5 = -1;
  gpio.header.stamp = ros::Time::now();
  gpio.gpio5_0 = -1;gpio.gpio5_1 = -1;gpio.gpio5_2 = -1;
  gpio.gpio5_3 = -1;gpio.gpio5_4 = -1;gpio.gpio5_5 = -1;
  gpio.gpio33_0 = -1;gpio.gpio33_1 = -1;gpio.gpio33_2 = -1;
  gpio.gpio33_3 = -1;gpio.gpio33_4 = -1;gpio.gpio33_5 = -1;
  gyro.header.stamp = ros::Time::now();
  gyro.angle = 0;
  gyro.rate = 0;


  
  if(READ_GYRO_FLAG & packet[0]){
    publishGyro = true;
    gyro.rate = (packet[idx++]<<8);
    gyro.rate |= packet[idx++];
    gyro.angle = (packet[idx++]<<8);
    gyro.angle |= packet[idx++];
  }
  if(READ_GPIO_33_FLAG & packet[0]){
    //TBD GPIO parsing gpio33
    idx++;
  }
  //Now the 6 3.3V adc values
  if(READ_ADC0_33_FLAG & packet[0]){
    publishADC = true;
    adc.adc33_0 = packet[idx++]<<8;
    adc.adc33_0 |= packet[idx++];
  }
  if(READ_ADC1_33_FLAG & packet[0]){
    publishADC = true;
    adc.adc33_1 = packet[idx++]<<8;
    adc.adc33_1 |= packet[idx++];
  }
  if(READ_ADC2_33_FLAG & packet[0]){
    publishADC = true;
    adc.adc33_2 = packet[idx++]<<8;
    adc.adc33_2 |= packet[idx++];
  }
  if(READ_ADC3_33_FLAG & packet[0]){
    publishADC = true;
    adc.adc33_3 = packet[idx++]<<8;
    adc.adc33_3 |= packet[idx++];
  }
  if(READ_ADC4_33_FLAG & packet[0]){
    publishADC = true;
    adc.adc33_4 = packet[idx++]<<8;
    adc.adc33_4 |= packet[idx++];
  }
  if(READ_ADC5_33_FLAG & packet[0]){
    publishADC = true;
    adc.adc33_5 = packet[idx++]<<8;
    adc.adc33_5 |= packet[idx++];
  }
  if(READ_GPIO_5_FLAG & packet[1]){
    //TBD GPIO parsing gpio5
    idx++;
  }
  //Now the 6 5.0V adc values
  if(READ_VOLTS_FLAG & packet[1]){
    publishADC = true;
    //Volts is reported on adc5
    adc.adc5_5 = packet[idx++]<<8;
    adc.adc5_5 |= packet[idx++];
  }
  if(READ_ADC0_5_FLAG & packet[1]){
    publishADC = true;
    adc.adc5_0 = packet[idx++]<<8;
    adc.adc5_0 |= packet[idx++];
  }
  if(READ_ADC1_5_FLAG & packet[1]){
    publishADC = true;
    adc.adc5_1 = packet[idx++]<<8;
    adc.adc5_1 |= packet[idx++];
  }
  if(READ_ADC2_5_FLAG & packet[1]){
    publishADC = true;
    adc.adc5_2 = packet[idx++]<<8;
    adc.adc5_2 |= packet[idx++];
  }
  if(READ_ADC3_5_FLAG & packet[1]){
    publishADC = true;
    adc.adc5_3 = packet[idx++]<<8;
    adc.adc5_3 |= packet[idx++];
  }
  if(READ_ADC4_5_FLAG & packet[1]){
    publishADC = true;
    adc.adc5_4 = packet[idx++]<<8;
    adc.adc5_4 |= packet[idx++];
  }
  if(READ_CURRENT_FLAG & packet[1]){
    publishADC = true;
    adc.adc5_6 = packet[idx++]<<8;
    adc.adc5_6 |= packet[idx++];
    adc.adc5_7 = packet[idx++]<<8;
    adc.adc5_7 |= packet[idx++];
  }

  if(idx != length){
    ROS_ERROR("Error in parsePacket(), expected to use %d bytes, but used %d",length,idx);
  }

  //Now publish all of the messages
  if(publishADC) adc_pub.publish(adc);
  if(publishGPIO) gpio_pub.publish(gpio);
  if(publishGyro) gyro_pub.publish(gyro);
}

/**
 * Process/parse a received byte.  This is the state machine to read
 * incoming bytes and depacketize them.  Once it is parsed correctly
 * (and checksum passed), then it calls the parsePacket() function.
 **/
void HoverboardLL::processByte(unsigned char c){
  enum states {START_BYTE0, START_BYTE1, LENGTH, DATA, CHECKSUM};
  static enum states state = START_BYTE0;
#define MAX_PACKET_SIZE 64
  static unsigned char packet[MAX_PACKET_SIZE];
  static int packetIdx = 0;
  static int packetSize = 0;
  unsigned char checksum;

  switch(state){
  case START_BYTE0:
    if(c == 0x55){ 
      state = START_BYTE1;
    }else{
      ROS_WARN("Resyncing, skipping byte %c (%#x)",((c>=' ') && (c <= '~'))?c:'?',c);
    }
    break;
  case START_BYTE1:
    if(c == 0x60){ 
      state = LENGTH;
      packetIdx = 0;
    }else{
      ROS_WARN("Invalid second start byte (expected 0x60), skipping byte %c (%#x)",((c>=' ') && (c <= '~'))?c:'?',c);
    }
    break;
  case LENGTH:
    packetSize = c;
    if(packetSize >= MAX_PACKET_SIZE){
      ROS_ERROR("Packet size %d oob",packetSize);
      state = START_BYTE0;
      break;
    }
    state = DATA;
    break;
  case DATA:
    packet[packetIdx++] = c;
    if(packetIdx >= packetSize){
      state = CHECKSUM;
    }
    break;
  case CHECKSUM:
    checksum = 0x60 ^ packetSize;
    for(int i = 0; i <packetIdx; i++){
      checksum ^= packet[i];
    }
    if(c == checksum){
      parsePacket(packet,packetIdx);
    }else{
      ROS_ERROR("Invalid packet computed checksum %#x, expected %#x (prev %#x), pkt data size %#x",
                checksum,c,packet[packetIdx-1],packetIdx);
    }
    state = START_BYTE0;
    break;
  default:
    ROS_ERROR("ERROR: Entered default state!");
    state = START_BYTE0;
    break;
  }


}

/**
 * Process any received ata
 **/
void HoverboardLL::receiveData(const rxtx::rxtxData::ConstPtr& data){
  for(unsigned int i = 0; i < data->data.size(); i++){
    processByte(data->data.at(i));
  }
}

int main(int argc, char **argv){
  unsigned long cnt = 0;
	ros::init(argc, argv, "HoverboardLL");
	HoverboardLL hb;

  //Publish things again every once in a while to make sure params are correct
  ros::Rate loop_rate(200);
  //While ros still thinks things look good
  while (ros::ok()){
    //Do any processing
    ros::spinOnce();

    //Every once in a while republish
    if(cnt++ % 50 == 1){
      hb.republish();
    }

    //Sleep for the remaining time of this loop iteration
    loop_rate.sleep();
  }
  return 0;
}
