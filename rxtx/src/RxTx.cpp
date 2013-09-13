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
******************************************************************************/

#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <rxtx/rxtxData.h>
#include <std_msgs/UInt8MultiArray.h>

using namespace std;

class RxTx {
public:
  RxTx();
  void parse();
private:

  //Callback for data we will send out
  void sendData(const rxtx::rxtxData::ConstPtr& data);
  //Callback for data we send with the MA format
  void sendDataMA(const std_msgs::UInt8MultiArray::ConstPtr& data);
  //The actual sender
  void sendData(const vector<unsigned char> data);

  //Node handle
  ros::NodeHandle nh;

  /**
   * Serial port we are using
   **/
  cereal::CerealPort * serialPort;

  /**
   * Portname to use
   **/
  string portname;

  /**
   * Baudrate to use
   **/
  int baud;

  /**
   * Publisher for received data we will send out
   **/
  ros::Publisher receive_pub;
  //Multi-array version
  ros::Publisher receive_MA_pub;

  /**
   * Subscriber for data we will send out
   **/
  ros::Subscriber send_sub;
  ros::Subscriber send_MA_sub;

};

/**
 * Constructor to advertise messages
 **/
RxTx::RxTx(void){
  //Get/store the serial port
  //portname = "/dev/ttyACM0";
  nh.param("RxTx/SerialPort",portname,portname);
  nh.setParam("RxTx/SerialPort",portname);

  //baud = 9600;
  nh.param("RxTx/BaudRate",baud,baud);
  nh.setParam("RxTx/BaudRate",baud);

  //Create and connect the serial port
  serialPort = new cereal::CerealPort();
  try{
    switch(baud){
    case 9600:
      serialPort->open(portname.c_str(),9600);
      break;
    case 19200:
      serialPort->open(portname.c_str(),19200);
      break;
    case 38400:
      serialPort->open(portname.c_str(),38400);
      break;
    case 57600:
      serialPort->open(portname.c_str(),57600);
      break;
    case 115200:
      serialPort->open(portname.c_str(),115200);
      break;
    default:
      ROS_INFO("Unknown baud rate %d, defaulting to 38400",baud);
      baud = 38400;
      serialPort->open(portname.c_str(),38400);
    }

    ROS_INFO("Opened serial port %s with baud %d",portname.c_str(), baud);
  }catch(cereal::Exception& e){
    ROS_FATAL("Error opening serial port %s",portname.c_str());
  }

  //Publisher to send out received data
  receive_pub = nh.advertise<rxtx::rxtxData>("rxtx/receive",10);
  receive_MA_pub = nh.advertise<std_msgs::UInt8MultiArray>("rxtx/receiveMA",10);
  //Subscriber to send data over the serial port
  send_sub = nh.subscribe("rxtx/send",100,&RxTx::sendData,this);
  send_MA_sub = nh.subscribe("rxtx/sendMA",100,&RxTx::sendDataMA,this);

}


void RxTx::sendData(const vector<unsigned char> data){
  char c;
  for(unsigned int i = 0; i < data.size(); i++){
    c = data.at(i);
    ROS_DEBUG_NAMED("RawTX","Sending %c (%#x)",((c>=' ') && (c <= '~'))?c:'?',c);
    serialPort->write((char *)&c,1);
  }
}

/**
 * Send received data out over the serial port
 **/
void RxTx::sendData(const rxtx::rxtxData::ConstPtr& data){
  sendData(data->data);
}

/**
 * Multi-Array version of sendData
 **/
void RxTx::sendDataMA(const std_msgs::UInt8MultiArray::ConstPtr& data){
  sendData(data->data);
}


/**
 * Parse any data that has come in on the serial line
 **/
void RxTx::parse(void){
  rxtx::rxtxData data;
  std_msgs::UInt8MultiArray dataMA;
  int cnt = 0;
  data.header.stamp = ros::Time::now();
  //Set it to 1 dimensional
  dataMA.layout.dim.resize(1);
  dataMA.layout.dim[0].size = 0;

  unsigned char c;
  try{
    //Just loop trying to get one byte each time (not as good as
    //trying to get a bunch of bytes...but ok).  Last argument is
    //number of MS to wait for a byte
    while(serialPort->read((char *)&c,1,2)){
      ROS_DEBUG_NAMED("RawRX","Got %c (%#x)",((c>=' ') && (c <= '~'))?c:'?',c);
      data.data.push_back(c);
      dataMA.data.push_back(c);
      dataMA.layout.dim[0].size++;
      //Make sure we don't get stuck here
      if(cnt++ > 128) break;
    }
  }catch(cereal::TimeoutException e){
    //Don't do anything if there is a serial timeout
  }

  //Send out the received data stream
  if(cnt > 0){
    //    for (int i = 0; i < data.data.size(); i++)
    // ROS_INFO("Received Byte: %u", data.data[i]);
    receive_pub.publish(data);
    receive_MA_pub.publish(dataMA);
  }

}


int main(int argc, char **argv){
  ros::init(argc, argv, "rxtx");

  //Create the rxtx serial port
  RxTx rxtxPort;

  //Run loop at 200Hz, this is how often we check for new bytes, but
  //everytime we get new bytes we suck them all in.
  ros::Rate loop_rate(200);

  //While ros still thinks things look good
  while (ros::ok()){
    //Parse any new data that has come in
    rxtxPort.parse();
    //Do any processing
    ros::spinOnce();

    //Sleep for the remaining time of this loop iteration
    loop_rate.sleep();
  }
  return 0;
}
