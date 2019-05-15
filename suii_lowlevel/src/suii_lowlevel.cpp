/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bram Fenijn
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <suii_lowlevel/lowlevelcontroller.h>
#include <suii_lowlevel/serialMsg.h>

#include <std_srvs/Empty.h>

#include <suii_lowlevel/suii_status.h>

#include "std_msgs/Bool.h"

bool g_isActive = false;
 

 LowLevelController* g_lowLevelController;

// void ledCallback(const slambot_core::Pid::ConstPtr &pidMsg)
// {
//  // ROS_INFO("cmdCallback");
//   //slambot_core::Pid pids = (slambot_core::Pid)*pidMsg;
//   ROS_INFO("change led state");
// }

ros::Publisher g_estop_pub;
ros::Publisher g_status_pub;
ros::ServiceClient g_resetOdrive;
//ros::Rate g_r;

void resetOdrives()
{
  ROS_INFO("[suii_lowlevel] send reset in 5 sec!");
  std_srvs::Empty msg;

  ros::Duration(5.0).sleep();
  ROS_INFO("[suii_lowlevel] resetting odrives");
  if (g_resetOdrive.call(msg))
  {
    ROS_INFO("[suii_lowlevel] send reset to odrives");
  }
  else
  {
    ROS_ERROR("[suii_lowlevel] Failed to call service reset to odrives");
  }
  
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suii_lowlevel");
  ros::NodeHandle n;
  ros::NodeHandle priv_nh("~");
  ROS_INFO("starting suii_lowlevel");
  ros::Rate r(100);
  std::string port;
  int baud_rate;

  priv_nh.param("port", port, std::string("/dev/ttyUSB0"));
  priv_nh.param("baud_rate", baud_rate, 115200);

  boost::asio::io_service io;

  try {
    ROS_INFO("connecting to lowLevelController %s@%i",port.c_str(),baud_rate);
    g_lowLevelController = new LowLevelController(port, baud_rate, io);
    ROS_INFO("connected to lowLevelController");
    g_estop_pub = n.advertise<std_msgs::Bool>("estop", 50);
    g_status_pub = n.advertise<suii_lowlevel::suii_status>("lowlevel_status", 50);
    g_resetOdrive = n.serviceClient<std_srvs::Empty>("/motors/bootup");
    
     
    while (ros::ok()) 
    {
      ros::spinOnce();

      SerialMsg sMsg;
      int8_t status = g_lowLevelController->readMsg(&sMsg);
      if(status == 1)
      {
        g_lowLevelController->sendPong();
        switch(sMsg.type)
        {
          case 1: //ping
          {
            //ROS_INFO("resived ping");
            break;
          }
          case 2: //status
          {
            StatusMsg statusMsgSerial;
            memcpy(&statusMsgSerial,sMsg.data,sizeof(statusMsgSerial));
            //ROS_INFO("state: %i",statusMsgSerial.state);
            
            std_msgs::Bool estopMsg;
            suii_lowlevel::suii_status statusMsg;
            
            estopMsg.data = statusMsgSerial.estop;
            statusMsg.enabled = (statusMsgSerial.state == 3);
            statusMsg.estop = statusMsgSerial.estop;
            statusMsg.error = false;
            statusMsg.battery = -1.0;
            statusMsg.state = statusMsgSerial.state;

            g_estop_pub.publish(estopMsg);
            g_status_pub.publish(statusMsg);

            if(statusMsgSerial.state == 3)
            {
              if(!g_isActive)
              {
                g_isActive = true;
                resetOdrives();
              }
            }
            else
            {
              g_isActive = false;
            }

            break;
          }
          default:
          {
            break;
          }
        }
      }
      else
      {
         //ROS_INFO("receive failed, error: %i",status);
      }

      r.sleep();
    }
    ROS_INFO("stop");
    g_lowLevelController->close();

    delete g_lowLevelController;
    g_lowLevelController = NULL;

    return 0;
  } catch (boost::system::system_error ex) {
    ROS_ERROR("Error instantiating suii_lowlevel object. Are you sure you have the correct port and baud rate? Error was %s", ex.what());
    return -1;
  }
}
