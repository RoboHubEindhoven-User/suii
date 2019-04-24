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

#include <suii_lowlevel/lowlevelcontroller.h>
#include <suii_lowlevel/serialMsg.h>
#include <iostream>

#define RXSTARTBYTE 0xAA
#define TXSTARTBYTE 0x55

LowLevelController::LowLevelController(const std::string &port, uint32_t baud_rate, boost::asio::io_service &io) : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_)
{
	serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
}

void LowLevelController::sendVelocity(int16_t speedLeft, int16_t speedRight)
{
	uint8_t data[4];
	data[0] = speedLeft & 0xFF;
	data[1] = (speedLeft >> 8) & 0xFF;
	data[2] = speedRight & 0xFF;
	data[3] = (speedRight >> 8) & 0xFF;
	sendMsg(1, data, 4);
}
void LowLevelController::sendLidarSpeed(uint16_t speed)
{
	uint8_t data[2];
	data[0] = speed & 0xFF;
	data[1] = (speed >> 8) & 0xFF;

	sendMsg(2, data, 2);
}
void LowLevelController::sendPids(float kp,float ki, float kd)
{
	PidMsg pids;
	pids.kp = kp;
	pids.ki = ki;
	pids.kd = kd;
	uint8_t data[sizeof(PidMsg)];
	memcpy(data,&pids,sizeof(PidMsg));
	sendMsg(4, data, sizeof(PidMsg));
}
void LowLevelController::sendMsg(uint8_t type, uint8_t *data, uint8_t len)
{
	uint8_t txBuffer[32];
	uint8_t size = len + 4;

	txBuffer[0] = TXSTARTBYTE;
	txBuffer[1] = type;
	txBuffer[2] = len;
	memcpy(&txBuffer[3],data,len);
	txBuffer[len + 3] = Crc8(txBuffer, len + 3);

	boost::asio::write(serial_, boost::asio::buffer(txBuffer, size));
}

int8_t LowLevelController::readMsg(SerialMsg *serialMsg)
{
	if(serialMsg == NULL)
	{
		return -1;
	}
	
	enum State{start,metadata,data,crc};
	State state = start;

	uint16_t timeoutCounter = 6;
	while(timeoutCounter)
	{
		
		switch(state)
		{
			case start:
			{
				uint8_t inByte = 0;
				boost::asio::read(serial_, boost::asio::buffer(&inByte,1));
				//std::cout << "startbyte: " << (int) inByte << std::endl; 
				if(inByte == RXSTARTBYTE)
				{
					//std::cout << "startbyte Oke: " << (int) inByte << std::endl; 
					serialMsg->start = inByte;
					state = metadata;
				}
				break;
			} 
			case metadata:
			{
				uint8_t inBytes[2];
				boost::asio::read(serial_, boost::asio::buffer(&inBytes,2));
				serialMsg->type = inBytes[0];
				serialMsg->size = inBytes[1];
				//std::cout << "type: " << (int) serialMsg->type << " size: " << (int) serialMsg->size <<  std::endl; 
				if(serialMsg->size > 32)
				{
					return -3;
				}
				state = data;
				break;
			}
			case data:
			{
				uint8_t readCount = serialMsg->size + 1;
				uint8_t inBytes[readCount];
				 
				boost::asio::read(serial_, boost::asio::buffer(&inBytes,readCount));
				serialMsg->crc = inBytes[serialMsg->size];
				memcpy(serialMsg->data,inBytes,serialMsg->size);
				//std::cout << "count: " << (int) readCount << " crc: " << (int) inBytes[4] <<  std::endl; 
				uint8_t calcCrc = Crc8((uint8_t*)serialMsg,serialMsg->size+3);

				//std::cout << "R crc: " << (int) serialMsg->crc << " C crc: " << (int) calcCrc <<  std::endl; 

				if(calcCrc == serialMsg->crc)
				{
					//succes
					//std::cout << "timeoutCounter: " << (int) timeoutCounter << std::endl; 
					return 1;
				}
				else
				{
					return -4;
				}
				state = start;
				break;
			}
		}

		timeoutCounter--;
	}
	
	if(timeoutCounter == 0)
	{
		//timeout
		return -2;
	}
	return -1;
}

uint8_t LowLevelController::Crc8(uint8_t *data, uint8_t len)
{
	uint8_t crc = 0xff;
	for (uint8_t i = 0; i < len; i++)
	{
		volatile uint8_t useByte = data[i];
		//std::cout << "[Crc8] useByte: " << (int) useByte << " [" << (int) i << "]" <<  std::endl; 
		crc ^= useByte;
		for (uint8_t j = 0; j < 8; j++)
		{
			if ((crc & 0x80) != 0)
				crc = (uint8_t)((crc << 1) ^ 0x31);
			else
				crc <<= 1;
		}
	}
	return crc;
}
