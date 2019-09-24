/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
 // %Tag(FULLTEXT)%
 // %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%
// %Tag(MSG_HEADER)%
#include "std_msgs/String.h"
// %EndTag(MSG_HEADER)%
#include <conio.h>
#include <stdio.h>
#include <sstream>
#include "../include/MCCL_Fun.h"


#define CARD_INDEX				    0
#define BASE_ADDRESS            0x240  //in PCI, user can input arbitrary number
#define IRQ_NO                      5  //in PCI, user can input arbitrary number
#define INTERPOLATION_TIME          2  // ms
#define ESC_KEY                    27

int g_nGroupIndex = -1;


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char** argv)
{
	
	char cKey;
	WORD wCardType;
	int	 nRet, nCommandMode;
	double dfCurPosX, dfCurPosY, dfCurPosZ, dfCurPosU, dfCurPosV, dfCurPosW, dfCurPosA, dfCurPosB;
	SYS_MAC_PARAM      stMacParam;
	SYS_ENCODER_CONFIG stENCConfig;
	SYS_CARD_CONFIG    stCardConfig[MAX_CARD_NUM];
	 wCardType    = 2;
	nCommandMode = OCM_PULSE;//  P Command
	
	int resp = MCC_SetSysMaxSpeed(100);//  set max. feed rate
	printf("rsp %d" , resp);
	    //  set mechanism parameters
	stMacParam.wPosToEncoderDir          = 0;
	stMacParam.dwPPR			         = 1000;
	stMacParam.wRPM                      = 3000; 
	stMacParam.dfPitch                   = 1.0;
	stMacParam.dfGearRatio               = 1.0;
	stMacParam.dfHighLimit               = 50000.0;
	stMacParam.dfLowLimit                = -50000.0; 
	stMacParam.dfHighLimitOffset         = 0;
	stMacParam.dfLowLimitOffset          = 0;
	stMacParam.wPulseMode                = DDA_FMT_PD; 
	stMacParam.wPulseWidth               = 100; 
	stMacParam.wCommandMode              = nCommandMode;
	stMacParam.wOverTravelUpSensorMode   = 2;//  not checking
	stMacParam.wOverTravelDownSensorMode = 2;

	//  set encoder configures
	stENCConfig.wType                    = ENC_TYPE_AB;
	stENCConfig.wAInverse                = _NO_;
	stENCConfig.wBInverse                = _NO_;
	stENCConfig.wCInverse                = _NO_;
	stENCConfig.wABSwap                  = _NO_;
	stENCConfig.wInputRate               = 4;//  set encoder input rate : x4


	//mechanism and ENC parameters are seperated from each other, we set them all the same.
	for (WORD wChannel = 0;wChannel < MAX_AXIS_NUM;wChannel++)
	{
		MCC_SetMacParam(&stMacParam, wChannel, CARD_INDEX);      //  mechanism parameters are the same for all axes
		MCC_SetEncoderConfig(&stENCConfig, wChannel, CARD_INDEX);//  encoder configures are the same for all axes
	}
    
	//  set group parameters
	MCC_CloseAllGroups();
	g_nGroupIndex = MCC_CreateGroup(0, 1, 2, 3, 4, 5, 6, 7,  CARD_INDEX);

	if( g_nGroupIndex < GROUP_VALID )
	{
		printf("Groups create error !\n\n");
		return 0;
	}

	
	//  stCardConfig is used to set card's base address and card style, and set one card's attributes now
	stCardConfig[CARD_INDEX].wCardType    = wCardType;   //  2 : 4-axis card
								                         //  3 : 6-axis card
								                         //  4 : 8-axis card

	stCardConfig[CARD_INDEX].wCardAddress = BASE_ADDRESS;//  base address, PCI card ignores this setting
	stCardConfig[CARD_INDEX].wIRQ_No      = IRQ_NO;      //  IRQ No., PCI card ignores this setting
	stCardConfig[CARD_INDEX].wPaddle      = 0;

	nRet = MCC_InitSimulation(INTERPOLATION_TIME,// set interpolation time interval and get some errors happening or not
						  stCardConfig,
						  1);		         //  only use one card

	if (nRet == NO_ERR)
	{
		printf("Initialization is successfull !\n\n");
		//MCC_SetServoOn(0, CARD_INDEX);//  set channel 0 servv on 
		//MCC_SetServoOn(1, CARD_INDEX);//  set channel 1 servv on 

		MCC_SetAbsolute(g_nGroupIndex);     //  use Absolute coordinate mode

		//  you must regulate accleration and deceleration time depending on different speed for a smooth moving		
		//  set line, arc and circle motion's accleration time
		MCC_SetAccTime(300, g_nGroupIndex);//  set accleration time to be 300 ms
		 
		//  set line, arc and circle motion's deceleration time
		MCC_SetDecTime(300, g_nGroupIndex);//  set decleration time to be 300 ms

		MCC_SetFeedSpeed(10, g_nGroupIndex);//  set line, arc and circle motion's feed rate (unit : mm/sec)

		nRet = MCC_Line(10, 10, 0, 0, 0, 0, 0, 0, g_nGroupIndex);
		if (nRet < NO_ERR)
			printf(" MCC_Line() error !  Return Value : %d", nRet);
		MCC_DelayMotion(200);

	}
	else
	{
		printf("Motion Initialization Error !\n\n");
		return 0;									  
	}

	while (1)
	{
		if (_kbhit())
		{
			cKey = getch();

			if (cKey == ESC_KEY)
			{
				MCC_AbortMotionEx(150, g_nGroupIndex);//  stop current motion with a decelerating style and discard all motion commands in a command queue
				// if you want to stop current motion immediatly, use the first parameter be 0
				break;
			}
		}

		//  get current position (unit : mm)
		MCC_GetCurPos(&dfCurPosX, &dfCurPosY, &dfCurPosZ, 
					  &dfCurPosU, &dfCurPosV, &dfCurPosW, 
					  &dfCurPosA, &dfCurPosB, g_nGroupIndex);

		printf("Current Pos. : x = %6.2f    y = %6.2f    \r", dfCurPosX, dfCurPosY);
		MCC_TimeDelay(10);
	}

	MCC_CloseSystem();//  close motion control C library (MCCL)


    printf("rsp %d" , resp);
    int a = getchar();
				
				
				
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	 // %Tag(INIT)%
	ros::init(argc, argv, "MCCL_talker");
	// %EndTag(INIT)%

	  /**
	   * NodeHandle is the main access point to communications with the ROS system.
	   * The first NodeHandle constructed will fully initialize this node, and the last
	   * NodeHandle destructed will close down the node.
	   */
	   // %Tag(NODEHANDLE)%
	ros::NodeHandle n;
	// %EndTag(NODEHANDLE)%

	  /**
	   * The advertise() function is how you tell ROS that you want to
	   * publish on a given topic name. This invokes a call to the ROS
	   * master node, which keeps a registry of who is publishing and who
	   * is subscribing. After this advertise() call is made, the master
	   * node will notify anyone who is trying to subscribe to this topic name,
	   * and they will in turn negotiate a peer-to-peer connection with this
	   * node.  advertise() returns a Publisher object which allows you to
	   * publish messages on that topic through a call to publish().  Once
	   * all copies of the returned Publisher object are destroyed, the topic
	   * will be automatically unadvertised.
	   *
	   * The second parameter to advertise() is the size of the message queue
	   * used for publishing messages.  If messages are published more quickly
	   * than we can send them, the number here specifies how many messages to
	   * buffer up before throwing some away.
	   */
	   // %Tag(PUBLISHER)%
	ros::Publisher MCCL_Joint_Pub = n.advertise<std_msgs::String>("MCCL_Joint", 1000);
	// %EndTag(PUBLISHER)%

	// %Tag(LOOP_RATE)%
	ros::Rate loop_rate(10);
	// %EndTag(LOOP_RATE)%

	  /**
	   * A count of how many messages we have sent. This is used to create
	   * a unique string for each message.
	   */
	   // %Tag(ROS_OK)%
	int count = 0;
	while (ros::ok())
	{
		// %EndTag(ROS_OK)%
			/**
			 * This is a message object. You stuff it with data, and then publish it.
			 */
			 // %Tag(FILL_MESSAGE)%
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		// %EndTag(FILL_MESSAGE)%

		// %Tag(ROSCONSOLE)%
		ROS_INFO("%s", msg.data.c_str());
		// %EndTag(ROSCONSOLE)%
			/**
			 * The publish() function is how you send messages. The parameter
			 * is the message object. The type of this object must agree with the type
			 * given as a template parameter to the advertise<>() call, as was done
			 * in the constructor above.
			 */
			 // %Tag(PUBLISH)%
		MCCL_Joint_Pub.publish(msg);
		// %EndTag(PUBLISH)%

		// %Tag(SPINONCE)%
		ros::spinOnce();
		// %EndTag(SPINONCE)%

		// %Tag(RATE_SLEEP)%
		loop_rate.sleep();
		// %EndTag(RATE_SLEEP)%
		++count;
	}


	return 0;
}
// %EndTag(FULLTEXT)%