# include "RosMCCL.h"

/*ROS_MCCL::ROS_MCCL( ros::NodeHandle* nh)
: nh_(*nh)
{
  //position_sub_ = nh_.subscribe("cmd_position", 1, &ROS_MCCL::positionCallback, this);
  //position_pub_ = nh_.advertise<Pose>("CurPos", 1);
  //set_pen_srv_ = nh_.advertiseService("set_pen", &ROS_MCCL::setPenCallback, this);

 // initParameters();

}*/
 ROS_MCCL::ROS_MCCL(int on)
 {
     ON =on;
//printf(" MCC_Line() error !  Return Value : %d", ON);
 }

void ROS_MCCL::positionCallback(const geometry_msgs::Pose::ConstPtr& pos)
{
    int nRet = MCC_Line(pos->position->x, pos->position->y, 0, 0, 0, 0, 0, 0, g_nGroupIndex);
    if (nRet < NO_ERR)
			printf(" MCC_Line() error !  Return Value : %d", nRet);
	MCC_DelayMotion(200);
}

ROS_MCCL::update(double dt)
{

}
/*void initParameters()
{
    
    int resp = MCC_SetSysMaxSpeed(100);//  set max. feed rate
	printf("resp %d" , resp);
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
	for (WORD wChannel = 0;wChannel < AXIS_NUM;wChannel++)
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
    // INIT
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

		

	}
	else
	{
		printf("Motion Initialization Error !\n\n");
		return 0;									  
	}
  
}*/
