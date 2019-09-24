#include "example.h"
ROS_MCCL::ROS_MCCL () {
  width = 5;
  height = 5;
}

ROS_MCCL::ROS_MCCL (int a, int b) {
  width = a;
  height = b;
}
ROS_MCCL::ROS_MCCL(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
    ROS_INFO("in class constructor of ROS_MCCL initializating .... ");
    if(initParameters())
	{
		initializeSubscribers(); // package up the messy work of creating subscribers; do this overhead in constructor
		initializePublishers();
		initializeServices();
		
		//initialize variables here, as needed
		val_Point.X=0.0; 
		val_Point.Y=0.0; 
		val_Point.Z=0.0; 
		val_Point.RX=0.0;
		val_Point.RY=0.0;
		val_Point.RZ=0.0;

		val_Joint.J1 = 0.0;
		val_Joint.J2 = 0.0;
		val_Joint.J3 = 0.0;
		val_Joint.J4 = 0.0;
		val_Joint.J5 = 0.0;
		val_Joint.J6 = 0.0;
		// can also do tests/waits to make sure all required services, topics, etc are alive
	}
    
}

void ROS_MCCL::initializeSubscribers()
{
    ROS_INFO("Initializing Subscribers");
    line_subs = nh_.subscribe("line_MCCL", 1, 		&ROS_MCCL::line_subscriberCallback,this);  
	ptp_subs = nh_.subscribe("ptp_MCCL", 1, 		&ROS_MCCL::ptp_subscriberCallback,this);  
	//cur_pos = nh_.subscribe("curPos_MCCL", 1, 		&ROS_MCCL::curPos_subscriberCallback,this);  
	//circle_subs = nh_.subscribe("circle_MCCL", 1, 	&ROS_MCCL::circle_subscriberCallback,this);  
	//arc_subs = nh_.subscribe("arc_MCCL", 1, 		&ROS_MCCL::arc_subscriberCallback,this);  
    // add more subscribers here, as needed
}

//member helper function to set up services:
// similar syntax to subscriber, required for setting up services outside of "main()"
void ROS_MCCL::initializeServices()
{
    ROS_INFO("Initializing Services");
    minimal_service_ = nh_.advertiseService("exampleMinimalService",
                                                   &ROS_MCCL::serviceCallback,
                                                   this);  
    // add more services here, as needed
}

//member helper function to set up publishers;
void ROS_MCCL::initializePublishers()
{
    ROS_INFO("Initializing Publishers");
    Curpos_publisher_ = nh_.advertise<ros_basics_tutorial_mccl::CartesianPoint& pointMsg>("Curpos_MCCL", 1, true); 
    //add more publishers, as needed
    // note: COULD make minimal_publisher_ a public member function, if want to use it within "main()"
}
void ROS_MCCL::line_subscriberCallback(const ros_basics_tutorial_mccl::CartesianPoint& pointMsg) {
    // the real work is done in this callback function
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"
    val_Point = moveMsg; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value x: %f , y : %f ",pointMsg.X,pointMsg.Y);
	nRet = MCC_Line( pointMsg.X,
				 	pointMsg.Y,
					pointMsg.Z,
					pointMsg.RX, 
					pointMsg.RY, 
					pointMsg.RZ, 0, 0, g_nGroupIndex);
  	ROS_INFO("Line callback : %d" , nRet);

}
void ROS_MCCL::ptp_subscriberCallback(const ros_basics_tutorial_mccl::Joint& jointMsg) {
    // it wakes up every time a new message is published on "exampleMinimalSubTopic"
    val_Joint = jointMsg; // copy the received data into member variable, so ALL member funcs of ExampleRosClass can access it
    ROS_INFO("myCallback activated: received value J1: %f , J2 : %f ",jointMsg.J1,jointMsg.J2);
	nRet = MCC_PtP( jointMsg.J1,
				 	jointMsg.J2,
					jointMsg.J3,
					jointMsg.J4, 
					jointMsg.J5, 
					jointMsg.J6, 0, 0, g_nGroupIndex);
  	ROS_INFO("MCC_PtP callback : %d" , nRet);
}




//member function implementation for a service callback function
bool ROS_MCCL::serviceCallback(ros_basics_tutorial_mccl::SetSpeedRequest& request, ros_basics_tutorial_mccl::SetSpeedResponse& response) {
    ROS_INFO("service callback activated");
    response.status = true; // boring, but valid response info
    return true;
}

bool ROS_MCCL::initParameters()
{
    int resp = MCC_SetSysMaxSpeed(100);//  set max. feed rate
	printf("resp %d \n" , resp);
     wCardType    = 2;
	nCommandMode = OCM_PULSE;//  P Command
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
		return false;
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
	//printf("Initialization : %d", nRet);

	if (nRet == NO_ERR)
	{
		printf("Initialization is successfull initVal  = %d !\n\n" , nRet );
		//MCC_SetServoOn(0, CARD_INDEX);//  set channel 0 servv on 
		//MCC_SetServoOn(1, CARD_INDEX);//  set channel 1 servv on 

		MCC_SetAbsolute(g_nGroupIndex);     //  use Absolute coordinate mode
		//  you must regulate accleration and deceleration time depending on different speed for a smooth moving		
		//  set line, arc and circle motion's accleration time
		MCC_SetAccTime(300, g_nGroupIndex);//  set accleration time to be 300 ms
		//  set line, arc and circle motion's deceleration time
		MCC_SetDecTime(300, g_nGroupIndex);//  set decleration time to be 300 ms

		MCC_SetFeedSpeed(10, g_nGroupIndex);//  set line, arc and circle motion's feed rate (unit : mm/sec)

        return true;
	}
	else
	{
		printf("Motion Initialization Error , initError  = %d  !\n\n", nRet);
		return false;									  
	}
}

int main (int argc, char **argv)
 {
     ros::init(argc, argv, "exampleRosClass"); //node name
	
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor

	ROS_MCCL rect (3,4);
	ROS_MCCL rectb;
	cout << "rect area: " << rect.area() << endl;
	cout << "rectb area: " << rectb.area() << endl;

    ROS_INFO("main: instantiating an object of type ExampleRosClass");
    ROS_MCCL exampleRosClass(&nh);  //instantiate an ExampleRosClass object and pass in pointer to nodehandle for constructor to use
    ROS_INFO("main: going into spin; let the callbacks do all the work");
    ros::spin();


  return 0;
}



