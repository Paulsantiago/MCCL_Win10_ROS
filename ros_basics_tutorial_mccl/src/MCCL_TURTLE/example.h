#ifndef MCCL_ROS_CLASS_H_
#define MCCL_ROS_CLASS_H_

#include <iostream>
//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include "ros_basics_tutorial_mccl/SetSpeed.h"
#include "ros_basics_tutorial_mccl/Move.h"

# include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "../include/MCCL_Fun.h"


using namespace std;

#define PI 3.14159265
#define CARD_INDEX				    0
#define BASE_ADDRESS            0x240  //in PCI, user can input arbitrary number
#define IRQ_NO                      5  //in PCI, user can input arbitrary number
#define INTERPOLATION_TIME          2  // ms
#define ESC_KEY                    27
#define AXIS_NUM                    2


class ROS_MCCL {
    int width, height;



  public:
    ROS_MCCL ();
    ROS_MCCL (int,int);
    ROS_MCCL(ros::NodeHandle* nodehandle); //"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    int area (void) {return (width*height);}
    //bool update(double dt);
    SYS_MAC_PARAM      stMacParam;
    SYS_ENCODER_CONFIG stENCConfig;
    SYS_CARD_CONFIG    stCardConfig[AXIS_NUM];
private:
    // put private member data here;  "private" data will only be available to member functions of this class;
    ros::NodeHandle nh_; // we will need this, to pass between "main" and constructor
    ros::Subscriber line_subs; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber ptp_subs; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber circle_subs; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber arc_subs; //these will be set up within the class constructor, hiding these ugly details
    ros::Subscriber cur_pos; //these will be set up within the class constructor, hiding these ugly details
  
    ros::Publisher  Curpos_publisher_;


    ros::ServiceServer minimal_service_;
    

    ros_basics_tutorial_mccl::CartesianPoint val_Point; //example member variable: better than using globals; convenient way to pass data from a subscriber to other member functions
    ros_basics_tutorial_mccl::Joint val_Joint; // member variables will retain their values even as callbacks come and go
    
    int g_nGroupIndex = -1;
    double dfCurPosX, dfCurPosY, dfCurPosZ, dfCurPosU, dfCurPosV, dfCurPosW, dfCurPosA, dfCurPosB;
    double dfPosX ;
    double dfPosY ;
    WORD wCardType;
    int	 nRet, nCommandMode;
    
     // member methods as well:
    void initializeSubscribers(); // we will define some helper methods to encapsulate the gory details of initializing subscribers, publishers and services
    void initializePublishers();
    void initializeServices();
    bool initParameters();
    void line_subscriberCallback(const ros_basics_tutorial_mccl::CartesianPoint& pointMsg);
    void ptp_subscriberCallback(const ros_basics_tutorial_mccl::Joint& jointMsg);
    void curPos_subscriberCallback(const ros_basics_tutorial_mccl::Joint& jointMsg);
    //void circle_subscriberCallback(const ros_basics_tutorial_mccl::CartesianPoint& pointMsg);
    //void arc_subscriberCallback(const ros_basics_tutorial_mccl::CartesianPoint& pointMsg);

    //prototype for callback for example service
    bool serviceCallback(ros_basics_tutorial_mccl::SetSpeedRequest & request, ros_basics_tutorial_mccl::SetSpeedResponse & response);
}; // note: a class definition requires a semicolon at the end of the definition

#endif  // this closes the header-include trick...ALWAYS need one of these to match #ifndef