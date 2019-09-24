#ifndef ROS_MCCL_H
#define ROS_MCCL_H

// This prevents a MOC error with versions of boost >= 1.48
//#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <ros/ros.h>
# include <boost/shared_ptr.hpp>
# include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
 #include "../include/MCCL_Fun.h"

//#endif
 //#include "ros_basics_tutorial_mccl/SetSpeed.h"
#define PI 3.14159265
#define CARD_INDEX				    0
#define BASE_ADDRESS            0x240  //in PCI, user can input arbitrary number
#define IRQ_NO                      5  //in PCI, user can input arbitrary number
#define INTERPOLATION_TIME          2  // ms
#define ESC_KEY                    27
#define AXIS_NUM                    2
class ROS_MCCL
{
    int ON;
    public:
    //ROS_MCCL(ros::NodeHandle* nh);
    ROS_MCCL(int);
    bool update(double dt);
    SYS_MAC_PARAM      stMacParam;
    SYS_ENCODER_CONFIG stENCConfig;
    SYS_CARD_CONFIG    stCardConfig[AXIS_NUM];
        

    //private:

    /*void positionCallback(const geometry_msgs::Pose::ConstPtr& position);
    
    ros::NodeHandle nh_;
    bool pen_on_;
    ros::Subscriber position_sub_;
    ros::Publisher position_pub_;
    ros::ServiceServer set_line_speed_;
    ros::ServiceServer set_ptp_speed_;
    int g_nGroupIndex = -1;
    double dfCurPosX, dfCurPosY, dfCurPosZ, dfCurPosU, dfCurPosV, dfCurPosW, dfCurPosA, dfCurPosB;
    double dfPosX ;
    double dfPosY ;
    WORD wCardType;
    int	 nRet, nCommandMode;*/
        //wCardType    = 2;
        //nCommandMode = OCM_PULSE;//  P Command


};


#endif