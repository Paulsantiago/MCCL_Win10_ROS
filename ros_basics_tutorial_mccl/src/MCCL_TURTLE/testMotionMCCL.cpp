
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

int InPos = 1 ;
//float dfCurPosX,dfCurPosX;
/*void InPositionCallback(const std_msgs::String::ConstPtr& msg)
{
    
    MCC_GetCurPos(&dfCurPosX, &dfCurPosY, &dfCurPosZ, 
					  &dfCurPosU, &dfCurPosV, &dfCurPosW, 
					  &dfCurPosA, &dfCurPosB, g_nGroupIndex);

	pose_message->position.x = dfCurPosX ; 
	pose_message->position.y = dfCurPosY;
	pose_message->position.z = dfCurPosZ;

}*/

void InPositionCallback(const std_msgs::String::ConstPtr& msg)
{
  if(msg->data.c_str() == "1")
  {
      ROS_INFO("Still moving: [%s]", msg->data.c_str());
      InPos = 1 ;
  }
  else
  {
     ROS_INFO("Still moving: [%s]", msg->data.c_str());
     InPos = 0 ;
  }
	
}




int main(int argc, char **argv)
{
  
// %Tag(INIT)%
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

// %Tag(PUBLISHER)% 
  ros::Publisher pos_pub = n.advertise<geometry_msgs::Pose>("SetPositionMCCL", 10);
  //ros::Subscriber get_Curpos_sub = n.subscribe("GetCurPositionMCCL", 5 , GetCurPosCallBack );
  ros::Subscriber get_InPos_sub = n.subscribe("GetInPositionMCCL", 1, InPositionCallback);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(20);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {

    if(InPos == 0)
    {
      geometry_msgs::Pose msg;
      msg.position.x = 5 * double(rand())/double(RAND_MAX);
      msg.position.y = 10 * double(rand())/double(RAND_MAX);  
      // %Tag(ROSCONSOLE)%
          ROS_INFO("pos x : %f", msg.position.x);
          ROS_INFO("pos y : %f", msg.position.y);
          pos_pub.publish(msg);
      // %EndTag(ROSCONSOLE)%
    }
   
// %Tag(PUBLISH)%
  
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
