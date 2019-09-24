// %Tag(ROS_HEADER)%
#include "ros/ros.h"
// %EndTag(ROS_HEADER)%

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
// %Tag(INIT)%
  ros::init(argc, argv, "TSimCMD");
// %EndTag(INIT)%

  ros::NodeHandle n;

// %Tag(PUBLISHER)%
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
// %EndTag(PUBLISHER)%

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(10);
// %EndTag(LOOP_RATE)%

// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok())
  {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    geometry_msgs::Twist Vel;

    Vel.linear.x = double(rand())/double(RAND_MAX);
    Vel.linear.y = double(rand())/double(RAND_MAX);
    Vel.angular.z = 4*double(rand())/double(RAND_MAX) - 1;
    //ss << "hello Twist " << count;
    //Vel.linear.x = double
    //vel_pub = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%d", Vel.linear.x);
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    vel_pub.publish(Vel);
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