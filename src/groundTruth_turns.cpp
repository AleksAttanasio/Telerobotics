#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include <geometry_msgs/PoseStamped.h>

int main (int argc, char **argv){

  ros::init(argc, argv, "GTPublisher");
  ros::NodeHandle n;

  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("path", 1000);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("position", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok()){
    nav_msgs::Path msg;
    geometry_msgs::PoseStamped poseMsg;

    msg.poses.resize(5);	
    msg.poses[0].pose.position.x = -1.4; 
    msg.poses[0].pose.position.y = -4.5; 
    msg.poses[0].pose.position.z = 7.5;

    msg.poses[1].pose.position.x = 1.49; 
    msg.poses[1].pose.position.y = 4.8; 
    msg.poses[1].pose.position.z = 7.5;


    msg.poses[2].pose.position.x = 7.4; 
    msg.poses[2].pose.position.y = 2.975; 
    msg.poses[2].pose.position.z = 7.5;

    msg.poses[3].pose.position.x = 9.29; 
    msg.poses[3].pose.position.y = 9.0; 
    msg.poses[3].pose.position.z = 7.5;

    msg.poses[4].pose.position.x = 7.15; 
    msg.poses[4].pose.position.y = 9.68; 
    msg.poses[4].pose.position.z = 7.5;

    poseMsg.pose.position.x =9.68;
    poseMsg.pose.position.y =7.15;
    poseMsg.pose.position.z =-7.5;

    pose_pub.publish(poseMsg);
    path_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
  return 0;
}
