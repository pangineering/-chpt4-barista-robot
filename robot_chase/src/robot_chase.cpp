#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_chase");
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Publisher cmdVelPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Rate rate(10.0); // Adjust the rate as needed

  while (ros::ok()) {
    geometry_msgs::TransformStamped transformStamped;

    try {
      // Get the latest transform between rick/base_link and morty/base_link
      transformStamped =
          tfBuffer.lookupTransform("rick/base_link", "morty/base_link",
                                   ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    // Compute the distance and angular error between Rick and Morty
    double distanceError =
        sqrt(pow(transformStamped.transform.translation.x, 2) +
             pow(transformStamped.transform.translation.y, 2));
    double angularError = atan2(transformStamped.transform.translation.y,
                                transformStamped.transform.translation.x);

    // Create the velocity command
    geometry_msgs::Twist cmdVel;
    cmdVel.linear.x =
        0.5 * distanceError; // Adjust the proportional gain as needed
    cmdVel.angular.z =
        1.0 * angularError; // Adjust the proportional gain as needed

    // Publish the velocity command
    cmdVelPub.publish(cmdVel);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
