//
//  WallCentering.cpp
//  Version 2.0
//  Created by Hongrui Zheng on 10/30/14.
//  Revised by Carol Young on 11/03/14.
//  Revised by Weiyu Liu on 11/10/14
//
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


//functions

void publish( double _angular, double _linear );
void scan_callback( const sensor_msgs::LaserScan& _msg );
void odometry_callback( const nav_msgs::Odometry& _msg );


// global variables

bool turnRight = false, turnLeft = false, goStraight = true;
int scanPoint = 50;
float cutoff = 0.1;
float gain = .25;

ros::Publisher pub;

int main( int argc, char** argv ) {

    ros::init(argc, argv, "follow");

    ros::NodeHandle nh;
    ros::NodeHandle odm;

    ros::Subscriber scan = nh.subscribe( "scan", 1, scan_callback);
    ros::Subscriber subodm = odm.subscribe( "odom", 1, odometry_callback);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(10);

    while( ros::ok()) {
        ros::spinOnce();
    }

    return(0);
}

void publish( double _angular, double _linear ) {

    geometry_msgs::Twist vel;
    vel.angular.z = _angular;
    vel.linear.x = _linear;

    pub.publish(vel);

    return;
}

void odometry_callback( const nav_msgs::Odometry& _msg ){

    // when turnRight is true and goStraight is false
    // publish following velocities

    if (turnRight && !goStraight) {
        publish(-1 * gain * 1, 0.1);
    }
    // when turnLeft is true and goStraight is false
    // publish following velocities

    if (turnLeft && !goStraight) {
        publish(gain * 1, 0.1);
    }
    // when goStraight is true, publish following velocities

    if (goStraight) {
        publish(0.0, 0.1);
    }

    return;
}

void scan_callback( const sensor_msgs::LaserScan& _msg ){

    turnRight = false;
    turnLeft = false; 
    goStraight = false;
    
    // get the length of the ranges[] array
    int arrayLength = (_msg.angle_max - _msg.angle_min) / _msg.angle_increment;

    // get the distance to the right
    float rightDis = _msg.ranges[scanPoint];

    // left and right edge of the data has a difference of 12 points
    // get the distance to the left
    float leftDis = _msg.ranges[arrayLength - scanPoint - 12];

    // The gain is used to adjust the angular velocity to make the turn more smoothly.
    gain = fabs(rightDis - leftDis);

    //ROS_INFO("%f, %f ", rightDis, leftDis);

    // Here the action states are determined. A cutoff are used to ignore negligible noises.
    if (rightDis > leftDis + cutoff) {
        turnRight = true;
    } else if (rightDis + cutoff < leftDis) {
        turnLeft = true;
    } else {
        goStraight = true;	
    }

    return;
}
