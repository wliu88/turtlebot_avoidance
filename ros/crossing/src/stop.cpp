//
//  Stop.cpp
//  Crossing_Xcode
//
//  Created by Weiyu Liu on 10/25/14.
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


//global variables

float posX = 0;
float startX = 0;
int numOfRecordOdm = 0;
double odmTolerance;// = 0.1; // in meter  //?????

bool measureOrNot = false;
float rightmostDis = 0;
double criticalDis;// = 0.5;  //?????
int numOfRecord = 0;
float forwardDis = 0;

int scanPoint;// = 50;  //?????


//int Num_comp=3;
//float ang_gain = .1, lin_gain = .07, d=1;

ros::Publisher pub;

int main( int argc, char** argv ) {
    
    ros::init(argc, argv, "follow");
    
    ros::NodeHandle nh;
    ros::NodeHandle odm;
    
    // This is experimental control of ros parameters
    ros::NodeHandle p("~");
    
    p.param("Odometry_Tolerance",odmTolerance, 0.01 );
    p.param("Critical_Distance",criticalDis, 0.5 );
    p.param("Scan_Point",scanPoint, 10 );
    
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

// odemetry contorls velocity of robot. Initially robot is moving forward.
void odometry_callback( const nav_msgs::Odometry& _msg ){
    
    publish(0.0,0.1);
    
    if (measureOrNot == true) {
        posX = sqrt(pow(_msg.pose.pose.position.x,2)+pow(_msg.pose.pose.position.y,2));
        
        // set up initial position
        if (numOfRecordOdm == 0) {
            startX = posX;
            numOfRecordOdm++;
        }
        
        float error = fabs(posX - startX - forwardDis);
        if (error < odmTolerance) {
            publish(0.0, 0.0);
        }
        ROS_INFO("%f",error);
    }
    
    return;
}

void scan_callback( const sensor_msgs::LaserScan& _msg ){
    
    // Only using the right-most point of measured ranges to trigometrically determine the distance
    // to cross. Other determination is the sudden change of left-most point of measured ranges.
    
    //float num;
    //float num = (_msg.angle_max - _msg.angle_min) / _msg.angle_increment;
    //int length = sizeof(_msg.ranges) / sizeof(_msg.ranges[0]);
    
    // 1.
    // for the first measurement
    if (measureOrNot == false) {
        if(numOfRecord == 0) {
            if(_msg.ranges[scanPoint] < _msg.range_min || _msg.ranges[scanPoint] > _msg.range_max) {
                ROS_INFO("Please move the turtlebot to valid starting position");
                ros::shutdown();
            } else {
                rightmostDis = _msg.ranges[scanPoint];
                numOfRecord++;
            }
            // for the following measurements
        } else {
            if(_msg.ranges[scanPoint] > _msg.range_min && _msg.ranges[scanPoint] < _msg.range_max)	{
                if(_msg.ranges[scanPoint] > rightmostDis + criticalDis) {
                    measureOrNot = true;
                } else {
                    rightmostDis = _msg.ranges[scanPoint];
                    numOfRecord++;
                }
            }
        }
        
        // 2.	
        // Condition Satisfied. Proceed to determine forward distance.
        if (measureOrNot == true) {
            forwardDis = rightmostDis * cos(_msg.angle_min + scanPoint*_msg.angle_increment);		
        }
        
        ROS_INFO("%f",_msg.ranges[scanPoint]);
        ROS_INFO("%f",forwardDis);
        return;
    }
}

