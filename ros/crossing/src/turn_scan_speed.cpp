//Turtlebot turning/scanning and speed measuring
//Author: Hongrui Zheng
//Oct.24 2014

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
bool turn = true;
bool turnRight = true;
bool turnLeft = false;
bool found = false;
bool measure = true;
bool jumpFound;
int countTol = 50;
int turnCount = 0;
int scanPoint = 20;
int preSeq =0;
float previousAvg;



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
        if (turnCount <= countTol && turnRight) {
            publish(-0.0, 0.0);
            turnCount++;
            if (turnCount = countTol) {
                turnRight = false;
                turnLeft = true;
            }
        }
        if (turnCount >= -countTol && turnLeft) {
            publish(0.0, 0.0);
            turnCount--;
            if (turnCount = -countTol) {
                turnRight = true;
                turnLeft = false;
            }
        }
    return;
}

void scan_callback( const sensor_msgs::LaserScan& _msg ){


    int i,j,cursor = 20, seq=_msg.header.seq;
    float critDis = 0.5;
    float arrayLength = (_msg.angle_max - _msg.angle_min) / _msg.angle_increment;
    float timeIncrement = _msg.time_increment;
    float speed, avg, previousAvg, sum =0;
    int total=0;

        // Find the jump and 6 consecutive points
        for (i = scanPoint; i < (arrayLength - scanPoint); i++) {
            if (fabs(_msg.ranges[i] - _msg.ranges[i+1]) >= critDis) {
                jumpFound = true;
                cursor = i;
            }
        }
        // calculate the sum and the average
        for (j = cursor; j < (cursor + 5); j++) {
		if(_msg.ranges[j]< _msg.range_max && _msg.ranges[j]> _msg.range_min )
            {sum += _msg.ranges[j];
		total++;}
		
        }
	if(total == 0){
	        for (j = cursor-5; j < (cursor ); j++) {
		if(_msg.ranges[j]< _msg.range_max && _msg.ranges[j]> _msg.range_min )
            {sum += _msg.ranges[j];
		total++;}
		}
		if(total==0)
		{	avg = 100;}
		else
		{avg = sum/total;}
	
	
	
	}
	else{
        avg = sum/total;}
        
	//calculate the speed (thisavg - previousavg)/(1/looprate)
        //speed = (previousAvg - avg) / ((seq-preSeq) *_msg.scan_time);
//speed = (previousAvg - avg) / (.1);
        //update
        previousAvg = avg;
        //ROS_INFO("Speed of moving object is %f m/s", speed);
	ROS_INFO("Moving object is at %f m", avg);
	//ROS_INFO("time is %f s", ((seq-preSeq) *_msg.scan_time));



        // Find the closest set of *6* points
        // for (i=numComp; i < (num - numComp); i++) {
        //     total = 2* numComp +1;
        //     sum = 0;
        //     for (j = i - numComp; j < (i + numComp); j++) {
        //         if(_msg.ranges[j] > _msg.range_min && _msg.ranges[j] < _msg.range_max ) {
        //             sum =_msg.ranges[j] + sum;
        //         } else  {
        //             total = total -1;
        //         }
        //     }

        //     if(sum/total < dis && sum/total > 0 ) {
        //         dis = sum/total;
        //         ang = _msg.angle_min + i* _msg.angle_increment;
        //     }
        // }

        // normalizedDis = dis * cos(ang); // distance in x direction with respect to object
        // speed = (speed + (normalizedDis - previousDis) / interval)/2; // averaging
        // ROS_INFO("Speed of moving object is &f m/s", speed);

        // // update
        // previousDis = normalizedDis;
  preSeq=seq;
    return;
}
