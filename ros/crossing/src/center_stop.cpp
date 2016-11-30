//
//  center_stop.cpp
//  
//  Revised by Weiyu Liu, Carol Young, Hongrui Zheng on 11/10/14. 
//  Revised by Weiyu Liu on 11/17/14
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
double odmTolerance = 0.01; // in meter

bool measureOrNot = false;
bool wallCentering = true;
bool detectingEnd = false;
float rightmostDis = 0;
double criticalDis = 0.5;
int numOfRecord = 0;
float forwardDis = 0;

bool turnRight = false, turnLeft = false, goStraight = true;
int scanPoint = 50;
float cutoff = 0.1;
float gain = .25;
int straight = 0;
float old_right, old_left;

//int Num_comp=3;
//float ang_gain = .1, lin_gain = .07, d=1;

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

// odemetry contorls velocity of robot. Initially robot is moving forward.
void odometry_callback( const nav_msgs::Odometry& _msg ){
    
    if (measureOrNot == false) {
	if (turnRight && !goStraight) {
        publish(-1 * gain * 1, 0.1);
    	}

    	if (turnLeft && !goStraight) {
        publish(gain * 1, 0.1);
    	}

    	if (goStraight) {
        publish(0.0, 0.1);
    	}
    }
    
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
        } else {
	    publish(0.0,0.1);
	}
    } 
    
    return;
}

void scan_callback( const sensor_msgs::LaserScan& _msg ){

	// get the length of the ranges[] array
	int arrayLength = (_msg.angle_max - _msg.angle_min) / _msg.angle_increment;
 	float leftDis;
	float rightDis;
	
        // get the distance to the right
	if(_msg.ranges[arrayLength - scanPoint - 12] > _msg.range_min || _msg.ranges[arrayLength - scanPoint - 12] < _msg.range_max) {
                leftDis = _msg.ranges[arrayLength - scanPoint - 12];
	} else{
		leftDis = old_left;
	}

        // left and right edge of the data has a difference of 12 points
        // get the distance to the left
        if(_msg.ranges[ scanPoint] > _msg.range_min || _msg.ranges[scanPoint] < _msg.range_max) {
                rightDis = _msg.ranges[scanPoint];
	} else {
		rightDis = old_right;
	}

    	if(measureOrNot == false) {
    		turnRight = false;
    		turnLeft = false; 
    		goStraight = false;

   		// The gain is used to adjust the angular velocity to make the turn more smoothly.
    		gain = fabs(rightDis - leftDis);

    		// Here the action states are determined. A cutoff are used to ignore negligible noises.
    		if (rightDis > leftDis + cutoff) {
        		turnRight = true;
    		} else if (rightDis + cutoff < leftDis) {
        		turnLeft = true;
    		 }else {
        		goStraight = true;
        		straight++;	
    		}


    		// Only using the right-most point of measured ranges to trigometrically determine the distance
    		// to cross. Other determination is the sudden change of left-most point of measured ranges.

        	if(numOfRecord == 0) {
			old_right=rightDis;
			old_left=leftDis;
			numOfRecord++;
		}

 		if((rightDis > old_right + criticalDis) ||(leftDis > old_left + criticalDis) ) {
			measureOrNot = true;
        	}

		old_right=rightDis;
		old_left=leftDis; 
        }
        
        // 2.	
        // Condition Satisfied. Proceed to determine forward distance.
        if (measureOrNot == true) {
            forwardDis = old_right * cos(_msg.angle_min + scanPoint*_msg.angle_increment);		
        }
        
   return;
}

