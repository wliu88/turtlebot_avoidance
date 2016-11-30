#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>


#define PI 3.14159265

//functions


void scan_callback( const sensor_msgs::LaserScan& _msg );


//global variables

ros::Publisher pub;

int main( int argc, char** argv ) {

  ros::init(argc, argv, "human_tracker");
  
    ros::NodeHandle nh;
 
    ros::Subscriber scan = nh.subscribe( "scan", 1, scan_callback);
     pub = nh.advertise<geometry_msgs::Pose2D>("object", 1);
  ros::Rate loop_rate(10);

  while( ros::ok()) {
    ros::spinOnce();
  }

  return(0);
}


void scan_callback( const sensor_msgs::LaserScan& _msg ){

float threshold = 0.8;
int objectBegin = 0;
int objectEnd = 0;
bool detectionOn = false;
bool detected = false;

geometry_msgs::Pose2D pose;

int num = 640;

float array[640]={2,0};
float object[640] = {0};

float diff = 0;



// Find the closest set of *6* points

for (int i=0;i<num-2;i++){
	if(isnan(_msg.ranges[i+1])==true) {
		array[i+1] = array[i]; 				
	} else {
		array[i+1] = _msg.ranges[i+1];
	}
}

for (int i = 1; i<= num-1; i++) {
	diff = fabs(array[i]-array[i-1]);
	if(diff >= threshold) {
		if(detectionOn == false && detected ==false) {
			objectBegin = i;
			detectionOn = true;
		} else if (detectionOn == true){
			objectEnd = i-1;
			detectionOn = false;
			detected = true;
		}
	}
}

float avg = 0;
float position = 0;
int count = 0;

for (int i =0;i <num; i++) {
        if(i >= objectBegin && i <= objectEnd) {
		if(isnan(_msg.ranges[i])==false) {
			object[i] = _msg.ranges[i];
			avg = object[i] + avg;
			position = i + position;
			count++;		
		}
	}	
}

avg = avg/count;
position = position/count;

ROS_INFO("%i ...... %i", objectBegin, objectEnd);

if(detected==true) {

pose.x =avg;
pose.y =position;
pose.theta =  (_msg.angle_min + (position * _msg.angle_increment)) *180 /PI;

pub.publish(pose);


}
return;
}

