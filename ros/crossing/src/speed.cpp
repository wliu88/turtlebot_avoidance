#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>


//functions
void scan_callback( const sensor_msgs::LaserScan& _msg );


//global variables
int numComp=3;
float normalizedDis = 0;
float previousDis = 0;


int main( int argc, char** argv ) {

  ros::init(argc, argv, "follow");
  
    ros::NodeHandle nh;
  
    ros::Subscriber scan = nh.subscribe( "scan", 1, scan_callback);

  ros::Rate loop_rate(10);

  while( ros::ok()) {
    ros::spinOnce();
  }

  return(0);
}


void scan_callback( const sensor_msgs::LaserScan& _msg ){
	int total,i,j;
	float num = (_msg.angle_max - _msg.angle_min) / _msg.angle_increment;
	float interval = _msg.time_increment;
	float dis = 10, ang =-10, sum;
	float av,lv;
	float speed;

	// Find the closest set of *6* points
	for (i=numComp; i < (num - numComp); i++) {
		total = 2* numComp +1;
		sum = 0;
		for (j = i - numComp; j < (i + numComp); j++) {
			if(_msg.ranges[j] > _msg.range_min && _msg.ranges[j] < _msg.range_max ) {
				sum =_msg.ranges[j] + sum;
			} else	{
				total = total -1;
			}
		}
			
		if(sum/total < dis && sum/total > 0 ) {
			dis = sum/total;
			ang = _msg.angle_min + i* _msg.angle_increment;
		}
	}
	
	normalizedDis = dis * cos(ang); // distance in x direction with respect to object
	speed = (speed + (normalizedDis - previousDis) / interval)/2; // averaging
	ROS_INFO("Speed of moving object is %f m/s", speed);	
	
	// update
	previousDis = normalizedDis;

	return;
}
