#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <math.h>
#include <stdlib.h>
#define PI 3.14159265


//functions
void scan_callback( const sensor_msgs::LaserScan& _msg);
ros::Publisher pub;
ros::Publisher pub2;


// global variables
int direction; // 0 for left
                   // 1 for right
                   // 2 for center
int direction_vote[3];
float previous_distance;
float previous_angle;
int direction_count;


int main( int argc, char** argv ) {
    
// global variable
direction = 0;
direction_vote[0] = 0;
direction_vote[1] = 0;
direction_vote[2] = 0;
previous_distance = 10;
previous_angle = 0;
direction_count = 0;

    ros::init(argc, argv, "follow");
    
    ros::NodeHandle nh;
    
    ros::Subscriber scan = nh.subscribe( "scan", 10, scan_callback);
    pub = nh.advertise<std_msgs::Float32MultiArray>("xy", 1); // debug
    //pub2 = nh.advertise<std_msgs::Float32>("y", 1); // debug
    
    
    ros::Rate loop_rate(5);
    
    while( ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return(0);
}


void scan_callback( const sensor_msgs::LaserScan& _msg ){
    
    // 1. preliminary information
    //ROS_INFO("length is %ld\n", sizeof(_msg.ranges)/ sizeof(_msg.ranges[1]));
    //ROS_INFO("Ranges array length is %d\n", length);
    //ROS_INFO("Angle_max is %f\n", _msg.angle_max);
    //ROS_INFO("Angle_min is %f\n", _msg.angle_min);
    
    
    // 2. declaration and initialization
    int length = 640; //(int) round((_msg.angle_max - _msg.angle_min) / _msg.angle_increment);
    float ranges_copy[length];
    float range_min = _msg.range_min;
    float range_max = _msg.range_max;
    float theta = 0;
    float rho = 0;
    float alpha = 0;
    int rmatch = 0;
    int max = 0;
    int row_max = 0;
    int col_max = 0;
    
    // copy ranges to local array
    // may also use memcpy
    for (int index = 0; index < length; index++) {
        ranges_copy[index] = _msg.ranges[index];
    }
    
    
    // 3. first vote dealing with left half
    
    // initialize vote matrix
    int vote[25][18];
    for (int row = 0; row < 25; row++) {
        for (int col = 0; col < 18; col++) {
            vote[row][col] = 0;
        }
    }
    
    // vote
    for (int index = 0; index < 320; index++) {
        theta = 28.0 / 320 * (320 - index); // may use angle_max and angle_min
        rho = ranges_copy[length - index - 1];
        if ((rho <= range_max) && (rho >= range_min)) {
            for (int index2 = 0; index2 < 18; index2++) {
                alpha = index2 * 2.5 - 22.5;
                rmatch = round(rho * sin((theta + alpha) * PI / 180) / 0.1);
                if (rmatch >= 0 && rmatch < 25) {
                    vote[rmatch][index2]++;
                }
            }
        }
    }
    
    //printf("first vote: \n");
    for (int row = 0; row < 25; row++) {
        for (int col = 0; col < 18; col++) {
            if (vote[row][col] >= max) {
                // in this case, if more than one maximum values exist, take the last one, the program may be
                // modified to accomendate multiple maximum values.
                max = vote[row][col];
                row_max = row;
                col_max = col;
            }
            //printf("%d, ", vote[row][col]);
        }
    }
    
    alpha = col_max * 2.5 - 22.5;
    for (int index = 0; index < 320; index++) {
        theta = 28.0 / 320 * (320 - index);
        rho = ranges_copy[length - index - 1];
        if (rho <= range_max && rho > range_min) {
            rmatch = round(rho * sin((theta + alpha) * PI / 180) / 0.1);
            if (abs(rmatch - row_max) < 4) {
                ranges_copy[length - index - 1] = 0;
            }
        }
    }

    
    // 4. second vote dealing with right half
    
    // initialize vote matrix
    max = 0;
    row_max = 0;
    col_max = 0;
    rmatch = 0;
    
    // initialize vote matrix
    for (int row = 0; row < 25; row++) {
        for (int col = 0; col < 18; col++) {
            vote[row][col] = 0;
        }
    }
    
    for (int index = 320; index < 640; index++) {
        theta = 28.0 / 320 * (index - 320);
        rho = ranges_copy[length - index - 1];
        if ((rho <= range_max) && (rho >= range_min)) {
            for (int index2 = 0; index2 < 18; index2++) {
                alpha = index2 * 2.5 - 22.5;
                rmatch = round(rho * sin((theta + alpha) * PI / 180) / 0.1);
                if (rmatch >= 0 && rmatch < 25) {
                    vote[rmatch][index2]++;
                }
            }
        }
    }
    
    //printf("\nsecond vote: \n");
    for (int row = 0; row < 25; row++) {
        for (int col = 0; col < 18; col++) {
            if (vote[row][col] > max) {
                max = vote[row][col];
                row_max = row;
                col_max = col;
            }
            //printf("%d, ", vote[row][col]);
        }
    }
    
    alpha = col_max * 2.5 - 22.5;
    for (int index = 320; index < 640; index++) {
        theta = 28.0 / 320 * (index - 320);
        if (index == 403) {
            int i =0;
        }
        rho = ranges_copy[length - index - 1];
        if (rho <= range_max && rho >= range_min) {
            rmatch = round(rho * sin((theta + alpha) * PI / 180) / 0.1);
            if (abs(rmatch - row_max) < 4) {
                ranges_copy[length - index - 1] = 0;
            }
        }
    }

    // debug: to publish a message containing vote
    /*
    std_msgs::Int32MultiArray out;
    for (int row = 0; row < 25; row++) {
        for (int col = 0; col < 18; col++) {
            out.data[row * 18 + col] = vote2[row][col];
        }
    }
    pub2.publish(out);
    
    ROS_INFO("max row 2 is %d\n", row_max2);
    ROS_INFO("max col 2 is %d\n", col_max2);
    */
    
    
    // debug: to publish a message containing modified range values
    /*
    sensor_msgs::LaserScan out = _msg;
    for( int i=0; i< 640; i++) {
        out.ranges[i] =ranges_copy[i];
    }
    pub.publish(out);
    */
    
    
    
    // 5. detect segments
    int index = 2;
    int begin_point = 0;
    int end_point = 0;
    int count = 0;
    float min = 0;
    int arrSeg[10] = {0,0,0,0,0,0,0,0,0,0};
    float arrMin[10] = {0,0,0,0,0,0,0,0,0,0};
    int seg_index = 0;
    int min_index = 0;
    float current = 0;
    float previous = 0;
    float threshold = 0.5;
    
    
    // ?? problem without using threshold to detect jump
    while (index < 640) {
        if (current == 0) {
            current = ranges_copy[index];
            previous = current;
        } else {
            previous = current;
            current = ranges_copy[index];
        }
        if (current <= range_max && current >= range_min && fabs(current - previous) < threshold) { //possible place to add threshold
            if (begin_point == 0) {
                begin_point = index;
                arrSeg[seg_index++] = begin_point;
                end_point = index;
                min = current;
            }
            if (current < min) {
                min = current;
            }
            count = 0;
        } else {
            count++;
            if (count > 5) {
                if (begin_point != 0) {
                    end_point = index - count;
                    if (end_point - begin_point > 20) {
                        arrSeg[seg_index++] = end_point;
                        arrMin[min_index++] = min;
                        begin_point = 0;
                        count = 0;
                    } else {
                        arrSeg[--seg_index] = 0;
                        count = 0;
                        begin_point = 0;
                        end_point = 0;
                    }
                }
            }
        }
        index++;
    }
    
    
    
    // 6. print out results
    /*
    ROS_INFO("Printing segments:\n");
    for (int index = 0; index < seg_index; index++) {
        ROS_INFO("%d", 640 - arrSeg[index] - 1);
    }
    ROS_INFO("Printing mins:\n");
    for (int index = 0; index < min_index; index++) {
        ROS_INFO("%f", arrMin[index]);	
    }
    */
    
    // 7. find out direction
    float min_distance = 10;
    int index_min_distance = 0;
    for (int index = 0; index < min_index; index++) {
        if (arrMin[index] < min_distance) {
            min_distance = arrMin[index];
            index_min_distance = index;
        }
    }
    
    int start = arrSeg[index_min_distance];
    int end = arrSeg[index_min_distance + 1];
    int center = (start + end) / 2;
    float angle = (center - 320) / 320.0 * 28.0; // use 320.0 for float promotion
    //min_distance = min_distance * cos(angle / 180 * PI);
    float y = min_distance * cos(angle / 180 * PI);
    float x = min_distance * sin(angle / 180 * PI);
    ROS_INFO("x: %f", x);
    ROS_INFO("y: %f", y);
    
    std_msgs::Float32MultiArray out;
    out.data.clear();
    out.data.push_back(x);
    out.data.push_back(y);
    pub.publish(out);
    
    //std_msgs::Float32 out2;
    //out2.data = y;
    //pub.publish(out2);
    
    
    //ROS_INFO("distance: %f", min_distance);
    //ROS_INFO("angle: %f", angle);
    if (direction_count < 10) {
        if (previous_distance - min_distance > 0.1) {
            float horiz_location = min_distance * sin(angle / 180 * PI);
                //ROS_INFO("Horiz: %f", horiz_location);
            if (horiz_location > 0.25) {
                direction_vote[0]++;
            } else if (horiz_location < -0.25) {
                direction_vote[1]++;
            } else {
                direction_vote[2]++;
            }
            direction_count++;
	    previous_distance = min_distance;
        }
    }
    //ROS_INFO("Count %d", direction_count);
    if (direction_count == 10) {
        int max = 0;
        for (int index = 0; index < 3; index++) {
            if (direction_vote[index] > max) {
                direction = index;
		max = direction_vote[index];
            }
        }
        ROS_INFO("direction is %d", direction);
	direction_count++;
    }
    
    previous_angle = angle;

    return;
}
