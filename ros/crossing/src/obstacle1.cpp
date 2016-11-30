#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <stdlib.h>
#include <queue>
#define PI 3.14159265


//functions
void scan_callback( const sensor_msgs::LaserScan& _msg);
void odometry_callback( const nav_msgs::Odometry& _msg );
void publish( double _angular, double _linear );

ros::Publisher pub;
//ros::Publisher pub2;


// global variables
/*
int direction = 0; // 0 for left // 1 for right // 2 for center
int direction_vote[3] = {0,0,0};
float previous_distance = 10;
float previous_angle = 0;
int direction_count = 0;
*/

const int time_num = 40;
std::queue<float> x_queue;
std::queue<float> y_queue;
float x_arr[time_num];
float y_arr[time_num];
float previous_x = -10;
float previous_y = -10;
int plotted = 0;

float end_x = 0;

bool turnRight = false, turnLeft = false, goStraight = true;
int scanPoint = 200;
float cutoff = 0.1;
float speed, gain = .8;


int main( int argc, char** argv ) {
    
    ros::init(argc, argv, "follow");
    
    ros::NodeHandle nh;
    ros::NodeHandle odm;
    
    ros::Subscriber scan = nh.subscribe( "scan", 10, scan_callback);
    ros::Subscriber subodm = odm.subscribe( "odom", 1, odometry_callback);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //pub = nh.advertise<std_msgs::Float32>("x", 1); // debug
    //pub2 = nh.advertise<std_msgs::Float32>("y", 1); // debug
    
    
    ros::Rate loop_rate(5);
    
    while( ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
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
        publish(-1 * speed, 0.3);
    }
    // when turnLeft is true and goStraight is false
    // publish following velocities
    
    if (turnLeft && !goStraight) {
        publish(speed, 0.3);
    }
    // when goStraight is true, publish following velocities
    
    if (goStraight) {
        publish(0.07, 0.3);
    }
    
    return;
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

    /*
    std_msgs::Float32 out;
    out.data.clear();
    out.data.push_back(x);
    pub.publish(out);
    
    std_msgs::Float32 out2;
    out2.data.clear();
    out2.data.push_back(y);
    pub.publish(out2);
    */
    
    
    if (previous_x == -10) {
        if (fabs(x) < 4 && y < 9 && y > 2.5) {
            previous_x = x;
            previous_y = y;
        }
    } else {
        if (fabs(x) < 4 && fabs(x - previous_x) < 0.3 && y < 9 && y > 2.5 && fabs(y - previous_y) < 1) {
            previous_x = x;
            previous_y = y;
        }
        
        if (x_queue.size() > time_num) {
            x_queue.pop();
            y_queue.pop();
        }
        x_queue.push(previous_x);
        y_queue.push(previous_y);
        
        if (previous_y < 3 && plotted == 0) {
            
            float sum_x = 0;
            float sum_y = 0;
            float sum_time = 0;
            // copy queue to array
            for (int i = 0; i < time_num; i++) {
                x_arr[i] = x_queue.front();
                x_queue.pop();
                y_arr[i] = y_queue.front();
                y_queue.pop();
                sum_x += x_arr[i];
                sum_y += y_arr[i];
                sum_time += i + 1;
            }
            
            float x_avg = sum_x / time_num;
            float y_avg = sum_y / time_num;
            float time_avg = sum_time / time_num;
            
            float cov_x = 0;
            float var_x = 0;
            float cov_y = 0;
            float var_y = 0;
            
            for (int i = 0; i < time_num; i++) {
                cov_x += (i+1 - time_avg) * (x_arr[i] - x_avg);
                var_x += (i+1 - time_avg) * (i+1 - time_avg);
                cov_y += (i+1 - time_avg) * (y_arr[i] - y_avg);
                var_y += (i+1 - time_avg) * (i+1 - time_avg);
            }
            
            float beta_x = cov_x / var_x;
            float alpha_x = x_avg - beta_x * time_avg;
            float mid_x = beta_x * time_num + alpha_x;
	    end_x = mid_x + beta_x * 50;
            float beta_y = cov_y / var_y;
            float alpha_y = y_avg - beta_y * time_avg;
            
            ROS_INFO("beta_x: %f", beta_x);
            ROS_INFO("alpha_x: %f", alpha_x);
            ROS_INFO("mid_x: %f", mid_x);
            ROS_INFO("end_x: %f", end_x);
            ROS_INFO("beta_y: %f", beta_y);
            ROS_INFO("alpha_y: %f", alpha_y);
            
            plotted = 1;
        }
    }
    
    if (plotted == 1) {
        float rightDiff = _msg.ranges[scanPoint + 1];
        float leftDiff = _msg.ranges[640 - scanPoint];
        if (end_x > 0) {
            speed = fabs(rightDiff - 1.2);
            if (rightDiff > 1.2 + cutoff) {
                turnRight = true;
                turnLeft = false;
                goStraight = false;
            } else if (rightDiff + cutoff < 1.2) {
                turnLeft = true;
                goStraight = false;
                turnRight = false;
            } else {
                goStraight = true;
                turnLeft = false;
                turnRight = false;
            }
        }
        if (end_x < 0) {
            speed = fabs(leftDiff - 1.2);
            if (leftDiff > 1.2 + cutoff) {
                turnRight = false;
                turnLeft = true;
                goStraight = false;
            } else if (leftDiff + cutoff < 1.2) {
                turnLeft = false;
                goStraight = false;
                turnRight = true;
            } else {
                goStraight = true;
                turnLeft = false;
                turnRight = false;
            }
        }
    
    }

    return;
}
