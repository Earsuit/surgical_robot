#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/motor_commands.h"
#include "surgical_robot/joystick_reading.h"

#define MIDDLE_1_x 527
#define MIDDLE_1_y 520
#define MIDDLE_2_x 523
#define MIDDLE_2_y 506
#define SCALE 1000.0
#define MAX 1023.0

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* angle, int* middleValue,const surgical_robot::joystick_readingConstPtr &);
typedef const boost::function<void(const surgical_robot::joystick_readingConstPtr & )> sub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"openloop");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_commands>("motor_commands",100);
    surgical_robot::motor_commands msg;

    // Array to store the angles for motor 1,2,3
    float angle[3] = {0,0,0};
    int middleValue[4] = {MIDDLE_1_x,MIDDLE_1_y,MIDDLE_2_x,MIDDLE_2_y};

    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),angle,middleValue,_1);
    ros::Subscriber motor_command_pub = n.subscribe("joystick_reading",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* angle, int* middleValue,const surgical_robot::joystick_readingConstPtr & joystick){
    ROS_INFO("Joystick reading received: %d, %d, %d, %d", joystick->joystick_1_x,joystick->joystick_1_y,joystick->joystick_2_x,joystick->joystick_2_y);
    float x1 = joystick->joystick_1_x;
    float y1 = joystick->joystick_1_y;
    float x2 = joystick->joystick_2_x;
    float y2 = joystick->joystick_2_y;

    if(joystick->joystick_1_press){
        middleValue[0] = x1;
        middleValue[1] = y1;
    }

    if(joystick->joystick_2_press){
        middleValue[2] = x2;
        middleValue[3] = y2;
    }

    float x1Diff = (abs(x1 - middleValue[0])<5) ? 0 : x1 - middleValue[0];
    float y1Diff = (abs(y1 - middleValue[1])<5) ? 0 : y1 - middleValue[1];
    float x2Diff = (abs(x2 - middleValue[2])<5) ? 0 : x2 - middleValue[2];
    float y2Diff = (abs(y2 - middleValue[3])<5) ? 0 : y2 - middleValue[3]; 
    
    angle[0] +=  x1Diff / SCALE;
    angle[1] +=  y1Diff / SCALE;
    angle[2] +=  x2Diff / SCALE;

    msg.motor_1 = angle[0]*M_PI/180;
    msg.motor_2 = angle[1]*M_PI/180;
    msg.motor_3 = angle[2]*M_PI/180;
    msg.motor_4 = y2Diff*MAX/middleValue[3];
    ROS_INFO("Motor commands (deg): %f, %f, %f, %f",angle[0],angle[1],angle[2],msg.motor_4);
    motor_commands_pub.publish(msg);
}
