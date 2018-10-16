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
#define MAX_10Bit 1023.0
#define MAX_8Bit 255.0

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, int* middleValue,const surgical_robot::joystick_readingConstPtr &);
typedef const boost::function<void(const surgical_robot::joystick_readingConstPtr & )> sub_callback;

float constraint(float u, float upperBound,float lowerBound);

int main(int argc, char** argv){
    ros::init(argc,argv,"openloop");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_commands>("motor_commands",100);
    surgical_robot::motor_commands msg;

    // Middle value for joysticks
    int middleValue[4] = {MIDDLE_1_x,MIDDLE_1_y,MIDDLE_2_x,MIDDLE_2_y};

    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),middleValue,_1);
    ros::Subscriber motor_command_pub = n.subscribe("joystick_reading",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, int* middleValue,const surgical_robot::joystick_readingConstPtr & joystick){
    ROS_INFO("Joystick reading received: %d, %d, %d, %d, %d, %d", joystick->joystick_1_x,joystick->joystick_1_y,joystick->joystick_2_x,joystick->joystick_2_y,joystick->joystick_1_press,joystick->joystick_2_press);
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

    msg.motor_1 = constraint(x1Diff*MAX_10Bit/middleValue[0],MAX_10Bit,-MAX_10Bit);
    msg.motor_2 = constraint(y1Diff*MAX_10Bit/middleValue[0],MAX_10Bit,-MAX_10Bit);
    msg.motor_3 = constraint(x2Diff*MAX_10Bit/middleValue[0],MAX_10Bit,-MAX_10Bit);
    msg.motor_4 = constraint(y2Diff*MAX_8Bit/middleValue[0],MAX_8Bit,-MAX_8Bit);
    
    ROS_INFO("Motor commands (pwm): %f, %f, %f, %f",msg.motor_1,msg.motor_2,msg.motor_3,msg.motor_4);
    motor_commands_pub.publish(msg);
}

float constraint(float u, float upperBound,float lowerBound){
    if(u > upperBound)
        return upperBound;
    else if (u < lowerBound)
        return lowerBound;
}
