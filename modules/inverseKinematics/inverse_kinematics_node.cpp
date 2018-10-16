#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/motor_commands.h"
#include "surgical_robot/joystick_reading.h"

#define SCALE 1000
#define L1 0.043
#define L2 0.042
#define L3 0.03
#define MAX 255.0
#define MIDDLE_1_x 527
#define MIDDLE_1_y 520
#define MIDDLE_2_x 523
#define MIDDLE_2_y 506

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* pos, int* middleValue ,const surgical_robot::joystick_readingConstPtr &);
typedef const boost::function<void(const surgical_robot::joystick_readingConstPtr & )> sub_callback;

float constraint(float u, float upperBound,float lowerBound);

int main(int argc, char** argv){
    ros::init(argc,argv,"inverse_kinematics");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_commands>("motor_commands",100);
    surgical_robot::motor_commands msg;

    // Array to store end effector pos
    float pos[3] = {115,0,0};
    // Middle value for joysticks
    int middleValue[4] = {MIDDLE_1_x,MIDDLE_1_y,MIDDLE_2_x,MIDDLE_2_y};

    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),pos,middleValue,_1);
    ros::Subscriber motor_command_pub = n.subscribe("joystick_reading",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* pos,int* middleValue,const surgical_robot::joystick_readingConstPtr & joystick){
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
    
    pos[0] += x1Diff / SCALE
    pos[1] += y1Diff / SCALE
    pos[2] += x2Diff / SCALE

    float xd = pos[0];
    float yd = pos[1];
    float zd = pos[2];

    float theta1 = 3.141592653589793-atan((L1+sqrt(-(L2*L2)*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)+L1*L1-zd*zd+(L2*L2)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+(L3*L3)*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)*(1.0/(L3*L3)*(yd*yd)-1.0)-(L3*L3)*(1.0/(L3*L3)*(yd*yd)-1.0)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-L2*L3*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0-(L1*L2*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)*2.0)/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)-(L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)*2.0)/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*2.0)-(L2*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)-(L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(-zd+L2*sqrt(-1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+1.0)+L3*sqrt(-1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+1.0)*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)))*2.0;
    float theta2 = -3.141592653589793+acos((L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)/(L1*(L2*2.0+L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)));
    float theta3 = asin(yd/L3);

    msg.motor_1 = theta1;
    msg.motor_2 = theta2;
    msg.motor_3 = theta3;
    msg.motor_4 = constraint(y2Diff*MAX/middleValue[3],MAX,-MAX);
    ROS_INFO("Motor commands: %f, %f, %f, %f",theta1,theta2,theta3,msg.motor_4);
    motor_commands_pub.publish(msg);
}

float constraint(float u, float upperBound,float lowerBound){
    if(u > upperBound)
        return upperBound;
    else if (u < lowerBound)
        return lowerBound;
    
    return u;
}

