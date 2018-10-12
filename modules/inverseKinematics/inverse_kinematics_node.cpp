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

#define MIDDLE 512
#define SCALE 1000

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* pos,const surgical_robot::joystick_readingConstPtr &);
typedef const boost::function<void(const surgical_robot::joystick_readingConstPtr & )> sub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"inverse_kinematics");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_commands>("motor_commands",100);
    surgical_robot::motor_commands msg;

    // Array to store end effector pos
    float pos[3] = {115,0,0};

    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),pos,_1);
    ros::Subscriber motor_command_pub = n.subscribe("joystick_reading",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* pos,const surgical_robot::joystick_readingConstPtr & joystick){
    ROS_INFO("Joystick reading received: %d, %d, %d, %d", joystick->joystick_1_x,joystick->joystick_1_y,joystick->joystick_2_x,joystick->joystick_2_y);
    pos[0] += ((joystick->joystick_1_x > MIDDLE)?joystick->joystick_1_x%MIDDLE:-joystick->joystick_1_x%MIDDLE)/SCALE;
    pos[1] += ((joystick->joystick_1_y > MIDDLE)?joystick->joystick_1_y%MIDDLE:-joystick->joystick_1_y%MIDDLE)/SCALE;
    pos[2] += ((joystick->joystick_2_x > MIDDLE)?joystick->joystick_2_x%MIDDLE:-joystick->joystick_2_x%MIDDLE)/SCALE;

    float xd = pos[0];
    float yd = pos[1];
    float zd = pos[2];

    float L1 = 0.043;
    float L2 = 0.042;
    float L3 = 0.03;

    float theta1 = 3.141592653589793-atan((L1+sqrt(-(L2*L2)*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)+L1*L1-zd*zd+(L2*L2)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+(L3*L3)*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)*(1.0/(L3*L3)*(yd*yd)-1.0)-(L3*L3)*(1.0/(L3*L3)*(yd*yd)-1.0)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-L2*L3*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0-(L1*L2*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)*2.0)/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)-(L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)*2.0)/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*2.0)-(L2*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)-(L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(-zd+L2*sqrt(-1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+1.0)+L3*sqrt(-1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+1.0)*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)))*2.0;
    float theta2 = -3.141592653589793+acos((L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)/(L1*(L2*2.0+L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)));
    float theta3 = asin(yd/L3);

    msg.motor_1 = theta1;
    msg.motor_2 = theta2;
    msg.motor_3 = theta3;
    msg.motor_4 = ((joystick->joystick_2_y > MIDDLE)?joystick->joystick_2_y%MIDDLE:-joystick->joystick_2_y%MIDDLE)/SCALE;
    ROS_INFO("Motor commands: %f, %f, %f, %f",theta1,theta2,theta3,msg.motor_4);
    motor_commands_pub.publish(msg);
}
