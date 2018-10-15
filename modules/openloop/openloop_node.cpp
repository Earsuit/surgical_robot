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

#define MIDDLE_1_x 512
#define MIDDLE_1_y 512
#define MIDDLE_2_x 512
#define MIDDLE_2_y 512
#define SCALE 1000
#define MAX 1023

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* angle,const surgical_robot::joystick_readingConstPtr &);
typedef const boost::function<void(const surgical_robot::joystick_readingConstPtr & )> sub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"openloop");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_commands>("motor_commands",100);
    surgical_robot::motor_commands msg;

    // Array to store the angles for motor 1,2,3
    float angle[3] = {0,0,0};

    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),pos,_1);
    ros::Subscriber motor_command_pub = n.subscribe("joystick_reading",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg, float* angle,const surgical_robot::joystick_readingConstPtr & joystick){
    ROS_INFO("Joystick reading received: %d, %d, %d, %d", joystick->joystick_1_x,joystick->joystick_1_y,joystick->joystick_2_x,joystick->joystick_2_y);
    angle[0] += ((joystick->joystick_1_x > MIDDLE_1_x)?joystick->joystick_1_x%MIDDLE_1_x:-joystick->joystick_1_x%MIDDLE_1_x)/SCALE;
    angle[1] += ((joystick->joystick_1_y > MIDDLE_1_y)?joystick->joystick_1_y%MIDDLE_1_y:-joystick->joystick_1_y%MIDDLE_1_y)/SCALE;
    angle[2] += ((joystick->joystick_2_x > MIDDLE_2_x)?joystick->joystick_2_x%MIDDLE_2_x:-joystick->joystick_2_x%MIDDLE_2_x)/SCALE;

    msg.motor_1 = angle[0];
    msg.motor_2 = angle[1];
    msg.motor_3 = angle[2];
    msg.motor_4 = ((joystick->joystick_2_y > MIDDLE_2_y)?(joystick->joystick_2_y%MIDDLE_2_y)*MAX/MIDDLE_2_y:(-joystick->joystick_2_y%MIDDLE_2_y)*MAX/MIDDLE_2_y);
    ROS_INFO("Motor commands: %f, %f, %f, %f",msg.motor_1,msg.motor_2,msg.motor_3,msg.motor_4);
    motor_commands_pub.publish(msg);
}
