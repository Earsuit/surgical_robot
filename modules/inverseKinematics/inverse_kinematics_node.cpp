#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/motor_commands.h"
#include "surgical_robot/end_effector_pos.h"

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg,const surgical_robot::end_effector_posConstPtr &);
typedef const boost::function<void(const surgical_robot::end_effector_posConstPtr & )> sub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"inverse kinematics");
    ros::NodeHandle n;

    //loop rate
    // int rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    // rate = rate>0?rate:RATE_LOWER_LIMIT;
    // ros::Rate loop_rate(rate);

    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_feedback>("motor_commands",100);
    surgical_robot::motor_commands msg;

    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),_1);
    ros::Subscriber motor_command_pub = n.subscribe("end_effector_pos",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg,const surgical_robot::end_effector_posConstPtr & pos){
    msg.motor_1 = rand();
    msg.motor_2 = rand();
    msg.motor_3 = rand();
    msg.motor_4 = rand();
    motor_commands_pub.publish(msg);
}
