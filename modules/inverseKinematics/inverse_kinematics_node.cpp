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
#include "surgical_robot/end_effector_pos.h"

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg,const surgical_robot::end_effector_posConstPtr &);
typedef const boost::function<void(const surgical_robot::end_effector_posConstPtr & )> sub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"inverse_kinematics");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::motor_commands>("motor_commands",100);
    surgical_robot::motor_commands msg;
    
    sub_callback sub_callback = boost::bind(subscriberCallback,boost::ref(motor_commands_pub),boost::ref(msg),_1);
    ros::Subscriber motor_command_pub = n.subscribe("end_effector_pos",1000,sub_callback);

    ros::spin();
    return 0;
}

void subscriberCallback(ros::Publisher& motor_commands_pub,surgical_robot::motor_commands &msg,const surgical_robot::end_effector_posConstPtr & pos){
    float xd = pos.x;
    float yd = pos.y;
    float zd = pos.z;

    float L1 = 0.048;
    float L2 = 0.042;
    float L3 = 0.048;

    float theta1 = 3.141592653589793-atan((L1+sqrt(-(L2*L2)*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)+L1*L1-zd*zd+(L2*L2)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+(L3*L3)*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)*(1.0/(L3*L3)*(yd*yd)-1.0)-(L3*L3)*(1.0/(L3*L3)*(yd*yd)-1.0)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-L2*L3*(1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)-1.0)*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0-(L1*L2*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)*2.0)/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)-(L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)*2.0)/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*2.0)-(L2*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)-(L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0))/(-zd+L2*sqrt(-1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+1.0)+L3*sqrt(-1.0/pow(L1*L2*2.0+L1*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)*pow(L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0,2.0)+1.0)*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)))*2.0;
    float theta2 = -3.141592653589793+acos((L1*L1+L2*L2+L3*L3-xd*xd-yd*yd-zd*zd+L2*L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)/(L1*(L2*2.0+L3*sqrt(-1.0/(L3*L3)*(yd*yd)+1.0)*2.0)));
    float theta3 = asin(yd/L3);

    msg.motor_1 = theta1;
    msg.motor_2 = theta2;
    msg.motor_3 = theta3;
    msg.motor_4 = pos.grab;
    motor_commands_pub.publish(msg);
}
