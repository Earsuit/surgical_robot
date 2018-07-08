#include "ros/ros.h"
#include "surgical_robot/motor_feedback.h"
#include <sstream>
#include <cstdlib>
// #include "MiniSerial.h"

#define DEFAULT_RATE 100

int main(int argc, char** argv){
    ros::init(argc,argv,"serial_rx");
    ros::NodeHandle n;
    ros::Publisher motor_feedback_pub = n.advertise<surgical_robot::motor_feedback>("motor_feedback",1000);
    int rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    rate = rate>0?rate:1;
    ros::Rate loop_rate(rate);
    unsigned int count = 0;
    while(ros::ok()){
        surgical_robot::motor_feedback msg;
        msg.motor_1 = 100;
        msg.motor_2 = 200;
        msg.motor_3 = 300;
        msg.motor_4 = 400;
        std::cout<<"Num "<<count<<": "<<msg.motor_1<<", "<<msg.motor_2<<", "<<msg.motor_3<<", "<<msg.motor_4<<std::endl;
        motor_feedback_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        count++;
    }
    return 0;
}