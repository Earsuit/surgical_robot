#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/system_identification.h"
#include "MiniSerial.h"

#define DEFAULT_RATE 20
#define RX_BUFFER_SIZE 27
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define PACKAGE_LEN 14
#define NOT_FOUND -1
#define RATE_LOWER_LIMIT 1

int findPackage(uint8_t* buffer,int size);

int main(int argc, char** argv){
    ros::init(argc,argv,"serial");
    ros::NodeHandle n;

    //start serial comunication
    MiniSerial serial(argv[0]);
    serial.begin(115200);
    uint8_t buffer[RX_BUFFER_SIZE];

    //publisher
    ros::Publisher system_identification_pub = n.advertise<surgical_robot::system_identification>("system_identification",1000);
    surgical_robot::system_identification msg;
    msg.motor_angle = msg.current = msg.voltage =0;
    
    //loop rate
    int rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    rate = rate>0?rate:RATE_LOWER_LIMIT;
    ros::Rate loop_rate(rate);

    while(ros::ok()){
        serial.flush();
        int numOfBytes = serial.read(buffer,RX_BUFFER_SIZE);
        ROS_DEBUG("Received: %d",numOfBytes);
        int index = findPackage(buffer,numOfBytes);
        int sizef = sizeof(float);
        if(index!=NOT_FOUND){
            memcpy(&msg.motor_angle,buffer+index,3*sizef);
        }else
            ROS_WARN("Package not found!");
        ROS_INFO("%f, %f, %f",msg.motor_angle,msg.current,msg.voltage);
        system_identification_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

int findPackage(uint8_t* buffer,int size){
    if(size>=PACKAGE_LEN){
        for(int i=0;i<=(size-PACKAGE_LEN);i++){
            ROS_DEBUG("BYTE: %x",buffer[i]);
            if(buffer[i] == PACKAGE_HEAD && buffer[i+PACKAGE_LEN-1] == PACKAGE_TAIL){
                return i+1;
            }
        }
    }
    return NOT_FOUND;
}