#include "ros/ros.h"
#include <ros/console.h>
#include "surgical_robot/motor_feedback.h"
#include "surgical_robot/motor_commands.h"
#include <sstream>
#include <cstdlib>
#include "MiniSerial.h"
#include <stdint.h>
#include <cstring>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#define DEFAULT_RATE 20
#define RX_BUFFER_SIZE 35
#define TX_BUFFER_SIZE 18
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define PACKAGE_LEN 18
#define NOT_FOUND -1
#define RATE_LOWER_LIMIT 1

int findPackage(uint8_t* buffer,int size);
void commandCallback(MiniSerial &serial,const surgical_robot::motor_commandsConstPtr &);
typedef const boost::function<void(const surgical_robot::motor_commandsConstPtr & )> callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"serial");
    ros::NodeHandle n;

    //start serial comunication
    MiniSerial serial(argv[0]);
    serial.begin(115200);
    uint8_t buffer[RX_BUFFER_SIZE];

    //publisher
    ros::Publisher motor_feedback_pub = n.advertise<surgical_robot::motor_feedback>("motor_feedback",1000);
    surgical_robot::motor_feedback msg;
    msg.motor_1 = msg.motor_2 = msg.motor_3 = msg.motor_4 =0;
    //subscriber
    callback sub_callback = boost::bind(commandCallback,boost::ref(serial),_1);
    ros::Subscriber motor_command_pub = n.subscribe("motor_command",1000,sub_callback);

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
            memcpy(&msg.motor_1,&buffer[index],sizef);
            memcpy(&msg.motor_2,&buffer[index+sizef],sizef);
            memcpy(&msg.motor_3,&buffer[index+sizef*2],sizef);
            memcpy(&msg.motor_4,&buffer[index+sizef*3],sizef); 
        }else
            ROS_WARN("Package not found!");
        ROS_INFO("%f, %f, %f, %f",msg.motor_1,msg.motor_2,msg.motor_3,msg.motor_4);
        motor_feedback_pub.publish(msg);
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

void commandCallback(MiniSerial &serial,const surgical_robot::motor_commandsConstPtr &msg){
    ROS_INFO("Commands received: %f, %f, %f, %f", msg->motor_1,msg->motor_2,msg->motor_3,msg->motor_4);
    uint8_t buffer[TX_BUFFER_SIZE];
    int sizef = sizeof(float);
    buffer[0] = PACKAGE_HEAD;
    memcpy(buffer+1,&msg->motor_1,sizef);
    memcpy(buffer+1+sizef,&msg->motor_2,sizef);
    memcpy(buffer+1+sizef*2,&msg->motor_3,sizef);
    memcpy(buffer+1+sizef*3,&msg->motor_4,sizef); 
    buffer[TX_BUFFER_SIZE-1] = PACKAGE_TAIL;
    serial.write(buffer,TX_BUFFER_SIZE);
}