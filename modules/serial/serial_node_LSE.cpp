#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/system_identification.h"
#include "surgical_robot/motor_commands.h"
#include "rs232.h"

#define DEFAULT_RATE 20
#define DEFAULT_BAUDRATE 115200
#define RX_BUFFER_SIZE 18
#define TX_BUFFER_SIZE 4
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define RX_PACKAGE_LEN 18
#define NOT_FOUND -1
#define RATE_LOWER_LIMIT 1
#define SYSTEM_IDENTIFICATION_LENGTH 4 

int findPackage(uint8_t* buffer,int size);
void subscriberCallback(int port,const surgical_robot::motor_commandsConstPtr &);
typedef const boost::function<void(const surgical_robot::motor_commandsConstPtr & )> sub_callback;

void publisherCallback(int ,uint8_t*,ros::Publisher&,surgical_robot::system_identification &,const ros::TimerEvent&);
typedef const boost::function<void(const ros::TimerEvent&)> pub_callback;


//first argument - loop rate; second argument - baud rate; third argument - port number(see library document)
int main(int argc, char** argv){
    ros::init(argc,argv,"serial");
    ros::NodeHandle n;

    //loop rate
    float rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    rate = 1.0/(rate>0?rate:RATE_LOWER_LIMIT);

    //start serial comunication, eight databits, no parity, one stopbit
    int baudRate = (argv[2]==NULL)?DEFAULT_BAUDRATE:atoi(argv[2]);
    if(RS232_OpenComport(atoi(argv[3]),baudRate,"8N1")){
        ROS_ERROR("Can not open comport");
        return 0;
    }
    uint8_t buffer[RX_BUFFER_SIZE];

    //publisher
    ros::Publisher system_identification_pub = n.advertise<surgical_robot::system_identification>("system_identification",1000);
    surgical_robot::system_identification msg;
    msg.motor_angle = msg.motor_v = msg.current = msg.voltage = 0;
    pub_callback publisher_callback =  boost::bind(publisherCallback,atoi(argv[3]),buffer,boost::ref(system_identification_pub),boost::ref(msg),_1);
    ros::Timer timer = n.createTimer(ros::Duration(rate),publisher_callback);
    
    //subscriber
    // sub_callback subscriber_callback = boost::bind(subscriberCallback,atoi(argv[3]),_1);
    // ros::Subscriber motor_command_sub = n.subscribe("motor_command",1000,subscriber_callback);

    //using two threads 
    ros::AsyncSpinner s(2);
    s.start();
    ros::waitForShutdown();
    return 0;
}

inline int findPackage(uint8_t* buffer,int size){
    if(size>=RX_PACKAGE_LEN){
        for(int i=0;i<=(size-RX_PACKAGE_LEN);i++){
            ROS_DEBUG("BYTE: %x",buffer[i]);
            if(buffer[i] == PACKAGE_HEAD && buffer[i+RX_PACKAGE_LEN-1] == PACKAGE_TAIL){
                return i+1;
            }
        }
    }
    return NOT_FOUND;
}

void subscriberCallback(int port,const surgical_robot::motor_commandsConstPtr &msg){
    ROS_INFO("Commands received: %d, %d",msg->motor_1_v,msg->motor_1_dir);
    uint8_t buffer[TX_BUFFER_SIZE];
    int sizef = sizeof(msg->motor_1_v);
    //only have one motoe in lse 
    buffer[0] = PACKAGE_HEAD;
    buffer[1] = msg->motor_1_v;
    buffer[2] = msg->motor_1_dir;
    buffer[3] = PACKAGE_TAIL;
    RS232_SendBuf(port,buffer,TX_BUFFER_SIZE);
} 

void publisherCallback(int port,uint8_t* buffer,ros::Publisher& pub,surgical_robot::system_identification & msg,const ros::TimerEvent& timer){
    int numOfBytes = RS232_PollComport(port,buffer,RX_BUFFER_SIZE);
    ROS_DEBUG("Received: %d",numOfBytes);
    int index = findPackage(buffer,numOfBytes);
    // int index = 1;
    int sizef = sizeof(msg.motor_angle);
    if(index!=NOT_FOUND){
        memcpy(&msg.motor_angle,buffer+index,SYSTEM_IDENTIFICATION_LENGTH*sizef);
        if(msg.current<0)
            msg.voltage = -msg.voltage;
        ROS_INFO("%0.2f, %0.2f, %0.2f,%0.2f",msg.motor_angle,msg.motor_v,msg.current,msg.voltage);
        pub.publish(msg);
    }else
        ROS_WARN("Package not found!"); 
}