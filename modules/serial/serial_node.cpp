#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/motor_feedback.h"
#include "surgical_robot/motor_commands.h"
#include "rs232.h"

#define DEFAULT_RATE 20
#define DEFAULT_BAUDRATE 115200
#define RX_BUFFER_SIZE 35
#define TX_BUFFER_SIZE 10
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define RX_PACKAGE_LEN 18
#define NOT_FOUND -1
#define RATE_LOWER_LIMIT 1
#define MOTOR_FEEDBACK_LENGTH 4 
#define MOTOR_COMMANDS_LENGTH 8

int findPackage(uint8_t* buffer,int size);
void subscriberCallback(int port,const surgical_robot::motor_commandsConstPtr &);
typedef const boost::function<void(const surgical_robot::motor_commandsConstPtr & )> sub_callback;

void publisherCallback(int ,uint8_t*,ros::Publisher&,surgical_robot::motor_feedback &,const ros::TimerEvent&);
typedef const boost::function<void(const ros::TimerEvent&)> pub_callback;

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
    ros::Publisher motor_feedback_pub = n.advertise<surgical_robot::motor_feedback>("motor_feedback",1000);
    surgical_robot::motor_feedback msg;
    msg.motor_1 = msg.motor_2 = msg.motor_3 = msg.motor_4 =0;
    pub_callback publisher_callback =  boost::bind(publisherCallback,atoi(argv[3]),buffer,boost::ref(motor_feedback_pub),boost::ref(msg),_1);
    ros::Timer timer = n.createTimer(ros::Duration(rate),publisher_callback);

    //subscriber
    sub_callback sub_callback = boost::bind(subscriberCallback,atoi(argv[3]),_1);
    ros::Subscriber motor_command_pub = n.subscribe("motor_command",1000,sub_callback);

    // using two threads 
    ros::AsyncSpinner s(2);
    s.start();
    ros::waitForShutdown();

    return 0;
}

int findPackage(uint8_t* buffer,int size){
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
    ROS_INFO("Commands received: %d, %d, %d, %d, %d, %d, %d, %d", msg->motor_1_v,msg->motor_1_dir,msg->motor_2_v,msg->motor_2_dir,msg->motor_3_v,msg->motor_3_dir,msg->motor_4_v,msg->motor_4_dir);
    uint8_t buffer[TX_BUFFER_SIZE];
    int sizef = sizeof(msg->motor_1_v);
    buffer[0] = PACKAGE_HEAD;
    memcpy(buffer+1,&msg->motor_1_v,MOTOR_COMMANDS_LENGTH*sizef);
    buffer[TX_BUFFER_SIZE-1] = PACKAGE_TAIL;
    RS232_SendBuf(port,buffer,TX_BUFFER_SIZE);
}

void publisherCallback(int port,uint8_t* buffer,ros::Publisher& pub,surgical_robot::motor_feedback & msg,const ros::TimerEvent& timer){
    int numOfBytes = RS232_PollComport(port,buffer,RX_BUFFER_SIZE);
    ROS_DEBUG("Received: %d",numOfBytes);
    int index = findPackage(buffer,numOfBytes);
    int sizef = sizeof(float);
    if(index!=NOT_FOUND){
        memcpy(&msg.motor_1,buffer+index,MOTOR_FEEDBACK_LENGTH*sizef);
    }else
        ROS_WARN("Package not found!");
    
    ROS_INFO("%f, %f, %f, %f",msg.motor_1,msg.motor_2,msg.motor_3,msg.motor_4);
    pub.publish(msg);
}