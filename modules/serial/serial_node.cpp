#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <boost/bind.hpp>
#include "surgical_robot/motor_commands.h"
#include "rs232.h"

#define DEFAULT_RATE 20
#define DEFAULT_BAUDRATE 115200
#define TX_BUFFER_SIZE 13
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define RATE_LOWER_LIMIT 1
#define MOTOR_COMMANDS_LENGTH 4

void subscriberCallback(uint8_t* buffer,int port,const surgical_robot::motor_commandsConstPtr &);
typedef const boost::function<void(const surgical_robot::motor_commandsConstPtr & )> sub_callback;

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

    uint8_t buffer[TX_BUFFER_SIZE];
    // buffer[0] = PACKAGE_HEAD;
    // buffer[TX_BUFFER_SIZE-1] = PACKAGE_TAIL;
    
    //subscriber
    sub_callback sub_callback = boost::bind(subscriberCallback,buffer,atoi(argv[3]),_1);
    ros::Subscriber motor_command_pub = n.subscribe("motor_command",100,sub_callback);

    ros::spin();

    return 0;
}

void subscriberCallback(uint8_t* buffer,int port,const surgical_robot::motor_commandsConstPtr &msg){
    ROS_INFO("Commands received: %f, %f, %f, %d", msg->motor_1,msg->motor_2,msg->motor_3,msg->motor_4);
    int sizef = sizeof(msg->motor_1);
    memcpy(buffer,&msg->motor_1,(MOTOR_COMMANDS_LENGTH-1)*sizef);
    memcpy(buffer+12,&msg->motor_4,sizeof(msg->motor_4));
    RS232_SendBuf(port,buffer,TX_BUFFER_SIZE);
}