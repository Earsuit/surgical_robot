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
#define TX_BUFFER_SIZE 16
#define PACKAGE_HEAD 0x51
#define PACKAGE_TAIL 0x71
#define RATE_LOWER_LIMIT 1
#define MOTOR_COMMANDS_LENGTH 4

void subscriberCallback(uint8_t* buffer,int port,const surgical_robot::motor_commandsConstPtr &);
typedef const boost::function<void(const surgical_robot::motor_commandsConstPtr & )> sub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"serial");
    ros::NodeHandle n;

    //start serial comunication, eight databits, no parity, one stopbit
    int baudRate = (argv[2]==NULL)?DEFAULT_BAUDRATE:atoi(argv[1]);
    if(RS232_OpenComport(atoi(argv[2]),baudRate,"8N1")){
        ROS_ERROR("Can not open comport");
        return 0;
    }

    uint8_t buffer[TX_BUFFER_SIZE];
    
    //subscriber
    sub_callback sub_callback = boost::bind(subscriberCallback,buffer,atoi(argv[2]),_1);
    ros::Subscriber motor_command_pub = n.subscribe("motor_commands",100,sub_callback);

    ros::spin();

    RS232_CloseComport(atoi(argv[2]));

    return 0;
}

void subscriberCallback(uint8_t* buffer,int port,const surgical_robot::motor_commandsConstPtr &msg){
    ROS_INFO("Commands received: %f, %f, %f, %f", msg->motor_1,msg->motor_2,msg->motor_3,msg->motor_4);
    int sizef = sizeof(msg->motor_1);
    memcpy(buffer,&msg->motor_1,MOTOR_COMMANDS_LENGTH*sizef);
    RS232_SendBuf(port,buffer,TX_BUFFER_SIZE);
}