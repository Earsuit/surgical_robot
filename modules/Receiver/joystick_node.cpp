#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include <stdint.h>
#include <cstring>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include "surgical_robot/end_effector_pos.h"

#define MIDDLE 512
#define SCALE 1000

// first argument: loop rate
int main(int argc,char** argv){
    ros::init(argc,argv,"joystick");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher motor_commands_pub = n.advertise<surgical_robot::end_effector_pos>("end_effector_pos",100);
    surgical_robot::end_effector_pos msg;
    // in cm
    msg.x = 11.5;
    msg.y = 0;
    msg.z = 0;
    msg.grab = 0;
    ros::Rate loop_rate(atoi(argv[1]));

    // using CS0 pin 
    SpiOpenPort(0);
    //dx,dy,dz,grab
    int output[4];
    unsigned char command[3];

    while (ros::ok()){
        // x1,y1,x2,y2
        for(int i=0;i<4;i++){
            // 0xC0: start bit and single mode, 0x00: extra clock to do the conversion 
            command[0] = (0xC0 | (i & 0x07) << 3);
            command[1] = 0x00;
            command[2] = 0x00;
            SpiWriteAndRead(0,command,3);
            output[i] =  (0x3FF & ((command[0]& 0x01)<<9) | ((command[1] & 0xFF) << 1) | ((command[2] & 0x80) >> 7));
        }

        msg.x += ((output[0]>MIDDLE)?output[0]%MIDDLE:-output[0]%MIDDLE)/SCALE;
        msg.y += ((output[1]>MIDDLE)?output[1]%MIDDLE:-output[1]%MIDDLE)/SCALE;
        msg.z += ((output[2]>MIDDLE)?output[0]%MIDDLE:-output[2]%MIDDLE)/SCALE;
        msg.grab = (output[2]>MIDDLE)?1:0;
        motor_commands_pub(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}