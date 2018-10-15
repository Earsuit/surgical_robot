#include "ros/ros.h"
#include <ros/console.h>
#include <sstream>
#include <cstdlib>
#include "surgical_robot/joystick_reading.h"
#include "SPI.h"
#include "GPIO.h"

// first argument: loop rate
int main(int argc,char** argv){
    ros::init(argc,argv,"joystick");
    ros::NodeHandle n;

    // Create the instance of publisher and msg
    ros::Publisher joystick_reading_pub = n.advertise<surgical_robot::joystick_reading>("joystick_reading",100);
    surgical_robot::joystick_reading msg;
    ros::Rate loop_rate(atoi(argv[1]));

    // using CS0 pin 
    SpiOpenPort(0);
    unsigned char command[3];
    int output[4];

    GPIO gpio;
    // Only pin 19 is available for input
    gpio.pinMode(19,INPUT);
    gpio.pull_up_off_down(19,UP);

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

        msg.joystick_1_x = output[0];
        msg.joystick_1_y = output[1];
        msg.joystick_2_x = output[2];
        msg.joystick_2_y = output[3];
        msg.joystick_1_press = (gpio.pinLevel(19) == LOW);
        msg.joystick_2_press = (gpio.pinLevel(19) == LOW);

        ROS_INFO("Joystick reading: %d, %d, %d, %d, %d, %d.",msg.joystick_1_x,msg.joystick_1_y,msg.joystick_2_x,msg.joystick_2_y,msg.joystick_1_press,msg.joystick_2_press);
        joystick_reading_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    SpiClosePort(0);
}