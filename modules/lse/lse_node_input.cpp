#include "ros/ros.h"
#include "surgical_robot/motor_commands.h"
#include <stdint.h>
#include <math.h>

#define DEFAULT_RATE 20
#define DEFAULT_SLOPE 1
#define RATE_LOWER_LIMIT 1
#define MY_PI 3.14159265358979
#define CLK_WISE 1
#define ANTI_CLK_WISE 2

void chripCallback(double , float , ros::Publisher&,surgical_robot::motor_commands &, const ros::WallTimerEvent &);
typedef const boost::function<void(const ros::WallTimerEvent &)> pub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"lse_input");
    ros::NodeHandle n;

    //loop rate
    float rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    rate = 1.0/(rate>0?rate:RATE_LOWER_LIMIT);

    //slope
    float slope = (argv[2]==NULL)?DEFAULT_SLOPE:atoi(argv[2]);

    //publisher
    ros::Publisher pub = n.advertise<surgical_robot::motor_commands>("motor_command",1000);
    surgical_robot::motor_commands msg;
    msg.motor_2_v = msg.motor_2_dir = msg.motor_3_v = msg.motor_3_dir = msg.motor_4_v = msg.motor_4_dir = 0;
    //start time
    double start = ros::WallTime::now().toSec();
    pub_callback publisher_callback = boost::bind(chripCallback,start,slope,boost::ref(pub),boost::ref(msg),_1);
    ros::WallTimer timer = n.createWallTimer(ros::WallDuration(rate),publisher_callback);

    ros::spin();

    return 0;
}

void chripCallback(double start,float slope,ros::Publisher& pub,surgical_robot::motor_commands & msg, const ros::WallTimerEvent & tmer){
    double now = ros::WallTime::now().toSec();
    double d = now-start;
    double v = sin(2*MY_PI*(slope*d*d));
    msg.motor_1_v = abs(v*255);
    msg.motor_1_dir = v>0?CLK_WISE:ANTI_CLK_WISE;
    pub.publish(msg);
}