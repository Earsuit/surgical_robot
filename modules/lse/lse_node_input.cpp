#include "ros/ros.h"
#include "surgical_robot/motor_commands.h"
#include <stdint.h>

#define DEFAULT_RATE 20
#define RATE_LOWER_LIMIT 1

void chripCallback(double , ros::Publisher&,surgical_robot::motor_commands &, const ros::WallTimerEvent &);
typedef const boost::function<void(const ros::WallTimerEvent &)> pub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"lse_input");
    ros::NodeHandle n;

    //loop rate
    float rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    rate = 1.0/(rate>0?rate:RATE_LOWER_LIMIT);

    //publisher
    ros::Publisher pub = n.advertise<surgical_robot::motor_commands>("motor_command",1000);
    surgical_robot::motor_commands msg;
    //start time
    double start = ros::WallTime::now().toSec();
    pub_callback publisher_callback = boost::bind(chripCallback,start,boost::ref(pub),boost::ref(msg),_1);
    ros::WallTimer timer = n.createWallTimer(ros::WallDuration(rate),publisher_callback);

    ros::spin();

    return 0;
}

void chripCallback(double start, ros::Publisher& pub,surgical_robot::motor_commands & msg, const ros::WallTimerEvent & tmer){
    double now = ros::WallTime::now().toSec();
    double d = now-start;
}