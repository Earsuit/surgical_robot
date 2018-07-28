#include "ros/ros.h"
#include "surgical_robot/motor_commands.h"

#define DEFAULT_RATE 20
#define RATE_LOWER_LIMIT 1

void chripCallback(int , ros::Publisher&,surgical_robot::motor_commands &, const ros::TimerEvent &);
typedef const boost::function<void(const ros::TimerEvent &)> pub_callback;

int main(int argc, char** argv){
    ros::init(argc,argv,"lse_input");
    ros::NodeHandle n;

    //loop rate
    int rate = (argv[1]==NULL)?DEFAULT_RATE:atoi(argv[1]);
    rate = 1/(rate>0?rate:RATE_LOWER_LIMIT);

    //publisher
    ros::Publisher pub = n.advertise<surgical_robot::motor_commands>("motor_command",1000);
    surgical_robot::motor_commands msg;
    pub_callback publisher_callback = boost::bind(chripCallback,rate,boost::ref(pub),boost::ref(msg),_1);
    ros::Timer timer = n.createTimer(ros::Duration(rate),publisher_callback);

    ros::spin();

    return 0;
}

void chripCallback(int rate, ros::Publisher& pub,surgical_robot::motor_commands & msg, const ros::TimerEvent & tmer){
    ros::Time now = ros::Time::now();
    ROS_INFO("Time: %d, %d",now.sec,now.nsec);
}