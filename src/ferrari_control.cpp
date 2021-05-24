
#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

void init_termios(int);
void reset_termios(void);
char getch(void);

std_msgs::Bool cmdMode;
std_msgs::Int16 cmdSteer;
std_msgs::Int16 cmdThrottle;

class KeyboardController
{
    public:
        KeyboardController(ros::NodeHandle& nh);
        ~KeyboardController();
        void callbackGearNum(const std_msgs::Int16& msg);
        void callbackSteer(const std_msgs::Int16& msg);
        void callbackThrottle(const std_msgs::Int16& msg);

        ros::NodeHandle nh_;

        int steer, throttle;
        bool autoMode;

    private:
        ros::Subscriber subGearNum;
        ros::Subscriber subSteer;
        ros::Subscriber subThrottle;

        ros::Publisher pubAutoMode;
        ros::Publisher pubAutoSteer;
        ros::Publisher pubAutoThrottle;
};

KeyboardController::KeyboardController(ros::NodeHandle& nh) : nh_(nh)
{
    steer = 1500;
    throttle = 1500;
    autoMode = false;

    subGearNum = nh_.subscribe("/car/gear_num",1, &KeyboardController::callbackGearNum, this);
    subSteer = nh_.subscribe("/rc_cmd/steer",1, &KeyboardController::callbackSteer, this);
    subThrottle = nh_.subscribe("/rc_cmd/throttle",1, &KeyboardController::callbackThrottle, this);

    pubAutoMode = nh_.advertise<std_msgs::Bool>("/auto_mode", 1, true);
    pubAutoSteer = nh_.advertise<std_msgs::Int16>("/auto_cmd/steer", 1, true);
    pubAutoThrottle = nh_.advertise<std_msgs::Int16>("/auto_cmd/throttle", 1, true);
};

KeyboardController::~KeyboardController() 
{
    cmdThrottle.data = 1500;
    cmdSteer.data = 1500;
    cmdMode.data = false;
    pubAutoMode.publish(cmdMode);
    pubAutoSteer.publish(cmdSteer);
    pubAutoThrottle.publish(cmdThrottle);
    ROS_INFO("AstarPlanner destructor.");
}


int main(int argc, char **argv){
    ros::init(argc, argv, "keyboard_controller"); 
    // for subscribe
    ros::NodeHandle nh;
    KeyboardController controller(nh);

    cmdMode.data = true;
    pubAutoMode.publish(cmdMode);
    init_termios(0);

    while(true) {
        char c = getch();
        if (c == 'w') {
            controller.steer = 1450;
            
        }
        else if (c == 's') {
            controller.steer = 1550;
        }
        if (c == 'd') {
            controller.throttle = 1550;
        }
        else if (c == 'a') {
            controller.throttle = 1450;
        }
        if (c == 'q') {
            break;
        }
        cmdSteer.data = controller.steer;
        cmdThrottle.data = controller.throttle;
        pubAutoSteer.publish(cmdSteer);
        pubAutoThrottle.publish(cmdThrottle);
        ros::spin_once();
    }
    reset_termios();
    return 0;
}