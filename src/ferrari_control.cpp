
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

        ros::Publisher pubAutoMode;
        ros::Publisher pubAutoSteer;
        ros::Publisher pubAutoThrottle;

    private:
        ros::Subscriber subGearNum;
        ros::Subscriber subSteer;
        ros::Subscriber subThrottle;
};

void KeyboardController::callbackGearNum(const std_msgs::Int16& msg)
{

    return;
}

void KeyboardController::callbackSteer(const std_msgs::Int16& msg)
{

    return;
}

void KeyboardController::callbackThrottle(const std_msgs::Int16& msg)
{

    return;
}

KeyboardController::KeyboardController(ros::NodeHandle& nh) : nh_(nh)
{
    steer = 1500;
    throttle = 1500;
    autoMode = true;

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
    ROS_INFO("Ferrari controller destructor.");
}


int main(int argc, char **argv){
    ros::init(argc, argv, "keyboard_controller"); 
    // for subscribe
    ros::NodeHandle nh;
    KeyboardController controller(nh);

    cmdMode.data = controller.autoMode;
    init_termios(0);

    while(true) {
        char c = getch();
        controller.pubAutoMode.publish(cmdMode);
        if (c == 'w') {
            controller.throttle = 1400;
        }
        else if (c == 's') {
            controller.throttle = 1600;
        }
        if (c == 'd') {
            controller.steer = 1700;
        }
        else if (c == 'a') {
            controller.steer = 1300;
        }
        else if (c == 'x') {
            controller.steer = 1500;
        }
        if (c == 'q') {
            break;
        }
        cmdSteer.data = controller.steer;
        cmdThrottle.data = controller.throttle;
        controller.pubAutoSteer.publish(cmdSteer);
        controller.pubAutoThrottle.publish(cmdThrottle);
        ros::spinOnce();
    }

    reset_termios();
    return 0;
}