#include "shm_msgs.h"
#include <fstream>

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"


ros::Subscriber file_name_sub, toggle_sub;
ros::Publisher status_pub;
ros::Timer status_timer;
ros::Time start_time_;

bool is_logging_ = false;

SHMmsgs *shm_msgs_;
std::string FILE_DIRECTORY = "/home/yong20/data/"; // Change this to your desired directory
std::string filename_ = "tocabi_log.csv";
std::ofstream log_file_;

void toggleCallback(const std_msgs::Bool::ConstPtr& msg);
void fileNameCallback(const std_msgs::String::ConstPtr& msg);
void statusTimerCallback(const ros::TimerEvent&);
void publishStatus(const std::string &s);