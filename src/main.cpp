#include "tocabi_logger/tocabi_data.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tocabi_logger", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    file_name_sub = nh.subscribe("/tc_rec/record_filename", 1, &fileNameCallback);
    toggle_sub = nh.subscribe("/tc_rec/is_logging", 1, &toggleCallback);
    status_pub = nh.advertise<std_msgs::String>("/tc_rec/logger_status", 1);
    status_timer = nh.createTimer(ros::Duration(0.1), &statusTimerCallback);

    int shm_id_;
    init_shm(shm_msg_key, shm_id_, &shm_msgs_);

    ros::Rate rate(250);
    while (ros::ok())
    {
        ros::spinOnce();
        if (is_logging_ && log_file_.is_open())
        {
            log_file_ << std::fixed << std::setprecision(8);
            //////////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////DATA LOGGING//////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////
            
            // 1. Joint Position
            for (int i = 0; i < 33; i++)
            {
                log_file_ << shm_msgs_->pos[i] << "\t";
            }
            // 2. Joint Velocity
            for (int i = 0; i < 33; i++)
            {
                log_file_ << shm_msgs_->vel[i] << "\t";
            }
            // 3. Joint Torque
            for (int i = 0; i < 33; i++)
            {
                log_file_ << shm_msgs_->torqueActual[i] << "\t";
            }
            // 4. Virtual Position(xyz & quaternion) and Velocity(lin & ang)
            for (int i = 0; i < 7; i++)
            {
                log_file_ << shm_msgs_->pos_virtual[i] << "\t";
            }
            for (int i = 0; i < 6; i++)
            {
                log_file_ << shm_msgs_->vel_virtual[i] << "\t";
            }
            // 5. Base linear Acceleration (by IMU)
            for (int i = 0; i < 3; i++)
            {
                log_file_ << shm_msgs_->imu_acc[i] << "\t";
            }
            // 6. FT sensor data (foot contact force) - left foot(fx, fy, fz, mx, my, mz) and right foot(fx, fy, fz, mx, my, mz)
            for (int i = 0; i < 12; i++)
            {
                log_file_ << shm_msgs_->ftSensor[i] << "\t";
            }

        }
        rate.sleep();
    }
    if (log_file_.is_open()) {
        log_file_.close();
        publishStatus("Data saved!");
    }
    deleteSharedMemory(shm_id_, shm_msgs_);
    std::cout << "Shared memory deleted." << std::endl;
    return 0;
}

void fileNameCallback(const std_msgs::String::ConstPtr& msg) {
    filename_ = FILE_DIRECTORY + msg->data;
}

void toggleCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data && !is_logging_) {
        // start
        log_file_.open(filename_, std::ofstream::out | std::ofstream::app);
        if (log_file_.is_open()) {
            is_logging_ = true;
            start_time_ = ros::Time::now();
            publishStatus("Data saving... 0.0s");
        } 
        else {
            publishStatus("Failed to open file");
        }
    }
    else if (!msg->data && is_logging_) {
        // stop
        log_file_.close();
        is_logging_ = false;
        publishStatus("Data saved!");
    }
}

void statusTimerCallback(const ros::TimerEvent&) {
    if (!is_logging_) return;

    double elapsed = (ros::Time::now() - start_time_).toSec();
    std::ostringstream ss;
    ss << "Data saving... "
       << std::fixed << std::setprecision(1)
       << elapsed << "s";
    publishStatus(ss.str());
}

void publishStatus(const std::string &s) {
    std_msgs::String st;
    st.data = s;
    status_pub.publish(st);
}
