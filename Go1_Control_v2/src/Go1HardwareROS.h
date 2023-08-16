//
// Created by Amir 02/23
// Ref. ShuoYang Github
//

#ifndef GO1_HARDWAREROS_H
#define GO1_HARDWAREROS_H
//
//Pinocchio
#include <iostream>
#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/fcl-pinocchio-conversions.hpp"
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

// std
#include <Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <ctime>
#include <sstream>
#include <iomanip>

////////

///
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
//#include <filesystem>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>

// control parameters
#include "Go1Parameters.h"
#include "Go1ControlStates.h"
#include "Go1RobotControl.h"
#include "Go1FilterEKF.h"
#include "utils/Utils.h"
#include "legKinematics/Go1Kinematics.h"
// Go1 hardware
#include "unitree_legged_sdk/unitree_legged_sdk.h" //Need to redirect

#define FOOT_FILTER_WINDOW_SIZE 5

////////////////////////////////////
class Go1HardwareROS {
public:
    Go1HardwareROS(ros::NodeHandle &_nh);

    ~Go1HardwareROS() {
        destruct = true;
        thread_.join();
    }

    bool update_foot_forces_grf(double dt);

    bool main_update(double t, double dt);

    bool send_cmd();

    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);

    int getch(); // added for enabling keybord control
    void keyboardCallback(const std_msgs::Char::ConstPtr& msg); //const std_msgs::String::ConstPtr& msg
    //
    
    //void keyboardCallback(const ros::TimerEvent& event);              //TODO I will edit this function to receive keyboard input

    //sub_keyBoardControl = nh_.subscribe("/keyBoard", 1000, &Go1HardwareROS::keyboardCallback, this); //Added a keyboard cntrol option

    //

    /*
    // DataWritingToFile
    std::vector<Eigen::VectorXd> StoredDataVector;
    std::ofstream myfile;
    myfile.open("TrialDataRecording.csv", ios::app);
    myfile << EstimatedBasePosVel_BaseOrientAngularVel_JointPosVelTorque_FootForce_IMUAccelAngVelQuaternion<<"\n"
    //
    */

private:
    ros::NodeHandle nh;
    ros::Publisher pub_joint_cmd;
    ros::Publisher pub_joint_angle;
    ros::Publisher pub_imu;
    ros::Publisher pub_joy_msg;

    sensor_msgs::JointState joint_foot_msg;
    sensor_msgs::Imu imu_msg;
    ros::Subscriber sub_joy_msg;
    ros::Subscriber sub_keyBoardControl;

    // debug estimated position
    ros::Publisher pub_estimated_pose;

    //To run keyboard input
    ros::Timer keyboardtimer;

    // Go1 hardware
    UNITREE_LEGGED_SDK::UDP udp;
    UNITREE_LEGGED_SDK::Safety safe;
    UNITREE_LEGGED_SDK::LowState state = {0};
    UNITREE_LEGGED_SDK::LowCmd cmd = {0};
    // Go1 hardware reading thread
    std::thread thread_;
    bool destruct = false;

    void udp_init_send();

    void receive_low_state();

    // Go1 hardware switch foot order
    // Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    // Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

    // Go1 hardware foot force filter
    Eigen::Matrix<double, NUM_LEG, FOOT_FILTER_WINDOW_SIZE> foot_force_filters;
    Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;


    // joystic command
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.15;

    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_exit = false;

    int tp_myfile =0;

    // following cade is also in VILEOM
    // add leg kinematics
    // the leg kinematics is relative to body frame, which is the center of the robot
    // following are some parameters that defines the transformation between IMU frame(b) and robot body frame(r)
    Eigen::Vector3d p_br;
    Eigen::Matrix3d R_br;
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double leg_offset_x[4] = {};
    double leg_offset_y[4] = {};
    // for each leg, there is an offset between the body frame and the hip motor (fx, fy)
    double motor_offset[4] = {};
    double upper_leg_length[4] = {};
    double lower_leg_length[4] = {};

    // std::vector<Eigen::VectorXd> rho_fix_list;
    // std::vector<Eigen::VectorXd> rho_opt_list;

    Go1Kinematics fk_jac_go1;
    // variables related to control and estimation
    Go1ControlStates control_states_go1;
    Go1RobotControl _main_control_go1;
    Go1FilterEKF estimates_go1;
    /////////////////////
    ////// Pinocchio
    // Load robot model from URDF file
    //const std::string dir = PINOCCHIO_MODEL_DIR;
    const std::string filename = std::string("/home/amir/ros_catkin_ws/catkin_ws/src/Go1_Control_v2/src/legKinematics/Go1.urdf");
    pinocchio::Model model;

};

#endif //GO1_HARDWAREROS_H
