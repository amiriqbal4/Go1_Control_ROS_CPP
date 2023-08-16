//
// Created by Amir 02/23
// Ref. ShuoYang Github
//

#ifndef GO1_ROBOTCONTROL_H
#define GO1_ROBOTCONTROL_H
///

///
#include <iostream>
#include <string>
#include <chrono>

// to debug
#include <ros/console.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <Eigen/Core>

#include "Go1Parameters.h"
#include "Go1ControlStates.h"
#include "utils/Utils.h"
#include "ConvexMpc.h"

#include "utils/filter.hpp"


class Go1RobotControl {
public:
    Go1RobotControl();

    Go1RobotControl(ros::NodeHandle &_nh);

    void update_plan(Go1ControlStates &state, double dt);

    void generate_swing_legs_ctrl(Go1ControlStates &state, double dt);

    void compute_joint_torques(Go1ControlStates &state);

    Eigen::Matrix<double, 3, NUM_LEG> compute_grf(Go1ControlStates &state, double dt);

    Eigen::Vector3d compute_walking_surface(Go1ControlStates &state);

private:
    BezierUtils bezierUtils[NUM_LEG];

    Eigen::Matrix<double, 6, 1> root_acc;
    Eigen::Matrix<double, 6, 1> desired_inertial_force; // Equivalent to b_d
    // allocate the problem weight matrices
    Eigen::DiagonalMatrix<double, 6> Q;
    double R;
    // ground friction coefficient
    double mu;
    double F_min;
    double F_max;
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;


    OsqpEigen::Solver solver;

    ////////////////////
    //Eigen::VectorXd hlipGains;
    OsqpEigen::Solver solver_hlip;

    Eigen::Matrix2d Phi_rem; // StateTransitionMatrix for the nominal remaining time in swing phase

    Eigen::VectorXd GainsHLIP; // HLIP-based stepping controller gain
    Eigen::SparseMatrix<double> hlip_hessian; //SteppingControllerHessian
    Eigen::VectorXd hlip_gradient; //SteppingControllerQPgradient
    Eigen::VectorXd hlip_ub; 
    Eigen::VectorXd hlip_lb; 
    Eigen::SparseMatrix<double> hlip_linearConst;

    Eigen::Matrix<double, 6, 4> hlip_dense_linearConst; // SteppingController LinearConstraints lb<= hlip_A_linearConst*gain_k<= hlip_b_linearConst
    Eigen::Matrix<double, 4, 4> hlip_dense_hessian;

    Eigen::Vector2d StanceFeetSumXY;
    Eigen::Vector2d StanceFeetMP, StanceFeetMP_prev;
    Eigen::Vector2d rel_error_pos;
    Eigen::Vector2d rel_error_vel;
    Eigen::Vector2d u_ref;
    Eigen::Vector2d X_hlip, Y_hlip, X_hlip_minus, Y_hlip_minus, X_hlip_ref_minus, Y_hlip_ref_minus, X_hlip_error_minus, Y_hlip_error_minus;
    double x_footprint_offset, y_footprint_offset;
    Eigen::VectorXd x_footprint_default, y_footprint_default;

    // Eigen::Matrix<double, 2*GAIN_SIZE, 1> hlip_b_linearConst; // linearconstraint b

    ////////////////////



    //add a number of ROS debug topics
    ros::NodeHandle nh;
    ros::Publisher pub_foot_start[NUM_LEG];
    ros::Publisher pub_foot_end[NUM_LEG];
    ros::Publisher pub_foot_path[NUM_LEG];
    visualization_msgs::Marker foot_start_marker[NUM_LEG];
    visualization_msgs::Marker foot_end_marker[NUM_LEG];
    visualization_msgs::Marker foot_path_marker[NUM_LEG];

    //debug topics
//    ros::Publisher pub_root_lin_vel;
//    ros::Publisher pub_root_lin_vel_d;
    ros::Publisher pub_terrain_angle;

    ros::Publisher pub_foot_pose_target_FL;
    ros::Publisher pub_foot_pose_target_FR;
    ros::Publisher pub_foot_pose_target_RL;
    ros::Publisher pub_foot_pose_target_RR;

    ros::Publisher pub_foot_pose_target_rel_FL;
    ros::Publisher pub_foot_pose_target_rel_FR;
    ros::Publisher pub_foot_pose_target_rel_RL;
    ros::Publisher pub_foot_pose_target_rel_RR;

    ros::Publisher pub_foot_pose_error_FL;
    ros::Publisher pub_foot_pose_error_FR;
    ros::Publisher pub_foot_pose_error_RL;
    ros::Publisher pub_foot_pose_error_RR;

    ros::Publisher pub_euler;

    //MPC does not start for the first 10 ticks to prevent uninitialized NAN goes into joint_torques
    int mpc_init_counter;

    std::string use_sim_time;

    // filters
    MovingWindowFilter filter_z_imu_acceleration;
    MovingWindowFilter terrain_angle_filter;
    MovingWindowFilter recent_contact_x_filter[NUM_LEG];
    MovingWindowFilter recent_contact_y_filter[NUM_LEG];
    MovingWindowFilter recent_contact_z_filter[NUM_LEG];
};


#endif //GO1_ROBOTCONTROL_H
