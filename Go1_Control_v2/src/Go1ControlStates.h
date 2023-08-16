//
// Created by shuoy on 10/18/21.
// Modified by Amir on 02/07/23.
//
//

#ifndef GO1_CPP_CTRLSTATES_H
#define GO1_CPP_CTRLSTATES_H

#include <Eigen/Dense>
#include <ros/ros.h>

#include "Go1Parameters.h"

class Go1ControlStates {
public:
    // this init function sets all variables to that used in orbit.issac.go1 controller
    Go1ControlStates() {
        reset();
    }

    void reset() {
        stance_leg_control_type = 1;
        use_terrain_adapt = 1;
        movement_mode = 0;
        counter_per_swing = 120;
        counter_per_gait = counter_per_swing * 2;
        counter = 0;
        gait_counter.setZero(); // use to keep track of step duration
        gait_counter_speed.setZero();
        gait_type = 1; //trotting
        gait_type_last = 1;
        // init gait counter
        // TODO: add other gait patterns?
        gait_counter_reset();

        root_pos_d.setZero();
        root_euler_d.setZero();
        root_lin_vel_d.setZero();
        root_ang_vel_d.setZero();

        robot_mass = 12.5;  // Should we use the lumped inertia of the entire robot or just the trunk?
        go1_lumped_inertia << 0.1486, 0.0005, -0.01,
            0.0005, 0.3433, 0.0001,
            -0.01, 0.0001, 0.3376;
        /*
        go1_lumped_inertia << 0.0158533, 0.0, 0.0,
                0.0, 0.0377999, 0.0,
                0.0, 0.0, 0.0456542;
        */
        //         // Check this later FL FR RL RR
        // default_foot_pos << 0.18, 0.18, -0.18, -0.18,
        //         0.13, -0.13, 0.13, -0.13,
        //         -0.2, -0.2, -0.2, -0.2;
                // Check this later FR FL RR RL
        default_foot_pos << 0.188, 0.188, -0.188, -0.188,
                -0.13, 0.13, -0.13, 0.13,
                -0.2, -0.2, -0.2, -0.2;

        q_weights.resize(13);
        r_weights.resize(12);

        q_weights << 80.0, 80.0, 1.0,
                0.0, 0.0, 270.0,
                1.0, 1.0, 20.0,
                20.0, 20.0, 20.0,
                0.0;
        r_weights << 1e-5, 1e-5, 1e-6,
                1e-5, 1e-5, 1e-6,
                1e-5, 1e-5, 1e-6,
                1e-5, 1e-5, 1e-6;

        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setZero();
        root_rot_mat_z.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();
        root_acc.setZero();

        foot_force.setZero();

        joint_pos.setZero();
        joint_vel.setZero();

        walking_surface_height_tmp = 0;
        walking_surface_height = 0;
        walking_surface_fit_count = 0;

        foot_pos_target_world.setZero();
        foot_pos_target_abs.setZero();
        foot_pos_target_rel.setZero();
        foot_pos_start.setZero();
        foot_pos_world.setZero();
        foot_pos_abs.setZero();
        foot_pos_rel.setZero();
        foot_pos_abs_mpc.setZero();
        foot_pos_rel_last_time.setZero();
        foot_pos_target_last_time.setZero();
        foot_pos_cur.setZero();
        foot_pos_recent_contact.setZero();
        foot_vel_world.setZero();
        foot_vel_abs.setZero();
        foot_vel_rel.setZero();
        j_foot.setIdentity();

        for (int i = 0; i < NUM_LEG; ++i) {
            contacts[i] = false;
            plan_contacts[i] = false;
            early_contacts[i] = false;
        }

        gait_counter_speed << 1, 1, 1, 1;

        double kp_foot_x = 300.0;
        double kp_foot_y = 400.0;
        double kp_foot_z = 400.0;

        double kd_foot_x = 8.0;
        double kd_foot_y = 8.0;
        double kd_foot_z = 8.0;

        kp_foot <<
                kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
                kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
                kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
        kd_foot <<
                kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
                kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
                kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

        km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

        kp_linear = Eigen::Vector3d(1000.0, 1000.0, 1000.0);
        kd_linear = Eigen::Vector3d(200.0, 70.0, 120.0);
        kp_angular = Eigen::Vector3d(650.0, 35.0, 1.0);
        kd_angular = Eigen::Vector3d(4.5, 4.5, 30.0);


        // torques_gravity << -0.66, 0.27, -0.12, 0.66, 0.27, -0.12,-0.66, 0.27, -0.12, 0.66, 0.27, -0.12; //Gravity compensation for walking at 0.32m height
        // torques_gravity << -0.65, 0, 0, 0.65, 0, 0,-0.65, 0, 0, 0.65, 0, 0;
        // torques_gravity << -0.66, 0.0, -0.12, 0.66, 0.0, -0.12,-0.66, 0.0, -0.12, 0.66, 0.0, -0.12; 
        torques_gravity << -0.66, -0.37, 0.16, 0.66, -0.37, 0.0,-0.66, -0.37, 0.0, 0.66, -0.37, 0.16;
        // torques_gravity << -0.66, 0.37, -0.16, 0.66, 0.37, -0.16,-0.66, 0.37, -0.16, 0.66, 0.37, -0.16; //Gravity compensation for walking at 0.32m height

        joint_torques.setZero();

        HLIP_Gains << 1.0, 0.18, 1.0, 0.18;

        power_level = 5;
    }

    void resetFromROSParam(ros::NodeHandle &_nh) {


        //////////////////////////////////////////////////////////////////////////////////////////
        ///// _nh.param("parameter_name", variable_to_store_value, default_value);////////////////
        ///// if parameter server gives a value for "variable_to_store_value", then /////////////
        ///// this value is assigned to the parameter otherwise "default_value" is stored ///////
        /////////////////////////////////////////////////////////////////////////////////////////
        _nh.param("stance_leg_control_type", stance_leg_control_type, 1);
        _nh.param("use_terrain_adapt", use_terrain_adapt, 1);
        

        _nh.param("go1_robot_mass", robot_mass, 11.50);

        double go1_lumped_inertia_xx;
        double go1_lumped_inertia_xy;
        double go1_lumped_inertia_xz;
        double go1_lumped_inertia_yz;
        double go1_lumped_inertia_yy;
        double go1_lumped_inertia_zz;

        // _nh.param("go1_lumped_inertia_xx", go1_lumped_inertia_xx, 0.1486);
        // _nh.param("go1_lumped_inertia_xy", go1_lumped_inertia_xy, 0.0005);
        // _nh.param("go1_lumped_inertia_xz", go1_lumped_inertia_xz, -0.01);
        // _nh.param("go1_lumped_inertia_yz", go1_lumped_inertia_yz, 0.0001);
        // _nh.param("go1_lumped_inertia_yy", go1_lumped_inertia_yy, 0.3433);
        // _nh.param("go1_lumped_inertia_zz", go1_lumped_inertia_zz, 0.3376);
        _nh.param("go1_lumped_inertia_xx", go1_lumped_inertia_xx, 0.1486);
        _nh.param("go1_lumped_inertia_xy", go1_lumped_inertia_xy, 0.0);
        _nh.param("go1_lumped_inertia_xz", go1_lumped_inertia_xz, 0.0);
        _nh.param("go1_lumped_inertia_yz", go1_lumped_inertia_yz, 0.0);
        _nh.param("go1_lumped_inertia_yy", go1_lumped_inertia_yy, 0.3433);
        _nh.param("go1_lumped_inertia_zz", go1_lumped_inertia_zz, 0.3376);

        go1_lumped_inertia << go1_lumped_inertia_xx, go1_lumped_inertia_xy, go1_lumped_inertia_xz,
                go1_lumped_inertia_xy, go1_lumped_inertia_yy, go1_lumped_inertia_yz,
                go1_lumped_inertia_xz, go1_lumped_inertia_yz, go1_lumped_inertia_zz;



        double go1_default_foot_pos_FR_x;
        double go1_default_foot_pos_FR_y;
        double go1_default_foot_pos_FR_z;

        double go1_default_foot_pos_FL_x;
        double go1_default_foot_pos_FL_y;
        double go1_default_foot_pos_FL_z;

        double go1_default_foot_pos_RR_x;
        double go1_default_foot_pos_RR_y;
        double go1_default_foot_pos_RR_z;

        double go1_default_foot_pos_RL_x;
        double go1_default_foot_pos_RL_y;
        double go1_default_foot_pos_RL_z;


        _nh.param("go1_default_foot_pos_FR_x", go1_default_foot_pos_FR_x, 0.188);
        _nh.param("go1_default_foot_pos_FR_y", go1_default_foot_pos_FR_y, -0.125);
        _nh.param("go1_default_foot_pos_FR_z", go1_default_foot_pos_FR_z, -0.2);

        _nh.param("go1_default_foot_pos_FL_x", go1_default_foot_pos_FL_x, 0.188);
        _nh.param("go1_default_foot_pos_FL_y", go1_default_foot_pos_FL_y, 0.125);
        _nh.param("go1_default_foot_pos_FL_z", go1_default_foot_pos_FL_z, -0.2);

        _nh.param("go1_default_foot_pos_RR_x", go1_default_foot_pos_RR_x, -0.188);
        _nh.param("go1_default_foot_pos_RR_y", go1_default_foot_pos_RR_y, -0.125);
        _nh.param("go1_default_foot_pos_RR_z", go1_default_foot_pos_RR_z, -0.2);

        _nh.param("go1_default_foot_pos_RL_x", go1_default_foot_pos_RL_x, -0.188);
        _nh.param("go1_default_foot_pos_RL_y", go1_default_foot_pos_RL_y, 0.125);
        _nh.param("go1_default_foot_pos_RL_z", go1_default_foot_pos_RL_z, -0.2);


        default_foot_pos << go1_default_foot_pos_FR_x, go1_default_foot_pos_FL_x, go1_default_foot_pos_RR_x, go1_default_foot_pos_RL_x, 
                go1_default_foot_pos_FR_y, go1_default_foot_pos_FL_y,  go1_default_foot_pos_RR_y, go1_default_foot_pos_RL_y,
                go1_default_foot_pos_FR_z, go1_default_foot_pos_FL_z, go1_default_foot_pos_RR_z,  go1_default_foot_pos_RL_z;

        double q_weights_0, q_weights_1, q_weights_2, q_weights_3, q_weights_4, q_weights_5, q_weights_6, q_weights_7, q_weights_8, q_weights_9, q_weights_10, q_weights_11, q_weights_12;

        _nh.param("q_weights_0", q_weights_0, 80.0);
        _nh.param("q_weights_1", q_weights_1, 80.0);
        _nh.param("q_weights_2", q_weights_2, 1.0);

        _nh.param("q_weights_3", q_weights_3, 0.0);
        _nh.param("q_weights_4", q_weights_4, 0.0);
        _nh.param("q_weights_5", q_weights_5, 270.0);

        _nh.param("q_weights_6", q_weights_6, 1.0);
        _nh.param("q_weights_7", q_weights_7, 1.0);
        _nh.param("q_weights_8", q_weights_8, 20.0);

        _nh.param("q_weights_9", q_weights_9, 20.0);
        _nh.param("q_weights_10", q_weights_10, 20.0);
        _nh.param("q_weights_11", q_weights_11, 20.0);

        _nh.param("q_weights_12", q_weights_12, 0.0);

        q_weights << q_weights_0, q_weights_1, q_weights_2,
                q_weights_3, q_weights_4, q_weights_5,
                q_weights_6, q_weights_7, q_weights_8,
                q_weights_9, q_weights_10, q_weights_11,
                q_weights_12;

        double r_weights_0, r_weights_1, r_weights_2, r_weights_3, r_weights_4, r_weights_5, r_weights_6, r_weights_7, r_weights_8, r_weights_9, r_weights_10, r_weights_11;

        _nh.param("r_weights_0", r_weights_0, 1e-5);
        _nh.param("r_weights_1", r_weights_1, 1e-5);
        _nh.param("r_weights_2", r_weights_2, 1e-6);

        _nh.param("r_weights_3", r_weights_3, 1e-5);
        _nh.param("r_weights_4", r_weights_4, 1e-5);
        _nh.param("r_weights_5", r_weights_5, 1e-6);

        _nh.param("r_weights_6", r_weights_6, 1e-5);
        _nh.param("r_weights_7", r_weights_7, 1e-5);
        _nh.param("r_weights_8", r_weights_8, 1e-6);

        _nh.param("r_weights_9", r_weights_9, 1e-5);
        _nh.param("r_weights_10", r_weights_10, 1e-5);
        _nh.param("r_weights_11", r_weights_11, 1e-6);

        r_weights << r_weights_0, r_weights_1, r_weights_2,
                r_weights_3, r_weights_4, r_weights_5,
                r_weights_6, r_weights_7, r_weights_8,
                r_weights_9, r_weights_10, r_weights_11;

        double go1_kp_foot_x, go1_kp_foot_y, go1_kp_foot_z, go1_kd_foot_x, go1_kd_foot_y, go1_kd_foot_z, go1_km_foot_x, go1_km_foot_y, go1_km_foot_z;

        _nh.param("go1_kp_foot_x", go1_kp_foot_x, 150.0);
        _nh.param("go1_kp_foot_y", go1_kp_foot_y, 150.0);
        _nh.param("go1_kp_foot_z", go1_kp_foot_z, 200.0);

        _nh.param("go1_kd_foot_x", go1_kd_foot_x, 0.0);
        _nh.param("go1_kd_foot_y", go1_kd_foot_y, 0.0);
        _nh.param("go1_kd_foot_z", go1_kd_foot_z, 0.0);

        _nh.param("go1_km_foot_x", go1_km_foot_x, 0.1);
        _nh.param("go1_km_foot_y", go1_km_foot_y, 0.1);
        _nh.param("go1_km_foot_z", go1_km_foot_z, 0.04);

        kp_foot <<
                go1_kp_foot_x, go1_kp_foot_x, go1_kp_foot_x, go1_kp_foot_x,
                go1_kp_foot_y, go1_kp_foot_y, go1_kp_foot_y, go1_kp_foot_y,
                go1_kp_foot_z, go1_kp_foot_z, go1_kp_foot_z, go1_kp_foot_z;
        kd_foot <<
                go1_kd_foot_x, go1_kd_foot_x, go1_kd_foot_x, go1_kd_foot_x,
                go1_kd_foot_y, go1_kd_foot_y, go1_kd_foot_y, go1_kd_foot_y,
                go1_kd_foot_z, go1_kd_foot_z, go1_kd_foot_z, go1_kd_foot_z;

        km_foot = Eigen::Vector3d(go1_km_foot_x, go1_km_foot_y, go1_km_foot_z);

        double go1_kp_linear_x;
        double go1_kp_linear_y;
        double go1_kp_linear_z;
        _nh.param("go1_kp_linear_x", go1_kp_linear_x, 120.0);
        _nh.param("go1_kp_linear_y", go1_kp_linear_y, 120.0);
        _nh.param("go1_kp_linear_z", go1_kp_linear_z, 500.0);

        double go1_kd_linear_x;
        double go1_kd_linear_y;
        double go1_kd_linear_z;
        _nh.param("go1_kd_linear_x", go1_kd_linear_x, 70.0);
        _nh.param("go1_kd_linear_y", go1_kd_linear_y, 70.0);
        _nh.param("go1_kd_linear_z", go1_kd_linear_z, 120.0);

        double go1_kp_angular_x;
        double go1_kp_angular_y;
        double go1_kp_angular_z;
        _nh.param("go1_kp_angular_x", go1_kp_angular_x, 250.0);
        _nh.param("go1_kp_angular_y", go1_kp_angular_y, 35.0);
        _nh.param("go1_kp_angular_z", go1_kp_angular_z, 1.0);

        double go1_kd_angular_x;
        double go1_kd_angular_y;
        double go1_kd_angular_z;
        _nh.param("go1_kd_angular_x", go1_kd_angular_x, 1.5);
        _nh.param("go1_kd_angular_y", go1_kd_angular_y, 1.5);
        _nh.param("go1_kd_angular_z", go1_kd_angular_z, 30.0);

        kp_linear_lock_x = go1_kp_linear_x;
        kp_linear_lock_y = go1_kp_linear_y;

        kp_linear = Eigen::Vector3d(go1_kp_linear_x, go1_kp_linear_y, go1_kp_linear_z);
        kd_linear = Eigen::Vector3d(go1_kd_linear_x, go1_kd_linear_y, go1_kd_linear_z);
        kp_angular = Eigen::Vector3d(go1_kp_angular_x, go1_kp_angular_y, go1_kp_angular_z);
        kd_angular = Eigen::Vector3d(go1_kd_angular_x, go1_kd_angular_y, go1_kd_angular_z);

        double go1_gait_counter_speed_FL;
        double go1_gait_counter_speed_FR;
        double go1_gait_counter_speed_RL;
        double go1_gait_counter_speed_RR;

       
        _nh.param("go1_gait_counter_speed_FR", go1_gait_counter_speed_FR, 1.0);
        _nh.param("go1_gait_counter_speed_FL", go1_gait_counter_speed_FL, 1.0);
        _nh.param("go1_gait_counter_speed_RR", go1_gait_counter_speed_RR, 1.0);
        _nh.param("go1_gait_counter_speed_RL", go1_gait_counter_speed_RL, 1.0);
        

        gait_counter_speed
                << go1_gait_counter_speed_FR, go1_gait_counter_speed_FL, go1_gait_counter_speed_RR, go1_gait_counter_speed_RL;

        _nh.param("go1_hardware_power_level", power_level, 2);
    }

    void gait_counter_reset() {
        if (gait_type == 1) {
            gait_counter << 0, counter_per_swing, counter_per_swing, 0;
        }
    }

    // variables
    int stance_leg_control_type; // 0: QP, 1: MPC
    int movement_mode;  // 0: standstill, 1: start to locomote
    int use_terrain_adapt; 
    double control_dt = MAIN_UPDATE_FREQUENCY / 1000.0;

    // period of one gait cycle
    double plan_dt;
    int counter_per_plan;
    double counter_per_gait;
    double counter_per_swing;
    int counter;
    Eigen::Vector4d gait_counter;
    Eigen::Vector4d gait_counter_speed;

    int gait_type;
    int gait_type_last;

    // control target
    Eigen::Vector3d root_pos_d;
    Eigen::Vector3d root_euler_d;
    Eigen::Vector3d root_lin_vel_d;
    Eigen::Vector3d root_lin_vel_d_world;
    Eigen::Vector3d root_ang_vel_d;
    Eigen::Vector3d root_ang_vel_d_world;

    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;

    // important kinematics constants
    double robot_mass;

    Eigen::Matrix3d go1_lumped_inertia;

    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;

    // MPC parameters
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;

    // terrain related
    double terrain_pitch_angle;  // the estimated terrain angle on pitch direction

    // important kinematics variables
    Eigen::Vector3d root_pos;
    Eigen::Quaterniond root_quat;
    Eigen::Vector3d root_euler;
    Eigen::Matrix3d root_rot_mat;
    Eigen::Matrix3d root_rot_mat_z;
    Eigen::Vector3d root_lin_vel;
    Eigen::Vector3d root_ang_vel;
    Eigen::Vector3d root_acc;

    Eigen::Vector4d foot_force;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    Eigen::Vector4d isaac_contact_flag;

    Eigen::Matrix<double, NUM_DOF, 1> joint_pos;
    Eigen::Matrix<double, NUM_DOF, 1> joint_vel;

    double walking_surface_height_tmp;
    double walking_surface_height;
    int walking_surface_fit_count;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // in the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // in the robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs_mpc;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel;
    Eigen::Matrix<double, 12, 12> j_foot;

    bool contacts[NUM_LEG];         // flag to decide leg in the stance/swing mode
    bool plan_contacts[NUM_LEG];    // planed flag for stance/swing mode
    bool early_contacts[NUM_LEG];   // true if foot hit objects during swing

    // controller variables
    double kp_lin_x;
    double kd_lin_x;
    double kf_lin_x;
    double kp_lin_y;
    double kd_lin_y;
    double kf_lin_y;

    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, 1> km_foot;

    double kp_linear_lock_x, kp_linear_lock_y;
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;

    Eigen::Matrix<double, NUM_DOF, 1> torques_gravity;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_acc_abs; //frame aligned to world frame;
    Eigen::Vector3d imu_ang_vel;

    // state estimation
    bool estimated_contacts[NUM_LEG];  // true if the estimator thinks the foot has contact
    Eigen::Vector3d estimated_root_pos;
    Eigen::Vector3d estimated_root_vel;
    // HLIP gains
    Eigen::Vector4d HLIP_Gains;


    // hardware
    int power_level;
};

#endif //GO1_CPP_CTRLSTATES_H
