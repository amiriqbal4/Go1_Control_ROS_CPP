//
// Created by Amir 02/23
// Ref. ShuoYang Github
//

#include "HardwareA1ROS.h"


// DataWritingToFile
std::time_t fileOpen_time = std::time(nullptr);
std::string unique_id = std::to_string(fileOpen_time);
std::vector<Eigen::VectorXd> StoredDataVector;
std::ofstream myfile("TrialDataRecording_"+unique_id+".csv", ios::app);
//myfile << "EstimatedBasePosVel_BaseOrientAngularVel_JointPosVelTorque_FootForce_IMUAccelAngVelQuaternion" << "\n";
//

// constructor
HardwareA1ROS::HardwareA1ROS(ros::NodeHandle &_nh)
        : safe(UNITREE_LEGGED_SDK::LeggedType::Go1), udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007) {
//    UNITREE_LEGGED_SDK::InitEnvironment();
    nh = _nh;
    // ROS publisher
    pub_joint_cmd = nh.advertise<sensor_msgs::JointState>("/hardware_a1/joint_torque_cmd", 100);

    // debug joint angle and foot force
    pub_joint_angle = nh.advertise<sensor_msgs::JointState>("/hardware_a1/joint_foot", 100);

    // imu data
    pub_imu = nh.advertise<sensor_msgs::Imu>("/hardware_a1/imu", 100);

    joint_foot_msg.name = {"FL0", "FL1", "FL2",
                           "FR0", "FR1", "FR2",
                           "RL0", "RL1", "RL2",
                           "RR0", "RR1", "RR2",
                           "FL_foot", "FR_foot", "RL_foot", "RR_foot"};
    joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/hardware_a1/estimation_body_pose", 100);



    //sub_joy_msg = nh.subscribe("/joy", 1000, &HardwareA1ROS::joy_callback, this);
    //
    
    sub_keyBoardControl = nh.subscribe("/keyBoardControl", 1000, &HardwareA1ROS::keyboardCallback, this); //Added a keyboard cntrol option

    // Create timer to read keyboard input
    //keyboardtimer = nh.createTimer(ros::Duration(0.01), keyboardCallback);



    udp.InitCmdData(cmd);
    udp_init_send();



    _root_control = A1RobotControl(nh);
    a1_ctrl_states.reset();
    a1_ctrl_states.resetFromROSParam(nh);

    // init leg kinematics
    // set leg kinematics related parameters
    // body_to_a1_body
    p_br = Eigen::Vector3d(-0.2293, 0.0, -0.067);
    R_br = Eigen::Matrix3d::Identity();
    // leg order: 0-FL  1-FR  2-RL  3-RR
    leg_offset_x[0] = 0.1805;
    leg_offset_x[1] = 0.1805;
    leg_offset_x[2] = -0.1805;
    leg_offset_x[3] = -0.1805;
    leg_offset_y[0] = 0.047;
    leg_offset_y[1] = -0.047;
    leg_offset_y[2] = 0.047;
    leg_offset_y[3] = -0.047;
    motor_offset[0] = 0.0838;
    motor_offset[1] = -0.0838;
    motor_offset[2] = 0.0838;
    motor_offset[3] = -0.0838;
    upper_leg_length[0] = upper_leg_length[1] = upper_leg_length[2] = upper_leg_length[3] = 0.20;
    lower_leg_length[0] = lower_leg_length[1] = lower_leg_length[2] = lower_leg_length[3] = 0.20;

    for (int i = 0; i < NUM_LEG; i++) {
        Eigen::VectorXd rho_fix(5);
        rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], upper_leg_length[i], lower_leg_length[i];
        Eigen::VectorXd rho_opt(3);
        rho_opt << 0.0, 0.0, 0.0;
        rho_fix_list.push_back(rho_fix);
        rho_opt_list.push_back(rho_opt);
    }

    //init swap order, very important
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;


    // a1 hardware foot force filter reset
    foot_force_filters.setZero();
    foot_force_filters_idx.setZero();
    foot_force_filters_sum.setZero();

    int tp_myfile= 0; //add to contro frequency of data recording


    // start hardware reading thread after everything initialized
    thread_ = std::thread(&HardwareA1ROS::receive_low_state, this);
}

bool HardwareA1ROS::update_foot_forces_grf(double dt) {
    a1_ctrl_states.foot_forces_grf = _root_control.compute_grf(a1_ctrl_states, dt);
    return true;
}

bool HardwareA1ROS::main_update(double t, double dt) {
    if (joy_cmd_exit) {
        return false;
    }

    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into a1_ctrl_states
    joy_cmd_body_height += joy_cmd_velz * dt;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }

    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;

    if (joy_cmd_ctrl_state_change_request) {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; //TODO: how to toggle more states?
        joy_cmd_ctrl_state_change_request = false; //erase this change request;
    }

    // root_lin_vel_d is in robot frame
    a1_ctrl_states.root_lin_vel_d[0] = joy_cmd_velx;
    a1_ctrl_states.root_lin_vel_d[1] = joy_cmd_vely;

    // root_ang_vel_d is in robot frame
    a1_ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    a1_ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    a1_ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;
    a1_ctrl_states.root_euler_d[0] = joy_cmd_roll_rate;
    a1_ctrl_states.root_euler_d[1] = joy_cmd_pitch_rate;
    a1_ctrl_states.root_euler_d[2] += joy_cmd_yaw_rate * dt;
    a1_ctrl_states.root_pos_d[2] = joy_cmd_body_height;

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        // in walking mode, in this mode the robot should execute gait
        a1_ctrl_states.movement_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        a1_ctrl_states.movement_mode = 0;
        a1_ctrl_states.root_pos_d.segment<2>(0) = a1_ctrl_states.root_pos.segment<2>(0);
        a1_ctrl_states.kp_linear(0) = a1_ctrl_states.kp_linear_lock_x;
        a1_ctrl_states.kp_linear(1) = a1_ctrl_states.kp_linear_lock_y;
    } else {
        a1_ctrl_states.movement_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (a1_ctrl_states.movement_mode == 1) {
        if (a1_ctrl_states.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy
            a1_ctrl_states.root_pos_d.segment<2>(0) = a1_ctrl_states.root_pos.segment<2>(0);
            a1_ctrl_states.kp_linear.segment<2>(0).setZero();
        } else {
            a1_ctrl_states.kp_linear(0) = a1_ctrl_states.kp_linear_lock_x;
            a1_ctrl_states.kp_linear(1) = a1_ctrl_states.kp_linear_lock_y;
        }
    }

    _root_control.update_plan(a1_ctrl_states, dt);
    _root_control.generate_swing_legs_ctrl(a1_ctrl_states, dt);

    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = a1_ctrl_states.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = a1_ctrl_states.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = a1_ctrl_states.estimated_root_pos(2);
    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = a1_ctrl_states.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = a1_ctrl_states.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = a1_ctrl_states.estimated_root_vel(2);
    pub_estimated_pose.publish(estimate_odom);

    return true;
}

bool HardwareA1ROS::send_cmd() {
    _root_control.compute_joint_torques(a1_ctrl_states);

    // send control cmd to robot via unitree hardware interface
    // notice a1_ctrl_states.joint_torques uses order FL, FR, RL, RR
    // notice cmd uses order FR, FL, RR, RL
    cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i < NUM_DOF; i++) {
        cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
        cmd.motorCmd[i].Kd = 0;
        int swap_i = swap_joint_indices(i);
        cmd.motorCmd[i].tau = a1_ctrl_states.joint_torques(swap_i);
    }

    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, a1_ctrl_states.power_level);
    udp.SetSend(cmd);
    udp.Send();

    return true;
}

void HardwareA1ROS::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
    // left updown
    joy_cmd_velz = joy_msg->axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    //A
    if (joy_msg->buttons[0] == 1) {
        joy_cmd_ctrl_state_change_request = true;
    }


    // right updown
    joy_cmd_velx = joy_msg->axes[3] * JOY_CMD_VELX_MAX;
    joy_cmd_velx = 0.10;// trying to pass forward velocity
    // right horiz
    joy_cmd_vely = joy_msg->axes[2] * JOY_CMD_VELY_MAX;
    // left horiz
    joy_cmd_yaw_rate  = joy_msg->axes[0]*JOY_CMD_YAW_MAX;
    // cross button, left and right
    joy_cmd_roll_rate = joy_msg->axes[6] * JOY_CMD_ROLL_MAX * (-1);
    // cross button, up and down
    joy_cmd_pitch_rate = joy_msg->axes[7] * JOY_CMD_PITCH_MAX;


    // lb
    if (joy_msg->buttons[4] == 1) {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}


// Added a keyboard intervention function
///////////////////////////////////////////////
// Non-blocking keyboard input function
int HardwareA1ROS::getch()
{
    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int c = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    // std::cout<< " Inside getch function the value of shit c is = "<< c << std::endl;
    return c;
    
}

// int c = HardwareA1ROS::getch();

// std::cout<< " outside getch function the value of shit c is = "<< c<< std::endl;

// std::cout << "Check KeyBoard Control Vx =" <<  joy_cmd_velx << std::endl;

// Callback function for keyboard input
void HardwareA1ROS::keyboardCallback(const std_msgs::Char::ConstPtr& msg) //const std_msgs::String::ConstPtr& msg
{
    //int c = getch();
    char c =msg->data;

    if (c == 'w' || c == 'W' || c == 65)  // Up arrow key or 'W' key
    {
        joy_cmd_velx += STEP_VELX;
        if (joy_cmd_velx > JOY_CMD_VELX_MAX) joy_cmd_velx = JOY_CMD_VELX_MAX;
    }
    else if (c == 's' || c == 'S' || c == 66)  // Down arrow key or 'S' key
    {
        joy_cmd_velx -= STEP_VELX;
        if (joy_cmd_velx < -JOY_CMD_VELX_MAX) joy_cmd_velx = -JOY_CMD_VELX_MAX;
    }
    else if (c == 'a' || c == 'A' || c == 68)  // Left arrow key or 'A' key
    {
        joy_cmd_yaw_rate += STEP_YAW_RATE;
        if (joy_cmd_yaw_rate > JOY_CMD_YAW_RATE_MAX) joy_cmd_yaw_rate = JOY_CMD_YAW_RATE_MAX;
    }
    else if (c == 'd' || c == 'D' || c == 67)  // Right arrow key or 'D' key
    {
        joy_cmd_yaw_rate -= STEP_YAW_RATE;
        if (joy_cmd_yaw_rate < -JOY_CMD_YAW_RATE_MAX) joy_cmd_yaw_rate = -JOY_CMD_YAW_RATE_MAX;
    }
    // else if (c == 'o' || c == 'O' )  // reset yaw rate to zero
    // {
    //     joy_cmd_yaw_rate = 0.0;
    // }
    //
    else if (c == 'f' || c == 'F' )  // Letter 'l' or 'L' key
    {
        joy_cmd_vely += STEP_VELY;
        if (joy_cmd_vely > JOY_CMD_VELY_MAX) joy_cmd_vely = JOY_CMD_VELY_MAX;
    }
    else if (c == 'h' || c == 'H')  // Letter 'r' or 'R' key
    {
        joy_cmd_vely -= STEP_VELY;
        if (joy_cmd_vely < -JOY_CMD_VELY_MAX) joy_cmd_vely = -JOY_CMD_VELY_MAX;
    }
    //
    else if (c == 't' || c == 'T')  // Letter 't' or 'T' key
    {
        joy_cmd_body_height += STEP_BODY_HEIGHT;
        if (joy_cmd_body_height > JOY_CMD_BODY_HEIGHT_MAX) joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    else if (c == 'g' || c == 'G')  // Letter 'g' or 'G' key
    {
        joy_cmd_body_height -= STEP_BODY_HEIGHT;
        if (joy_cmd_body_height < JOY_CMD_BODY_HEIGHT_MIN) joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }
    //
    else if (c == 'p' || c == 'P' || c == 67)  // Letter 'P' or 'p' change control_state
    {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; 
        if (joy_cmd_ctrl_state==1)
            {
                joy_cmd_velx=0.05;
            }
        
        //TODO: how to toggle more states?
        //joy_cmd_ctrl_state_change_request = false; //erase this change request;
    }


    //


    else if (c == 'x' || c == 'X')  // Stop key
    {
        std::cout << "You have pressed the exit button!!!!" << std::endl;
        joy_cmd_exit = true;
    }
}

//Todo: modify it to receive keyboard input
// 
// 

//////////////////////////////////
void HardwareA1ROS::udp_init_send() {
    cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i < NUM_DOF; i++) {
        cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF;        // 禁止位置环
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF;        // 禁止速度环
        cmd.motorCmd[i].Kd = 0;
        cmd.motorCmd[i].tau = 0;
    }
    safe.PositionLimit(cmd);
    udp.SetSend(cmd);
    udp.Send();
}

void HardwareA1ROS::receive_low_state() {
    ros::Time prev = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration dt(0);
    while (destruct == false) {
//        std::cout << "OBSERVE THREAD: delta time is:"  << std::setprecision(10) << dt.toSec() << std::endl;
//         std::cout << udp.targetIP << std::endl;
        udp.Recv();
//         std::cout << "receive" << std::endl;
        udp.GetRecv(state);
//         std::cout << state.motorState[0].q << std::endl;
//         std::cout << state.imu.accelerometer[0] << std::endl;

        // fill data to a1_ctrl_states, notice the order in state is FR, FL, RR, RL
        // fill data to a1_ctrl_states, notice the order in a1_ctrl_states is FL, FR, RL, RR
        /* TODO: fill data */


        a1_ctrl_states.root_quat = Eigen::Quaterniond(state.imu.quaternion[0],
                                                      state.imu.quaternion[1],
                                                      state.imu.quaternion[2],
                                                      state.imu.quaternion[3]);
        a1_ctrl_states.root_rot_mat = a1_ctrl_states.root_quat.toRotationMatrix();
        a1_ctrl_states.root_euler = Utils::quat_to_euler(a1_ctrl_states.root_quat);
        double yaw_angle = a1_ctrl_states.root_euler[2];

        // std::cout<< "root euler angle = "<< a1_ctrl_states.root_euler <<std::endl;

        a1_ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
        // a1_ctrl_states.root_pos     | do not fill
        // a1_ctrl_states.root_lin_vel | do not fill

        a1_ctrl_states.imu_acc = Eigen::Vector3d(state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
        a1_ctrl_states.imu_ang_vel = Eigen::Vector3d(state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
        a1_ctrl_states.root_ang_vel = a1_ctrl_states.root_rot_mat * a1_ctrl_states.imu_ang_vel;

        // joint states
                // Get dt (in seconds)
        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        double dt_s = dt.toSec();

        for (int i = 0; i < NUM_DOF; ++i) {
            int swap_i = swap_joint_indices(i);
            a1_ctrl_states.joint_vel[i] = state.motorState[swap_i].dq;
            // a1_ctrl_states.joint_vel[i] = (state.motorState[swap_i].q - a1_ctrl_states.joint_pos[i])/dt_s;
            a1_ctrl_states.joint_pos[i] = state.motorState[swap_i].q;
        }

        // foot force, add a filter here
        for (int i = 0; i < NUM_LEG; ++i) {
            int swap_i = swap_foot_indices(i);
            double value = static_cast<double>(state.footForce[swap_i]);

            foot_force_filters_sum[i] -= foot_force_filters(i, foot_force_filters_idx[i]);
            foot_force_filters(i, foot_force_filters_idx[i]) = value;
            foot_force_filters_sum[i] += value;
            foot_force_filters_idx[i]++;
            foot_force_filters_idx[i] %= FOOT_FILTER_WINDOW_SIZE;

            a1_ctrl_states.foot_force[i] = foot_force_filters_sum[i] / static_cast<double>(FOOT_FILTER_WINDOW_SIZE);
        }

        // publish joint angle and foot force
        for (int i = 0; i < NUM_DOF; ++i) {
            joint_foot_msg.position[i] = a1_ctrl_states.joint_pos[i];
            joint_foot_msg.velocity[i] = a1_ctrl_states.joint_vel[i];
        }
        for (int i = 0; i < NUM_LEG; ++i) {
            // publish plan contacts to help state estimation
            joint_foot_msg.velocity[NUM_DOF + i] = a1_ctrl_states.plan_contacts[i];
            joint_foot_msg.effort[NUM_DOF + i] = a1_ctrl_states.foot_force[i];
        }
        joint_foot_msg.header.stamp = ros::Time::now();
        pub_joint_angle.publish(joint_foot_msg);

        imu_msg.header.stamp = ros::Time::now();
        imu_msg.angular_velocity.x = state.imu.gyroscope[0];
        imu_msg.angular_velocity.y = state.imu.gyroscope[1];
        imu_msg.angular_velocity.z = state.imu.gyroscope[2];

        imu_msg.linear_acceleration.x = state.imu.accelerometer[0];
        imu_msg.linear_acceleration.y = state.imu.accelerometer[1];
        imu_msg.linear_acceleration.z = state.imu.accelerometer[2]; 
        pub_imu.publish(imu_msg);


//        std::cout << "a1_ctrl_states.foot_force.transpose()" << std::endl;
//        std::cout << a1_ctrl_states.foot_force.transpose() << std::endl;

        // TODO: shall we call estimator update here, be careful the runtime should smaller than the HARDWARE_FEEDBACK_FREQUENCY

        // state estimation
        auto t1 = ros::Time::now();
        if (!a1_estimate.is_inited()) {
            a1_estimate.init_state(a1_ctrl_states);
        } else {
            a1_estimate.update_estimation(a1_ctrl_states, dt_s);
        }
        auto t2 = ros::Time::now();
        ros::Duration run_dt = t2 - t1;

        // FL, FR, RL, RR
        // use estimation pos and vel to get foot pos and foot vel in world frame
        //Modify this function for Go1
        for (int i = 0; i < NUM_LEG; ++i) {
            a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i) = a1_kin.fk(
                    a1_ctrl_states.joint_pos.segment<3>(3 * i),
                    rho_opt_list[i], rho_fix_list[i]);
            a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i) = a1_kin.jac(
                    a1_ctrl_states.joint_pos.segment<3>(3 * i),
                    rho_opt_list[i], rho_fix_list[i]);
            Eigen::Matrix3d tmp_mtx = a1_ctrl_states.j_foot.block<3, 3>(3 * i, 3 * i);
            Eigen::Vector3d tmp_vec = a1_ctrl_states.joint_vel.segment<3>(3 * i);
            a1_ctrl_states.foot_vel_rel.block<3, 1>(0, i) = tmp_mtx * tmp_vec;

            a1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) =
                    a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_pos_rel.block<3, 1>(0, i);
            a1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) =
                    a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_vel_rel.block<3, 1>(0, i);

            // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
            // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
            // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
            a1_ctrl_states.foot_pos_world.block<3, 1>(0, i) =
                    a1_ctrl_states.foot_pos_abs.block<3, 1>(0, i) + a1_ctrl_states.root_pos;
            a1_ctrl_states.foot_vel_world.block<3, 1>(0, i) =
                    a1_ctrl_states.foot_vel_abs.block<3, 1>(0, i) + a1_ctrl_states.root_lin_vel;
        }
        double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
        // sleep for interval_ms
        std::cout<<"run_time for a loop in ms = "<<1000*run_dt.toSec()<<std::endl;
        double interval_time = interval_ms / 1000.0;
        if (interval_time > run_dt.toSec()) {
            ros::Duration(interval_time - run_dt.toSec()).sleep();
        }
/////////////////////////////////////////////////////////////////////
       /*
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, 0)= go1_kin.FR_foot(a1_ctrl_states.joint_pos); // Relative to base coordinatesystem
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, 1)= go1_kin.FL_foot(a1_ctrl_states.joint_pos); //
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, 2)= go1_kin.RR_foot(a1_ctrl_states.joint_pos); //
        a1_ctrl_states.foot_pos_rel.block<3, 1>(0, 3)= go1_kin.RL_foot(a1_ctrl_states.joint_pos); //

        a1_ctrl_states.j_foot.block<3, 12>(0, 0) = go1_kin.NumJac(a1_ctrl_states.joint_pos, &Go1Kinematics::FR_foot);
        a1_ctrl_states.j_foot.block<3, 12>(3, 0) = go1_kin.NumJac(a1_ctrl_states.joint_pos, &Go1Kinematics::FL_foot);
        a1_ctrl_states.j_foot.block<3, 12>(6, 0) = go1_kin.NumJac(a1_ctrl_states.joint_pos, &Go1Kinematics::RR_foot);
        a1_ctrl_states.j_foot.block<3, 12>(9, 0) = go1_kin.NumJac(a1_ctrl_states.joint_pos, &Go1Kinematics::RL_foot);

        Eigen::Matrix<double, NUM_DOF, 1> TempMat;

        TempMat = a1_ctrl_states.j_foot * a1_ctrl_states.joint_vel; // foot velocity relative

        Eigen::Map<Eigen::MatrixXd> mat_map(TempMat.data(), 3, 4);

        a1_ctrl_states.foot_vel_rel = mat_map; 

        
        std::cout << "Size check; Row = " <<  a1_ctrl_states.j_foot.rows()<<" Col =" << a1_ctrl_states.j_foot.cols()<< " Vector =" << a1_ctrl_states.joint_vel.size() << std::endl;

        a1_ctrl_states.foot_pos_abs = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_pos_rel; //

        a1_ctrl_states.foot_vel_abs = a1_ctrl_states.root_rot_mat * a1_ctrl_states.foot_vel_rel;

        a1_ctrl_states.foot_pos_world = a1_ctrl_states.foot_pos_abs.colwise() + a1_ctrl_states.root_pos; // FootPosWorld

        a1_ctrl_states.foot_vel_world = a1_ctrl_states.foot_vel_abs.colwise() + a1_ctrl_states.root_lin_vel; //FootVelWorld
        std::cout << "foot_Pos Abs = " <<   a1_ctrl_states.foot_pos_abs   << std::endl;



        std::cout << "FR = " <<  go1_kin.FR_foot(a1_ctrl_states.joint_pos) << std::endl;
        std::cout << "FL = " <<  go1_kin.FL_foot(a1_ctrl_states.joint_pos) << std::endl;
        std::cout << "RR = " <<  go1_kin.RR_foot(a1_ctrl_states.joint_pos) << std::endl;
        std::cout << "RL = " <<  go1_kin.RL_foot(a1_ctrl_states.joint_pos) << std::endl;
        std::cout << "Joint pos  = " <<  a1_ctrl_states.joint_pos << std::endl;
        // std::cout << "FR_foot Jacobian  = " <<  go1_kin.NumJac(a1_ctrl_states.joint_pos, &Go1Kinematics::FR_foot) << std::endl;
        // std::cout << "FR_Jacobian  = " << go1_kin.NumJac(a1_ctrl_states.joint_pos, &Go1Kinematics::FR_foot);
        // std::cout << "FL = " <<  FL_foot(a1_ctrl_states.joint_pos) << std::endl;
        // std::cout << "RR = " <<  RR_foot(a1_ctrl_states.joint_pos) << std::endl;
        // std::cout << "RL = " <<  RL_foot(a1_ctrl_states.joint_pos) << std::endl;
        // std::cout << "RL = " <<  RL_foot(a1_ctrl_states.joint_pos) << std::endl;
        // std::cout << "Joint pos  = " <<  a1_ctrl_states.joint_pos << std::endl;

*/
///////////////////////////////////////////////////////////////////
        
        //// Datafile
        /*
        if (std::filesystem::is_empty(myfile)) {

            myfile << "EstimatedBasePosVel_BaseOrientAngularVel_JointPosVelTorque_FootForce_IMUAccelAngVelQuaternion" << "\n";

        }*/
        /*
        std::cout << "Check if file is empty_Boolean= " <<  (myfile.tellp() == 0) << std::endl;
        if ((myfile.tellp() == 0)){
            StoredDataVector.push_back(a1_ctrl_states.root_pos);
            StoredDataVector.push_back(a1_ctrl_states.root_lin_vel);
            StoredDataVector.push_back(a1_ctrl_states.root_euler);
            StoredDataVector.push_back(a1_ctrl_states.root_ang_vel);
            StoredDataVector.push_back(a1_ctrl_states.joint_pos);
            StoredDataVector.push_back(a1_ctrl_states.joint_vel);
            StoredDataVector.push_back(a1_ctrl_states.joint_torques);
            StoredDataVector.push_back(a1_ctrl_states.foot_force);
            StoredDataVector.push_back(a1_ctrl_states.imu_acc);
            StoredDataVector.push_back(a1_ctrl_states.imu_ang_vel);
            //StoredDataVector.push_back(a1_ctrl_states.root_quat); // To store quaternion need to cast it as Eigen::VectorXd
        
            for (const auto& v : StoredDataVector) {        
                for (int i = 0; i < v.size(); i++) {
                    myfile << v(i);
                    if (i != v.size() - 1) {
                        myfile << ",";
                    }
                }
            }
            myfile << "\n";
            StoredDataVector.clear();
            
        } *//// This loop run once to initialize data recording
        //////
      
        if (tp_myfile % 10 == 0){
            StoredDataVector.push_back(a1_ctrl_states.root_pos);
            // std::cout << "root_pos = " <<  a1_ctrl_states.root_pos << std::endl;
            StoredDataVector.push_back(a1_ctrl_states.root_lin_vel);
            // std::cout << "root_lin_vel = " <<  a1_ctrl_states.root_lin_vel << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.root_euler);
            // std::cout << "root_euler = " <<  a1_ctrl_states.root_euler << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.root_ang_vel);
            // std::cout << "root_ang_vel = " <<  a1_ctrl_states.root_ang_vel << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.joint_pos);
            // std::cout << "joint_pos = " <<  a1_ctrl_states.joint_pos << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.joint_vel);
            // std::cout << "joint_vel = " <<  a1_ctrl_states.joint_vel << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.joint_torques);
            // std::cout << "joint_torques = " <<  a1_ctrl_states.joint_torques << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.foot_force);
            // std::cout << "foot_force = " <<  a1_ctrl_states.foot_force << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.imu_acc);
            // std::cout << "imu_acc = " <<  a1_ctrl_states.imu_acc << std::endl;
 
            StoredDataVector.push_back(a1_ctrl_states.imu_ang_vel);
            // std::cout << "imu_ang_vel = " <<  a1_ctrl_states.imu_ang_vel << std::endl;

            
            //StoredDataVector.push_back(a1_ctrl_states.root_quat); // To store quaternion need to cast it as Eigen::VectorXd
        
            for (const auto& v : StoredDataVector) {   
                // std::cout << "StoredDataVector = " <<  v << std::endl;
                // std::cout << "StoredDataVector Size = " <<  v.size() << std::endl;
      
                for (int i = 0; i < v.size(); i++) {
                    myfile << v(i);
                    if (i != v.size() - 1) {
                        myfile << ",";
                    }
                }
                myfile << ",";
            }
            myfile << "\n";
            StoredDataVector.clear();
            //tp_myfile =0;
            
        } // this loop stores data at 100HZ
        tp_myfile +=1;
        
        // std::cout << "Check KeyBoard Control Vx =" <<  joy_cmd_velx << std::endl;
        // std::cout << "Check if tp behave as expected? = " <<  tp_myfile << std::endl;

        // std::cout << "joy_cmd_ctrl_state =" <<  joy_cmd_ctrl_state << "  and base height = " << joy_cmd_body_height << std::endl;


    };
}
