//
// Created by Amir 02/23
// Ref. ShuoYang Github
//

#include "Go1HardwareROS.h"


// DataWritingToFile
std::time_t fileOpen_time = std::time(nullptr);
std::string unique_id = std::to_string(fileOpen_time);

std::vector<Eigen::VectorXd> StoredDataVector;
std::ofstream myfile("TrialDataRecording_"+unique_id+".csv", ios::app);
//myfile << "EstimatedBasePosVel_BaseOrientAngularVel_JointPosVelTorque_FootForce_IMUAccelAngVelQuaternion" << "\n";
//

/*
// Get the current date and time
auto fileStartTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
std::stringstream ddmmyy;
ddmmyy << std::put_time(std::localtime(&fileStartTime), "%d_%m_%Y_%H_%M_%S");

std::vector<Eigen::VectorXd> StoredDataVector;
std::ofstream myfile("TrialDataRecording_" + ddmmyy.str() + ".csv", std::ios::app);
*/
// constructor
Go1HardwareROS::Go1HardwareROS(ros::NodeHandle &_nh)
        : safe(UNITREE_LEGGED_SDK::LeggedType::Go1), udp(UNITREE_LEGGED_SDK::LOWLEVEL, 8090, "192.168.123.10", 8007) {
//    UNITREE_LEGGED_SDK::InitEnvironment();
    nh = _nh;
    // ROS publisher
    pub_joint_cmd = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_torque_cmd", 100);

    // debug joint angle and foot force
    pub_joint_angle = nh.advertise<sensor_msgs::JointState>("/hardware_go1/joint_foot", 100);

    // imu data
    pub_imu = nh.advertise<sensor_msgs::Imu>("/hardware_go1/imu", 100);

    joint_foot_msg.name = { "FR0", "FR1", "FR2",
                            "FL0", "FL1", "FL2",
                            "RR0", "RR1", "RR2",
                            "RL0", "RL1", "RL2",
                            "FR_foot", "FL_foot", "RR_foot","RL_foot"};
    joint_foot_msg.position.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.velocity.resize(NUM_DOF + NUM_LEG);
    joint_foot_msg.effort.resize(NUM_DOF + NUM_LEG);

    pub_estimated_pose = nh.advertise<nav_msgs::Odometry>("/hardware_go1/estimation_body_pose", 100);



    //sub_joy_msg = nh.subscribe("/joy", 1000, &Go1HardwareROS::joy_callback, this);
    //
    
    sub_keyBoardControl = nh.subscribe("/keyBoardControl", 1000, &Go1HardwareROS::keyboardCallback, this); //Added a keyboard cntrol option

    // Create timer to read keyboard input
    //keyboardtimer = nh.createTimer(ros::Duration(0.01), keyboardCallback);



    udp.InitCmdData(cmd);
    udp_init_send();



    _main_control_go1 = Go1RobotControl(nh);
    control_states_go1.reset();
    control_states_go1.resetFromROSParam(nh);
/////////////////////////////////////////////


    // hardware foot force filter reset
    foot_force_filters.setZero();
    foot_force_filters_idx.setZero();
    foot_force_filters_sum.setZero();

    int tp_myfile= 0; //add to controt frequency of data recording


    // start hardware reading thread after everything initialized
    thread_ = std::thread(&Go1HardwareROS::receive_low_state, this);
}

bool Go1HardwareROS::update_foot_forces_grf(double dt) {
    control_states_go1.foot_forces_grf = _main_control_go1.compute_grf(control_states_go1, dt);
    return true;
}


bool Go1HardwareROS::main_update(double t, double dt) {
    if (joy_cmd_exit) {
        return false;
    }

    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into control_states_go1
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
    control_states_go1.root_lin_vel_d[0] = joy_cmd_velx;
    control_states_go1.root_lin_vel_d[1] = joy_cmd_vely;

    // root_ang_vel_d is in robot frame
    control_states_go1.root_ang_vel_d[0] = joy_cmd_roll_rate;
    control_states_go1.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    control_states_go1.root_ang_vel_d[2] = joy_cmd_yaw_rate;
    control_states_go1.root_euler_d[0] = joy_cmd_roll_rate;
    control_states_go1.root_euler_d[1] = joy_cmd_pitch_rate;
    control_states_go1.root_euler_d[2] += joy_cmd_yaw_rate * dt;
    control_states_go1.root_pos_d[2] = joy_cmd_body_height;

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        // in walking mode, in this mode the robot should execute gait
        control_states_go1.movement_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        control_states_go1.movement_mode = 0;
        control_states_go1.root_pos_d.segment<2>(0) = control_states_go1.root_pos.segment<2>(0);
        control_states_go1.kp_linear(0) = control_states_go1.kp_linear_lock_x;
        control_states_go1.kp_linear(1) = control_states_go1.kp_linear_lock_y;
    } else {
        control_states_go1.movement_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (control_states_go1.movement_mode == 1) {
        if (control_states_go1.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy
            control_states_go1.root_pos_d.segment<2>(0) = control_states_go1.root_pos.segment<2>(0);
            control_states_go1.kp_linear.segment<2>(0).setZero();
        } else {
            control_states_go1.kp_linear(0) = control_states_go1.kp_linear_lock_x;
            control_states_go1.kp_linear(1) = control_states_go1.kp_linear_lock_y;
        }
    }

    _main_control_go1.update_plan(control_states_go1, dt);
    _main_control_go1.generate_swing_legs_ctrl(control_states_go1, dt);

    nav_msgs::Odometry estimate_odom;
    estimate_odom.pose.pose.position.x = control_states_go1.estimated_root_pos(0);
    estimate_odom.pose.pose.position.y = control_states_go1.estimated_root_pos(1);
    estimate_odom.pose.pose.position.z = control_states_go1.estimated_root_pos(2);
    // make sure root_lin_vel is in world frame
    estimate_odom.twist.twist.linear.x = control_states_go1.estimated_root_vel(0);
    estimate_odom.twist.twist.linear.y = control_states_go1.estimated_root_vel(1);
    estimate_odom.twist.twist.linear.z = control_states_go1.estimated_root_vel(2);
    pub_estimated_pose.publish(estimate_odom);

    return true;
}

bool Go1HardwareROS::send_cmd() {
    _main_control_go1.compute_joint_torques(control_states_go1);

    // send control cmd to robot via unitree hardware interface
    // notice control_states_go1.joint_torques uses order FL, FR, RL, RR
    // notice cmd uses order FR, FL, RR, RL
    cmd.levelFlag = UNITREE_LEGGED_SDK::LOWLEVEL;
    for (int i = 0; i < NUM_DOF; i++) {
        cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        cmd.motorCmd[i].q = UNITREE_LEGGED_SDK::PosStopF; // shut down position control
        cmd.motorCmd[i].Kp = 0;
        cmd.motorCmd[i].dq = UNITREE_LEGGED_SDK::VelStopF; // shut down velocity control
        cmd.motorCmd[i].Kd = 0;
        // int swap_i = swap_joint_indices(i);
        cmd.motorCmd[i].tau = control_states_go1.joint_torques(i);
    }

    safe.PositionLimit(cmd);
    safe.PowerProtect(cmd, state, control_states_go1.power_level);
    udp.SetSend(cmd);
    udp.Send();

    return true;
}

void Go1HardwareROS::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg) {
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
int Go1HardwareROS::getch()
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

// std::cout << "Check KeyBoard Control Vx =" <<  joy_cmd_velx << std::endl;

// Callback function for keyboard input
void Go1HardwareROS::keyboardCallback(const std_msgs::Char::ConstPtr& msg) //const std_msgs::String::ConstPtr& msg
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
    else if (c == 'o' || c == 'O' )  // reset yaw rate to zero
    {
        joy_cmd_yaw_rate = 0.0;
        joy_cmd_vely =0.0;
    }
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
    //
    else if (c == 'u' || c == 'U')  // Letter 'u' or 'U' key
    {
        control_states_go1.counter_per_swing += STEP_COUNTER_PER_SWING;
        if (control_states_go1.counter_per_swing > COUNTER_PER_SWING_MAX) control_states_go1.counter_per_swing = COUNTER_PER_SWING_MAX;
        control_states_go1.counter_per_gait = control_states_go1.counter_per_swing*2;
    }
    else if (c == 'y' || c == 'Y')  // Letter 'g' or 'G' key
    {
        control_states_go1.counter_per_swing -= STEP_COUNTER_PER_SWING;
        if (control_states_go1.counter_per_swing < COUNTER_PER_SWING_MIN) control_states_go1.counter_per_swing = COUNTER_PER_SWING_MIN;
        control_states_go1.counter_per_gait = control_states_go1.counter_per_swing*2;
    }
    //
    else if (c == 'p' || c == 'P' )  // Letter 'P' or 'p' change control_state
    {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2; 
        if (joy_cmd_ctrl_state==1)
            {
                joy_cmd_velx=0.10;
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
void Go1HardwareROS::udp_init_send() {
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

void Go1HardwareROS::receive_low_state() {
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

        // fill data to control_states_go1, notice the order in state is FR, FL, RR, RL
        // fill data to control_states_go1, notice the order in control_states_go1 is FL, FR, RL, RR
        /* TODO: fill data */


        control_states_go1.root_quat = Eigen::Quaterniond(state.imu.quaternion[0],
                                                      state.imu.quaternion[1],
                                                      state.imu.quaternion[2],
                                                      state.imu.quaternion[3]);
        control_states_go1.root_rot_mat = control_states_go1.root_quat.toRotationMatrix();
        control_states_go1.root_euler = Utils::quat_to_euler(control_states_go1.root_quat);
        double yaw_angle = control_states_go1.root_euler[2];

        // std::cout<< "root euler angle = "<< control_states_go1.root_euler <<std::endl;

        control_states_go1.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());
        // control_states_go1.root_pos     | do not fill
        // control_states_go1.root_lin_vel | do not fill

        control_states_go1.imu_acc = Eigen::Vector3d(state.imu.accelerometer[0], state.imu.accelerometer[1], state.imu.accelerometer[2]);
        control_states_go1.imu_ang_vel = Eigen::Vector3d(state.imu.gyroscope[0], state.imu.gyroscope[1], state.imu.gyroscope[2]);
        control_states_go1.root_ang_vel = control_states_go1.root_rot_mat * control_states_go1.imu_ang_vel;

        control_states_go1.imu_acc_abs = control_states_go1.root_rot_mat * control_states_go1.imu_acc;

        // joint states
                // Get dt (in seconds)
        now = ros::Time::now();
        dt = now - prev;
        prev = now;
        double dt_s = dt.toSec();

        for (int i = 0; i < NUM_DOF; ++i) {
            // int swap_i = swap_joint_indices(i);
            control_states_go1.joint_vel[i] = state.motorState[i].dq;
            // control_states_go1.joint_vel[i] = (state.motorState[swap_i].q - control_states_go1.joint_pos[i])/dt_s;
            control_states_go1.joint_pos[i] = state.motorState[i].q;
        }

        // foot force, add a filter here
        for (int i = 0; i < NUM_LEG; ++i) {
            // int swap_i = swap_foot_indices(i);
            double value = static_cast<double>(state.footForce[i]);

            foot_force_filters_sum[i] -= foot_force_filters(i, foot_force_filters_idx[i]);
            foot_force_filters(i, foot_force_filters_idx[i]) = value;
            foot_force_filters_sum[i] += value;
            foot_force_filters_idx[i]++;
            foot_force_filters_idx[i] %= FOOT_FILTER_WINDOW_SIZE;

            control_states_go1.foot_force[i] = foot_force_filters_sum[i] / static_cast<double>(FOOT_FILTER_WINDOW_SIZE);
        }

        // publish joint angle and foot force
        for (int i = 0; i < NUM_DOF; ++i) {
            joint_foot_msg.position[i] = control_states_go1.joint_pos[i];
            joint_foot_msg.velocity[i] = control_states_go1.joint_vel[i];
        }
        for (int i = 0; i < NUM_LEG; ++i) {
            // publish plan contacts to help state estimation
            joint_foot_msg.velocity[NUM_DOF + i] = control_states_go1.plan_contacts[i];
            joint_foot_msg.effort[NUM_DOF + i] = control_states_go1.foot_force[i];
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


//        std::cout << "control_states_go1.foot_force.transpose()" << std::endl;
//        std::cout << control_states_go1.foot_force.transpose() << std::endl;

        // TODO: shall we call estimator update here, be careful the runtime should smaller than the HARDWARE_FEEDBACK_FREQUENCY

        // state estimation
        auto t1 = ros::Time::now();
        if (!estimates_go1.is_inited()) {
            estimates_go1.init_state(control_states_go1);
        } else {
            estimates_go1.update_estimation(control_states_go1, dt_s);
        }
        auto t2 = ros::Time::now();
        ros::Duration run_dt = t2 - t1;

        // FR, FL, RR, RL
        ////////////////////////////////////////////////////
    
        // Go1 foot position in Base coordinate system    
        control_states_go1.foot_pos_rel.block<3, 1>(0, 0)= fk_jac_go1.FR_foot(control_states_go1.joint_pos); // Relative to base coordinatesystem
        control_states_go1.foot_pos_rel.block<3, 1>(0, 1)= fk_jac_go1.FL_foot(control_states_go1.joint_pos); //
        control_states_go1.foot_pos_rel.block<3, 1>(0, 2)= fk_jac_go1.RR_foot(control_states_go1.joint_pos); //
        control_states_go1.foot_pos_rel.block<3, 1>(0, 3)= fk_jac_go1.RL_foot(control_states_go1.joint_pos); //


        // Go1 foot Jacobian in Base coordinate system
        control_states_go1.j_foot.block<3, 12>(0, 0) = fk_jac_go1.NumJac(control_states_go1.joint_pos, &Go1Kinematics::FR_foot);
        control_states_go1.j_foot.block<3, 12>(3, 0) = fk_jac_go1.NumJac(control_states_go1.joint_pos, &Go1Kinematics::FL_foot);
        control_states_go1.j_foot.block<3, 12>(6, 0) = fk_jac_go1.NumJac(control_states_go1.joint_pos, &Go1Kinematics::RR_foot);
        control_states_go1.j_foot.block<3, 12>(9, 0) = fk_jac_go1.NumJac(control_states_go1.joint_pos, &Go1Kinematics::RL_foot);

        Eigen::Matrix<double, NUM_DOF, 1> TempMat;
        TempMat = control_states_go1.j_foot * control_states_go1.joint_vel; // foot velocity relative
        Eigen::Map<Eigen::MatrixXd> mat_map(TempMat.data(), 3, 4);
        control_states_go1.foot_vel_rel = mat_map; 

        
        // std::cout << "Size check; Row = " <<  control_states_go1.j_foot.rows()<<" Col =" << control_states_go1.j_foot.cols()<< " Vector =" << control_states_go1.joint_vel.size() << std::endl;

        control_states_go1.foot_pos_abs = control_states_go1.root_rot_mat * control_states_go1.foot_pos_rel; //

        control_states_go1.foot_vel_abs = control_states_go1.root_rot_mat * control_states_go1.foot_vel_rel;

        control_states_go1.foot_pos_world = control_states_go1.foot_pos_abs.colwise() + control_states_go1.root_pos; // FootPosWorld

        control_states_go1.foot_vel_world = control_states_go1.foot_vel_abs.colwise() + control_states_go1.root_lin_vel; //FootVelWorld

        //////////////////////////////////////////
        double interval_ms = HARDWARE_FEEDBACK_FREQUENCY;
        // sleep for interval_ms
        std::cout<<"run_time for a loop in ms = "<<1000*run_dt.toSec()<<std::endl;
        double interval_time = interval_ms / 1000.0;
        if (interval_time > run_dt.toSec()) {
            ros::Duration(interval_time - run_dt.toSec()).sleep();
        }

        
        // pinocchio::urdf::buildModel(filename, model);
        // pinocchio::Data data(model);

        // // Print the model information
        // // pinocchio::urdf::printModel(model);
        // // model.printModel();
        // // Eigen::Vector3d gravity = model.gravity();
        // Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

        // // pinocchio::forwardKinematics(model, data, control_states_go1.joint_pos, control_states_go1.joint_vel);
        // std::cout << "size of Joint Pos = " << q.size() << std::endl;
        
        // // Print information about the joints
        // for (int i = 0; i < model.njoints; ++i) {
        //     const auto& joint = model.joints[i];
        //     std::cout << "Joint " << i << ": " << joint.shortname() << ", type = " << joint.type << std::endl;
        // }

        // // Print information about the links
        // for (int i = 0; i < model.nlinks; ++i) {
        //     const auto& link = model.links[i];
        //     std::cout << "Link " << i << ": " << link.name << ", parent = " << link.parent << std::endl;
        // }

/////////////////////////////////////////////////////////////////////


    /*
            std::cout << "foot_Pos Abs = " <<   control_states_go1.foot_pos_abs   << std::endl;



            std::cout << "FR = " <<  fk_jac_go1.FR_foot(control_states_go1.joint_pos) << std::endl;
            std::cout << "FL = " <<  fk_jac_go1.FL_foot(control_states_go1.joint_pos) << std::endl;
            std::cout << "RR = " <<  fk_jac_go1.RR_foot(control_states_go1.joint_pos) << std::endl;
            std::cout << "RL = " <<  fk_jac_go1.RL_foot(control_states_go1.joint_pos) << std::endl;
            std::cout << "Joint pos  = " <<  control_states_go1.joint_pos << std::endl;
            // std::cout << "FR_foot Jacobian  = " <<  fk_jac_go1.NumJac(control_states_go1.joint_pos, &Go1Kinematics::FR_foot) << std::endl;
            // std::cout << "FR_Jacobian  = " << fk_jac_go1.NumJac(control_states_go1.joint_pos, &Go1Kinematics::FR_foot);
            // std::cout << "FL = " <<  FL_foot(control_states_go1.joint_pos) << std::endl;
            // std::cout << "RR = " <<  RR_foot(control_states_go1.joint_pos) << std::endl;
            // std::cout << "RL = " <<  RL_foot(control_states_go1.joint_pos) << std::endl;
            // std::cout << "RL = " <<  RL_foot(control_states_go1.joint_pos) << std::endl;
            // std::cout << "Joint pos  = " <<  control_states_go1.joint_pos << std::endl;

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
            StoredDataVector.push_back(control_states_go1.root_pos);
            StoredDataVector.push_back(control_states_go1.root_lin_vel);
            StoredDataVector.push_back(control_states_go1.root_euler);
            StoredDataVector.push_back(control_states_go1.root_ang_vel);
            StoredDataVector.push_back(control_states_go1.joint_pos);
            StoredDataVector.push_back(control_states_go1.joint_vel);
            StoredDataVector.push_back(control_states_go1.joint_torques);
            StoredDataVector.push_back(control_states_go1.foot_force);
            StoredDataVector.push_back(control_states_go1.imu_acc);
            StoredDataVector.push_back(control_states_go1.imu_ang_vel);
            //StoredDataVector.push_back(control_states_go1.root_quat); // To store quaternion need to cast it as Eigen::VectorXd
        
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
            StoredDataVector.push_back(control_states_go1.root_pos);
            // std::cout << "root_pos = " <<  control_states_go1.root_pos << std::endl;
            StoredDataVector.push_back(control_states_go1.root_lin_vel);
            // std::cout << "root_lin_vel = " <<  control_states_go1.root_lin_vel << std::endl;
 
            StoredDataVector.push_back(control_states_go1.root_euler);
            // std::cout << "root_euler = " <<  control_states_go1.root_euler << std::endl;
 
            StoredDataVector.push_back(control_states_go1.root_ang_vel);
            // std::cout << "root_ang_vel = " <<  control_states_go1.root_ang_vel << std::endl;
 
            StoredDataVector.push_back(control_states_go1.joint_pos);
            // std::cout << "joint_pos = " <<  control_states_go1.joint_pos << std::endl;
 
            StoredDataVector.push_back(control_states_go1.joint_vel);
            // std::cout << "joint_vel = " <<  control_states_go1.joint_vel << std::endl;
 
            StoredDataVector.push_back(control_states_go1.joint_torques);
            // std::cout << "joint_torques = " <<  control_states_go1.joint_torques << std::endl;
 
            StoredDataVector.push_back(control_states_go1.foot_force);
            // std::cout << "foot_force = " <<  control_states_go1.foot_force << std::endl;
 
            StoredDataVector.push_back(control_states_go1.imu_acc);
            // std::cout << "imu_acc = " <<  control_states_go1.imu_acc << std::endl;
 
            StoredDataVector.push_back(control_states_go1.imu_ang_vel);
            // std::cout << "imu_ang_vel = " <<  control_states_go1.imu_ang_vel << std::endl;
            Eigen::VectorXd KeyContCommand(4);
            KeyContCommand <<joy_cmd_body_height,joy_cmd_velx,joy_cmd_vely,joy_cmd_yaw_rate;
            StoredDataVector.push_back(KeyContCommand);

            
            //StoredDataVector.push_back(control_states_go1.root_quat); // To store quaternion need to cast it as Eigen::VectorXd
        
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
        // std::cout << "Parameter z_0 = " << joy_cmd_body_height << " vx = " <<joy_cmd_velx <<" vy = "<< joy_cmd_vely << " wz = " << joy_cmd_yaw_rate << std::endl;


    };
}
