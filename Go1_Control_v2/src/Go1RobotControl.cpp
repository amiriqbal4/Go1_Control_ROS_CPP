//
// Created by shuoy on 10/19/21.
// Edited by Amir for Go1 Control
//

#include "Go1RobotControl.h"

Go1RobotControl::Go1RobotControl() {
    std::cout << "init GO1RobotControl" << std::endl;
    // init QP solver
    // init some parameters
    Q.diagonal() << 1.0, 1.0, 50.0, 400.0, 400.0, 100.0; //<< 1.0, 1.0, 1.0, 400.0, 400.0, 100.0; 
    R = 1e-3;
    mu = 0.6;
    F_min = 0;
    F_max = 180;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
    lowerBound.setZero();
    upperBound.resize(NUM_LEG + 4 * NUM_LEG);
    upperBound.setZero();

    // init mpc skip counter
    mpc_init_counter = 0;

    // constraint matrix fixed
    for (int i = 0; i < NUM_LEG; ++i) {
        // extract F_zi
        linearMatrix.insert(i, 2 + i * 3) = 1;
        // friction pyramid
        // 1. F_xi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY;
        // 2. F_xi > -uF_zi    ===> -F_xi -uF_zi < 0
        linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
        // 3. F_yi < uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
        linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
        // 4. -F_yi > uF_zi
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
        linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
        lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;
    }
    // debug linearMatrix
//    std::cout << Eigen::MatrixXd(linearMatrix) << std::endl;

    terrain_angle_filter = MovingWindowFilter(100);
    for (int i = 0; i < NUM_LEG; ++i) {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
// HLIP_GainComputation
    GainsHLIP.resize(4);
    GainsHLIP.setZero();
    hlip_ub.resize(6);
    hlip_ub.setZero();
    hlip_lb.resize(6);
    hlip_lb.setZero();
    // hlip_hessian.resize(4,4);
    hlip_gradient.resize(4);
    hlip_gradient.setZero();
    // hlip_linearConst.resize(2,4);
    hlip_dense_hessian.setZero();
    hlip_dense_linearConst.setZero();
    StanceFeetMP_prev.setZero();

}

Go1RobotControl::Go1RobotControl(ros::NodeHandle &_nh) : Go1RobotControl() {
    std::cout << "init nh" << std::endl;
    nh = _nh;
    _nh.param("use_sim_time", use_sim_time);
    // initial debug publisher
    for (int i = 0; i < NUM_LEG; ++i) {
        std::string id = std::to_string(i);
        std::string start_topic = "/isaac_go1/foot" + id + "/start_pos";
        std::string end_topic = "/isaac_go1/foot" + id + "/end_pos";
        std::string path_topic = "/isaac_go1/foot" + id + "/swing_path";

        pub_foot_start[i] = nh.advertise<visualization_msgs::Marker>(start_topic, 100);
        pub_foot_end[i] = nh.advertise<visualization_msgs::Marker>(end_topic, 100);
        pub_foot_path[i] = nh.advertise<visualization_msgs::Marker>(path_topic, 100);

        // set basic info of markers
        foot_start_marker[i].header.frame_id = "go1_world";
        foot_start_marker[i].ns = "basic_shapes";
        foot_start_marker[i].id = 10 + i;
        foot_start_marker[i].type = visualization_msgs::Marker::CYLINDER;
        foot_start_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_start_marker[i].scale.x = 0.08;
        foot_start_marker[i].scale.y = 0.08;
        foot_start_marker[i].scale.z = 0.02;
        foot_start_marker[i].pose.orientation.x = 0.0;
        foot_start_marker[i].pose.orientation.y = 0.0;
        foot_start_marker[i].pose.orientation.z = 0.0;
        foot_start_marker[i].pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        foot_start_marker[i].color.r = 1.0f;
        foot_start_marker[i].color.g = 0.0f;
        foot_start_marker[i].color.b = 0.0f;
        foot_start_marker[i].color.a = 1.0;

        foot_end_marker[i].lifetime = ros::Duration();

        foot_end_marker[i].header.frame_id = "go1_world";
        foot_end_marker[i].ns = "basic_shapes";
        foot_end_marker[i].id = 20 + i;
        foot_end_marker[i].type = visualization_msgs::Marker::CYLINDER;
        foot_end_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_end_marker[i].scale.x = 0.08;
        foot_end_marker[i].scale.y = 0.08;
        foot_end_marker[i].scale.z = 0.02;
        foot_end_marker[i].pose.orientation.x = 0.0;
        foot_end_marker[i].pose.orientation.y = 0.0;
        foot_end_marker[i].pose.orientation.z = 0.0;
        foot_end_marker[i].pose.orientation.w = 1.0;

        // Set the color -- be sure to set alpha to something non-zero!
        foot_end_marker[i].color.r = 0.0f;
        foot_end_marker[i].color.g = 0.0f;
        foot_end_marker[i].color.b = 1.0f;
        foot_end_marker[i].color.a = 1.0;

        foot_end_marker[i].lifetime = ros::Duration();

        foot_path_marker[i].header.frame_id = "go1_world";
        foot_path_marker[i].ns = "basic_shapes";
        foot_path_marker[i].id = 30 + i;
        foot_path_marker[i].type = visualization_msgs::Marker::LINE_STRIP;
        foot_path_marker[i].action = visualization_msgs::Marker::ADD;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        foot_path_marker[i].scale.x = 0.02;
        foot_path_marker[i].pose.position.x = 0.0;
        foot_path_marker[i].pose.position.y = 0.0;
        foot_path_marker[i].pose.position.z = 0.0;
        foot_path_marker[i].pose.orientation.w = 1.0;
        foot_path_marker[i].pose.orientation.x = 0.0;
        foot_path_marker[i].pose.orientation.y = 0.0;
        foot_path_marker[i].pose.orientation.z = 0.0;
        foot_path_marker[i].points.resize(10); // fix to be 10 points
        foot_path_marker[i].colors.resize(10); // fix to be 10 points
        for (int k = 0; k < 10; k++) {
            foot_path_marker[i].colors[k].r = 0.0f;
            foot_path_marker[i].colors[k].g = 1.0f;
            foot_path_marker[i].colors[k].b = 0.0f;
            foot_path_marker[i].colors[k].a = 1.0f;
        }

        foot_path_marker[i].lifetime = ros::Duration();
    }
    pub_terrain_angle = nh.advertise<std_msgs::Float64>("go1_debug/terrain_angle", 100);
}

void Go1RobotControl::update_plan(Go1ControlStates &state, double dt) {
    state.counter += 1;
    if (!state.movement_mode) {
        // movement_mode == 0, standstill with all feet in contact with ground
        for (bool &plan_contact: state.plan_contacts) plan_contact = true;
        state.gait_counter_reset();
    } else {
        // movement_mode == 1, walk
        for (int i = 0; i < NUM_LEG; ++i) {
            // state.gait_counter(i) = state.gait_counter(i) + state.gait_counter_speed(i);
            state.gait_counter(i) += state.gait_counter_speed(i);
            state.gait_counter(i) = std::fmod(state.gait_counter(i), state.counter_per_gait);
            if (state.gait_counter(i) <= state.counter_per_swing) {
                state.plan_contacts[i] = true;
            } else {
                state.plan_contacts[i] = false;
                //ToDo: Change state.gait_counter here for variable duration stepping
            }
        }
    }

    // update foot plan: state.foot_pos_target_world
    Eigen::Vector3d lin_vel_world = state.root_lin_vel; // world frame linear velocity
    Eigen::Vector3d lin_vel_rel = state.root_rot_mat_z.transpose() * lin_vel_world; // robot body frame linear velocity

    // Raibert Heuristic, calculate foothold position
    //ToDo add the H-LIP based foothold planner: delta_x and delta_y are the step lengths in x and y directions
    state.foot_pos_target_rel = state.default_foot_pos;
    //state.foot_pos_target_rel(2) = -state.root_pos_d (2);

 
    ////////////////////////////////////////////////
    // Eigen::Vector2d StanceFeetSumXY;
    // Eigen::Vector2d StanceFeetMP, StanceFeetMP_prev;
    // Eigen::Vector2d rel_error_pos;
    // Eigen::Vector2d rel_error_vel;
    // Eigen::Vector2d u_ref;
    double swing_progress_pram = 0.0;
    StanceFeetSumXY.setZero();
    int temp=0;
    for (int i = 0; i < NUM_LEG; ++i) {
        double is_stance = state.contacts[i] ? 1.0 : 0.0;
        StanceFeetSumXY += is_stance* state.foot_pos_rel.block<2, 1>(0, i);
        double is_swing = state.contacts[i] ? 0.0 : 1.0;
        if (is_swing){
        swing_progress_pram +=  float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing); //ranges between 0 to 2
        temp+=1;
        }
    }
    /// Added this to ensure update only during 2 feet contact
    x_footprint_default = state.default_foot_pos.row(0).segment(0, state.default_foot_pos.cols());
    y_footprint_default = state.default_foot_pos.row(1).segment(0, state.default_foot_pos.cols());
    x_footprint_offset = 0.25*x_footprint_default.sum();
    y_footprint_offset = 0.25*y_footprint_default.sum();
    std:: cout << "StanceFeetSumXY : " << StanceFeetSumXY <<  "state.foot_pos_rel : " << state.foot_pos_rel << "x_footprint_offset : " << x_footprint_offset << "y_footprint_offset : " << y_footprint_offset << std::endl;

    if ( temp == 2 ){
        StanceFeetMP = StanceFeetSumXY/2.0;
        StanceFeetMP_prev = StanceFeetMP;
        } else {
            StanceFeetMP = StanceFeetMP_prev;
            std:: cout << " ElseStanceFeetMP : " << StanceFeetMP << std::endl;
        }
    std:: cout << " StanceFeetMP : " << StanceFeetMP << std::endl;

    rel_error_pos = state.root_lin_vel_d.segment<2>(0)*state.control_dt*state.counter_per_swing*(1.0-swing_progress_pram)/2.0 -StanceFeetSumXY; //vd*Dtau/2

    Eigen::Vector3d rootVelRobotframe;
    rootVelRobotframe = state.root_rot_mat_z.transpose() * state.root_lin_vel; // currentVelocityRobotFrame Used for computing current HLIPState as well
    rel_error_vel = rootVelRobotframe.segment<2>(0) - state.root_lin_vel_d.segment<2>(0);
    u_ref = state.root_lin_vel_d.segment<2>(0)*state.control_dt*state.counter_per_swing;
   /////////////////
    double a_ver_plus_g;
    double f_M;
    filter_z_imu_acceleration = MovingWindowFilter(5);
    a_ver_plus_g = filter_z_imu_acceleration.CalculateAverage((state.imu_acc_abs(2)));
    std::cout<< "state.imu_acc_abs(2) : " << state.imu_acc_abs(2) << "a_ver_plus_g : " << a_ver_plus_g << " state.root_pos_d (2): " << state.root_pos_d (2)<<std::endl;

    if (state.imu_acc_abs(2)>0){
        f_M = state.imu_acc_abs(2) / state.root_pos_d (2);
    } 
    if(state.imu_acc_abs(2)>15.0){
        f_M = 15.0 / state.root_pos_d(2);
    }
    if(state.imu_acc_abs(2)< 5.0){
        f_M =25.0;
    }
        
        double a1= cosh(state.control_dt*state.counter_per_swing*sqrt(f_M));
        double a2= sinh(state.control_dt*state.counter_per_swing*sqrt(f_M))/sqrt(f_M);
        double a3= sinh(state.control_dt*state.counter_per_swing*sqrt(f_M))*sqrt(f_M);
        double a4= cosh(state.control_dt*state.counter_per_swing*sqrt(f_M));
        //
        double a1r= cosh((1.0-0.5*swing_progress_pram)*state.control_dt*state.counter_per_swing*sqrt(f_M)); // 0.5 because swing_progress_pram is averaged for the nominally 2 swinging legs
        double a2r= sinh((1.0-0.5*swing_progress_pram)*state.control_dt*state.counter_per_swing*sqrt(f_M))/sqrt(f_M);
        double a3r= sinh((1.0-0.5*swing_progress_pram)*state.control_dt*state.counter_per_swing*sqrt(f_M))*sqrt(f_M);
        double a4r= cosh((1.0-0.5*swing_progress_pram)*state.control_dt*state.counter_per_swing*sqrt(f_M));

        std::cout << "f_M : " << f_M << "state.control_dt : " << state.control_dt << "a1 : " << a1<< std:: endl;
    Phi_rem << a1r, a2r, 
                a3r, a4r; //STM for remainingTime in swing

    // hlip state

        X_hlip << -StanceFeetMP(0)-x_footprint_offset, rootVelRobotframe(0);
        Y_hlip << -StanceFeetMP(1)-y_footprint_offset, rootVelRobotframe(1);
        X_hlip_ref_minus << state.root_lin_vel_d(0)*state.control_dt*state.counter_per_swing/2.0, state.root_lin_vel_d(0); //
        Y_hlip_ref_minus << state.root_lin_vel_d(1)*state.control_dt*state.counter_per_swing/2.0, state.root_lin_vel_d(1); //

        X_hlip_minus = Phi_rem*X_hlip;
        Y_hlip_minus = Phi_rem*Y_hlip;
        X_hlip_error_minus = X_hlip_minus - X_hlip_ref_minus;
        Y_hlip_error_minus = Y_hlip_minus - Y_hlip_ref_minus;

    
    // e<< root
    for (int i=0; i<4; ++i){
        hlip_dense_hessian(i,i) = 2.0*(a1*a1+a3*a3);

        hlip_dense_linearConst(i+2,i) =1.0;
    }

    hlip_gradient << -2*(a1*a1+a3*a3), -2*(a1*a2+a3*a4),-2*(a1*a1+a3*a3), -2*(a1*a2+a3*a4);

    // dense_hlip_linearConst << rel_error_pos(0), rel_error_vel(0),
    //                 rel_error_pos(1), rel_error_vel(1);

    hlip_dense_linearConst(0,0) = X_hlip_error_minus(0);
    hlip_dense_linearConst(0,1) = X_hlip_error_minus(1);
    hlip_dense_linearConst(1,2) = Y_hlip_error_minus(0);
    hlip_dense_linearConst(1,3) = Y_hlip_error_minus(1);


    hlip_lb << -FOOT_DELTA_X_LIMIT - u_ref(0), -FOOT_DELTA_Y_LIMIT - u_ref(1), 0.5, 0.0, 0.5, 0.0;
    hlip_ub << FOOT_DELTA_X_LIMIT + u_ref(0), FOOT_DELTA_Y_LIMIT + u_ref(1), 1.5, 0.5, 1.5, 0.5;

    hlip_hessian = hlip_dense_hessian.sparseView();
    hlip_linearConst = hlip_dense_linearConst.sparseView();

           // instantiate the solver
        OsqpEigen::Solver solver_hlip;
        // settings
        solver_hlip.settings()->setVerbosity(false);
        solver_hlip.settings()->setWarmStart(false);
        solver_hlip.data()->setNumberOfVariables(4);
        solver_hlip.data()->setNumberOfConstraints(6);
        solver_hlip.data()->setLinearConstraintsMatrix(hlip_linearConst);
        solver_hlip.data()->setHessianMatrix(hlip_hessian);
        solver_hlip.data()->setGradient(hlip_gradient);
        solver_hlip.data()->setLowerBound(hlip_lb);
        solver_hlip.data()->setUpperBound(hlip_ub);
        auto t1 = std::chrono::high_resolution_clock::now();
        solver_hlip.initSolver();
        auto t2 = std::chrono::high_resolution_clock::now();
        solver_hlip.solve();
        auto t3 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

        // std::cout<< "Phi_rem : "<< Phi_rem << "X_hlip : " << X_hlip << "hlip_lb : " << hlip_lb << "hlip_ub : "<< hlip_ub << " hlip_dense_linearConst : " << hlip_dense_linearConst << std::endl;
        // std::cout << "HLIP qp solver initialization time: " << ms_double_1.count() << "ms; HLIP qp solve time: " << ms_double_2.count() << "ms" << std::endl;

        GainsHLIP = solver_hlip.getSolution(); //4x1
        for (int i = 0; i < 4; ++i) {
            if (!isnan(GainsHLIP[i]))
                state.HLIP_Gains[i] = GainsHLIP[i];
                state.HLIP_Gains << 1.0, 0.18, 1.0, 0.18;
            }

        std::cout << "HLIP Gains : " << state.HLIP_Gains << std:: endl;
        ///
        // StepLength = Vd_rf*DeltaTau +K.segment<2>(0)*(Phi_rem*X_hlip)- X_hlip_ref_minus)
        /*
        double stepLengthX = state.root_lin_vel_d(0)*state.control_dt*state.counter_per_swing + (state.HLIP_Gains.segment<2>(0).dot(X_hlip_error_minus));
        double stepLengthY = state.root_lin_vel_d(1)*state.control_dt*state.counter_per_swing + (state.HLIP_Gains.segment<2>(2).dot(Y_hlip_error_minus));

        // std::cout << "stepLengthX: " << stepLengthX << " stepLengthY : " << stepLengthY << "X_hlip_minus : "<< X_hlip_minus <<"X_hlip_ref_minus : " << X_hlip_ref_minus << std:: endl;

        // Uncomment to try HLIP-based Planning

        for (int i = 0; i < NUM_LEG; ++i) {

            if (stepLengthX < -FOOT_DELTA_X_LIMIT) {
                stepLengthX = -FOOT_DELTA_X_LIMIT;
            }
            if (stepLengthX > FOOT_DELTA_X_LIMIT) {
                stepLengthX = FOOT_DELTA_X_LIMIT;
            }
            if (stepLengthY < -FOOT_DELTA_Y_LIMIT) {
                stepLengthY = -FOOT_DELTA_Y_LIMIT;
            }
            if (stepLengthY > FOOT_DELTA_Y_LIMIT) {
                stepLengthY = FOOT_DELTA_Y_LIMIT;
            }

        state.foot_pos_target_rel(0, i) += stepLengthX;
        state.foot_pos_target_rel(1, i) += stepLengthY;
        std::cout << "stepLengthX : " << stepLengthX << " stepLengthY : " << stepLengthY << std::endl;

        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
        //  std::cout << "FeetNo = " << i << " Dx = " << delta_x <<" Dy = " << delta_y << "FeetTargetRel = "<<  state.foot_pos_target_rel.block<3, 1>(0, i) << "FeetTargetWorld_Abs = "<<  state.foot_pos_target_world.block<3, 1>(0, i)<< state.foot_pos_target_abs.block<3, 1>(0, i)<< std::endl;
    
    }
    */


 
    ////////////////////////////////////////////////


    ///////////////////////////////////////////
    // Uncomment to try RaibertHeuresticBasedPlanning
    
    for (int i = 0; i < NUM_LEG; ++i) {
        double delta_x =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(0) - state.root_lin_vel_d(0)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(0);
        double delta_y =
                std::sqrt(std::abs(state.default_foot_pos(2)) / 9.8) * (lin_vel_rel(1) - state.root_lin_vel_d(1)) +
                ((state.counter_per_swing / state.gait_counter_speed(i)) * state.control_dt) / 2.0 *
                state.root_lin_vel_d(1);
        // std::cout << "delta_x =" << delta_x << "lin_vel_rel(0) = " << lin_vel_rel(0) << " state.root_lin_vel_d(0) = " <<  state.root_lin_vel_d(0) << std::endl;
        if (delta_x < -FOOT_DELTA_X_LIMIT) {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT) {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT) {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT) {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }

        state.foot_pos_target_rel(0, i) += delta_x;
        state.foot_pos_target_rel(1, i) += delta_y;

        state.foot_pos_target_abs.block<3, 1>(0, i) = state.root_rot_mat * state.foot_pos_target_rel.block<3, 1>(0, i);
        state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.root_pos;
        //  std::cout << "FeetNo = " << i << " Dx = " << delta_x <<" Dy = " << delta_y << "FeetTargetRel = "<<  state.foot_pos_target_rel.block<3, 1>(0, i) << "FeetTargetWorld_Abs = "<<  state.foot_pos_target_world.block<3, 1>(0, i)<< state.foot_pos_target_abs.block<3, 1>(0, i)<< std::endl;
    // std::cout << " delta_x : " << delta_x << "stepLengthX : " << stepLengthX << " delta_y : " << delta_y << "stepLengthY : " << stepLengthY << std::endl;
    }
    
    std::cout <<  "state.foot_pos_rel : " << state.foot_pos_rel << std::endl;
    
}

void Go1RobotControl::generate_swing_legs_ctrl(Go1ControlStates &state, double dt) {
    state.joint_torques.setZero();

    // get current foot pos and target foot pose
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    spline_time.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    foot_pos_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    foot_vel_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

    // the foot force of swing foot and stance foot, both are in robot frame
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;

    for (int i = 0; i < NUM_LEG; ++i) {
        foot_pos_cur.block<3, 1>(0, i) = state.root_rot_mat_z.transpose() * state.foot_pos_abs.block<3, 1>(0, i);

        // foot_pos_cur.block<3, 1>(0, i) = state.foot_pos_rel.block<3, 1>(0, i); //check effect of this compared to the above line

        // from foot_pos_cur to foot_pos_final computes an intermediate point using BezierUtils
        if (state.gait_counter(i) <= state.counter_per_swing) {
            // stance foot
            spline_time(i) = 0.0;
            // in this case the foot should be stance
            // keep refreshing foot_pos_start in stance mode
            state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
        } else {
            // in this case the foot should be swing
            spline_time(i) = float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);
        }

        foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                              state.foot_pos_start.block<3, 1>(0, i),
                                                                              state.foot_pos_target_rel.block<3, 1>(0, i),
                                                                              0.0);

        foot_vel_cur.block<3, 1>(0, i) = (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

        foot_vel_target.block<3, 1>(0, i) = (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
        state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

        foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
        foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
        foot_forces_kin.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                            foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
    }
    state.foot_pos_cur = foot_pos_cur;

    //////////////////
    // std::cout << "FeetStartPoint = " <<  state.foot_pos_start << "\n" << "FeetTargetPoints = "<<  state.foot_pos_target_rel << std::endl;
    // std::cout << "FeetForceInput = " << foot_forces_kin << "\n" << "FeetPosAbs = "<<  state.foot_pos_abs<< "FeetPosRel = "<<  state.foot_pos_rel<< std::endl;

    /////////////////////

    // detect early contact
    bool last_contacts[NUM_LEG];

    for (int i = 0; i < NUM_LEG; ++i) {
        if (state.gait_counter(i) <= state.counter_per_swing * 1.5) {
            state.early_contacts[i] = false;
        }
        if (!state.plan_contacts[i] &&
            (state.gait_counter(i) > state.counter_per_swing * 1.5) &&
            (state.foot_force(i) > FOOT_FORCE_LOW)) {
            state.early_contacts[i] = true;
        }

        // actual contact
        last_contacts[i] = state.contacts[i];
        state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];

        // record recent contact position if the foot is in touch with the ground
        if (state.contacts[i]) {
//            state.foot_pos_recent_contact.block<3, 1>(0, i) = state.root_rot_mat.transpose() * (state.foot_pos_world.block<3, 1>(0, i));
//            state.foot_pos_recent_contact.block<3, 1>(0, i) = state.foot_pos_abs.block<3, 1>(0, i);
            state.foot_pos_recent_contact.block<3, 1>(0, i)
                    << recent_contact_x_filter[i].CalculateAverage(state.foot_pos_abs(0, i)),
                    recent_contact_y_filter[i].CalculateAverage(state.foot_pos_abs(1, i)),
                    recent_contact_z_filter[i].CalculateAverage(state.foot_pos_abs(2, i));
        }
    }

    std::cout << "foot_pos_recent_contact z: " << state.foot_pos_recent_contact.block<1, 4>(2, 0) << std::endl;

    state.foot_forces_kin = foot_forces_kin;
}

void Go1RobotControl::compute_joint_torques(Go1ControlStates &state) {
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    joint_torques.setZero();
    mpc_init_counter++;
    // for the first 10 ticks, just return zero torques.
    if (mpc_init_counter < 10) {
        state.joint_torques = joint_torques;
    } else {
        // for each leg, if it is a swing leg (contact[i] is false), use foot_force_kin to get joint_torque
        // for each leg, if it is a stance leg (contact[i] is true), use foot_forces_grf to get joint_torque
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Matrix3d jac = state.j_foot.block<3, 3>(3 * i, 3 * i);
            if (state.contacts[i]) {
                // stance leg
                joint_torques.segment<3>(i * 3) = jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i);
                // std::cout << " StanceTorque : " << jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i)<<std::endl;
            } else {
                // swing leg
                Eigen::Vector3d force_tgt = state.km_foot.cwiseProduct(state.foot_forces_kin.block<3, 1>(0, i));
                joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt); //state.j_foot.block<3, 3>(3*i, 3 * i).transpose()*force_tgt;//jac.lu().solve(force_tgt);   // jac * tau = F
            
                //  std::cout << "jac.lu().solve(): " << jac.lu().solve(force_tgt) << " v J^TF : " << state.j_foot.block<3, 3>(3*i, 3 * i).transpose()*force_tgt << "StanceTorqe :  " << jac.transpose() * -state.foot_forces_grf.block<3, 1>(0, i)<< std::endl;
            }
        }
        // gravity compensation
        
        joint_torques += state.torques_gravity;

        // prevent nan
        for (int i = 0; i < 12; ++i) {
            if (!isnan(joint_torques[i]))
                state.joint_torques[i] = joint_torques[i];
        }
    }
}

Eigen::Matrix<double, 3, NUM_LEG> Go1RobotControl::compute_grf(Go1ControlStates &state, double dt) {
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // first get parameters needed to construct the solver hessian and gradient
    // use euler angle to get desired angle
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // limit euler error to pi/2
    if (euler_error(2) > 3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) - 3.1415926 * 2 - state.root_euler(2);
    } else if (euler_error(2) < -3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) + 3.1415926 * 2 - state.root_euler(2);
    }

    // only do terrain adaptation in MPC
    if (state.stance_leg_control_type == 1) {
        Eigen::Vector3d surf_coef = compute_walking_surface(state);
        Eigen::Vector3d flat_ground_coef;
        flat_ground_coef << 0, 0, 1;
        double terrain_angle = 0;
        // only record terrain angle when the body is high
        if (state.root_pos[2] > 0.1) {
            terrain_angle = terrain_angle_filter.CalculateAverage(Utils::cal_dihedral_angle(flat_ground_coef, surf_coef));
        } else {
            terrain_angle = 0;
        }

        if (terrain_angle > 0.5) {
            terrain_angle = 0.5;
        }
        if (terrain_angle < -0.5) {
            terrain_angle = -0.5;
        }
        // FR, FL, RR,  RL
        double F_R_diff = state.foot_pos_recent_contact(2, 0) + state.foot_pos_recent_contact(2, 1) - state.foot_pos_recent_contact(2, 2) -
                        state.foot_pos_recent_contact(2, 3);

        if (state.use_terrain_adapt) {
            if (F_R_diff > 0.05) {
            state.root_euler_d[1] = -terrain_angle;
            } else {
            state.root_euler_d[1] = terrain_angle;
            }
        }


        std_msgs::Float64 terrain_angle_msg;
        terrain_angle_msg.data = terrain_angle * (180 / 3.1415926);
        pub_terrain_angle.publish(terrain_angle_msg); // publish in deg
        // std::cout << "desire pitch in deg: " << state.root_euler_d[1] * (180 / 3.1415926) << std::endl;
        // std::cout << "terrain angle: " << terrain_angle << std::endl;

        // save calculated terrain pitch angle
        // TODO: limit terrain pitch angle to -30 to 30? 
        state.terrain_pitch_angle = terrain_angle;
    }
    if (state.stance_leg_control_type == 0) { // 0: QP
        // desired acc in world frame
        root_acc.setZero();
        desired_inertial_force.setZero();
        root_acc.block<3, 1>(0, 0) = state.kp_linear.cwiseProduct(state.root_pos_d - state.root_pos);

        root_acc.block<3, 1>(0, 0) += state.root_rot_mat * state.kd_linear.cwiseProduct(
                state.root_lin_vel_d - state.root_rot_mat.transpose() * state.root_lin_vel);

        root_acc.block<3, 1>(3, 0) = state.kp_angular.cwiseProduct(euler_error);

        root_acc.block<3, 1>(3, 0) += state.kd_angular.cwiseProduct(
                state.root_ang_vel_d - state.root_rot_mat.transpose() * state.root_ang_vel);

        // add gravity
        root_acc(2) += 9.8; 

        desired_inertial_force.block<3, 1>(0, 0) = state.robot_mass*root_acc.block<3, 1>(0, 0);
        desired_inertial_force.block<3, 1>(3, 0) = state.go1_lumped_inertia*root_acc.block<3, 1>(3, 0);

        // root_acc(2) += state.robot_mass * 9.8; ///// Check if this is correct m(a+g);I(dw)

        // Create inverse inertia matrix
        Eigen::Matrix<double, 6, DIM_GRF> inertia_inv;
        for (int i = 0; i < NUM_LEG; ++i) {
            inertia_inv.block<3, 3>(0, i * 3).setIdentity();
            // TODO: confirm this should be root_rot_mat instead of root_rot_mat
            inertia_inv.block<3, 3>(3, i * 3) = state.root_rot_mat_z.transpose() * Utils::skew(state.foot_pos_abs.block<3, 1>(0, i));
        }
        Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian;
        dense_hessian.setIdentity();
        dense_hessian *= R;
        dense_hessian += inertia_inv.transpose() * Q * inertia_inv;
        hessian = dense_hessian.sparseView();
        // accidentally wrote this as -2* before. Huge problem

        // gradient.block<3 * NUM_LEG, 1>(0, 0) = -inertia_inv.transpose() * Q * root_acc;
        gradient.block<3 * NUM_LEG, 1>(0, 0) = -inertia_inv.transpose() * Q * desired_inertial_force;

        // adjust bounds according to contact flag
        for (int i = 0; i < NUM_LEG; ++i) {
            double c_flag = state.contacts[i] ? 1.0 : 0.0;
            lowerBound(i) = c_flag * F_min;
            upperBound(i) = c_flag * F_max;
        }

        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(false);
        solver.data()->setNumberOfVariables(3 * NUM_LEG);
        solver.data()->setNumberOfConstraints(NUM_LEG + 4 * NUM_LEG);
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        solver.data()->setHessianMatrix(hessian);
        solver.data()->setGradient(gradient);
        solver.data()->setLowerBound(lowerBound);
        solver.data()->setUpperBound(upperBound);

        auto t1 = std::chrono::high_resolution_clock::now();
        solver.initSolver();
        auto t2 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t3 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

        std::cout << "qp solver initialization time: " << ms_double_1.count() << "ms; solve time: " << ms_double_2.count() << "ms" << std::endl;

        Eigen::VectorXd QPSolution = solver.getSolution(); //12x1
        for (int i = 0; i < NUM_LEG; ++i) {
            // the QP solves for world frame force
            // here we convert the force into robot frame
            foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * QPSolution.segment<3>(i * 3);
        }

    } else if (state.stance_leg_control_type == 1) { // 1: MPC
        ConvexMpc mpc_solver = ConvexMpc(state.q_weights, state.r_weights);
        mpc_solver.reset();

        // initialize the mpc state at the first time step
        // state.mpc_states.resize(13);
        state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
                state.root_pos[0], state.root_pos[1], state.root_pos[2],
                state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
                state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
                -9.8;

        // previously we use dt passed by outer thread. It turns out that this dt is not stable on hardware.
        // if the thread is slowed down, dt becomes large, then MPC will output very large force and torque value
        // which will cause over current. Here we use a new mpc_dt, this should be roughly close to the average dt
        // of thread 1 
        double mpc_dt = 0.0025;

        // in simulation, use dt has no problem
        if (use_sim_time == "true") {
            mpc_dt = dt;
        }

        // initialize the desired mpc states trajectory
        state.root_lin_vel_d_world = state.root_rot_mat * state.root_lin_vel_d;
        // state.mpc_states_d.resize(13 * PLAN_HORIZON);
        for (int i = 0; i < PLAN_HORIZON; ++i) {
            state.mpc_states_d.segment(i * 13, 13)
                    <<
                    state.root_euler_d[0],
                    state.root_euler_d[1],
                    state.root_euler[2] + state.root_ang_vel_d[2] * mpc_dt * (i + 1),
                    state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
                    state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1),
                    state.root_pos_d[2],
                    state.root_ang_vel_d[0],
                    state.root_ang_vel_d[1],
                    state.root_ang_vel_d[2],
                    state.root_lin_vel_d_world[0],
                    state.root_lin_vel_d_world[1],
                    0,
                    -9.8;
        }

        // a single A_c is computed for the entire reference trajectory
        auto t1 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_A_mat_c(state.root_euler);

        // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
        // from the reference trajectory and foot placement controller
        // state.foot_pos_abs_mpc = state.foot_pos_abs;
        auto t2 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < PLAN_HORIZON; i++) {
            // calculate current B_c matrix
            mpc_solver.calculate_B_mat_c(state.robot_mass,
                                         state.go1_lumped_inertia,
                                         state.root_rot_mat,
                                         state.foot_pos_abs);
            // state.foot_pos_abs_mpc.block<3, 1>(0, 0) = state.foot_pos_abs_mpc.block<3, 1>(0, 0) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 1) = state.foot_pos_abs_mpc.block<3, 1>(0, 1) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 2) = state.foot_pos_abs_mpc.block<3, 1>(0, 2) - state.root_lin_vel_d * mpc_dt;
            // state.foot_pos_abs_mpc.block<3, 1>(0, 3) = state.foot_pos_abs_mpc.block<3, 1>(0, 3) - state.root_lin_vel_d * mpc_dt;

            // state space discretization, calculate A_d and current B_d
            mpc_solver.state_space_discretization(mpc_dt);

            // store current B_d matrix
            mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
        }

        // calculate QP matrices
        auto t3 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_qp_mats(state);

        // solve
        auto t4 = std::chrono::high_resolution_clock::now();
        if (!solver.isInitialized()) {
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
            solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver.hessian);
            solver.data()->setGradient(mpc_solver.gradient);
            solver.data()->setLowerBound(mpc_solver.lb);
            solver.data()->setUpperBound(mpc_solver.ub);
            solver.initSolver();
        } else {
            solver.updateHessianMatrix(mpc_solver.hessian);
            solver.updateGradient(mpc_solver.gradient);
            solver.updateLowerBound(mpc_solver.lb);
            solver.updateUpperBound(mpc_solver.ub);
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t6 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
        std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
        std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
        std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

//        std::cout << "mpc cal A_mat_c: " << ms_double_1.count() << "ms" << std::endl;
//        std::cout << "mpc cal B_mat_d_list: " << ms_double_2.count() << "ms" << std::endl;
//        std::cout << "mpc cal qp mats: " << ms_double_3.count() << "ms" << std::endl;
//        std::cout << "mpc init time: " << ms_double_4.count() << "ms" << std::endl;
//        std::cout << "mpc solve time: " << ms_double_5.count() << "ms" << std::endl << std::endl;

        Eigen::VectorXd solution = solver.getSolution();
        // std::cout << solution.transpose() << std::endl;

        for (int i = 0; i < NUM_LEG; ++i) {
            if (!isnan(solution.segment<3>(i * 3).norm()))
                foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * solution.segment<3>(i * 3);
        }
    }
    return foot_forces_grf;
}

Eigen::Vector3d Go1RobotControl::compute_walking_surface(Go1ControlStates &state) {
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;

    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = state.foot_pos_recent_contact.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}
