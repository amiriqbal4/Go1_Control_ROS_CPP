/** Created by Amir on 12/12/22 **/

#ifndef HLIP_PLANNER_H
#define HLIP_PLANNER_H


#include <iostream>
#include <string>
#include <chrono>

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

#include "Go1Parameters.h"
#include "Go1ControlStates.h"
#include "utils/Utils.h"


// Stepping Controller gains
#define GAIN_SIZE 2
#define MEAS_SIZE 28


// implement a basic error state KF to estimate robot pose
// assume orientation is known from a IMU (state.root_rot_mat)
class HLIP_Planner {
public:
    HLIP_Planner ();
    Go1FilterEKF (bool assume_flat_ground_);
    void init_state(Go1ControlStates& state);
    void update_estimation(Go1ControlStates& state, double dt);
    bool is_inited() {return filter_initialized;}
private:
    bool filter_initialized = false;
    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
    Eigen::Matrix<double, GAIN_SIZE, 1> hlip_state; // HLIP state
    Eigen::Matrix<double, GAIN_SIZE, GAIN_SIZE> hlipA; // HLIP system matrix

    Eigen::Matrix<double, GAIN_SIZE, 1> gain_k; // HLIP-based stepping controller gain
    Eigen::Matrix<double, GAIN_SIZE, GAIN_SIZE> hlip_hessian; //SteppingControllerHessian
    Eigen::Matrix<double, GAIN_SIZE, 1> hlip_gradient; //SteppingControllerQPgradient
    Eigen::Matrix<double, 2*GAIN_SIZE, GAIN_SIZE> hlip_A_linearConst; // SteppingController LinearConstraints hlip_A_linearConst*gain_k<= hlip_b_linearConst
    Eigen::Matrix<double, 2*GAIN_SIZE, 1> hlip_b_linearConst; // linearconstraint b

    Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise


    ////////////////////////////



    OsqpEigen::Solver hlip_gain_solver;

};


#endif //HLIP_PLANNER_H