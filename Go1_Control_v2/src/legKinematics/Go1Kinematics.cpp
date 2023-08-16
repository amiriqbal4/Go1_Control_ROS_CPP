

/** Created by Amir on 02/08/23 **/

#include "Go1Kinematics.h"

/*
  Eigen::MatrixXd T_world2base(Eigen::VectorXd q){
        Eigen::Matrix<double,4,4> T_w2b_pos,T_w2b_orn,T_w2b_roll,T_w2b_pitch,T_w2b_yaw,T_w2b;


        T_w2b_pos<< 1,  0,  0,  q(0),
                    0,  1,  0,  q(1),
                    0,  0,  1,  q(2),
                    0,  0,  0,  1;
                    
        T_w2b_roll<< 1,         0,           0,  0,
                    0,  cos(q(3)),  -sin(q(3)),  0,
                    0,  sin(q(3)),   cos(q(3)),  0,
                    0,          0,           0,  1; 

        T_w2b_pitch<< cos(q(4)),  0,   sin(q(4)),  0,
                                0,  1,           0,  0,
                        -sin(q(4)),  0,   cos(q(4)),  0,
                                0,  0,           0,  1;    

        T_w2b_yaw<< cos(q(5)),  -sin(q(5)),   0,  0,
                    sin(q(5)),   cos(q(5)),   0,  0,
                            0,           0,   1,  0,
                            0,           0,   0,  1; 

        T_w2b =   T_w2b_pos* T_w2b_yaw*T_w2b_pitch*T_w2b_roll;  

        return T_w2b; // 4X4 Transformation matrix
    }
    */
    
//World to FR_foot   Go1Kinematics::
    Eigen::Vector3d Go1Kinematics::FR_foot(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_b2FR_hr,T_FR_hr2hp,T_FR_hp2kp,T_FR_kp2f,T_b2FR_foot;


        // T_w2b = T_world2base(q);

        T_b2FR_hr <<1,         0,          0,  0.1881,
                    0, cos(q(0)), -sin(q(0)),  -0.04675,
                    0, sin(q(0)),  cos(q(0)),        0,
                    0,         0,          0,        1;

        T_FR_hr2hp << cos(q(1)),    0,  sin(q(1)),      0,
                              0,    1,          0, -0.08,
                     -sin(q(1)),    0,  cos(q(1)),      0,
                              0,    0,          0,      1;

        T_FR_hp2kp << cos(q(2)),    0,  sin(q(2)),      0,
                              0,    1,          0,      0,
                     -sin(q(2)),    0,  cos(q(2)),  -0.213,
                              0,    0,          0,      1;                              

        T_FR_kp2f << 1,     0,      0,      0,
                     0,     1,      0,      0,
                     0,     0,      1,  -0.213,
                     0,     0,      0,      1; 

        // T_w2FR_foot = T_w2b*T_b2FR_hr*T_FR_hr2hp*T_FR_hp2kp*T_FR_kp2f;

        T_b2FR_foot = T_b2FR_hr*T_FR_hr2hp*T_FR_hp2kp*T_FR_kp2f;
        Eigen::Vector3d FR_foot = T_b2FR_foot.block<3,1>(0,3);
        return FR_foot;
    }

//World to FL_foot 
    Eigen::Vector3d Go1Kinematics::FL_foot(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_b2FL_hr,T_FL_hr2hp,T_FL_hp2kp,T_FL_kp2f,T_b2FL_foot;


        // T_w2b = T_world2base(q);

        T_b2FL_hr <<1,         0,          0,  0.1881,
                    0, cos(q(3)), -sin(q(3)),   0.04675,
                    0, sin(q(3)),  cos(q(3)),        0,
                    0,         0,          0,        1;

        T_FL_hr2hp << cos(q(4)),    0,  sin(q(4)),      0,
                               0,    1,           0,  0.08,
                     -sin(q(4)),    0,  cos(q(4)),      0,
                               0,    0,           0,      1;

        T_FL_hp2kp << cos(q(5)),    0,  sin(q(5)),      0,
                               0,    1,           0,      0,
                     -sin(q(5)),    0,  cos(q(5)),  -0.213,
                               0,    0,           0,      1;                              

        T_FL_kp2f << 1,     0,      0,      0,
                     0,     1,      0,      0,
                     0,     0,      1,  -0.213,
                     0,     0,      0,      1; 

        // T_w2FL_foot = T_w2b*T_b2FL_hr*T_FL_hr2hp*T_FL_hp2kp*T_FL_kp2f;

        T_b2FL_foot = T_b2FL_hr*T_FL_hr2hp*T_FL_hp2kp*T_FL_kp2f;

        Eigen::Vector3d FL_foot = T_b2FL_foot.block<3,1>(0,3);
        return FL_foot;
    }

//World to RR_foot 
    Eigen::Vector3d Go1Kinematics::RR_foot(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_b2RR_hr,T_RR_hr2hp,T_RR_hp2kp,T_RR_kp2f,T_b2RR_foot;


        // T_w2b = T_world2base(q);

        T_b2RR_hr <<1,          0,           0,  -0.1881,
                    0, cos(q(6)), -sin(q(6)),  -0.04675,
                    0, sin(q(6)),  cos(q(6)),        0,
                    0,          0,           0,        1;

        T_RR_hr2hp << cos(q(7)),    0,  sin(q(7)),      0,
                               0,    1,           0, -0.08,
                     -sin(q(7)),    0,  cos(q(7)),      0,
                               0,    0,           0,      1;

        T_RR_hp2kp << cos(q(8)),    0,  sin(q(8)),      0,
                               0,    1,           0,      0,
                     -sin(q(8)),    0,  cos(q(8)),  -0.213,
                               0,    0,           0,      1;                              

        T_RR_kp2f << 1,     0,      0,      0,
                     0,     1,      0,      0,
                     0,     0,      1,  -0.213,
                     0,     0,      0,      1; 

        // T_w2RR_foot = T_w2b*T_b2RR_hr*T_RR_hr2hp*T_RR_hp2kp*T_RR_kp2f;

        T_b2RR_foot = T_b2RR_hr*T_RR_hr2hp*T_RR_hp2kp*T_RR_kp2f;


        Eigen::Vector3d RR_foot = T_b2RR_foot.block<3,1>(0,3);
        return RR_foot;
    }

//World to RL_foot 
    Eigen::Vector3d Go1Kinematics::RL_foot(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_b2RL_hr,T_RL_hr2hp,T_RL_hp2kp,T_RL_kp2f,T_b2RL_foot;


        // T_w2b = T_world2base(q);

        T_b2RL_hr <<1,          0,           0, -0.1881,
                    0, cos(q(9)), -sin(q(9)),   0.04675,
                    0, sin(q(9)),  cos(q(9)),        0,
                    0,          0,           0,        1;

        T_RL_hr2hp << cos(q(10)),    0,  sin(q(10)),      0,
                               0,    1,           0,  0.08,
                     -sin(q(10)),    0,  cos(q(10)),      0,
                               0,    0,           0,      1;

        T_RL_hp2kp << cos(q(11)),    0,  sin(q(11)),      0,
                               0,    1,           0,      0,
                     -sin(q(11)),    0,  cos(q(11)),  -0.213,
                               0,    0,           0,      1;                              

        T_RL_kp2f << 1,     0,      0,      0,
                     0,     1,      0,      0,
                     0,     0,      1,  -0.213,
                     0,     0,      0,      1; 

        // T_w2RL_foot = T_w2b*T_b2RL_hr*T_RL_hr2hp*T_RL_hp2kp*T_RL_kp2f;

        T_b2RL_foot = T_b2RL_hr*T_RL_hr2hp*T_RL_hp2kp*T_RL_kp2f;


        Eigen::Vector3d RL_foot = T_b2RL_foot.block<3,1>(0,3);
        return RL_foot;
    }

// Hip_position_for reference //We do not need locations of these points.

/*
//World to FR_hip_ref 
    Eigen::Vector3d FR_hip(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_w2b,T_b2FR_hr,T_FR_hip_C,T_FR_hip_ref;


        T_w2b = T_world2base(q);

        T_b2FR_hr <<1,         0,          0,  0.21935,
                    0, cos(q(6)), -sin(q(6)),  -0.0875,
                    0, sin(q(6)),  cos(q(6)),        0,
                    0,         0,          0,        1;
                             
        T_FR_hip_C << 1,     0,      0,      0,
                        0,     1,      0, -0.059,
                        0,     0,      1,      0,
                        0,     0,      0,      1; 

        T_FR_hip_ref = T_w2b*T_b2FR_hr*T_FR_hip_C;

        Eigen::Vector3d FR_hip_p = T_FR_hip_ref.block<3,1>(0,3);
        return FR_hip_p;
    }

//World to FL_hip_ref
    Eigen::Vector3d FL_hip(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_w2b,T_b2FL_hr,T_FL_hip_C,T_FL_hip_ref;


        T_w2b = T_world2base(q);

        T_b2FL_hr <<1,         0,          0,  0.21935,
                    0, cos(q(9)), -sin(q(9)),   0.0875,
                    0, sin(q(9)),  cos(q(9)),        0,
                    0,         0,          0,        1;                           

        T_FL_hip_C << 1,     0,      0,      0,
                     0,     1,      0,  0.059,
                     0,     0,      1,      0,
                     0,     0,      0,      1; 

        T_FL_hip_ref = T_w2b*T_b2FL_hr*T_FL_hip_C;

        Eigen::Vector3d FL_hip_p = T_FL_hip_ref.block<3,1>(0,3);
        return FL_hip_p;
    }

//World to RR_hip_ref
    Eigen::Vector3d RR_hip(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_w2b,T_b2RR_hr,T_RR_hip_C,T_RR_hip_ref;


        T_w2b = T_world2base(q);

        T_b2RR_hr <<1,          0,           0,  -0.21935,
                    0, cos(q(12)), -sin(q(12)),   -0.0875,
                    0, sin(q(12)),  cos(q(12)),         0,
                    0,          0,           0,         1;                            

        T_RR_hip_C <<   1,     0,      0,       0,
                        0,     1,      0,  -0.059,
                        0,     0,      1,       0,
                        0,     0,      0,       1; 

        T_RR_hip_ref = T_w2b*T_b2RR_hr*T_RR_hip_C;

        Eigen::Vector3d RR_hip_p = T_RR_hip_ref.block<3,1>(0,3);
        return RR_hip_p;
    }

//World to RL_hip_ref
    Eigen::Vector3d RL_hip(Eigen::VectorXd q){

        Eigen::Matrix<double, 4, 4> T_w2b,T_b2RL_hr,T_RL_hip_C,T_RL_hip_ref;


        T_w2b = T_world2base(q);

        T_b2RL_hr <<1,          0,           0, -0.21935,
                    0, cos(q(15)), -sin(q(15)),   0.0875,
                    0, sin(q(15)),  cos(q(15)),        0,
                    0,          0,           0,        1;
                        

        T_RL_hip_C<< 1,     0,      0,      0,
                     0,     1,      0,  0.059,
                     0,     0,      1,      0,
                     0,     0,      0,      1; 

        T_RL_hip_ref = T_w2b*T_b2RL_hr*T_RL_hip_C;

        Eigen::Vector3d RL_hip_p = T_RL_hip_ref.block<3,1>(0,3);
        return RL_hip_p;
    }
//
*/
    Eigen::MatrixXd Go1Kinematics::NumJac(Eigen::VectorXd q,Eigen::Vector3d (Go1Kinematics::*function)(Eigen::VectorXd q)){
        double eps =1e-6;
        Eigen::Vector3d f0, f_qplus_delq;
        f0 = (this->*function)(q); // call the member function using the pointer-to-member-function
        // f0 = function(q);
        Eigen::VectorXd qplus_delq;
        Eigen::MatrixXd NumJac(f0.size(),q.size());// (3,18//)f0.size(),q.size() 
        for(int j=0; j<q.size();j++){
            Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(q.size());
            delta_q[j]=eps;
            qplus_delq = q +delta_q;
            f_qplus_delq = (this->*function)(qplus_delq);
            NumJac.col(j) = (f_qplus_delq-f0)/eps;
        }
        return NumJac;

    }

    

    // Eigen::Vector3d Go1Kinematics::JJ_foot(Eigen::VectorXd q, Eigen::VectorXd dq,Eigen::Vector3d (Go1Kinematics::*function)(Eigen::VectorXd q)){
    //     Eigen::Vector3d f0, f_qplus_delq;
    //     double eps =1e-6;
    //     f0 = (this->*function)(q);
    //     Eigen::VectorXd qplus_delq;
    //     Eigen::MatrixXd NumJac(f0.size(),q.size());// (3,18//)f0.size(),q.size() 
    //     for(int j=0; j<q.size();j++){
    //         Eigen::VectorXd delta_q = Eigen::VectorXd::Zero(q.size());
    //         delta_q[j]=eps;
    //         qplus_delq = q +delta_q;
    //         f_qplus_delq = (this->*function)(qplus_delq);
    //         NumJac.col(j) = (f_qplus_delq-f0)/eps;
    //     }
    // }