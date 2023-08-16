/** Created by Amir on 02/08/23 **/

#ifndef GO1_KINEMATICS_H
#define Go1_KINEMATICS_H

#include <eigen3/Eigen/Dense>

// class Forward_Kin 
// { 
    // private:
    //     Eigen::VectorXd q;
    //     Eigen::Vector3d (*function)(Eigen::VectorXd q);

    // public: 

    //Hold the  forward kinematic and NumericalJacobian functions
class Go1Kinematics {

    public:
        // Go1Kinematics() = default;
        // ~Go1Kinematics() = default;

        // Eigen::MatrixXd T_world2base(Eigen::VectorXd q);

        Eigen::Vector3d FL_foot(Eigen::VectorXd q);

        Eigen::Vector3d FR_foot(Eigen::VectorXd q);

        Eigen::Vector3d RL_foot(Eigen::VectorXd q);

        Eigen::Vector3d RR_foot(Eigen::VectorXd q);

        // Eigen::Vector3d JJ_FR_foot(Eigen::VectorXd q, Eigen::VectorXd dq);

        // Eigen::Vector3d JJ_FL_foot(Eigen::VectorXd q, Eigen::VectorXd dq);

        // Eigen::Vector3d JJ_RR_foot(Eigen::VectorXd q, Eigen::VectorXd dq);

        // Eigen::Vector3d JJ_RL_foot(Eigen::VectorXd q, Eigen::VectorXd dq);

        // Eigen::MatrixXd NumJac(Eigen::VectorXd q, Eigen::Vector3d (*function)(Eigen::VectorXd q));
        Eigen::MatrixXd NumJac(Eigen::VectorXd q, Eigen::Vector3d (Go1Kinematics::*function)(Eigen::VectorXd q));

        // Eigen::Vector3d FL_hip(Eigen::VectorXd q);

        // Eigen::Vector3d FR_hip(Eigen::VectorXd q);

        // Eigen::Vector3d RL_hip(Eigen::VectorXd q);

        // Eigen::Vector3d RR_hip(Eigen::VectorXd q);
    private:
    //     Eigen::VectorXd q;
    //     Eigen::Vector3d (*function)(Eigen::VectorXd q);
 };

#endif //Go1_KINEMATICS_H