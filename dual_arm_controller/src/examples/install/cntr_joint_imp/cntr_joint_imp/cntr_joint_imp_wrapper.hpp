#pragma once

#include <vector>
#include <string>
#include <memory>
#include "Eigen/Core"

class cntr_joint_impModelClass;

namespace cntr_joint_imp {

struct P_cntr_joint_imp{
Eigen::Matrix<double,7,1> K_theta;
Eigen::Matrix<double,7,1> D_theta;
Eigen::Matrix<double,1,1> enable_ffwd_vel;
Eigen::Matrix<double,1,1> enable_ffwd_acc;
P_cntr_joint_imp(){
K_theta.setZero();
D_theta.setZero();
enable_ffwd_vel.setZero();
enable_ffwd_acc.setZero();
}
};
struct U_cntr_joint_imp{
Eigen::Matrix<double,7,1> theta;
Eigen::Matrix<double,7,1> dtheta;
Eigen::Matrix<double,7,1> theta_d;
Eigen::Matrix<double,7,1> dtheta_d;
Eigen::Matrix<double,7,1> ddtheta_d;
Eigen::Matrix<double,7,7> M;
Eigen::Matrix<double,7,1> tau_ff;
U_cntr_joint_imp(){
theta.setZero();
dtheta.setZero();
theta_d.setZero();
dtheta_d.setZero();
ddtheta_d.setZero();
M.setZero();
tau_ff.setZero();
}
};
struct Y_cntr_joint_imp{
Eigen::Matrix<double,7,1> tau_J_d;
Y_cntr_joint_imp(){
tau_J_d.setZero();
}
};
struct L_cntr_joint_imp{
Eigen::Matrix<double,7,1> tau_J_d_K;
Eigen::Matrix<double,7,1> tau_J_d_D;
L_cntr_joint_imp(){
tau_J_d_K.setZero();
tau_J_d_D.setZero();
}
};
class cntr_joint_imp{
public:
cntr_joint_imp();
~cntr_joint_imp();
void initialize(bool log = false,unsigned long long l_len = 0,const std::string& path_logs="");
void step();
void terminate();
P_cntr_joint_imp p;
U_cntr_joint_imp u;
Y_cntr_joint_imp y;
L_cntr_joint_imp l;

private:
void write_input();
void write_output();
void write_log();
std::unique_ptr<cntr_joint_impModelClass> m_model;
std::vector<U_cntr_joint_imp> m_log_u;
std::vector<Y_cntr_joint_imp> m_log_y;
std::vector<L_cntr_joint_imp> m_log_l;
std::string m_path_logs;
unsigned long long m_cnt_step;
bool m_flag_log;};

}