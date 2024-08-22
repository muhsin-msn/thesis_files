#pragma once

#include <vector>
#include <string>
#include <memory>
#include "Eigen/Core"

class cartesian_dualModelClass;

namespace cartesian_dual {

struct P_cartesian_dual{
P_cartesian_dual(){
}
};
struct U_cartesian_dual{
Eigen::Matrix<double,4,4> TF_T_EE_d1;
Eigen::Matrix<double,4,4> TF_T_EE_d2;
Eigen::Matrix<double,4,4> TF_T_EE1;
Eigen::Matrix<double,4,4> TF_T_EE2;
Eigen::Matrix<double,7,1> coriolis1;
Eigen::Matrix<double,7,1> coriolis2;
Eigen::Matrix<double,6,7> B_J_EE1;
Eigen::Matrix<double,6,7> B_J_EE2;
Eigen::Matrix<double,7,1> dtheta1;
Eigen::Matrix<double,7,1> dtheta2;
U_cartesian_dual(){
TF_T_EE_d1.setZero();
TF_T_EE_d2.setZero();
TF_T_EE1.setZero();
TF_T_EE2.setZero();
coriolis1.setZero();
coriolis2.setZero();
B_J_EE1.setZero();
B_J_EE2.setZero();
dtheta1.setZero();
dtheta2.setZero();
}
};
struct Y_cartesian_dual{
Eigen::Matrix<double,14,1> tau_J_d;
Y_cartesian_dual(){
tau_J_d.setZero();
}
};
struct L_cartesian_dual{
L_cartesian_dual(){
}
};
class cartesian_dual{
public:
cartesian_dual();
~cartesian_dual();
void initialize(bool log = false,unsigned long long l_len = 0,const std::string& path_logs="");
void step();
void terminate();
P_cartesian_dual p;
U_cartesian_dual u;
Y_cartesian_dual y;
L_cartesian_dual l;

private:
void write_input();
void write_output();
void write_log();
std::unique_ptr<cartesian_dualModelClass> m_model;
std::vector<U_cartesian_dual> m_log_u;
std::vector<Y_cartesian_dual> m_log_y;
std::vector<L_cartesian_dual> m_log_l;
std::string m_path_logs;
unsigned long long m_cnt_step;
bool m_flag_log;};

}