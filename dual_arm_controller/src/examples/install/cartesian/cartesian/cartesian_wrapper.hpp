#pragma once

#include <vector>
#include <string>
#include <memory>
#include "Eigen/Core"

class cartesianModelClass;

namespace cartesian {

struct P_cartesian{
Eigen::Matrix<double,6,1> stiffness;
Eigen::Matrix<double,6,1> damping;
P_cartesian(){
stiffness.setZero();
damping.setZero();
}
};
struct U_cartesian{
Eigen::Matrix<double,4,4> TF_T_EE_d;
Eigen::Matrix<double,4,4> TF_T_EE;
Eigen::Matrix<double,6,7> B_J_EE;
Eigen::Matrix<double,7,1> dtheta;
Eigen::Matrix<double,7,1> coriolis;
U_cartesian(){
TF_T_EE_d.setZero();
TF_T_EE.setZero();
B_J_EE.setZero();
dtheta.setZero();
coriolis.setZero();
}
};
struct Y_cartesian{
Eigen::Matrix<double,7,1> tau_J_d;
Eigen::Matrix<double,6,1> K_x_2;
Eigen::Matrix<double,6,1> D_x_2;
Y_cartesian(){
tau_J_d.setZero();
K_x_2.setZero();
D_x_2.setZero();
}
};
struct L_cartesian{
L_cartesian(){
}
};
class cartesian{
public:
cartesian();
~cartesian();
void initialize(bool log = false,unsigned long long l_len = 0,const std::string& path_logs="");
void step();
void terminate();
P_cartesian p;
U_cartesian u;
Y_cartesian y;
L_cartesian l;

private:
void write_input();
void write_output();
void write_log();
std::unique_ptr<cartesianModelClass> m_model;
std::vector<U_cartesian> m_log_u;
std::vector<Y_cartesian> m_log_y;
std::vector<L_cartesian> m_log_l;
std::string m_path_logs;
unsigned long long m_cnt_step;
bool m_flag_log;};

}