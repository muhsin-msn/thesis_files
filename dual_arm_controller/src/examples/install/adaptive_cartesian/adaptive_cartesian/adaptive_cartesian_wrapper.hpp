#pragma once

#include <vector>
#include <string>
#include <memory>
#include "Eigen/Core"

class adaptive_cartesianModelClass;

namespace adaptive_cartesian {

struct P_adaptive_cartesian{
Eigen::Matrix<double,6,1> alpha;
Eigen::Matrix<double,6,1> beta;
Eigen::Matrix<double,6,1> beta1;
Eigen::Matrix<double,6,1> gamma_a;
Eigen::Matrix<double,6,1> gamma_c;
Eigen::Matrix<double,6,1> gamma_b;
Eigen::Matrix<double,6,1> K_0;
Eigen::Matrix<double,6,1> D_0;
Eigen::Matrix<double,6,1> F_ff_0;
Eigen::Matrix<double,6,1> L;
Eigen::Matrix<double,6,1> L1;
Eigen::Matrix<double,6,1> xi;
Eigen::Matrix<double,1,1> kappa;
Eigen::Matrix<double,1,1> TF_control;
Eigen::Matrix<double,6,1> K_max;
Eigen::Matrix<double,6,1> dK_max;
Eigen::Matrix<double,6,1> D_max;
Eigen::Matrix<double,6,1> dD_max;
Eigen::Matrix<double,6,1> F_ff_max;
Eigen::Matrix<double,6,1> dF_ff_max;
Eigen::Matrix<double,7,1> tau_max;
Eigen::Matrix<double,7,1> dtau_max;
P_adaptive_cartesian(){
alpha.setZero();
beta.setZero();
beta1.setZero();
gamma_a.setZero();
gamma_c.setZero();
gamma_b.setZero();
K_0.setZero();
D_0.setZero();
F_ff_0.setZero();
L.setZero();
L1.setZero();
xi.setZero();
kappa.setZero();
TF_control.setZero();
K_max.setZero();
dK_max.setZero();
D_max.setZero();
dD_max.setZero();
F_ff_max.setZero();
dF_ff_max.setZero();
tau_max.setZero();
dtau_max.setZero();
}
};
struct U_adaptive_cartesian{
Eigen::Matrix<double,4,4> TF_T_EE_d;
Eigen::Matrix<double,4,4> TF_T_EE;
Eigen::Matrix<double,6,1> TF_F_ff;
Eigen::Matrix<double,6,1> TF_F_ext;
Eigen::Matrix<double,7,1> coriolis;
Eigen::Matrix<double,6,7> B_J_EE;
Eigen::Matrix<double,7,1> dtheta;
Eigen::Matrix<double,6,1> K_x;
Eigen::Matrix<double,6,1> xi_x;
Eigen::Matrix<double,7,7> mass;
Eigen::Matrix<double,7,1> d_dtheta;
U_adaptive_cartesian(){
TF_T_EE_d.setZero();
TF_T_EE.setZero();
TF_F_ff.setZero();
TF_F_ext.setZero();
coriolis.setZero();
B_J_EE.setZero();
dtheta.setZero();
K_x.setZero();
xi_x.setZero();
mass.setZero();
d_dtheta.setZero();
}
};
struct Y_adaptive_cartesian{
Eigen::Matrix<double,7,1> tau_J_d;
Eigen::Matrix<double,6,1> F_add;
Eigen::Matrix<double,6,1> K_x_2;
Eigen::Matrix<double,6,1> D_x_2;
Y_adaptive_cartesian(){
tau_J_d.setZero();
F_add.setZero();
K_x_2.setZero();
D_x_2.setZero();
}
};
struct L_adaptive_cartesian{
L_adaptive_cartesian(){
}
};
class adaptive_cartesian{
public:
adaptive_cartesian();
~adaptive_cartesian();
void initialize(bool log = false,unsigned long long l_len = 0,const std::string& path_logs="");
void step();
void terminate();
P_adaptive_cartesian p;
U_adaptive_cartesian u;
Y_adaptive_cartesian y;
L_adaptive_cartesian l;

private:
void write_input();
void write_output();
void write_log();
std::unique_ptr<adaptive_cartesianModelClass> m_model;
std::vector<U_adaptive_cartesian> m_log_u;
std::vector<Y_adaptive_cartesian> m_log_y;
std::vector<L_adaptive_cartesian> m_log_l;
std::string m_path_logs;
unsigned long long m_cnt_step;
bool m_flag_log;};

}