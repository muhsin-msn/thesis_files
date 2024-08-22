#pragma once

#include <vector>
#include <string>
#include <memory>
#include "Eigen/Core"

class dual_arm_syncModelClass;

namespace dual_arm_sync {

struct P_dual_arm_sync{
Eigen::Matrix<double,6,1> alpha_l;
Eigen::Matrix<double,6,1> alpha_r;
Eigen::Matrix<double,6,1> beta_l;
Eigen::Matrix<double,6,1> beta_r;
Eigen::Matrix<double,6,1> gamma_a_l;
Eigen::Matrix<double,6,1> gamma_a_r;
Eigen::Matrix<double,6,1> gamma_b_l;
Eigen::Matrix<double,6,1> gamma_b_r;
Eigen::Matrix<double,6,1> K_0_l;
Eigen::Matrix<double,6,1> K_0_r;
Eigen::Matrix<double,6,1> F_ff_0_l;
Eigen::Matrix<double,6,1> F_ff_0_r;
Eigen::Matrix<double,6,1> L_l;
Eigen::Matrix<double,6,1> L_r;
Eigen::Matrix<double,6,1> xi_l;
Eigen::Matrix<double,6,1> xi_r;
Eigen::Matrix<double,1,1> kappa_l;
Eigen::Matrix<double,1,1> kappa_r;
Eigen::Matrix<double,1,1> TF_control_l;
Eigen::Matrix<double,1,1> TF_control_r;
Eigen::Matrix<double,6,1> K_max_l;
Eigen::Matrix<double,6,1> K_max_r;
Eigen::Matrix<double,6,1> dK_max_l;
Eigen::Matrix<double,6,1> dK_max_r;
Eigen::Matrix<double,6,1> F_ff_max_l;
Eigen::Matrix<double,6,1> F_ff_max_r;
Eigen::Matrix<double,6,1> dF_ff_max_l;
Eigen::Matrix<double,6,1> dF_ff_max_r;
Eigen::Matrix<double,7,1> tau_max_l;
Eigen::Matrix<double,7,1> tau_max_r;
Eigen::Matrix<double,7,1> dtau_max_l;
Eigen::Matrix<double,7,1> dtau_max_r;
P_dual_arm_sync(){
alpha_l.setZero();
alpha_r.setZero();
beta_l.setZero();
beta_r.setZero();
gamma_a_l.setZero();
gamma_a_r.setZero();
gamma_b_l.setZero();
gamma_b_r.setZero();
K_0_l.setZero();
K_0_r.setZero();
F_ff_0_l.setZero();
F_ff_0_r.setZero();
L_l.setZero();
L_r.setZero();
xi_l.setZero();
xi_r.setZero();
kappa_l.setZero();
kappa_r.setZero();
TF_control_l.setZero();
TF_control_r.setZero();
K_max_l.setZero();
K_max_r.setZero();
dK_max_l.setZero();
dK_max_r.setZero();
F_ff_max_l.setZero();
F_ff_max_r.setZero();
dF_ff_max_l.setZero();
dF_ff_max_r.setZero();
tau_max_l.setZero();
tau_max_r.setZero();
dtau_max_l.setZero();
dtau_max_r.setZero();
}
};
struct U_dual_arm_sync{
Eigen::Matrix<double,4,4> TF_T_EE_d1;
Eigen::Matrix<double,4,4> TF_T_EE_d2;
Eigen::Matrix<double,4,4> TF_T_EE1;
Eigen::Matrix<double,4,4> TF_T_EE2;
Eigen::Matrix<double,6,1> TF_F_ff1;
Eigen::Matrix<double,6,1> TF_F_ff2;
Eigen::Matrix<double,6,1> TF_F_ext1;
Eigen::Matrix<double,6,1> TF_F_ext2;
Eigen::Matrix<double,7,1> coriolis1;
Eigen::Matrix<double,7,1> coriolis2;
Eigen::Matrix<double,6,7> B_J_EE1;
Eigen::Matrix<double,6,7> B_J_EE2;
Eigen::Matrix<double,7,1> dtheta1;
Eigen::Matrix<double,7,1> dtheta2;
Eigen::Matrix<double,6,1> K_x1;
Eigen::Matrix<double,6,1> K_x2;
Eigen::Matrix<double,6,1> xi_x1;
Eigen::Matrix<double,6,1> xi_x2;
U_dual_arm_sync(){
TF_T_EE_d1.setZero();
TF_T_EE_d2.setZero();
TF_T_EE1.setZero();
TF_T_EE2.setZero();
TF_F_ff1.setZero();
TF_F_ff2.setZero();
TF_F_ext1.setZero();
TF_F_ext2.setZero();
coriolis1.setZero();
coriolis2.setZero();
B_J_EE1.setZero();
B_J_EE2.setZero();
dtheta1.setZero();
dtheta2.setZero();
K_x1.setZero();
K_x2.setZero();
xi_x1.setZero();
xi_x2.setZero();
}
};
struct Y_dual_arm_sync{
Eigen::Matrix<double,14,1> tau_J_d;
Y_dual_arm_sync(){
tau_J_d.setZero();
}
};
struct L_dual_arm_sync{
L_dual_arm_sync(){
}
};
class dual_arm_sync{
public:
dual_arm_sync();
~dual_arm_sync();
void initialize(bool log = false,unsigned long long l_len = 0,const std::string& path_logs="");
void step();
void terminate();
P_dual_arm_sync p;
U_dual_arm_sync u;
Y_dual_arm_sync y;
L_dual_arm_sync l;

private:
void write_input();
void write_output();
void write_log();
std::unique_ptr<dual_arm_syncModelClass> m_model;
std::vector<U_dual_arm_sync> m_log_u;
std::vector<Y_dual_arm_sync> m_log_y;
std::vector<L_dual_arm_sync> m_log_l;
std::string m_path_logs;
unsigned long long m_cnt_step;
bool m_flag_log;};

}