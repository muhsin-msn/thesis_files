#include <iostream>
#include "franka/exception.h"
#include "franka/robot.h"
#include "franka/model.h"
#include "cntr_joint_imp/cntr_joint_imp_wrapper.hpp"


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    franka::Robot robot(argv[1]);
    franka::Model model = robot.loadModel();
    try {
        robot.setCollisionBehavior({5,5,5,5,5,5,5},{100,100,100,80,80,40,40},{5,5,5,3,3,3},{100,100,100,30,30,30});
        robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
        robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

        cntr_joint_imp::cntr_joint_imp cntr_jnt;

        double damping=0.7;
        cntr_jnt.p.D_theta<<damping,damping,damping,damping,damping,damping,damping;
        cntr_jnt.p.enable_ffwd_acc<<0;
        cntr_jnt.p.enable_ffwd_vel<<0;
        cntr_jnt.p.K_theta<<1000,1000,1000,500,200,100,100;

        franka::RobotState state=robot.readOnce();

        cntr_jnt.u.theta=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.q).data()));
        cntr_jnt.u.theta_d=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.q).data()));
        cntr_jnt.u.dtheta=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.dq).data()));
        cntr_jnt.u.dtheta_d.setZero();
        cntr_jnt.u.ddtheta_d.setZero();
        cntr_jnt.u.tau_ff.setZero();
        cntr_jnt.u.M=Eigen::Matrix<double,7,7>(Eigen::Map<Eigen::Matrix<double,7,7> >(std::array<double,49>(model.mass(state)).data()));
        cntr_jnt.u.M.transposeInPlace();

        cntr_jnt.initialize();

        double time=0;



        
        auto callback = [=, &time,&cntr_jnt,&model](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
            time += period.toSec();


            cntr_jnt.u.theta=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.q).data()));
            // cntr_jnt.u.theta_d=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.q).data()));
            cntr_jnt.u.dtheta=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state.dq).data()));
            cntr_jnt.u.dtheta_d<<0,0,0,0,0,0,0;
            cntr_jnt.u.ddtheta_d<<0,0,0,0,0,0,0;
            cntr_jnt.u.M=Eigen::Matrix<double,7,7>(Eigen::Map<Eigen::Matrix<double,7,7> >(std::array<double,49>(model.mass(state)).data()));
            cntr_jnt.u.M.transposeInPlace();
            cntr_jnt.step();
            franka::Torques tau_J={cntr_jnt.y.tau_J_d[0],cntr_jnt.y.tau_J_d[1],cntr_jnt.y.tau_J_d[2],cntr_jnt.y.tau_J_d[3],
                                   cntr_jnt.y.tau_J_d[4],cntr_jnt.y.tau_J_d[5],cntr_jnt.y.tau_J_d[6]};

            if (time >= 10) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(tau_J);
            }
            return tau_J;
        };

        try{
            robot.control(callback);
            cntr_jnt.terminate();
        }catch(const franka::ControlException& e){
            std::cout<<e.what()<<std::endl;
            cntr_jnt.terminate();
            robot.automaticErrorRecovery();
        }catch(const franka::Exception& e){
            std::cout<<e.what()<<std::endl;
            cntr_jnt.terminate();
            robot.automaticErrorRecovery();
        }

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        robot.automaticErrorRecovery();
        return -1;
    }
    return 0;
}