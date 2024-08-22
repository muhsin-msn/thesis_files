
#include <iostream>
#include <array>
#include <thread>
#include <mutex>
#include "franka/exception.h"
#include "franka/robot.h"
#include "franka/model.h"
#include "adaptive_cartesian/adaptive_cartesian_wrapper.hpp"
#include <Eigen/Dense>
#include "include/pseudo_inversion.h"
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>

#include "sensor_msgs/JointState.h"
#include <cmath>
#include "examples_common.h"
#include "sensor_msgs/Joy.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <sstream>
#include <fstream>
class DualArmController {



public:
    DualArmController(const std::string& robot_right_hostname, const std::string& robot_left_hostname,  ros::NodeHandle& nh)
        :  robot_right_hostname_(robot_right_hostname), robot_left_hostname_(robot_left_hostname), running_(true) , nh_(nh){
             jointSubRight = nh_.subscribe<sensor_msgs::JointState>("/panda_joint_states_right", 10, &DualArmController::jointCallbackRight, this);
             jointSubLeft = nh_.subscribe<sensor_msgs::JointState>("/panda_joint_states_left", 10, &DualArmController::jointCallbackLeft, this);
             
             viveRightSub = nh_.subscribe<sensor_msgs::Joy>("/vive_right", 1, &DualArmController::viveRightCallback, this);
            viveLeftSub = nh_.subscribe<sensor_msgs::Joy>("/vive_left", 1, &DualArmController::viveLeftCallback, this);
             vibration_pub = nh_.advertise<std_msgs::Float64>("/vive_right_vibration", 1);
             external_force_right_pub = nh_.advertise<std_msgs::Float64MultiArray>("/external_force_right", 1);
            external_force_left_pub = nh_.advertise<std_msgs::Float64MultiArray>("/external_force_left", 1);
            
            external_stiffness_right_pub = nh_.advertise<std_msgs::Float64MultiArray>("/external_stiffness_right", 1);
            external_stiffness_left_pub = nh_.advertise<std_msgs::Float64MultiArray>("/external_stiffness_left", 1);


            external_damping_right_pub = nh_.advertise<std_msgs::Float64MultiArray>("/external_damping_right", 1);
            external_damping_left_pub = nh_.advertise<std_msgs::Float64MultiArray>("/external_damping_left", 1);

    }
    
    void run() {

       int iterationCount = 0;
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
     std::stringstream dataStreaml,dataStreamr, forceDataStreamR,forceStreamLeft,forceStreamRight;
     dataStreaml << "Time_l,J1_l,J2_l,J3_l,J4_l,J5_l,J6_l,J7_l,T1_l,T2_l,T3_l,T4_l,T5_l,T6_l,T7_l" << std::endl;
     dataStreamr << "Time_r,J1_r,J2_r,J3_r,J4_r,J5_r,J6_r,J7_r,T1_r,T2_r,T3_r,T4_r,T5_r,T6_r,T7_r" << std::endl;
     forceStreamLeft << "Time,Force_Left_X,Force_Left_Y,Force_Left_Z,Force_Left_Rx,Force_Left_Ry,Force_Left_Rz" << std::endl;
    forceStreamRight << "Time,Force_Right_X,Force_Right_Y,Force_Right_Z,Force_Right_Rx,Force_Right_Ry,Force_Right_Rz" << std::endl;

     forceDataStreamR << "Force_z" << std::endl;

        try {
            
           std::string myVariable1 = robot_right_hostname_;
           std::string myVariable2 = robot_left_hostname_;
            
             std::cout << "The value of myVariable is: " << myVariable1 << std::endl;
              std::cout << "The value of myVariable is: " << myVariable2 << std::endl;

             franka::Robot robot_right_(myVariable1);
            franka::Robot robot_left_(myVariable2);


            robot_right_.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

            robot_left_.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


        ros::Rate loopRate(10); // Set the loop rate to 10 Hz
        ros::Time startTime2 = ros::Time::now();

        while (ros::ok())
        {
             for (size_t i = 0; i < joint_positions_left.size(); ++i) {
        ROS_INFO_STREAM("Joint " << i << " position_left: " << joint_positions_left[i]);
    }   


    for (size_t i = 0; i < joint_positions_right.size(); ++i) {
        ROS_INFO_STREAM("Joint " << i << " position_right: " << joint_positions_right[i]);
    }

    std::string buttonsStr;
    for (int i = 0; i < receivedButtons.size(); ++i) {
        buttonsStr += std::to_string(receivedButtons[i]) + " ";
    }
    ROS_INFO("\033[1;32mReceived right buttons: %s\033[0m", buttonsStr.c_str());
    
    ROS_INFO("\033[1;33mPress the middle button in steady position to \033[1;33m\033[0mbreak the loop.\033[0m");

            ros::spinOnce();

 

             if (!receivedButtons.empty() && receivedButtons[2] == 1) {
        ROS_INFO("\033[1;32mButton [2] is pressed. Exiting the loop.\033[0m");
        break; // Exit the loop if button [2] is pressed
    }
            

            loopRate.sleep();
        } 

            

                /* std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore(); */


   while (ros::ok()) {
     ROS_INFO("\033[1;36mJoint position fixed. Keep your hands steady. Press menu button to continue.\033[0m");


            ros::spinOnce();
           
            // Check if the middle button (button[2]) is pressed to continue
            if (!receivedButtons.empty() && receivedButtons[3] == 1) {
                ROS_INFO("\033[1;33mMenu Button  is pressed. Continuing...\033[0m");
                break;
            }

            ros::Duration(0.1).sleep(); // Add a small delay
        }


             //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    
    MotionGenerator motion_generator_left(0.2, joint_positions_left);
    robot_left_.control(motion_generator_left);


    MotionGenerator motion_generator_right(0.2, joint_positions_right);
    robot_right_.control(motion_generator_right); 


 

    
    



            franka::RobotState state1i = robot_right_.readOnce();
            franka::RobotState state2i = robot_left_.readOnce();

            franka::Model model1_ = robot_right_.loadModel();
            franka::Model model2_ = robot_left_.loadModel();


      




            

            std::array<double, 16> initial_pose_1{};
        initial_pose_1 = state1i.O_T_EE;

        Eigen::Matrix<double,7,1> q1,q2  ,dq1,dq2;
         
         Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial1(state1i.q.data());
                 Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial2(state2i.q.data());

            std::array<double, 16> initial_pose_2{};
        initial_pose_2 = state2i.O_T_EE;

        

     cntr_cart_l.p.alpha <<  9, 9, 9,0,0, 0;
            cntr_cart_r.p.alpha << 9, 9, 9,0, 0,0;
            cntr_cart_l.p.beta << 1000, 1000, 1000,0,0, 0;
            cntr_cart_r.p.beta <<  1000, 1000, 1000,0,0, 0;
            cntr_cart_l.p.beta1 <<100, 100, 100,0,0, 0;
            cntr_cart_r.p.beta1 <<100, 100, 100, 0,0, 0;
            cntr_cart_l.p.gamma_a  << 0.1,0.1,0.1,0.1,0.1,0.1;
            cntr_cart_r.p.gamma_a  <<   0.1,0.1,0.1,  0.1,0.1,0.1;
            cntr_cart_l.p.gamma_b << 0.00005,0.00005,0.00005,0.00005,0.00005,0.00005;
            cntr_cart_r.p.gamma_b <<0.00005,0.00005,0.00005,0.00005,0.00005,0.00005;
            cntr_cart_l.p.K_0 <<  0, 0, 0, 0, 0, 0;
            cntr_cart_r.p.K_0 << 0, 0, 0, 0, 0, 0;
            cntr_cart_l.p.D_0 << 0,0,0,0,0,0;
            cntr_cart_r.p.D_0 << 0,0,0,0,0,0;
            cntr_cart_l.p.F_ff_0 << 0, 0, 0, 0, 0, 0;
            cntr_cart_r.p.F_ff_0<< 0, 0, 0, 0, 0, 0;
            cntr_cart_l.p.L <<   250.0, 250.0, 250.0, 20.0, 20.0, 20.0;
            cntr_cart_r.p.L <<  250.0, 250.0, 250.0,20.0, 20.0, 20.0;
            cntr_cart_l.p.L1 <<   250.0, 250.0, 250.0, 20.0, 20.0, 20.0;
            cntr_cart_r.p.L1 <<  250.0, 250.0, 250.0, 20.0, 20.0, 20.0;


            cntr_cart_l.p.xi << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;
            cntr_cart_r.p.xi << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;
            cntr_cart_l.p.kappa << 5;
            cntr_cart_r.p.kappa <<5;
            cntr_cart_l.p.TF_control << 1;
            cntr_cart_r.p.TF_control << 1;
            cntr_cart_l.p.K_max << 1000, 1000, 1000, 150, 150, 150;
            cntr_cart_r.p.K_max << 1000, 1000, 1000, 150, 150, 150;

            cntr_cart_l.p.D_max << 1000, 1000, 1000, 150, 150, 150;
            cntr_cart_r.p.D_max << 1000, 1000, 1000, 150, 150, 150;

            cntr_cart_l.p.dK_max << 5000, 5000, 5000, 400, 400, 400;
            cntr_cart_r.p.dK_max << 5000, 5000, 5000, 400, 400, 400;
            cntr_cart_l.p.F_ff_max << 61, 62, 63, 64, 65, 66;
            cntr_cart_r.p.F_ff_max << 61, 62, 63, 64, 65, 66;
            cntr_cart_l.p.dF_ff_max << 67, 68, 69, 70, 71, 72;
            cntr_cart_r.p.dF_ff_max << 67, 68, 69, 70, 71, 72;
            cntr_cart_l.p.tau_max << 87, 87, 87, 87, 12, 12, 12;
            cntr_cart_r.p.tau_max << 87, 87, 87, 87, 12, 12, 12;
            cntr_cart_l.p.dtau_max << 1000, 1000, 1000, 1000, 1000, 1000, 1000;
            cntr_cart_r.p.dtau_max << 1000, 1000, 1000, 1000, 1000, 1000, 1000;



    cntr_cart_r.u.TF_T_EE_d = Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(state1i.O_T_EE.data()));
    cntr_cart_l.u.TF_T_EE_d = Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(state2i.O_T_EE.data()));
    cntr_cart_r.u.TF_T_EE = Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(state1i.O_T_EE.data()));
    cntr_cart_l.u.TF_T_EE = Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(state2i.O_T_EE.data()));
    cntr_cart_r.u.TF_F_ff << 0, 0, 0, 0, 0, 0;
    cntr_cart_r.u.TF_F_ff << 0, 0, 0, 0, 0, 0;
    cntr_cart_l.u.TF_F_ext << 0, 0, 0, 0, 0, 0;
    cntr_cart_r.u.TF_F_ext << 0, 0, 0, 0, 0, 0;
    cntr_cart_r.u.dtheta = Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state1i.dq).data()));
    cntr_cart_l.u.dtheta = Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state2i.dq).data()));
    cntr_cart_l.u.K_x << 200, 200, 200, 20, 20, 20;
    cntr_cart_r.u.K_x << 200, 200, 200, 20, 20, 20;
    cntr_cart_l.u.xi_x << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;
    cntr_cart_r.u.xi_x << 0.7, 0.7, 0.7, 0.7, 0.7, 0.7;


       

            cntr_cart_l.initialize();
            cntr_cart_r.initialize();

            std::array<double, 14> output; 

            
            std::thread thread1([&]() {

                  


                while (!buttonPressed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }


                    

                std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                 callback1 = [&](const franka::RobotState& state1, franka::Duration period1) -> franka::Torques {
                    // Lock the mutex for thread safety
                        std::lock_guard<std::mutex> lock(mutex);
                        time_r += period1.toSec();
                           

                       
        std::array<double, 42> jacobian_array1 = model1_.zeroJacobian(franka::Frame::kEndEffector, state1);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian1(jacobian_array1.data());

        cntr_cart_r.u.coriolis=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>( model1_.coriolis(state1)).data()));

         
      
                

                 constexpr double kRadius1 = 0.3;
        double angle1 = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time_l));
        double delta_x1 = kRadius1 * std::sin(angle1);
        double delta_z1 = kRadius1 * (std::cos(angle1) - 1);

        std::array<double, 16> new_pose1 = initial_pose_1;
        new_pose1[12] -= delta_x1;
        new_pose1[14] -= delta_z1;

            //cntr_cart_r.u.TF_F_ff = Eigen::Matrix<double,6,1>(Eigen::Map<Eigen::Matrix<double,6,1> >(std::array<double,6>(state1.K_F_ext_hat_K).data()));



        cntr_cart_r.u.TF_T_EE = Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(state1.O_T_EE.data()));
                dq1 =Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state1.dq).data()));
                cntr_cart_r.u.dtheta<< dq1;
             //cntr_cart_r.u.TF_T_EE_d=Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(new_pose1.data()));

            
            cntr_cart_r.u.TF_T_EE_d << tf_matrix_r;
                cntr_cart_r.u.B_J_EE<< jacobian1;
                     
                                 q1=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state1.q).data()));

                    counterl++;













              external_wrench_state_r = state1.K_F_ext_hat_K;  
               
                force_with_time_r[0] = time_l; // Time as the first element
                for (int i = 0; i < 6; ++i) {
                    force_with_time_r[i + 1] = external_wrench_state_r[i]; // Append force values
                }



                    cntr_cart_r.step();




                    
                       Eigen::VectorXd::Map(&damping_r_1[0], 6) = cntr_cart_r.y.D_x_2;

                        std::copy_n(damping_r_1.begin(), 6, damping_r.begin());

                    

               
                damping_with_time_r[0] = time_r;
                for (int i = 0; i < 6; ++i) {
                    damping_with_time_r[i + 1] = damping_r[i]; 
                }



                Eigen::VectorXd::Map(&stiffness_r_1[0], 6) = cntr_cart_r.y.K_x_2;

                        std::copy_n(stiffness_r_1.begin(), 6, stiffness_r.begin());



               
                stiffness_with_time_r[0] = time_r; 
                for (int i = 0; i < 6; ++i) {
                    stiffness_with_time_r[i + 1] = stiffness_r[i]; 
                }

                           

                            
                            Eigen::VectorXd::Map(&tau_d_array_r[0], 7) = cntr_cart_r.y.tau_J_d;

                            std::copy_n(tau_d_array_r.begin(), 7, tau_d_array1.begin()); 

                          std::array<double, 7> tau_J_d_1{};
                             std::array<double, 7> tau_J_1{},tau_nullspace1{},tau_sum1{};;
                          /* std::copy_n(tau_d_array.begin() + 7, 7, tau_d_array2.begin());
                      //  std::fill(tau_d_array2.begin(), tau_d_array2.end(), 0.0); */

                            tau_J_d_1=state1.tau_J_d;    
                            
                            tau_J_1 = saturateTorqueRate(tau_d_array1, tau_J_d_1);

                        for (int i = 0; i < 7; i++) {
                dataStreaml << "," << tau_J_1[i];
                        }

                         dataStreaml << std::endl;





                            tau_nullspace1 = calculateTauNullspace(jacobian1,
                                                                q_initial1, q1, dq1);



                            for (int i = 0; i < 7; ++i) {
                                            tau_sum1[i] = tau_nullspace1[i] + tau_J_1[i];
                                        }


                             iterationCount++;     
                            franka::Torques tau_sum3 =tau_J_1;

                          
                            if (stopButtonPressed) {

                            
                             std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                         return franka::MotionFinished(tau_sum3); 

                            }
                            return tau_J_1 ;

                            
                };
               

              
        try {
            robot_right_.control(callback1);
        } catch (const franka::ControlException& e) {
            std::cout << e.what() << std::endl;
            robot_right_.automaticErrorRecovery();
              }
            
            });

            std::thread thread2([&]() {

                while (!buttonPressed) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
    }

               

                std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
                 callback2 = [&](const franka::RobotState& state2, franka::Duration period2) -> franka::Torques {
                  std::lock_guard<std::mutex> lock(mutex); 



                         time_l += period2.toSec();
                         std::array<double, 42> jacobian_array2 = model2_.zeroJacobian(franka::Frame::kEndEffector, state2);
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian2(jacobian_array2.data());

        
        cntr_cart_l.u.coriolis=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>( model2_.coriolis(state2)).data()));
         
                

        constexpr double kRadius2 = 0.3;
        double angle2 = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * time_r));
        double delta_x2 = kRadius2 * std::sin(angle2);
        double delta_z2 = kRadius2 * (std::cos(angle2) - 1);

        std::array<double, 16> new_pose2 = initial_pose_2;
        new_pose2[12] -= delta_x2;
        new_pose2[14] -= delta_z2;

        
                cntr_cart_l.u.TF_T_EE = Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(state2.O_T_EE.data()));
               dq2 =Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state2.dq).data()));
                cntr_cart_l.u.dtheta<< dq2;

                //cntr_cart_l.u.TF_F_ff = Eigen::Matrix<double,6,1>(Eigen::Map<Eigen::Matrix<double,6,1> >(std::array<double,6>(state2.K_F_ext_hat_K).data()));


               //cntr_cart_l.u.TF_T_EE_d=Eigen::Matrix<double,4,4>(Eigen::Matrix4d::Map(new_pose2.data()));
              cntr_cart_l.u.TF_T_EE_d<< tf_matrix_l;

               external_wrench_state_l = state2.K_F_ext_hat_K;

               
                force_with_time_l[0] = time_l; // Time as the first element
                for (int i = 0; i < 6; ++i) {
                    force_with_time_l[i + 1] = external_wrench_state_l[i]; // Append force values
                }
               
                cntr_cart_l.u.B_J_EE<< jacobian2;
        
                 q2=Eigen::Matrix<double,7,1>(Eigen::Map<Eigen::Matrix<double,7,1> >(std::array<double,7>(state2.q).data()));

                 counterr++;
          



                         cntr_cart_l.step();


                         //stiffness_l <<cntr_cart_l.y.K_x_2;

                         Eigen::VectorXd::Map(&stiffness_l_1[0], 6) = cntr_cart_l.y.K_x_2;

                        std::copy_n(stiffness_l_1.begin(), 6, stiffness_l.begin());

               
                stiffness_with_time_l[0] = time_l; // Time as the first element
                for (int i = 0; i < 6; ++i) {
                    stiffness_with_time_l[i + 1] = stiffness_l[i]; // Append force values
                }

                    //damping_l =cntr_cart_l.y.D_x_2;

                     Eigen::VectorXd::Map(&damping_l_1[0], 6) = cntr_cart_l.y.D_x_2;

                        std::copy_n(damping_l_1.begin(), 6, damping_l.begin());

               
                damping_with_time_l[0] = time_l; 
                for (int i = 0; i < 6; ++i) {
                    damping_with_time_l[i + 1] = damping_l[i];
                }
      
                    Eigen::VectorXd::Map(&tau_d_array_l[0], 7) = cntr_cart_l.y.tau_J_d;

                        std::copy_n(tau_d_array_l.begin(), 7, tau_d_array2.begin());


                              std::array<double, 7> tau_J_d_2{};
                             std::array<double, 7> tau_J_2{} ,tau_nullspace2{},tau_sum2{};
                          /* std::copy_n(tau_d_array.begin() + 7, 7, tau_d_array2.begin());
                      //  std::fill(tau_d_array2.begin(), tau_d_array2.end(), 0.0); */

                            tau_J_d_2=state2.tau_J_d;    
                            
                            tau_J_2 = saturateTorqueRate(tau_d_array2, tau_J_d_2);










                             tau_nullspace2 = calculateTauNullspace(jacobian2,
                                                                q_initial2, q2, dq2);

                                for (int i = 0; i < 7; ++i) {
                                            tau_sum2[i] = tau_nullspace2[i] + tau_J_2[i];
                                        }
                            franka::Torques tau_sum3 =tau_J_2;
                       

                         if (stopButtonPressed) {

                            std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                                  
                                return franka::MotionFinished(tau_sum3); 

                            }
                            return tau_J_2 ;
                };

               
                try {
                    robot_left_.control(callback2);
                } catch (const franka::ControlException& e) {
                    std::cout << e.what() << std::endl;
                    robot_left_.automaticErrorRecovery();
                }
            
            });

            
        int p=0;
            
            
            
            std::thread updateThread([&]() {
                 while (ros::ok() && !stopButtonPressed) {
                   // std::lock_guard<std::mutex> lock(mutex); 

                   
                     ros::Time lastVibrationCallbackTime = ros::Time::now();



                    //calculateOutput(); 
                    p++;
                    

                     tfCallback_r(tf_listener_r);

                  

std::cout << "TF_F_ff:\n" << cntr_cart_r.y.F_add << std::endl;
                  std::cout << "D_x_2:\n" << cntr_cart_r.y.D_x_2 << std::endl;
                   std::cout << "K_x:\n" <<  cntr_cart_r.y.K_x_2 << std::endl;

                    tfCallback_l(tf_listener_l);

                    if (buttonPressed) {

                         

                    broadcastTransform(cntr_cart_l.u.TF_T_EE, tf_broadcaster_l);
                    broadcastTransform2(cntr_cart_r.u.TF_T_EE, tf_broadcaster_r);
        } else {

            franka::RobotState state_right = robot_right_.readOnce();
                    franka::RobotState state_left = robot_left_.readOnce();

                    Eigen::Map<const Eigen::Matrix<double, 4, 4>> ee_pose_left(state_left.O_T_EE.data());
                    Eigen::Map<const Eigen::Matrix<double, 4, 4>> ee_pose_right(state_right.O_T_EE.data());

                   broadcastTransform(ee_pose_left, tf_broadcaster_l);
                   broadcastTransform2(ee_pose_right, tf_broadcaster_r);
            
        }



                     vibrationCallback(force_with_time_r);
                     externalForceRightCallback(force_with_time_r);
                     externalForceLeftCallback(force_with_time_l);
                     externalStiffnessRightCallback(stiffness_with_time_r);
                     externalStiffnessLeftCallback(stiffness_with_time_l);
                     externalDampingRightCallback(damping_with_time_r);
                     externalDampingLeftCallback(damping_with_time_l);


                   
                    
                    //std::this_thread::sleep_for(std::chrono::milliseconds(1));  
                    ros::spinOnce();
                   
                    
                   // std::this_thread::sleep_for(std::chrono::milliseconds(1)); 

                     std::this_thread::sleep_for(std::chrono::milliseconds(delayDuration));
                }

            });


            while (ros::ok()) {
    if (!receivedButtons.empty() && receivedButtons[4] == 1) {
        buttonPressed = true;

        ROS_INFO("\033[1;32mButton [4] is pressed. Starting robot control threads.\033[0m");
        //break; // Exit the loop if button [2] is pressed
    }

    /* if (!receivedButtons.empty() && receivedButtons[1] == 1) {
        centerButtonPressed = true;

        ROS_INFO("\033[1;32mButton [3] is pressed. switching tfs.\033[0m");
         // Exit the loop if button [2] is pressed
    } */

    if (!receivedButtons.empty() && receivedButtons[0] == 1) {
        stopButtonPressed = true;

        ros::shutdown(); 

        ROS_INFO("\033[1;32mButton [3] is pressed. stopping threads.\033[0m");
         // Exit the loop if button [2] is pressed
    }

    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
 }

            thread1.join();
            thread2.join();
            updateThread.join();
            
                    

        } catch (const franka::Exception& e) {
            std::cout << e.what() << std::endl;
            
        }
         
        
    }


    

    
    std::array<double, 7> calculateTauNullspace(const Eigen::MatrixXd& jacobian,
                                            const Eigen::VectorXd& q_initial,
                                            const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& dq) {
    Eigen::MatrixXd jacobian_transpose = jacobian.transpose();
    Eigen::MatrixXd jacobian_transpose_pinv; 
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    Eigen::MatrixXd identity_matrix = Eigen::MatrixXd::Identity(7, 7);

    Eigen::MatrixXd intermediate_result = identity_matrix - jacobian_transpose * jacobian_transpose_pinv;
    Eigen::VectorXd difference_vector = nullspace_stiffness_ * (q_initial - q) -
                                        (2.0 * sqrt(nullspace_stiffness_)) * dq;
    Eigen::VectorXd result_vector = intermediate_result * difference_vector;

    std::array<double, 7> tau_nullspace;
    for (int i = 0; i < 7; ++i) {
        tau_nullspace[i] = result_vector(i);
    }

    return tau_nullspace;
}

    


    std::array<double, 7> saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) { 
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_); 

  }
  return tau_d_saturated;
}

void tfCallback_r(const tf::TransformListener& listener)
{
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("world", "RightHandPn_2", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }



  //  rotation and translation components
    tf::Quaternion rotation = transform.getRotation();
    tf::Vector3 translation = transform.getOrigin();

    //  the transformation matrix
    Eigen::Matrix4d hand_tf;
    hand_tf.setIdentity();
    hand_tf.block(0, 0, 3, 3) = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
    hand_tf(0, 3) = translation.x();
    hand_tf(1, 3) = translation.y();
    hand_tf(2, 3) = translation.z();

    
    tf_matrix_r = hand_tf;
    // Print the transformation matrix
    //ROS_INFO("Received TF Matrix:");
    //ROS_INFO_STREAM(hand_tf);
}













void tfCallback_l(const tf::TransformListener& listener)
{
    tf::StampedTransform transform;
    try {
        listener.lookupTransform("world", "LeftHandPn_2", ros::Time(0), transform);
    } catch (tf::TransformException& ex) {
        ROS_WARN("%s", ex.what());
        return;
    }

    //  rotation and translation components
    tf::Quaternion rotation = transform.getRotation();
    tf::Vector3 translation = transform.getOrigin();

 

    //  the transformation matrix
    Eigen::Matrix4d hand_tf;
    hand_tf.setIdentity();
    hand_tf.block(0, 0, 3, 3) = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z()).toRotationMatrix();
    hand_tf(0, 3) = translation.x();
    hand_tf(1, 3) = translation.y();
    hand_tf(2, 3) = translation.z();

    
    tf_matrix_l = hand_tf;




    // ROS_INFO("Received TF Matrix:");
    // ROS_INFO_STREAM(hand_tf);
}


void jointCallbackRight(const sensor_msgs::JointStateConstPtr& msg)
{
    
      if (msg->position.size() == joint_positions_right.size()) {
    std::copy(msg->position.begin(), msg->position.end(), joint_positions_right.begin());
}
}

void jointCallbackLeft(const sensor_msgs::JointStateConstPtr& msg)
{
    
      if (msg->position.size() == joint_positions_left.size()) {
    std::copy(msg->position.begin(), msg->position.end(), joint_positions_left.begin());
}





      
 
}


void broadcastTransform(const Eigen::Matrix4d& TF_T_EE_L_B, tf::TransformBroadcaster& tf_broadcaster) {
    Eigen::Matrix3d rotation_matrix = TF_T_EE_L_B.block<3, 3>(0, 0);

    Eigen::Quaterniond rotation_quaternion(rotation_matrix);

    tf::Quaternion tf_rotation_quaternion(
        rotation_quaternion.x(),
        rotation_quaternion.y(),
        rotation_quaternion.z(),
        rotation_quaternion.w());

    tf::Transform tf_transform(tf_rotation_quaternion, tf::Vector3(TF_T_EE_L_B(0, 3), TF_T_EE_L_B(1, 3), TF_T_EE_L_B(2, 3)));

    tf::StampedTransform stamped_transform(tf_transform, ros::Time::now(), "panda_1_link0", "left_dmp_tf");
    tf_broadcaster.sendTransform(stamped_transform);
}

void broadcastTransform2(const Eigen::Matrix4d& TF_T_EE_L_B, tf::TransformBroadcaster& tf_broadcaster) {
    Eigen::Matrix3d rotation_matrix = TF_T_EE_L_B.block<3, 3>(0, 0);

    Eigen::Quaterniond rotation_quaternion(rotation_matrix);

    tf::Quaternion tf_rotation_quaternion(
        rotation_quaternion.x(),
        rotation_quaternion.y(),
        rotation_quaternion.z(),
        rotation_quaternion.w());

    tf::Transform tf_transform(tf_rotation_quaternion, tf::Vector3(TF_T_EE_L_B(0, 3), TF_T_EE_L_B(1, 3), TF_T_EE_L_B(2, 3)));

    tf::StampedTransform stamped_transform(tf_transform, ros::Time::now(), "panda_2_link0", "right_dmp_tf");
    tf_broadcaster.sendTransform(stamped_transform);
}

void viveRightCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
   receivedButtons = joy_msg->buttons;
   
    
    
}


void viveLeftCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
   receivedButtons_l = joy_msg->buttons;
   
    
    
}

double mapValue(double value, double in_min, double in_max, double out_min, double out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void externalForceLeftCallback(const std::array<double, 7>& force_with_time) {
    
 

    std::vector<double> force_vector(force_with_time.begin(), force_with_time.end());

    std_msgs::Float64MultiArray force_right_msg;
    force_right_msg.data = force_vector;
    external_force_right_pub.publish(force_right_msg);
}

void externalForceRightCallback(const std::array<double, 7>& force_with_time) {
 

   
    std::vector<double> force_vector(force_with_time.begin(), force_with_time.end());

    std_msgs::Float64MultiArray force_left_msg;
    force_left_msg.data = force_vector;
    external_force_left_pub.publish(force_left_msg);
}


void externalStiffnessLeftCallback(const std::array<double, 7>& stiffness_with_time) {
   
 

    
    std::vector<double> stiffness_vector(stiffness_with_time.begin(), stiffness_with_time.end());


    std_msgs::Float64MultiArray stiffness_right_msg;
    stiffness_right_msg.data = stiffness_vector;
    external_stiffness_right_pub.publish(stiffness_right_msg);
}

void externalStiffnessRightCallback(const std::array<double, 7>& stiffness_with_time) {
   
   
   
    std::vector<double> stiffness_vector(stiffness_with_time.begin(), stiffness_with_time.end());

    
    std_msgs::Float64MultiArray stiffness_left_msg;
    stiffness_left_msg.data = stiffness_vector;
    external_stiffness_left_pub.publish(stiffness_left_msg);
}


void externalDampingLeftCallback(const std::array<double, 7>& damping_with_time) {
   

 

    
    std::vector<double> damping_vector(damping_with_time.begin(), damping_with_time.end());

   
    std_msgs::Float64MultiArray damping_right_msg;
    damping_right_msg.data = damping_vector;
    external_damping_right_pub.publish(damping_right_msg);
}

void externalDampingRightCallback(const std::array<double, 7>& damping_with_time) {
    

   
    
    std::vector<double> damping_vector(damping_with_time.begin(), damping_with_time.end());

    
    std_msgs::Float64MultiArray damping_left_msg;
    damping_left_msg.data = damping_vector;
    external_damping_left_pub.publish(damping_left_msg);
}
void vibrationCallback(const std::array<double, 7>& external_wrench_state) {
  

     static ros::Time last_publish_time = ros::Time::now(); 
    ros::Duration elapsed_time = ros::Time::now() - last_publish_time;

  
    if (elapsed_time.toSec() >= 0.03) {
        
        
        double min_z_force = 3.5;  
        double max_z_force = 5.0; 

        double z_force = external_wrench_state[3]; 
        double vibration_intensity = mapValue(z_force, min_z_force, max_z_force, 0.0, 1.0);

        std_msgs::Float64 vibration_msg;
        vibration_msg.data = vibration_intensity;
        vibration_pub.publish(vibration_msg);

        last_publish_time = ros::Time::now();
    } else {
        std_msgs::Float64 vibration_msg;
        vibration_msg.data = 0.0;
        vibration_pub.publish(vibration_msg);
    }
}



    void stop() {
        running_ = false;
    }

private:
        uint64_t counterl = 0;
        uint64_t counterr = 0;
        std::mutex mutex;
        std::array<double, 7> tau_d_array_r{};
        std::array<double, 6> stiffness_l_1{},stiffness_r_1{},damping_l_1{},damping_r_1{},damping_r{}, damping_l{},stiffness_r{}, stiffness_l{};
        std::array<double, 7> tau_d_array_l{};

        std::array<double, 6> Kx_r{};
        std::array<double, 6> Kx_l{};



          std::array<double, 7> tau_d_array1{};
          std::array<double, 7> tau_d_array2{};
          adaptive_cartesian::adaptive_cartesian cntr_cart_l, cntr_cart_r ;

    const double delta_tau_max_{1.0};
    double nullspace_stiffness_{20.0};

     
    double desiredFrequency=1000;  
    int delayDuration = static_cast<int>(1000.0 / desiredFrequency);

     std::string robot_right_hostname_; 
    std::string robot_left_hostname_;  
    bool running_;
    
    
                 std::array<double, 16> initial_pose1, desired_pose1;
    std::array<double, 16> initial_pose2, desired_pose2;
     tf::TransformListener tf_listener_r, tf_listener_l;
     tf::TransformBroadcaster tf_broadcaster_r , tf_broadcaster_l;
    Eigen::Matrix4d tf_matrix_r, tf_matrix_l;
    ros::NodeHandle& nh_;

    std::array<double, 7>  joint_positions_left, joint_positions_right  ;

    ros::Subscriber jointSubLeft,jointSubRight,viveRightSub,viveLeftSub;
    ros::Publisher vibration_pub , external_force_left_pub,external_force_right_pub,external_stiffness_left_pub,external_stiffness_right_pub,external_damping_left_pub,external_damping_right_pub;


    std::vector<int> receivedButtons, receivedButtons_l;

    bool buttonPressed = false; 
    bool centerButtonPressed = false;
    bool stopButtonPressed = false;


    std::array<double, 6> external_wrench_state_r, external_wrench_state_l;
    std::array<double, 7> force_with_time_r,force_with_time_l,stiffness_with_time_r,stiffness_with_time_l,damping_with_time_r,damping_with_time_l;

    double time_l=0;
    double time_r=0;
    

      
};

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot_right_-hostname> <robot_left_-hostname>" << std::endl;
        return -1;
    }

    std::string robot_right_hostname = argv[1];
    std::string robot_left_hostname = argv[2];

 
    ros::init(argc, argv, "dual_arm_controller");
    ros::NodeHandle nh;

    DualArmController controller(robot_right_hostname, robot_left_hostname, nh);
    controller.run();
   

    return 0;
}