
#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES
#include <iostream>
#include "joint.h"
#include "bvh-parser_2.h"
#include "bvh.h"
#include "utils.h"
#include <boost/filesystem.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <vector>
#include <array>
#include "franka_ik_He.hpp"
#include <cmath>
namespace bf = boost::filesystem;

/** Human elbow joint angle in radius*/
float getDegAngle(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
    glm::vec3 v1 = glm::normalize(p1 - glm::vec3(0));
    glm::vec3 v2 = glm::normalize(p3 - p2);
    return(glm::acos(glm::dot(v1, v2)));
}

/**  Cartesian pose of the end effect-or*/
glm::mat4 get_O_T_EE(bvh::Bvh data, std::shared_ptr<bvh::Joint> start_joint, unsigned frame) {

    glm::mat4 O_T_EE(1.0);
    for (int i = 0; i < 50; i++) {
        O_T_EE = start_joint->ltm(frame)
            * start_joint->children()[0]->ltm(frame)
            * start_joint->children()[0]->children()[0]->ltm(frame);
    }
    return O_T_EE;
}

/**  convert glm::mat4 to std::array*/
std::array<double, 16> O_T_EE_array(glm::mat4 O_T_EE) {
    std::array<double, 16> O_T_EE_array;
    const float* pSource = (const float*)glm::value_ptr(O_T_EE);
    for (int i = 0; i < 16; i++) {
        O_T_EE_array[i] = pSource[i];
    }
    return O_T_EE_array;

}

int main()
{

    bvh::Bvh_parser_2 parser;
    bvh::Bvh data;
    bf::path sample_path = bf::path("/home/muhsin/workspaces/cpp_server/ws/src/motion_mapping/17N in Tristan_s OfficeAvatar00.bvh");
    //File path
    parser.parse(sample_path, &data);
    int frame = data.num_frames();
    unsigned channel = data.num_channels();

    std::cout << "Received BVH data:" << std::endl;
    std::cout << "Number of frames: " << frame << std::endl;
    std::cout << "Number of channel: " << channel << std::endl;
    std::cout << "Number of joints: " << data.joints().size() << std::endl;

    std::array<glm::mat4, 50> O_T_EE; // get Cartesian pose of the end effect-or of 50 frames.
    data.joints()[17]->set_parent(NULL);//data.joints()[17] is RightArm.
    data.recalculate_joints_ltm(data.joints()[17]);
    for (int i = 0; i < 50; i++) {
        O_T_EE[i] = get_O_T_EE(data, data.joints()[17], i);
    }


    std::cout << "O_T_EE values:" << std::endl;
    for (int i = 0; i < 50; i++) {
        std::cout << "Frame " << i << ":" << std::endl;
        for (int row = 0; row < 4; row++) {
            for (int col = 0; col < 4; col++) {
                std::cout << O_T_EE[i][row][col] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }



    //bvh::Joint* rightArmJoint = data.joints()[17];

    // Check if the joint exists and print its data
   /* if (rightArmJoint != nullptr)
    {
        std::cout << "Joint Name: " << rightArmJoint->name() << std::endl;

        // Print additional information as needed
        // Example: Print the joint's local transformation matrix (LTM)
        bvh::Matrix4 ltm = rightArmJoint->ltm();
        std::cout << "Local Transformation Matrix (LTM):" << std::endl;
        for (int row = 0; row < 4; ++row)
        {
            for (int col = 0; col < 4; ++col)
            {
                std::cout << ltm(row, col) << " ";
            }
            std::cout << std::endl;
        }
    }
    else
    {
        std::cout << "RightArm joint does not exist." << std::endl;
    }

    */
    /**  BVH data of 50 frames*/
    /* for (int i = 0; i < 50; i++) {
        std::vector<std::vector<double>> rightArm = data.joints()[17]->channel_data();
        for (int j = 0; j < 6; j++) {
            std::cout << rightArm[i][j] << ", ";

        }
        std::cout << "\n";
        std::vector<std::vector<double>> rightForeArm = data.joints()[18]->channel_data();
        for (int j = 0; j < 6; j++) {
            std::cout << rightForeArm[i][j] << ", ";

        }
        std::cout << "\n";
        std::vector<std::vector<double>> rightHand = data.joints()[19]->channel_data();
        for (int j = 0; j < 6; j++) {
            std::cout << rightHand[i][j] << ", ";

        }
        std::cout << "\n";
        std::cout << "\n";
    } */


    /* std::cout << "Right Arm Data:\n";
for (int i = 0; i < 50; i++) {
    std::vector<std::vector<double>> rightArm = data.joints()[17]->channel_data();
    for (int j = 0; j < 6; j++) {
        std::cout << rightArm[i][j] << ", ";
    }
    std::cout << "\n";
}

std::cout << "\nRight Forearm Data:\n";
for (int i = 0; i < 50; i++) {
    std::vector<std::vector<double>> rightForeArm = data.joints()[18]->channel_data();
    for (int j = 0; j < 6; j++) {
        std::cout << rightForeArm[i][j] << ", ";
    }
    std::cout << "\n";
}

std::cout << "\nRight Hand Data:\n";
for (int i = 0; i < 50; i++) {
    std::vector<std::vector<double>> rightHand = data.joints()[19]->channel_data();
    for (int j = 0; j < 6; j++) {
        std::cout << rightHand[i][j] << ", ";
    }
    std::cout << "\n";
}

std::cout << "\n"; */


    for (int i = 0; i < 50; i++) {
       // std::cout << "Iteration " << i + 1 << ":\n";

        std::vector<std::vector<double>> rightArm = data.joints()[17]->channel_data();
        //std::cout << "Right Arm Data: ";
        for (int j = 0; j < 6; j++) {
           // std::cout << rightArm[i][j] << ", ";
        }
        std::cout << "\n";

        std::vector<std::vector<double>> rightForeArm = data.joints()[18]->channel_data();
       // std::cout << "Right Forearm Data: ";
        for (int j = 0; j < 6; j++) {
          //  std::cout << rightForeArm[i][j] << ", ";
        }
        std::cout << "\n";

        std::vector<std::vector<double>> rightHand = data.joints()[19]->channel_data();
      //  std::cout << "Right Hand Data: ";
        for (int j = 0; j < 6; j++) {
          //  std::cout << rightHand[i][j] << ", ";
        }
        std::cout << "\n";

        std::cout << "\n";
    }



    /**  Panda's joint configuration of 50 frames*/
    for (int i = 0; i < 50; i++) {
        std::array<double, 16> T_EE = O_T_EE_array(O_T_EE[i]);
        std::array<double, 7> actual = { 0,0,0,0,0,0,0 };
        std::array< std::array<double, 7>, 4 > IK_EE;
        int flag = 1;
        float k = -2.8;
        while (flag) {
            IK_EE = franka_IK_EE(T_EE, k, actual);
            flag = 0;
            for (auto i : IK_EE) {
                for (auto j : i) {
                    if (std::isnan(j))
                        flag = 1;
                }
            }
            k = k + 0.1;
            if (k == 2.9)
                flag = 0;
        }
        glm::vec3 pos_RightArm = O_T_EE[i][3];
        glm::vec3 pos_RightForeArm = data.joints()[18]->ltm(i)[3];
        glm::vec3 pos_RightHand = data.joints()[19]->ltm(i)[3];
        float humanElbow = getDegAngle(pos_RightArm, pos_RightForeArm, pos_RightHand);
        float min_error = fabs(fabs(humanElbow) - fabs(IK_EE[0][3]));
        int m = 0;
        for (int i = 1; i < 4; i++) {
            float error = fabs(fabs(humanElbow) - fabs(IK_EE[i][3]));
            if (min_error > error) {
                min_error = error;
                m = i;
            }
        }

       // std::cout << "IK Solution for Iteration " << i + 1 << ":\n";
        for (int i = 0; i < 7; i++) {
          //  std::cout << IK_EE[0][i];
            std::cout << "\n";
        }
        std::cout << "\n";
    }

}
