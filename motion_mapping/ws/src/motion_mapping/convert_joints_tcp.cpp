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
#include <sstream>
#include <atomic>

#include "asio_service.h"
#include "server/asio/tcp_client.h"
#include "threads/thread.h"

#include <chrono>
#include <thread>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <stdio.h>
#include <std_msgs/Float32MultiArray.h>
/** Human elbow joint angle in radians */
/** Human elbow joint angle in radians */
/** Human elbow joint angle in radians */
std::vector<float> receivedMotionData;
std::vector<float> receivedMotionDataCopy;

/** Human elbow joint angle in radians */
class MotionCaptureClient : public CppServer::Asio::TCPClient
{
public:
    using CppServer::Asio::TCPClient::TCPClient;

    void DisconnectAndStop()
    {
        _stop = true;
        DisconnectAsync();
        while (IsConnected())
            CppCommon::Thread::Yield();
    }

protected:
    void onConnected() override
    {
        std::cout << "Motion capture TCP client connected a new session with Id " << id() << std::endl;
    }

    void onDisconnected() override
    {
        std::cout << "Motion capture TCP client disconnected a session with Id " << id() << std::endl;

        // Wait for a while...
        CppCommon::Thread::Sleep(1000);

        // Try to connect again
        if (!_stop)
            ConnectAsync();
    }

    int loopCounter = 0;
    void onReceived(const void* buffer, size_t size) override
    {
        // Process the received motion capture data as BVH
        bvh::Bvh_parser_2 parser;
        bvh::Bvh data;

        std::string bvhData((const char*)buffer, size); // Assuming you have BVH data in a string

        std::cout << "Received BVH data:" << std::endl;
        std::cout << bvhData << std::endl;
        std::cout << "Size of BVH data: " << bvhData.size() << std::endl;

        // Find the position of the first space character in the BVH data
        size_t firstSpacePos = bvhData.find(' ');

        // Find the position of the second space character in the BVH data
        size_t secondSpacePos = bvhData.find(' ', firstSpacePos + 1);

        // Check if both space positions are found
        if (firstSpacePos != std::string::npos && secondSpacePos != std::string::npos)
        {
            // Remove the first and second elements from the BVH data
            bvhData.erase(0, secondSpacePos + 1);
        }

        size_t trailingPos = bvhData.find("||");
        if (trailingPos != std::string::npos)
        {
            bvhData.erase(trailingPos);
        }

        // Use the modified BVH data for further processing
        std::cout << "Modified BVH data:" << std::endl;
        std::cout << bvhData << std::endl;
        std::cout << "Size of modified BVH data: " << bvhData.size() << std::endl;

        // Extract motion data from bvhData into a vector or array
        std::vector<float> motionData;
        std::stringstream ss(bvhData);
std::string value;
while (getline(ss, value, ' ')) {
    if (!value.empty()) {
        motionData.push_back(std::stof(value));
    }
}
        // ...

        receivedMotionData.clear();
        for (size_t i = 0; i < motionData.size(); ++i) {
            receivedMotionData.push_back(motionData[i]);
        }
    }

    void onError(int error, const std::string& category, const std::string& message) override
    {
        std::cout << "Motion capture TCP client caught an error with code " << error << " and category '" << category << "': " << message << std::endl;
    }

private:
    std::atomic<bool> _stop{ false };
};

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "motion_capture_node");
    ros::NodeHandle nh;
    ros::Publisher motionDataPublisher = nh.advertise<std_msgs::Float32MultiArray>("motion_data_topic", 10);

    // Motion capture server address
    std::string address = "172.23.255.222";
    if (argc > 1)
        address = argv[1];

    // Motion capture server port
    int port = 7002;
    if (argc > 2)
        port = std::atoi(argv[2]);

    std::cout << "Motion capture server address: " << address << std::endl;
    std::cout << "Motion capture server port: " << port << std::endl;

    // Create a new Asio service
    auto service = std::make_shared<AsioService>();

    // Start the Asio service
    std::cout << "Asio service starting...";
    service->Start();
    std::cout << "Done!" << std::endl;

    // Create a new TCP motion capture client
    auto client = std::make_shared<MotionCaptureClient>(service, address, port);

    // Connect the client
    std::cout << "Client connecting...";
    client->ConnectAsync();
    std::cout << "Done!" << std::endl;

    std::cout << "Press Enter to stop the client..." << std::endl;
    
    ros::Rate rate(1000);
    while (ros::ok()) {
        

        // Publish the received motion data
       /* std_msgs::Float32MultiArray motionDataMsg;
        motionDataMsg.data = receivedMotionData;
        motionDataPublisher.publish(motionDataMsg);*/



        receivedMotionDataCopy.resize(receivedMotionData.size());
    std::memcpy(receivedMotionDataCopy.data(), receivedMotionData.data(), receivedMotionData.size() * sizeof(float));

    // Publish the copied motion data
    std_msgs::Float32MultiArray motionDataMsg;
    motionDataMsg.data = receivedMotionDataCopy;
    motionDataPublisher.publish(motionDataMsg);
        // Add a delay to control the publishing rate
        //std::this_thread::sleep_for(std::chrono::milliseconds(16));
       
        rate.sleep();
         ros::spinOnce();
    }

    // Disconnect the client
    std::cout << "Client disconnecting...";
    client->DisconnectAndStop();
    std::cout << "Done!" << std::endl;

    // Stop the Asio service
    std::cout << "Asio service stopping...";
    service->Stop();
    std::cout << "Done!" << std::endl;

    return 0;
}