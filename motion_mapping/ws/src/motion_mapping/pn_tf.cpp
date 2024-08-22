#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>

#include <chrono> 
#include <std_msgs/Float32MultiArray.h>
#include <ros/callback_queue.h>

class NeuronSubscriber {
public:
  NeuronSubscriber(ros::NodeHandle& nh) : nh_(nh),rate_(300) {
    std::string topic_name = "/motion_data_topic";
    subscriber_ = nh_.subscribe<std_msgs::Float32MultiArray>(
        topic_name, 5, boost::bind(&NeuronSubscriber::callback, this, _1));

    tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher tf_pub_;
  }

  std::vector<std::string> link_children_names_ = {
    "Hips","RightUpLeg","RightLeg","RightFoot","LeftUpLeg","LeftLeg",
    "LeftFoot","Spine","Spine1","Spine2","Spine3","Neck","Head",
    "RightShoulder","RightArm","RightForeArm","RightHand",
    "RightHandThumb1","RightHandThumb2","RightHandThumb3",
    "RightInHandIndex","RightHandIndex1","RightHandIndex2",
    "RightHandIndex3","RightInHandMiddle","RightHandMiddle1",
    "RightHandMiddle2","RightHandMiddle3","RightInHandRing",
    "RightHandRing1","RightHandRing2","RightHandRing3","RightInHandPinky",
    "RightHandPinky1","RightHandPinky2","RightHandPinky3","LeftShoulder",
    "LeftArm","LeftForeArm","LeftHand","LeftHandThumb1","LeftHandThumb2",
    "LeftHandThumb3","LeftInHandIndex","LeftHandIndex1","LeftHandIndex2",
    "LeftHandIndex3","LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2",
    "LeftHandMiddle3","LeftInHandRing","LeftHandRing1","LeftHandRing2",
    "LeftHandRing3","LeftInHandPinky","LeftHandPinky1","LeftHandPinky2",
    "LeftHandPinky3"
  };

  std::vector<std::string> link_parents_names_ ={
    "world","Hips","RightUpLeg","RightLeg","Hips",
    "LeftUpLeg","LeftLeg","Hips","Spine","Spine1","Spine2","Spine3","Neck",
    "Spine3","RightShoulder","RightArm","RightForeArm","RightHand",
    "RightHandThumb1","RightHandThumb2","RightHand","RightInHandIndex",
    "RightHandIndex1","RightHandIndex2","RightHand","RightInHandMiddle",
    "RightHandMiddle1","RightHandMiddle2","RightHand","RightInHandRing",
    "RightHandRing1","RightHandRing2","RightHand","RightInHandPinky",
    "RightHandPinky1","RightHandPinky2","Spine3","LeftShoulder","LeftArm",
    "LeftForeArm","LeftHand","LeftHandThumb1","LeftHandThumb2",
    "LeftHand","LeftInHandIndex","LeftHandIndex1","LeftHandIndex2",
    "LeftHand","LeftInHandMiddle","LeftHandMiddle1","LeftHandMiddle2",
    "LeftHand","LeftInHandRing","LeftHandRing1","LeftHandRing2",
    "LeftHand","LeftInHandPinky","LeftHandPinky1","LeftHandPinky2"
  };

  void sendStaticTransform() {
    tf::Transform world_frame;
    world_frame.setOrigin(tf::Vector3(0, 0, 0));
    world_frame.setRotation(tf::Quaternion(0.70711, 0, 0, 0.70711));
    tf_broadcaster_->sendTransform(tf::StampedTransform(
        world_frame, ros::Time::now(), "world", "WorldPerceptionNeuron"));
  }

  ~NeuronSubscriber() {}

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
    ros::Rate rate_;

 // std::vector<std::string> link_children_names_, link_parents_names_;
  std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;

  int tfFrameID = 0;

  void eulerToQuaternion(float eulerY, float eulerX, float eulerZ,
                         tf::Quaternion& q) {
    Eigen::Matrix3f rxyz, rx, ry, rz;

    rx = Eigen::AngleAxisf(eulerX * M_PI / 180, Eigen::Vector3f::UnitX());
    ry = Eigen::AngleAxisf(eulerY * M_PI / 180, Eigen::Vector3f::UnitY());
    rz = Eigen::AngleAxisf(eulerZ * M_PI / 180, Eigen::Vector3f::UnitZ());

    // Check Ordering in Axis Neuron->Settings->Output Format! Here = YXZ
    rxyz = ry * rx *rz;

    Eigen::Quaternionf qf(rxyz);

    q.setW(qf.w());
    q.setX(qf.x());
    q.setY(qf.y());
    q.setZ(qf.z());
  }

  void callback(const std_msgs::Float32MultiArrayConstPtr& bone_data) {
    ROS_INFO_STREAM("Subscribed values:");
    // ros::Rate rate(40);
    uint startIdx = 0;
    for (uint joint_index = 0; joint_index < bone_data->data.size() / 6; joint_index++) {
      uint startIdx = joint_index * 6;

      tf::Transform pose;
      float eulerY, eulerX, eulerZ;
      tf::Quaternion rotation;
      tf::Vector3 position;

      position.setX(0.01 * bone_data->data[startIdx]);     
      position.setY(0.01 * bone_data->data[startIdx + 1]);
      position.setZ(0.01 * bone_data->data[startIdx + 2]);

      eulerY = bone_data->data[startIdx + 3];
      eulerX = bone_data->data[startIdx + 4];
      eulerZ = bone_data->data[startIdx + 5];


      eulerToQuaternion(eulerY, eulerX, eulerZ, rotation);

      pose.setOrigin(position);
      pose.setRotation(rotation);

     if (joint_index == 0) {
        pose.setOrigin(tf::Vector3(0.6, 0, 0));
        rotation.setRPY(M_PI / 2,0, M_PI_2 ); 
        pose.setRotation(rotation);
      }
      
      tf_broadcaster_->sendTransform(tf::StampedTransform(
          pose, ros::Time::now(), link_parents_names_.at(joint_index),
          link_children_names_.at(joint_index)));
     
      ROS_INFO_STREAM("Transform: "
                    << "Parent: " << link_parents_names_.at(joint_index) << ", "
                    << "Child: " << link_children_names_.at(joint_index));

      ROS_INFO_STREAM("Bone " << joint_index << ":");
      ROS_INFO_STREAM("  Position: (" << position.getX() << ", "
                                      << position.getY() << ", "
                                      << position.getZ() << ")");
      ROS_INFO_STREAM("  Rotation: (" << rotation.getW() << ", "
                                      << rotation.getX() << ", "
                                      << rotation.getY() << ", "
                                      << rotation.getZ() << ")");


                                   // rate_.sleep(); 
    }
  }
};

int main(int argc, char** argv) {
  ROS_INFO_STREAM("Started NeuronSubscriber");

  ros::init(argc, argv, "neuron_subscriber_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  

  NeuronSubscriber neuronSubscriber(nh);
// neuronSubscriber.sendStaticTransform();
  ros::spin();

  ROS_INFO_STREAM("Bye!");
  return 0;
}
