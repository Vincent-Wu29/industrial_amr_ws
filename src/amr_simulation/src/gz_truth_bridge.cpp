#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <set>

// Scene Pose_V carries the entity name in pose.name, whereas the generic TF
// bridge expects frame metadata. Preserve the name and vector's source stamp.
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("gz_truth_bridge");
  auto topic=node->declare_parameter<std::string>("source_topic","/amr/evaluation/poses");
  auto model=node->declare_parameter<std::string>("model_name","amr");
  auto publisher=node->create_publisher<tf2_msgs::msg::TFMessage>("/ground_truth/poses",rclcpp::QoS(5));
  auto logger=node->get_logger();
  gz::transport::Node transport;
  bool subscribed=transport.Subscribe<gz::msgs::Pose_V>(topic,[publisher,model,logger](const gz::msgs::Pose_V &input){
    if(!rclcpp::ok()) return;
    const std::set<std::string> names{model,"scenario_box","enclosure_west","enclosure_east","enclosure_north","enclosure_south"};
    tf2_msgs::msg::TFMessage output;
    for(const auto &pose:input.pose()) {
      if(!names.count(pose.name())) continue;
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id="world"; transform.child_frame_id=pose.name();
      transform.header.stamp.sec=static_cast<int32_t>(input.header().stamp().sec());
      transform.header.stamp.nanosec=static_cast<uint32_t>(input.header().stamp().nsec());
      transform.transform.translation.x=pose.position().x();
      transform.transform.translation.y=pose.position().y();
      transform.transform.translation.z=pose.position().z();
      transform.transform.rotation.x=pose.orientation().x();
      transform.transform.rotation.y=pose.orientation().y();
      transform.transform.rotation.z=pose.orientation().z();
      transform.transform.rotation.w=pose.orientation().w();
      output.transforms.push_back(transform);
    }
    if(!output.transforms.empty()) {
      try { publisher->publish(output); }
      catch(const rclcpp::exceptions::RCLError &error) {
        if(rclcpp::ok()) RCLCPP_ERROR(logger,"truth publish failed: %s",error.what());
      }
    }
  });
  if(!subscribed) { RCLCPP_FATAL(logger,"cannot subscribe to Gazebo truth"); rclcpp::shutdown(); return 1; }
  RCLCPP_INFO(logger,"Evaluation-only truth source: %s, model=%s",topic.c_str(),model.c_str());
  rclcpp::spin(node);
  transport.Unsubscribe(topic);
  rclcpp::shutdown();
  return 0;
}
