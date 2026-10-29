#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>
#include <cmath>
#include <string_view>

using namespace std::chrono_literals;
class MotionGuard : public rclcpp::Node {
 public:
  MotionGuard():Node("motion_guard") {
    timeout_=declare_parameter("heartbeat_timeout_s",.6);
    command_timeout_=declare_parameter("command_timeout_s",.4);
    max_linear_=declare_parameter("max_linear",.35); max_angular_=declare_parameter("max_angular",.8);
    pub_=create_publisher<geometry_msgs::msg::TwistStamped>("/diff_drive_controller/cmd_vel",rclcpp::QoS(1));
    status_=create_publisher<std_msgs::msg::Bool>("/motion_guard/enabled",rclcpp::QoS(1).reliable().transient_local());
    heartbeat_=create_subscription<std_msgs::msg::Empty>("/mission/heartbeat",10,[this](std_msgs::msg::Empty::ConstSharedPtr){ heartbeat_wall_=wall(); });
    scan_=create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::LaserScan::ConstSharedPtr m){ scan_wall_=wall(); scan_stamp_=rclcpp::Time(m->header.stamp).seconds(); });
    odom_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){ odom_wall_=wall(); odom_stamp_=rclcpp::Time(m->header.stamp).seconds(); });
    cmd_=create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel_nav",rclcpp::QoS(1),[this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){
      double age=now().seconds()-rclcpp::Time(m->header.stamp).seconds();
      bool finite=std::isfinite(m->twist.linear.x)&&std::isfinite(m->twist.linear.y)&&std::isfinite(m->twist.linear.z)&&
        std::isfinite(m->twist.angular.x)&&std::isfinite(m->twist.angular.y)&&std::isfinite(m->twist.angular.z);
      if(!enabled_) return;
      const bool health_ok=healthy();
      if(!health_ok || !finite) {
        RCLCPP_WARN(get_logger(),"input gate health=%d finite=%d scan_sim_age=%.6f odom_sim_age=%.6f clock_idle=%.6f",
          health_ok,finite,now().seconds()-scan_stamp_,now().seconds()-odom_stamp_,wall()-advance_wall_);
        disarm("HEALTH_OR_NONFINITE"); return;
      }
      bool nonzero=std::abs(m->twist.linear.x)>1e-6 || std::abs(m->twist.angular.z)>1e-6;
      if(nonzero && (age<-.05 || age>command_timeout_)) { disarm("COMMAND_STAMP"); return; }
      if(std::abs(m->twist.linear.x)>max_linear_+.01 || std::abs(m->twist.angular.z)>max_angular_+.01 ||
         std::abs(m->twist.linear.y)>1e-6 || std::abs(m->twist.linear.z)>1e-6 ||
         std::abs(m->twist.angular.x)>1e-6 || std::abs(m->twist.angular.y)>1e-6) { disarm("VELOCITY_LIMIT"); return; }
      command_wall_=wall(); seen_command_=true; last_nonzero_=nonzero;
      if(nonzero) pub_->publish(*m);  // Preserve original nonzero command stamp.
      else zero();  // Explicit stop is safe even when upstream gives it stamp=0.
    });
    service_=create_service<std_srvs::srv::SetBool>("/motion_guard/set_enabled",[this](std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr resp){
      if(!req->data) { disarm("MISSION_DISABLE"); resp->success=true; resp->message="disabled"; return; }
      enabled_=healthy(); command_wall_=wall(); enabled_wall_=wall(); seen_command_=false;
      resp->success=enabled_; resp->message=enabled_?"enabled":"clock/sensor/heartbeat freshness gate";
      publish_status();
    });
    timer_=create_wall_timer(50ms,[this]{
      double sim=now().seconds();
      if(sim<last_sim_-.001) disarm("CLOCK_RESET");
      if(sim>last_sim_+1e-9) advance_wall_=wall();
      last_sim_=sim;
      if(enabled_ && !healthy()) disarm("HEARTBEAT_SENSOR_OR_CLOCK");
      if(enabled_ && seen_command_ && wall()-command_wall_>command_timeout_) {
        if(last_nonzero_) disarm("NONZERO_COMMAND_TIMEOUT");
        else zero();  // Nav2 recovery Wait is allowed to remain stopped.
      }
      if(enabled_ && !seen_command_ && wall()-enabled_wall_>5.0) disarm("FIRST_COMMAND_TIMEOUT");
      if(!enabled_) zero();
      publish_status();
    });
  }
 private:
  static double wall() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
  bool healthy() const {
    double w=wall(),s=get_clock()->now().seconds();
    return s>0 && w-heartbeat_wall_<timeout_ && w-advance_wall_<.6 && w-scan_wall_<1 && w-odom_wall_<.5 &&
      s-scan_stamp_>=-.05 && s-scan_stamp_<1 && s-odom_stamp_>=-.05 && s-odom_stamp_<.5;
  }
  void zero() { geometry_msgs::msg::TwistStamped z; z.header.stamp=now(); z.header.frame_id="base_footprint"; pub_->publish(z); }
  void publish_status() { std_msgs::msg::Bool s; s.data=enabled_; status_->publish(s); }
  void disarm(const char *reason) {
    if(enabled_ && std::string_view(reason)!="MISSION_DISABLE")
      RCLCPP_WARN(get_logger(),"disarmed: %s (heartbeat_age=%.3f scan_age=%.3f odom_age=%.3f command_age=%.3f)",
        reason,wall()-heartbeat_wall_,wall()-scan_wall_,wall()-odom_wall_,wall()-command_wall_);
    enabled_=false; zero(); publish_status();
  }
  bool enabled_{false},seen_command_{false},last_nonzero_{false};
  double timeout_,command_timeout_,max_linear_,max_angular_;
  double heartbeat_wall_{-1e12},scan_wall_{-1e12},odom_wall_{-1e12},command_wall_{-1e12},advance_wall_{-1e12};
  double scan_stamp_{0},odom_stamp_{0},last_sim_{0},enabled_wall_{0};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<MotionGuard>()); rclcpp::shutdown(); return 0;
}
