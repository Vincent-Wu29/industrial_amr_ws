#include "amr_mission/mission_core.hpp"
#include "amr_mission/station_registry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <amr_interfaces/action/execute_transport.hpp>
#include <amr_interfaces/msg/task_snapshot.hpp>
#include <amr_interfaces/msg/mission_event.hpp>
#include <amr_interfaces/srv/get_transport.hpp>
#include <amr_interfaces/srv/get_station.hpp>
#include <amr_interfaces/srv/confirm_operation.hpp>
#include <amr_interfaces/srv/reconcile_load.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <random>
#include <sstream>

using namespace std::chrono_literals;
using Transport=amr_interfaces::action::ExecuteTransport;
using Nav=nav2_msgs::action::NavigateToPose;
using Snapshot=amr_interfaces::msg::TaskSnapshot;
using Confirm=amr_interfaces::srv::ConfirmOperation;
using Reconcile=amr_interfaces::srv::ReconcileLoad;
using TransportHandle=rclcpp_action::ServerGoalHandle<Transport>;
using NavHandle=rclcpp_action::ClientGoalHandle<Nav>;
namespace {
double wall() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
std::string uuid_string(const rclcpp_action::GoalUUID &u) {
  std::ostringstream out; for(auto b:u) out<<std::hex<<std::setfill('0')<<std::setw(2)<<static_cast<int>(b); return out.str();
}
std::string new_run() {
  std::ostringstream out; out<<std::hex<<std::chrono::system_clock::now().time_since_epoch().count()<<'-'<<std::random_device{}(); return out.str();
}
std::string json(const std::string &s) {
  std::ostringstream out; out<<'"';
  for(unsigned char c:s) { if(c=='"'||c=='\\') out<<'\\'<<c; else if(c<32) out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<static_cast<int>(c); else out<<c; }
  out<<'"'; return out.str();
}
std::string number(double v) { if(!std::isfinite(v)) return "null"; std::ostringstream s; s<<std::setprecision(12)<<v; return s.str(); }
}

class MissionNode : public rclcpp::Node {
 public:
  MissionNode():Node("mission_manager") {
    auto stations_file=declare_parameter<std::string>("stations_file","");
    auto map_file=declare_parameter<std::string>("map_file","");
    bt_=declare_parameter<std::string>("behavior_tree","");
    clean_world_=declare_parameter("reset_world_empty",false);
    auto padding=declare_parameter("footprint_padding",.12);
    registry_=std::make_unique<amr::StationRegistry>(stations_file,map_file,padding);
    amr::Limits limits;
    limits.nav_sim=declare_parameter("nav_timeout_s",180.0);
    limits.task_sim=declare_parameter("task_timeout_s",420.0);
    if(!std::isfinite(limits.nav_sim) || !std::isfinite(limits.task_sim) || limits.nav_sim<=0 ||
       limits.task_sim<limits.nav_sim || limits.task_sim>900) throw std::runtime_error("invalid navigation/task time budgets");
    core_=std::make_unique<amr::MissionCore>(new_run(),registry_->stations(),limits);
    auto artifacts=declare_parameter<std::string>("artifacts_dir","artifacts");
    auto dir=std::filesystem::path(artifacts)/core_->run_id(); std::filesystem::create_directories(dir);
    log_.open(dir/"events.jsonl",std::ios::out|std::ios::app);
    if(!log_) throw std::runtime_error("cannot open event log; motion remains disabled");
    std::ofstream manifest(dir/"manifest.yaml");
    manifest<<"run_id: "<<core_->run_id()<<"\nros_distro: jazzy\nmap_file: "<<json(map_file)
      <<"\nstations_file: "<<json(stations_file)<<"\nreset_world_empty: "<<(clean_world_?"true":"false")<<"\n";
    if(!manifest) throw std::runtime_error("cannot write manifest");
    buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_shared<tf2_ros::TransformListener>(*buffer_,this,false);
    nav_=rclcpp_action::create_client<Nav>(this,"/navigate_to_pose");
    guard_=create_client<std_srvs::srv::SetBool>("/motion_guard/set_enabled");
    state_pub_=create_publisher<Snapshot>("/mission/state",rclcpp::QoS(1).reliable().transient_local());
    events_pub_=create_publisher<amr_interfaces::msg::MissionEvent>("/mission/events",rclcpp::QoS(100).reliable());
    heartbeat_=create_publisher<std_msgs::msg::Empty>("/mission/heartbeat",10);
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",10);
    ready_sub_=create_subscription<std_msgs::msg::Bool>("/system/ready",10,[this](std_msgs::msg::Bool::ConstSharedPtr m){ ready_=m->data; ready_wall_=wall(); });
    guard_sub_=create_subscription<std_msgs::msg::Bool>("/motion_guard/enabled",rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::Bool::ConstSharedPtr m){ guard_enabled_=m->data; guard_wall_=wall(); });
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){
      double s=rclcpp::Time(m->header.stamp).seconds();
      bool still=std::isfinite(m->twist.twist.linear.x) && std::isfinite(m->twist.twist.angular.z) &&
        std::abs(m->twist.twist.linear.x)<.02 && std::abs(m->twist.twist.linear.y)<.02 && std::abs(m->twist.twist.angular.z)<.05;
      if(!still || s<odom_stamp_ || wall()-odom_wall_>.5) stopped_since_=-1;
      if(still && stopped_since_<0) stopped_since_=s;
      odom_stamp_=s; odom_wall_=wall();
    });
    query_=create_service<amr_interfaces::srv::GetTransport>("/mission/get_transport",[this](amr_interfaces::srv::GetTransport::Request::SharedPtr req,amr_interfaces::srv::GetTransport::Response::SharedPtr res){
      auto s=core_->find(req->task_id); res->found=s.has_value(); if(s) res->snapshot=message(*s);
    });
    station_query_=create_service<amr_interfaces::srv::GetStation>("/stations/get_station",[this](amr_interfaces::srv::GetStation::Request::SharedPtr req,amr_interfaces::srv::GetStation::Response::SharedPtr res){
      auto it=registry_->stations().find(req->station_id); res->found=it!=registry_->stations().end();
      if(res->found) { res->type=it->second.type; res->pose=pose(it->second); res->map_id=registry_->map_id(); res->has_approach=false; }
    });
    confirm_=create_service<Confirm>("/mission/confirm_operation",[this](Confirm::Request::SharedPtr req,Confirm::Response::SharedPtr res){
      auto r=core_->confirm(req->run_id,req->task_id,req->station_id,req->interaction_id,req->operation,req->outcome,observation());
      res->accepted=r.accepted; res->reason_code=r.code; res->message=r.message;
      if(!r.accepted) admission("confirmation_rejected",req->task_id,"",r);
      drain();
    });
    reconcile_=create_service<Reconcile>("/mission/reconcile_load",[this](Reconcile::Request::SharedPtr req,Reconcile::Response::SharedPtr res){
      amr::Reply r{false,amr::InvalidRequest,"run_id and operator_note required"};
      if(restart_required_) r={false,amr::NotReady,restart_reason_+"; full stack restart required"};
      else if(req->run_id==core_->run_id() && !req->operator_note.empty()) r=core_->reconcile(static_cast<amr::Load>(req->load_state),observation());
      res->accepted=r.accepted; res->reason_code=r.code; res->message=r.message;
      admission("reconcile",core_->current().request.id,"",r); drain();
    });
    server_=rclcpp_action::create_server<Transport>(this,"/mission/execute_transport",
      [this](const auto &uuid,auto goal){
        auto r=restart_required_?amr::Reply{false,amr::NotReady,restart_reason_+"; full stack restart required"}:
          core_->admit({goal->task_id,goal->source_station,goal->target_station,uuid_string(uuid)},observation());
        admission(r.accepted?"admission_accepted":"admission_rejected",goal->task_id,uuid_string(uuid),r);
        // drain on accepted callback: never run side effects before goal ownership exists.
        if(!r.accepted) drain();
        return r.accepted?rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE:rclcpp_action::GoalResponse::REJECT;
      },
      [this](auto handle){
        auto r=core_->cancel(handle->get_goal()->task_id,observation());
        admission("cancel_requested",handle->get_goal()->task_id,uuid_string(handle->get_goal_id()),r);
        // timer drains after rclcpp_action has changed the goal to CANCELING.
        return r.accepted?rclcpp_action::CancelResponse::ACCEPT:rclcpp_action::CancelResponse::REJECT;
      },
      [this](auto handle){ transport_=handle; drain(); });
    timer_=create_wall_timer(200ms,[this]{
      if(restart_required_ && !core_->active()) {
        if(exit_deadline_<0) exit_deadline_=wall()+.5;
        else if(wall()>=exit_deadline_) throw std::runtime_error(restart_reason_+": task ended; full stack reinitialization required");
      }
      heartbeat_->publish(std_msgs::msg::Empty{});
      auto o=observation();
      if(o.sim+1e-6<last_observed_sim_) {
        restart_required_=true;
        restart_reason_="CLOCK_RESET";
        auto snapshot=core_->current(); snapshot.error=amr::ClockReset;
        record(snapshot,"clock_reset","simulation context invalid; shutdown after bounded task cleanup");
      }
      last_observed_sim_=o.sim;
      if(clean_world_ && !bootstrapped_ && o.ready && o.stopped && o.guard_disabled) {
        auto r=core_->reconcile(amr::Load::Empty,o); bootstrapped_=r.accepted;
      }
      core_->tick(o);  // Clock reset and global gates take precedence over guard symptoms.
      if(core_->active() && (core_->current().state==amr::State::ToPickup || core_->current().state==amr::State::ToDropoff) &&
          !guard_enabled_ && wall()-guard_wall_<.5 && !enabling_ && guard_was_enabled_) {
        auto snapshot=core_->current(); snapshot.error=amr::NotReady;
        record(snapshot,"motion_context_invalidated","unexpected guard disarm; full stack reset required");
        core_->invalidate_context(amr::NotReady,"motion guard disarmed during navigation; full stack reset required",o);
      }
      drain(); publish_state(); publish_diagnostic(o);
    });
    RCLCPP_INFO(get_logger(),"Mission run %s: startup HOLD; reset_world_empty=%s",core_->run_id().c_str(),clean_world_?"true":"false");
  }
 private:
  geometry_msgs::msg::PoseStamped pose(const amr::Station &s) {
    geometry_msgs::msg::PoseStamped p; p.header.frame_id="map"; p.header.stamp=now();
    p.pose.position.x=s.x; p.pose.position.y=s.y; p.pose.orientation.z=std::sin(s.yaw/2); p.pose.orientation.w=std::cos(s.yaw/2); return p;
  }
  amr::Observation observation() {
    double sim=now().seconds(),w=wall();
    amr::Observation o{sim,w,ready_ && w-ready_wall_<1 && w-guard_wall_<.6 && sim>0 && nav_->action_server_is_ready() && guard_->service_is_ready(),
      stopped_since_>=0 && sim-stopped_since_>=.5 && w-odom_wall_<.5 && sim-odom_stamp_<.5 && sim-odom_stamp_>=-.05,
      false,!guard_enabled_ && w-guard_wall_<.6};
    const auto &id=core_->current().station;
    if(!id.empty()) {
      try {
        auto tf=buffer_->lookupTransform("map","base_footprint",tf2::TimePointZero);
        double age=sim-rclcpp::Time(tf.header.stamp).seconds();
        const auto &s=registry_->stations().at(id); const auto &q=tf.transform.rotation;
        double yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        o.docked=age>=-.1 && age<1.5 && std::hypot(tf.transform.translation.x-s.x,tf.transform.translation.y-s.y)<=.18 && std::abs(std::remainder(yaw-s.yaw,2*M_PI))<=.18;
      } catch(const std::exception &) {}
    }
    return o;
  }
  Snapshot message(const amr::Snapshot &s) {
    Snapshot m; m.run_id=core_->run_id(); m.task_id=s.request.id; m.state=static_cast<uint8_t>(s.state);
    m.source_station=s.request.source; m.target_station=s.request.target; m.current_station=s.station;
    m.interaction_id=s.interaction; m.load_state=static_cast<uint8_t>(s.load); m.error_code=s.error;
    m.message=s.message; m.requires_operator=s.requires_operator;
    auto sequence=task_event_sequences_.find(s.request.id);
    m.event_seq=sequence==task_event_sequences_.end()?0:sequence->second.first;
    m.elapsed_sim_s=s.elapsed_sim; m.elapsed_wall_s=s.elapsed_wall; m.distance_remaining_m=s.distance;
    if(s.request.uuid.size()==32) for(size_t i=0;i<16;++i) m.transport_goal_uuid.uuid[i]=static_cast<uint8_t>(std::stoul(s.request.uuid.substr(i*2,2),nullptr,16));
    return m;
  }
  void record(const amr::Snapshot &s,const std::string &type,const std::string &detail,const std::string &uuid="") {
    log_<<"{\"run_id\":"<<json(core_->run_id())<<",\"task_id\":"<<json(s.request.id)
      <<",\"event_seq\":"<<++log_sequence_<<",\"core_event_seq\":"<<s.event_seq
      <<",\"ros_stamp\":"<<number(now().seconds())<<",\"wall_offset_s\":"<<number(wall()-started_wall_)
      <<",\"event_type\":"<<json(type)<<",\"state\":"<<json(amr::MissionCore::state_name(s.state))
      <<",\"state_code\":"<<static_cast<int>(s.state)<<",\"error_code\":"<<s.error
      <<",\"goal_uuid\":"<<json(uuid.empty()?s.request.uuid:uuid)<<",\"nav_sequence\":"<<s.nav_sequence
      <<",\"nav_goal_uuid\":"<<json(s.nav_sequence?nav_uuid_:"")
      <<",\"interaction_id\":"<<json(s.interaction)<<",\"load_state\":"<<static_cast<int>(s.load)
      <<",\"elapsed_sim_s\":"<<number(s.elapsed_sim)<<",\"elapsed_wall_s\":"<<number(s.elapsed_wall)
      <<",\"detail\":"<<json(detail)<<"}\n";
    log_.flush();
    // Fail fast before publishing any uncommitted terminal result.
    if(!log_) throw std::runtime_error("LOG_IO_ERROR: cannot commit event; shutting down mission");
    if(type=="admission_accepted") task_event_sequences_[s.request.id]={log_sequence_,false};
    auto sequence=task_event_sequences_.find(s.request.id);
    if(sequence!=task_event_sequences_.end() && !sequence->second.second) {
      sequence->second.first=log_sequence_;
      if(type=="terminal") sequence->second.second=true;
    }
    amr_interfaces::msg::MissionEvent e; e.stamp=now(); e.run_id=core_->run_id(); e.task_id=s.request.id;
    e.event_seq=log_sequence_; e.event_type=type; e.state=static_cast<uint8_t>(s.state); e.error_code=s.error; e.detail=detail;
    e.request_goal_uuid=message(s).transport_goal_uuid;
    events_pub_->publish(e);
  }
  void admission(const std::string &type,const std::string &id,const std::string &uuid,const amr::Reply &r) {
    amr::Snapshot s; s.request.id=id; s.request.uuid=uuid; s.error=r.code; record(s,type,r.message,uuid);
  }
  void drain() {
    auto effects=core_->take_effects();
    for(const auto &e:effects) {
      switch(e.type) {
        case amr::EffectType::Event:
          record(e.snapshot,"state_event",e.detail);
          state_pub_->publish(message(e.snapshot));
          break;
        case amr::EffectType::EnableGuard: set_guard(true,e.sequence); break;
        case amr::EffectType::DisableGuard: guard_was_enabled_=false; set_guard(false,e.sequence); break;
        case amr::EffectType::SendNav: send_nav(e); break;
        case amr::EffectType::CancelNav:
          if(nav_handle_ && nav_sequence_==e.sequence) {
            record(e.snapshot,"nav_cancel_requested","targeted nav UUID="+nav_uuid_);
            nav_->async_cancel_goal(nav_handle_,[this](auto response){
              record(core_->current(),"nav_cancel_response","return_code="+std::to_string(response->return_code)+"; waiting for terminal result");
            });
          }
          break;
        case amr::EffectType::Terminal: {
          record(e.snapshot,"terminal",e.detail);
          state_pub_->publish(message(e.snapshot));
          auto result=std::make_shared<Transport::Result>(); auto m=message(e.snapshot);
          result->run_id=m.run_id; result->task_id=m.task_id; result->final_state=m.state; result->error_code=m.error_code;
          result->message=m.message; result->load_state=m.load_state; result->requires_operator=m.requires_operator;
          result->elapsed_sim_s=m.elapsed_sim_s; result->elapsed_wall_s=m.elapsed_wall_s;
          if(transport_) {
            if(e.snapshot.state==amr::State::Completed) transport_->succeed(result);
            else if(e.snapshot.state==amr::State::Canceled && transport_->is_canceling()) transport_->canceled(result);
            else transport_->abort(result);
            transport_.reset();
          }
          break;
        }
      }
    }
  }
  void set_guard(bool enabled,uint64_t sequence) {
    if(!guard_->service_is_ready()) {
      if(enabled) core_->guard_enabled(sequence,false,observation());
      return;
    }
    auto request=std::make_shared<std_srvs::srv::SetBool::Request>(); request->data=enabled;
    if(enabled) enabling_=true;
    guard_->async_send_request(request,[this,enabled,sequence](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future){
      auto response=future.get();
      if(enabled) {
        enabling_=false; guard_was_enabled_=response->success;
        if(response->success) { guard_enabled_=true; guard_wall_=wall(); }
        core_->guard_enabled(sequence,response->success,observation());
      }
      drain();
    });
  }
  void send_nav(const amr::Effect &e) {
    Nav::Goal goal; goal.pose=pose(registry_->stations().at(e.snapshot.station)); goal.behavior_tree=bt_;
    nav_sequence_=e.sequence;
    nav_uuid_.clear();
    recovery_count_=0;
    record(e.snapshot,"nav_attempt","station="+e.snapshot.station);
    rclcpp_action::Client<Nav>::SendGoalOptions options;
    options.goal_response_callback=[this,seq=e.sequence](NavHandle::SharedPtr handle){
      nav_handle_=handle;
      if(handle && seq==nav_sequence_) nav_uuid_=uuid_string(handle->get_goal_id());
      core_->nav_response(seq,static_cast<bool>(handle),observation()); drain();
    };
    options.feedback_callback=[this,seq=e.sequence](auto,auto feedback){
      core_->nav_feedback(seq,feedback->distance_remaining);
      if(seq==nav_sequence_ && feedback->number_of_recoveries!=recovery_count_) {
        recovery_count_=feedback->number_of_recoveries;
        record(core_->current(),"nav_recovery","count="+std::to_string(recovery_count_));
      }
    };
    options.result_callback=[this,seq=e.sequence](const NavHandle::WrappedResult &r){
      bool success=r.code==rclcpp_action::ResultCode::SUCCEEDED && r.result && r.result->error_code==0;
      bool unknown_abort=seq==nav_sequence_ && r.code==rclcpp_action::ResultCode::ABORTED && (!r.result || r.result->error_code==0);
      std::string detail="code="+std::to_string(static_cast<int>(r.code));
      if(r.result) detail+=" nav_error="+std::to_string(r.result->error_code)+" "+r.result->error_msg;
      if(unknown_abort) {
        restart_required_=true; restart_reason_="UNCLASSIFIED_NAV_ABORT";
        detail+="; internal navigation context uncertain; full stack restart required";
        auto snapshot=core_->current(); snapshot.error=amr::NavFailed;
        record(snapshot,"nav_context_invalidated",detail);
      }
      record(core_->current(),"nav_result",detail+(success?" success":" failure"));
      if(seq==nav_sequence_) nav_handle_.reset();
      core_->nav_result(seq,success,detail,observation(),!unknown_abort); drain();
    };
    nav_->async_send_goal(goal,options);
  }
  void publish_state() {
    auto m=message(core_->current());
    if(!core_->active()) {
      m=Snapshot{}; m.run_id=core_->run_id(); m.state=Snapshot::IDLE;
      m.load_state=static_cast<uint8_t>(core_->robot_load()); m.requires_operator=core_->hold()||restart_required_;
      m.message=m.requires_operator?"HOLD":"IDLE"; m.event_seq=log_sequence_;
      m.distance_remaining_m=std::numeric_limits<float>::quiet_NaN();
    }
    state_pub_->publish(m);
    if(transport_ && core_->active()) {
      auto f=std::make_shared<Transport::Feedback>(); f->run_id=m.run_id; f->state=m.state;
      f->current_station=m.current_station; f->interaction_id=m.interaction_id; f->load_state=m.load_state;
      f->elapsed_sim_s=m.elapsed_sim_s; f->distance_remaining_m=m.distance_remaining_m; transport_->publish_feedback(f);
    }
  }
  void publish_diagnostic(const amr::Observation &o) {
    diagnostic_msgs::msg::DiagnosticArray a; a.header.stamp=now(); diagnostic_msgs::msg::DiagnosticStatus s;
    s.name="mission/readiness"; s.hardware_id="sim_amr"; s.level=(!o.ready||core_->hold()||restart_required_)?1:0;
    s.message=(core_->hold()||restart_required_)?"HOLD":o.ready?"READY":"NOT_READY"; a.status.push_back(s); diagnostics_->publish(a);
  }
  std::unique_ptr<amr::StationRegistry> registry_;
  std::unique_ptr<amr::MissionCore> core_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp_action::Client<Nav>::SharedPtr nav_;
  rclcpp_action::Server<Transport>::SharedPtr server_;
  NavHandle::SharedPtr nav_handle_;
  std::shared_ptr<TransportHandle> transport_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr guard_;
  rclcpp::Publisher<Snapshot>::SharedPtr state_pub_;
  rclcpp::Publisher<amr_interfaces::msg::MissionEvent>::SharedPtr events_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready_sub_,guard_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<amr_interfaces::srv::GetTransport>::SharedPtr query_;
  rclcpp::Service<amr_interfaces::srv::GetStation>::SharedPtr station_query_;
  rclcpp::Service<Confirm>::SharedPtr confirm_;
  rclcpp::Service<Reconcile>::SharedPtr reconcile_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream log_;
  std::string bt_,nav_uuid_;
  std::map<std::string,std::pair<uint64_t,bool>> task_event_sequences_;
  uint64_t nav_sequence_{0},log_sequence_{0};
  int16_t recovery_count_{0};
  bool ready_{false},guard_enabled_{false},clean_world_{false},bootstrapped_{false},enabling_{false},guard_was_enabled_{false};
  double ready_wall_{-1e12},guard_wall_{-1e12},odom_wall_{-1e12},odom_stamp_{0},stopped_since_{-1};
  double started_wall_{wall()};
  double last_observed_sim_{0},exit_deadline_{-1};
  bool restart_required_{false};
  std::string restart_reason_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<MissionNode>()); }
  catch(const std::exception &e) { RCLCPP_FATAL(rclcpp::get_logger("mission_manager"),"%s",e.what()); rclcpp::shutdown(); return 1; }
  rclcpp::shutdown(); return 0;
}
