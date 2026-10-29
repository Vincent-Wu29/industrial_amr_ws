#include "amr_mission/mission_core.hpp"
#include <algorithm>
#include <cmath>
#include <utility>

namespace amr {
MissionCore::MissionCore(std::string run_id, std::map<std::string, Station> stations, Limits limits)
 : run_(std::move(run_id)), stations_(std::move(stations)), limits_(limits) {}
bool MissionCore::terminal(State s) { return s==State::Completed || s==State::Failed || s==State::Canceled; }
bool MissionCore::active() const { return current_.state!=State::Idle && !terminal(current_.state); }
bool MissionCore::hold() const {
  return fault_latched_ || !log_ok_ || unresolved_nav() || enable_pending_ || robot_load_!=Load::Empty;
}
bool MissionCore::valid_id(const std::string &s) {
  return !s.empty() && s.size()<=64 && std::all_of(s.begin(),s.end(),[](unsigned char c){
    return (c>='a' && c<='z') || (c>='A' && c<='Z') || (c>='0' && c<='9') || c=='_' || c=='-';
  });
}
const char *MissionCore::state_name(State s) {
  static const char *names[]={"IDLE","PENDING","NAVIGATING_TO_PICKUP","DOCKING_AT_PICKUP",
    "WAITING_FOR_LOAD","NAVIGATING_TO_DROPOFF","DOCKING_AT_DROPOFF","WAITING_FOR_UNLOAD",
    "STOPPING","COMPLETED","FAILED","CANCELED"};
  return names[static_cast<unsigned>(s)];
}
void MissionCore::emit(EffectType type, const std::string &detail) {
  if(type==EffectType::Event) current_.event_seq=++event_seq_;
  effects_.push_back({type,current_.nav_sequence,detail,current_});
}
void MissionCore::update_time(const Observation &o) {
  current_.elapsed_sim=clock_invalid_ ? std::numeric_limits<double>::quiet_NaN() : std::max(0.0,o.sim-current_.start_sim);
  current_.elapsed_wall=std::max(0.0,o.wall-current_.start_wall);
}
void MissionCore::transition(State s, const std::string &detail, const Observation &o) {
  auto from=state_name(current_.state);
  current_.state=s; phase_sim_=o.sim; phase_wall_=o.wall; update_time(o);
  emit(EffectType::Event,std::string(from)+" -> "+state_name(s)+": "+detail);
}
Reply MissionCore::admit(const Request &r, const Observation &o) {
  auto old=history_.find(r.id);
  if(old!=history_.end()) {
    bool same=old->second.request.source==r.source && old->second.request.target==r.target;
    return {false,static_cast<uint16_t>(same?Duplicate:Conflict),same?"task ID already used in this run":"task ID payload conflict"};
  }
  if(!valid_id(r.id) || r.source==r.target) return {false,InvalidRequest,"invalid task ID or same station"};
  auto a=stations_.find(r.source), b=stations_.find(r.target);
  if(a==stations_.end() || b==stations_.end()) return {false,UnknownStation,"station does not exist"};
  if(a->second.type!="PICKUP" || b->second.type!="DROPOFF") return {false,InvalidRequest,"station role mismatch"};
  if(active()) return {false,Busy,"execution slot occupied"};
  if(hold() || !o.ready || !o.stopped || !o.guard_disabled) return {false,NotReady,"HOLD or readiness/stop gate not satisfied"};
  if(history_.size()>=limits_.cache_size) return {false,CacheFull,"session ID cache full; archive and reset stack"};
  current_=Snapshot{}; current_.request=r; current_.load=Load::Empty;
  current_.requires_operator=false; current_.start_sim=o.sim; current_.start_wall=o.wall;
  cancel_intent_=false; cancel_sent_=false; clock_invalid_=false; last_sim_=o.sim;
  transition(State::Pending,"accepted",o); history_[r.id]=current_;
  return {true,Ok,"accepted"};
}
void MissionCore::start_nav(bool pickup, const Observation &o) {
  current_.station=pickup?current_.request.source:current_.request.target;
  current_.interaction.clear(); current_.distance=std::numeric_limits<float>::quiet_NaN();
  current_.nav_sequence=++sequence_; enable_pending_=true; cancel_sent_=false;
  transition(pickup?State::ToPickup:State::ToDropoff,"request motion permission",o);
  emit(EffectType::EnableGuard);
}
void MissionCore::guard_enabled(uint64_t seq, bool success, const Observation &o) {
  if(seq!=current_.nav_sequence) { if(success) emit(EffectType::DisableGuard,"stale enable response"); return; }
  enable_pending_=false;
  if(!active() || current_.state==State::Stopping) { emit(EffectType::DisableGuard,"enable after termination"); return; }
  if(!success || !o.ready) { stop(NotReady,"motion permission failed/readiness lost",false,o); return; }
  if(pending_ || nav_active_) { stop(NavUnavailable,"residual navigation goal",false,o); return; }
  pending_=true; sent_wall_=o.wall; emit(EffectType::SendNav,"send");
}
void MissionCore::nav_response(uint64_t seq, bool accepted, const Observation &o) {
  if(seq!=current_.nav_sequence) return;
  pending_=false; nav_active_=accepted;
  emit(EffectType::Event,accepted?"nav accepted":"nav rejected");
  if(!active() || current_.state==State::Stopping) {
    if(accepted) { cancel_sent_=true; emit(EffectType::CancelNav,"late acceptance cleanup"); }
    return;
  }
  if(!accepted) stop(NavRejected,"Nav2 rejected goal",false,o);
}
void MissionCore::nav_result(uint64_t seq, bool success, const std::string &detail, const Observation &o, bool context_valid) {
  if(seq!=current_.nav_sequence) return;
  if(!context_valid) { fault_latched_=true; success=false; }
  bool expected=pending_ || nav_active_;
  pending_=false; nav_active_=false;
  emit(EffectType::Event,"nav result: "+detail);
  if(!active() || current_.state==State::Stopping || !expected) return;
  if(!success) { stop(NavFailed,detail,false,o); return; }
  emit(EffectType::DisableGuard,"navigation finished");
  if(current_.state==State::ToPickup) transition(State::DockPickup,"nav_only pose and stop check",o);
  else if(current_.state==State::ToDropoff) transition(State::DockDropoff,"nav_only pose and stop check",o);
}
void MissionCore::nav_feedback(uint64_t seq, float distance) {
  if(seq==current_.nav_sequence && nav_active_ && active() && current_.state!=State::Stopping)
    current_.distance=distance;
}
void MissionCore::stop(uint16_t code, const std::string &message, bool cancel, const Observation &o) {
  if(!active() || current_.state==State::Stopping) return;
  if(current_.state==State::WaitLoad || current_.state==State::WaitUnload) current_.load=robot_load_=Load::Unknown;
  current_.interaction.clear(); current_.error=code; current_.message=message;
  cancel_intent_=cancel; transition(State::Stopping,message,o);
  emit(EffectType::DisableGuard,"termination");
  if(nav_active_ && !cancel_sent_) { cancel_sent_=true; emit(EffectType::CancelNav,"termination"); }
}
void MissionCore::fail(uint16_t code, const std::string &message, const Observation &o) { stop(code,message,false,o); }
void MissionCore::invalidate_context(uint16_t code,const std::string &message,const Observation &o) {
  fault_latched_=true; stop(code,message,false,o);
}
Reply MissionCore::cancel(const std::string &id, const Observation &o) {
  if(!active() || current_.request.id!=id || current_.state==State::Stopping)
    return {false,InvalidRequest,"task terminal or already stopping"};
  stop(UserCanceled,"user requested cancellation",true,o); return {true,Ok,"canceling; waiting for downstream terminal and stop"};
}
Reply MissionCore::confirm(const std::string &run,const std::string &task,const std::string &station,
 const std::string &interaction,uint8_t operation,uint8_t outcome,const Observation &o) {
  if(run!=run_ || !history_.count(task) || interaction.empty() || (operation!=1 && operation!=2) ||
    (outcome!=1 && outcome!=2)) return {false,InvalidRequest,"invalid confirmation correlation"};
  std::string key=task+"/"+station+"/"+interaction+"/"+std::to_string(operation);
  auto old=confirmations_.find(key);
  if(old!=confirmations_.end()) return {old->second==outcome,static_cast<uint16_t>(old->second==outcome?Ok:ConfirmConflict),
    old->second==outcome?"already applied":"conflicting confirmation"};
  if(task!=current_.request.id || station!=current_.station || interaction!=current_.interaction ||
    !((current_.state==State::WaitLoad && operation==1) || (current_.state==State::WaitUnload && operation==2)))
    return {false,InvalidRequest,"expired or mismatched interaction"};
  tick(o);
  if(current_.interaction!=interaction || current_.state==State::Stopping || terminal(current_.state))
    return {false,InvalidRequest,"interaction expired"};
  confirmations_[key]=outcome; emit(EffectType::Event,"device confirmation: "+std::to_string(operation)+"/"+std::to_string(outcome));
  if(outcome==2) { stop(DeviceFailed,"station reported operation failure",false,o); return {true,Ok,"failure applied"}; }
  current_.interaction.clear();
  if(operation==1) { current_.load=robot_load_=Load::Loaded; start_nav(false,o); }
  else { current_.load=robot_load_=Load::Empty; finish(State::Completed,o); }
  return {true,Ok,"confirmation applied"};
}
Reply MissionCore::reconcile(Load load, const Observation &o) {
  if(active() || unresolved_nav() || enable_pending_ || fault_latched_ || !o.ready || !o.stopped || !o.guard_disabled ||
    (load!=Load::Empty && load!=Load::Loaded)) return {false,NotReady,"cannot reconcile: cleanup/readiness/stop gate"};
  robot_load_=load;
  if(current_.state==State::Idle) { current_.load=load; current_.requires_operator=load!=Load::Empty; }
  Snapshot status; status.load=robot_load_; status.requires_operator=hold(); status.event_seq=++event_seq_;
  effects_.push_back({EffectType::Event,0,"operator reconciled robot load="+std::to_string(static_cast<int>(load)),status});
  return {true,Ok,"load reconciled"};
}
void MissionCore::finish(State state,const Observation &o) {
  current_.requires_operator=fault_latched_ || unresolved_nav() || enable_pending_ || current_.load!=Load::Empty || !log_ok_;
  if(state==State::Completed) { current_.error=Ok; current_.message="transport completed after unload confirmation"; }
  transition(state,current_.message,o); history_[current_.request.id]=current_;
  emit(EffectType::Terminal,current_.message);
}
void MissionCore::log_failed(const Observation &o) {
  log_ok_=false; fault_latched_=true;
  if(active() && current_.state!=State::Stopping) stop(LogError,"event log write failed",false,o);
}
void MissionCore::tick(const Observation &o) {
  if(!active()) return;
  update_time(o);
  if(o.sim+1e-6<last_sim_) { clock_invalid_=true; fault_latched_=true; stop(ClockReset,"ROS clock moved backwards",false,o); }
  last_sim_=o.sim;
  if(current_.state!=State::Stopping && (!o.ready || o.wall-current_.start_wall>=limits_.task_wall ||
     o.sim-current_.start_sim>=limits_.task_sim)) stop(!o.ready?NotReady:NavTimeout,"readiness lost or overall deadline",false,o);
  switch(current_.state) {
    case State::Pending:
      if(o.wall-phase_wall_>=limits_.pending_wall) stop(NotReady,"pending deadline",false,o);
      else if(o.guard_disabled && o.stopped) start_nav(true,o);
      break;
    case State::ToPickup: case State::ToDropoff:
      if(o.sim-phase_sim_>=limits_.nav_sim) stop(NavTimeout,"navigation segment deadline",false,o);
      else if((pending_ && o.wall-sent_wall_>=limits_.response_wall) ||
        (enable_pending_ && o.wall-phase_wall_>=limits_.response_wall))
        stop(NavUnavailable,"goal/guard response deadline; cleanup retained",false,o);
      break;
    case State::DockPickup: case State::DockDropoff:
      if(o.stopped && o.docked && o.guard_disabled) {
        bool pickup=current_.state==State::DockPickup;
        current_.interaction=run_+"-"+std::to_string(++interaction_seq_);
        transition(pickup?State::WaitLoad:State::WaitUnload,"stationary at station",o);
      } else if(o.sim-phase_sim_>=limits_.dock_sim || o.wall-phase_wall_>=limits_.dock_wall)
        stop(DockFailed,"pose/stop/guard verification deadline",false,o);
      break;
    case State::WaitLoad: case State::WaitUnload: {
      bool load=current_.state==State::WaitLoad;
      const auto &s=stations_.at(current_.station);
      if(o.sim-phase_sim_>=(load?s.load_timeout:s.unload_timeout))
        stop(load?LoadTimeout:UnloadTimeout,"station operation deadline",false,o);
      else if(!o.stopped || !o.guard_disabled) stop(StopUnconfirmed,"movement during station operation",false,o);
      break;
    }
    case State::Stopping:
      if(!unresolved_nav() && !enable_pending_ && o.stopped && o.guard_disabled) finish(cancel_intent_?State::Canceled:State::Failed,o);
      else if(o.wall-phase_wall_>=limits_.stop_wall) {
        current_.error=unresolved_nav()||enable_pending_?CancelUnconfirmed:StopUnconfirmed;
        current_.message="cleanup deadline exceeded; full stack reset required";
        fault_latched_=true; finish(State::Failed,o);
      }
      break;
    default: break;
  }
  if(active()) history_[current_.request.id]=current_;
}
std::optional<Snapshot> MissionCore::find(const std::string &id) const {
  if(id==current_.request.id && !id.empty() && active()) return current_;
  auto it=history_.find(id); if(it==history_.end()) return std::nullopt; return it->second;
}
std::vector<Effect> MissionCore::take_effects() { auto out=std::move(effects_); effects_.clear(); return out; }
}  // namespace amr
