#pragma once
#include <cstdint>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace amr {
enum class State : uint8_t {
  Idle=0, Pending=1, ToPickup=2, DockPickup=3, WaitLoad=4,
  ToDropoff=5, DockDropoff=6, WaitUnload=7, Stopping=8,
  Completed=9, Failed=10, Canceled=11
};
enum class Load : uint8_t { Empty=0, Loaded=1, Unknown=2 };
enum Error : uint16_t {
  Ok=0, InvalidRequest=100, UnknownStation=101, Duplicate=102, Conflict=103,
  Busy=104, CacheFull=105, NotReady=200, NavUnavailable=201,
  NavRejected=202, NavFailed=203, NavTimeout=204, NoProgress=205,
  DockFailed=300, LoadTimeout=301, UnloadTimeout=302, DeviceFailed=303,
  ConfirmConflict=304, CancelUnconfirmed=400, StopUnconfirmed=401,
  ClockReset=402, Interrupted=403, LogError=404, UserCanceled=500
};
struct Station {
  std::string id, type;
  double x{}, y{}, yaw{}, load_timeout{30}, unload_timeout{30};
};
struct Request { std::string id, source, target, uuid; };
struct Snapshot {
  Request request;
  State state{State::Idle};
  Load load{Load::Unknown};
  std::string station, interaction, message;
  uint16_t error{Ok};
  bool requires_operator{true};
  uint64_t event_seq{0}, nav_sequence{0};
  double start_sim{}, start_wall{}, elapsed_sim{}, elapsed_wall{};
  float distance{std::numeric_limits<float>::quiet_NaN()};
};
struct Observation {
  double sim{}, wall{};
  bool ready{false}, stopped{false}, docked{false}, guard_disabled{false};
};
struct Limits {
  double pending_wall{10}, nav_sim{180}, response_wall{5}, dock_sim{3},
    dock_wall{5}, stop_wall{5}, task_sim{420}, task_wall{900};
  size_t cache_size{10000};
};
enum class EffectType { EnableGuard, DisableGuard, SendNav, CancelNav, Event, Terminal };
struct Effect {
  EffectType type;
  uint64_t sequence{};
  std::string detail;
  Snapshot snapshot;
};
struct Reply { bool accepted; uint16_t code; std::string message; };

// Serialized by the adapter; no ROS objects, clocks, threads or IO.
class MissionCore {
 public:
  MissionCore(std::string run_id, std::map<std::string, Station> stations, Limits limits={});
  Reply admit(const Request &request, const Observation &observation);
  Reply cancel(const std::string &task_id, const Observation &observation);
  Reply confirm(const std::string &run, const std::string &task,
    const std::string &station, const std::string &interaction,
    uint8_t operation, uint8_t outcome, const Observation &observation);
  Reply reconcile(Load load, const Observation &observation);
  void tick(const Observation &observation);
  void guard_enabled(uint64_t sequence, bool success, const Observation &observation);
  void nav_response(uint64_t sequence, bool accepted, const Observation &observation);
  void nav_result(uint64_t sequence, bool success, const std::string &detail,
    const Observation &observation, bool context_valid=true);
  void nav_feedback(uint64_t sequence, float distance);
  void fail(uint16_t code, const std::string &message, const Observation &observation);
  void invalidate_context(uint16_t code, const std::string &message, const Observation &observation);
  void log_failed(const Observation &observation);
  const Snapshot &current() const { return current_; }
  std::optional<Snapshot> find(const std::string &id) const;
  std::vector<Effect> take_effects();
  bool active() const;
  bool hold() const;
  Load robot_load() const { return robot_load_; }
  bool unresolved_nav() const { return pending_ || nav_active_; }
  const std::string &run_id() const { return run_; }
  static bool terminal(State state);
  static bool valid_id(const std::string &id);
  static const char *state_name(State state);
 private:
  void transition(State state, const std::string &detail, const Observation &observation);
  void emit(EffectType type, const std::string &detail="");
  void start_nav(bool pickup, const Observation &observation);
  void stop(uint16_t code, const std::string &message, bool cancel, const Observation &observation);
  void finish(State state, const Observation &observation);
  void update_time(const Observation &observation);
  std::string run_;
  std::map<std::string, Station> stations_;
  Limits limits_;
  Snapshot current_;
  Load robot_load_{Load::Unknown};
  std::map<std::string, Snapshot> history_;
  std::map<std::string, uint8_t> confirmations_;
  std::vector<Effect> effects_;
  uint64_t sequence_{0}, event_seq_{0}, interaction_seq_{0};
  bool pending_{false}, nav_active_{false}, enable_pending_{false}, cancel_sent_{false};
  bool cancel_intent_{false}, fault_latched_{false}, log_ok_{true}, clock_invalid_{false};
  double phase_sim_{0}, phase_wall_{0}, sent_wall_{0}, last_sim_{0};
};
}  // namespace amr
