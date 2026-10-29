#include "amr_mission/mission_core.hpp"
#include <functional>
#include <iostream>
#include <stdexcept>
using namespace amr;
#define CHECK(x) do { if(!(x)) throw std::runtime_error(std::string(__FILE__)+":"+std::to_string(__LINE__)+" " #x); } while(false)
struct Fixture {
  std::map<std::string,Station> stations{{"A01",{"A01","PICKUP",3,3,0}},
    {"B02",{"B02","DROPOFF",17,12,3.14}}, {"HOME",{"HOME","HOME",2,7.5,0}}};
  MissionCore c{"run",stations};
  Observation o{1,1,true,true,true,true};
  Request r{"TASK-1","A01","B02","uuid"};
  Fixture() { CHECK(c.reconcile(Load::Empty,o).accepted); c.take_effects(); }
  void start() { CHECK(c.admit(r,o).accepted); c.tick(o); c.guard_enabled(c.current().nav_sequence,true,o); }
  void accepted() { c.nav_response(c.current().nav_sequence,true,o); }
  void arrive() { c.nav_result(c.current().nav_sequence,true,"success",o); c.tick(o); }
  Reply confirm(uint8_t op,uint8_t outcome=1,std::string run="run") {
    auto s=c.current(); return c.confirm(run,r.id,s.station,s.interaction,op,outcome,o);
  }
  void wait_load() { start(); accepted(); arrive(); CHECK(c.current().state==State::WaitLoad); }
  void wait_unload() { wait_load(); CHECK(confirm(1).accepted); c.guard_enabled(c.current().nav_sequence,true,o); accepted(); arrive(); CHECK(c.current().state==State::WaitUnload); }
  size_t effects(EffectType type) { auto es=c.take_effects(); size_t n=0; for(auto &e:es) if(e.type==type) ++n; return n; }
};
int main() {
  int passed=0,failed=0;
  auto test=[&](const char *name,const std::function<void()> &fn){
    try { fn(); ++passed; std::cout<<"PASS "<<name<<'\n'; }
    catch(const std::exception &e) { ++failed; std::cerr<<"FAIL "<<name<<": "<<e.what()<<'\n'; }
  };
  test("T01 independent load/unload, exactly one terminal",[]{ Fixture f; f.wait_unload(); CHECK(f.confirm(2).accepted); CHECK(f.c.current().state==State::Completed); CHECK(f.c.current().load==Load::Empty); CHECK(f.effects(EffectType::Terminal)==1); f.c.tick(f.o); CHECK(f.effects(EffectType::Terminal)==0); });
  test("T04 invalid IDs/types/stations",[]{ Fixture f; auto r=f.r; r.id="bad/id"; CHECK(f.c.admit(r,f.o).code==InvalidRequest); r=f.r; r.source="missing"; CHECK(f.c.admit(r,f.o).code==UnknownStation); r.source="HOME"; CHECK(f.c.admit(r,f.o).code==InvalidRequest); r=f.r; r.target=r.source; CHECK(!f.c.admit(r,f.o).accepted); });
  test("T05 duplicate in flight and after terminal",[]{ Fixture f; f.wait_unload(); CHECK(f.c.admit(f.r,f.o).code==Duplicate); f.confirm(2); CHECK(f.c.admit(f.r,f.o).code==Duplicate); CHECK(f.c.find(f.r.id)->state==State::Completed); });
  test("T06 conflicts and atomic execution slot",[]{ Fixture f; f.start(); auto r=f.r; r.target="B01"; CHECK(f.c.admit(r,f.o).code==Conflict); r=f.r; r.id="new"; CHECK(f.c.admit(r,f.o).code==Busy); });
  test("T07 cancel pickup requires result AND stopped",[]{ Fixture f; f.start(); f.accepted(); CHECK(f.c.cancel(f.r.id,f.o).accepted); CHECK(f.c.current().state==State::Stopping); f.c.tick(f.o); CHECK(f.c.current().state==State::Stopping); f.c.nav_result(1,false,"canceled",f.o); f.o.stopped=false; f.c.tick(f.o); CHECK(f.c.current().state==State::Stopping); f.o.stopped=true; f.c.tick(f.o); CHECK(f.c.current().state==State::Canceled); CHECK(!f.c.hold()); });
  test("T07 cancel load window produces UNKNOWN/HOLD",[]{ Fixture f; f.wait_load(); f.c.cancel(f.r.id,f.o); f.c.tick(f.o); CHECK(f.c.current().state==State::Canceled); CHECK(f.c.current().load==Load::Unknown); CHECK(f.c.hold()); });
  test("T07 cancel dropoff retains LOADED",[]{ Fixture f; f.wait_load(); f.confirm(1); f.c.guard_enabled(2,true,f.o); f.accepted(); f.c.cancel(f.r.id,f.o); f.c.nav_result(2,false,"canceled",f.o); f.c.tick(f.o); CHECK(f.c.current().load==Load::Loaded); CHECK(f.c.hold()); });
  test("T08 cancel before acceptance cleans late goal",[]{ Fixture f; f.start(); f.c.take_effects(); f.c.cancel(f.r.id,f.o); f.c.tick(f.o); CHECK(f.c.current().state==State::Stopping); f.c.nav_response(1,true,f.o); CHECK(f.effects(EffectType::CancelNav)==1); f.c.nav_result(1,false,"canceled",f.o); f.c.tick(f.o); CHECK(f.c.current().state==State::Canceled); });
  test("T08 response timeout retains cleanup after FAILED",[]{ Fixture f; f.start(); f.o.wall=7; f.c.tick(f.o); CHECK(f.c.current().state==State::Stopping); f.o.wall=13; f.c.tick(f.o); CHECK(f.c.current().state==State::Failed); CHECK(f.c.current().error==CancelUnconfirmed); f.c.take_effects(); f.c.nav_response(1,true,f.o); CHECK(f.effects(EffectType::CancelNav)==1); f.c.nav_result(1,false,"late canceled",f.o); CHECK(f.c.hold()); CHECK(!f.c.reconcile(Load::Empty,f.o).accepted); });
  test("T08 guard enable late after cancel is revoked",[]{ Fixture f; CHECK(f.c.admit(f.r,f.o).accepted); f.c.tick(f.o); f.c.cancel(f.r.id,f.o); f.c.take_effects(); f.c.guard_enabled(1,true,f.o); CHECK(f.effects(EffectType::DisableGuard)==1); f.c.tick(f.o); CHECK(f.c.current().state==State::Canceled); });
  test("T09 cancel first beats navigation success",[]{ Fixture f; f.start(); f.accepted(); f.c.cancel(f.r.id,f.o); f.arrive(); CHECK(f.c.current().state==State::Canceled); CHECK(f.c.current().interaction.empty()); });
  test("T09 unload commit first rejects cancel",[]{ Fixture f; f.wait_unload(); f.confirm(2); CHECK(!f.c.cancel(f.r.id,f.o).accepted); CHECK(f.c.current().state==State::Completed); });
  test("T09 cancel first rejects unload",[]{ Fixture f; f.wait_unload(); auto s=f.c.current(); f.c.cancel(f.r.id,f.o); CHECK(!f.c.confirm("run",f.r.id,s.station,s.interaction,2,1,f.o).accepted); f.c.tick(f.o); CHECK(f.c.current().state==State::Canceled); CHECK(f.c.current().load==Load::Unknown); });
  test("T10 cancel acceptance alone never completes",[]{ Fixture f; f.start(); f.accepted(); f.c.cancel(f.r.id,f.o); f.o.wall=7; f.c.tick(f.o); CHECK(f.c.current().state==State::Failed); CHECK(f.c.current().error==CancelUnconfirmed); CHECK(f.c.current().requires_operator); });
  test("T10 stale odom prevents canceled",[]{ Fixture f; f.start(); f.accepted(); f.c.cancel(f.r.id,f.o); f.c.nav_result(1,false,"canceled",f.o); f.o.stopped=false; f.o.wall=7; f.c.tick(f.o); CHECK(f.c.current().error==StopUnconfirmed); CHECK(f.c.current().state==State::Failed); });
  test("T11 load timeout and late success",[]{ Fixture f; f.wait_load(); auto s=f.c.current(); f.o.sim=32; CHECK(!f.c.confirm("run",f.r.id,s.station,s.interaction,1,1,f.o).accepted); CHECK(f.c.current().load==Load::Unknown); f.c.tick(f.o); CHECK(f.c.current().error==LoadTimeout); });
  test("T11 failure confirmation",[]{ Fixture f; f.wait_load(); CHECK(f.confirm(1,2).accepted); f.c.tick(f.o); CHECK(f.c.current().state==State::Failed); CHECK(f.c.current().error==DeviceFailed); CHECK(f.c.current().load==Load::Unknown); });
  test("T11 duplicate and conflicting confirmation",[]{ Fixture f; f.wait_load(); auto s=f.c.current(); CHECK(f.confirm(1).accepted); CHECK(f.c.confirm("run",f.r.id,s.station,s.interaction,1,1,f.o).accepted); CHECK(f.c.confirm("run",f.r.id,s.station,s.interaction,1,2,f.o).code==ConfirmConflict); CHECK(f.c.current().nav_sequence==2); });
  test("T12 stale run/interaction rejected",[]{ Fixture f; f.wait_load(); CHECK(!f.confirm(1,1,"old-run").accepted); auto s=f.c.current(); CHECK(!f.c.confirm("run",f.r.id,s.station,"old-window",1,1,f.o).accepted); CHECK(f.c.current().state==State::WaitLoad); });
  test("T13 readiness lost revokes permission",[]{ Fixture f; f.start(); f.accepted(); f.o.ready=false; f.c.tick(f.o); CHECK(f.c.current().state==State::Stopping); CHECK(f.c.current().error==NotReady); CHECK(f.effects(EffectType::DisableGuard)>=1); });
  test("T15 sim pause uses wall watchdog budget",[]{ Fixture f; f.start(); f.accepted(); f.o.wall=902; f.c.tick(f.o); CHECK(f.c.current().state==State::Stopping); CHECK(f.c.current().error==NavTimeout); });
  test("T15 clock reset latches HOLD",[]{ Fixture f; f.wait_load(); f.o.sim=0.1; f.c.tick(f.o); CHECK(f.c.current().error==ClockReset); CHECK(f.c.hold()); });
  test("T16 new process starts UNKNOWN",[]{ Fixture f; MissionCore restarted("new",f.stations); CHECK(restarted.hold()); CHECK(!restarted.admit(f.r,f.o).accepted); CHECK(!restarted.find(f.r.id)); CHECK(restarted.reconcile(Load::Empty,f.o).accepted); });
  test("T17 log failure prevents later admission",[]{ Fixture f; f.wait_load(); f.c.log_failed(f.o); f.c.tick(f.o); CHECK(f.c.current().error==LogError); CHECK(f.c.current().state==State::Failed); CHECK(f.c.hold()); });
  test("T18 stale result cannot advance next segment",[]{ Fixture f; f.wait_load(); f.confirm(1); f.c.nav_result(1,true,"stale",f.o); CHECK(f.c.current().state==State::ToDropoff); CHECK(f.c.current().nav_sequence==2); });
  test("nav rejection and nav failure are distinct",[]{ Fixture f; f.start(); f.c.nav_response(1,false,f.o); f.c.tick(f.o); CHECK(f.c.current().error==NavRejected); Fixture g; g.start(); g.accepted(); g.c.nav_result(1,false,"planner failure",g.o); g.c.tick(g.o); CHECK(g.c.current().error==NavFailed); });
  test("no docking without pose proof",[]{ Fixture f; f.start(); f.accepted(); f.o.docked=false; f.arrive(); CHECK(f.c.current().state==State::DockPickup); f.o.sim=5; f.c.tick(f.o); CHECK(f.c.current().error==DockFailed); });
  test("segment budget does not reset on feedback",[]{ Fixture f; f.start(); f.accepted(); f.o.sim=181; f.c.nav_feedback(1,0.1f); f.c.tick(f.o); CHECK(f.c.current().error==NavTimeout); });
  test("ID cache never silently evicts",[]{ Fixture f; Limits l; l.cache_size=1; MissionCore c("r",f.stations,l); c.reconcile(Load::Empty,f.o); CHECK(c.admit(f.r,f.o).accepted); c.cancel(f.r.id,f.o); c.tick(f.o); auto r=f.r; r.id="other"; CHECK(c.admit(r,f.o).code==CacheFull); CHECK(c.admit(f.r,f.o).code==Duplicate); });
  test("operator reconciliation never rewrites a terminal task",[]{ Fixture f; f.wait_load(); f.c.cancel(f.r.id,f.o); f.c.tick(f.o); auto final=f.c.find(f.r.id); CHECK(final->load==Load::Unknown); CHECK(f.c.reconcile(Load::Empty,f.o).accepted); CHECK(f.c.robot_load()==Load::Empty); CHECK(!f.c.hold()); CHECK(f.c.find(f.r.id)->load==Load::Unknown); CHECK(f.c.find(f.r.id)->requires_operator); auto r=f.r; r.id="next"; CHECK(f.c.admit(r,f.o).accepted); CHECK(f.c.find(f.r.id)->load==Load::Unknown); });
  test("unclassified navigation abort latches even when empty; stale abort cannot latch",[]{ Fixture f; f.start(); f.accepted(); f.c.nav_result(1,false,"internal acknowledgement uncertain",f.o,false); f.c.tick(f.o); CHECK(f.c.current().state==State::Failed && f.c.current().error==NavFailed); CHECK(f.c.current().load==Load::Empty && f.c.current().requires_operator && f.c.hold()); CHECK(!f.c.reconcile(Load::Empty,f.o).accepted); Fixture g; g.wait_unload(); g.c.nav_result(1,false,"stale invalid context",g.o,false); g.confirm(2); CHECK(g.c.current().state==State::Completed && !g.c.hold()); });
  test("unexpected guard loss requires reset even with empty cargo",[]{ Fixture f; f.start(); f.accepted(); f.c.invalidate_context(NotReady,"guard disarmed",f.o); f.c.nav_result(1,false,"canceled",f.o); f.c.tick(f.o); CHECK(f.c.current().state==State::Failed && f.c.current().requires_operator); CHECK(f.c.current().load==Load::Empty && f.c.hold()); CHECK(!f.c.reconcile(Load::Empty,f.o).accepted); });
  std::cout<<passed<<" passed, "<<failed<<" failed\n"; return failed?1:0;
}
