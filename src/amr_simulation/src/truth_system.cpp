#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose_v.pb.h>
#include <chrono>
#include <string>

namespace amr {
// Evaluation only. Reads ECM physics state; never sets pose, forces or commands.
// Its cadence explicitly restarts on backward time and remains alive when paused.
class TruthSystem : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPostUpdate,
                    public gz::sim::ISystemReset {
 public:
  void Configure(const gz::sim::Entity &,const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &,gz::sim::EventManager &) override {
    if(sdf && sdf->HasElement("model_name")) model_=sdf->Get<std::string>("model_name");
    publisher_=transport_.Advertise<gz::msgs::Pose_V>("/amr/evaluation/poses");
  }
  void Reset(const gz::sim::UpdateInfo &,gz::sim::EntityComponentManager &ecm) override {
    published_=false; reset_seen_=true; reported_loss_=false;
    entity_=ecm.EntityByComponents(gz::sim::components::Name(model_),gz::sim::components::Model());
    gzmsg<<"[AMR truth] Reset; model entity="<<entity_<<std::endl;
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,const gz::sim::EntityComponentManager &ecm) override {
    auto wall=std::chrono::steady_clock::now();
    if(published_ && info.simTime>=last_sim_ && info.simTime-last_sim_<std::chrono::milliseconds(50) &&
       wall-last_wall_<std::chrono::milliseconds(50)) return;
    if(entity_==gz::sim::kNullEntity || !ecm.HasEntity(entity_))
      entity_=ecm.EntityByComponents(gz::sim::components::Name(model_),gz::sim::components::Model());
    if(entity_==gz::sim::kNullEntity) {
      if((reset_seen_ || published_) && !reported_loss_) { gzwarn<<"[AMR truth] Runtime model absent after world reset"<<std::endl; reported_loss_=true; }
      return;
    }
    gz::msgs::Pose_V message;
    auto nanoseconds=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    auto stamp=message.mutable_header()->mutable_stamp();
    stamp->set_sec(nanoseconds/1000000000LL); stamp->set_nsec(nanoseconds%1000000000LL);
    auto append=[&](const std::string &name,gz::sim::Entity entity){
      if(entity==gz::sim::kNullEntity) return;
      const auto pose=gz::sim::worldPose(entity,ecm);
      auto output=message.add_pose(); output->set_name(name); output->set_id(entity);
      output->mutable_position()->set_x(pose.Pos().X());
      output->mutable_position()->set_y(pose.Pos().Y());
      output->mutable_position()->set_z(pose.Pos().Z());
      output->mutable_orientation()->set_x(pose.Rot().X());
      output->mutable_orientation()->set_y(pose.Rot().Y());
      output->mutable_orientation()->set_z(pose.Rot().Z());
      output->mutable_orientation()->set_w(pose.Rot().W());
    };
    append(model_,entity_);
    for(const std::string name:{"scenario_box","enclosure_west","enclosure_east","enclosure_north","enclosure_south"})
      append(name,ecm.EntityByComponents(gz::sim::components::Name(name),gz::sim::components::Model()));
    if(info.simTime<last_sim_) gzmsg<<"[AMR truth] Rewound sampling clock"<<std::endl;
    if(!publisher_.Publish(message)) gzwarn<<"[AMR truth] Publish failed"<<std::endl;
    last_sim_=info.simTime; last_wall_=wall; published_=true;
  }
 private:
  gz::transport::Node transport_;
  gz::transport::Node::Publisher publisher_;
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  std::string model_{"amr"};
  std::chrono::steady_clock::duration last_sim_{};
  std::chrono::steady_clock::time_point last_wall_{};
  bool published_{false};
  bool reset_seen_{false},reported_loss_{false};
};
}
GZ_ADD_PLUGIN(amr::TruthSystem,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPostUpdate,gz::sim::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(amr::TruthSystem,"amr::TruthSystem")
