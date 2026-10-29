#pragma once
#include "amr_mission/mission_core.hpp"
#include <string>

namespace amr {
class StationRegistry {
 public:
  StationRegistry(const std::string &stations_file, const std::string &map_file,double safety_padding=0.12);
  const std::map<std::string,Station> &stations() const { return stations_; }
  const std::string &map_id() const { return map_id_; }
 private:
  std::map<std::string,Station> stations_;
  std::string map_id_;
};
}  // namespace amr
