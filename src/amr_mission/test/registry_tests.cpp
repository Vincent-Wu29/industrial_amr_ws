#include "amr_mission/station_registry.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <functional>
#include <chrono>
int main(int argc,char **argv) {
  if(argc!=3) return 2;
  amr::StationRegistry valid(argv[1],argv[2]);
  if(valid.stations().size()!=6) return 1;
  std::ifstream input(argv[1]); std::ostringstream text; text<<input.rdbuf();
  auto path=std::filesystem::temp_directory_path()/("amr_stations_"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".yaml");
  auto reject=[&](std::string s){
    { std::ofstream f(path); f<<s; }
    bool rejected=false; try { amr::StationRegistry bad(path.string(),argv[2]); } catch(const std::exception &) { rejected=true; }
    if(!rejected) throw std::runtime_error("invalid registry accepted");
  };
  auto replace=[](std::string s,const std::string &a,const std::string &b){ s.replace(s.find(a),a.size(),b); return s; };
  try {
    reject(replace(text.str(),"warehouse_v1","other_map"));
    reject(replace(text.str(),"station_id: A02","station_id: A01"));
    reject(replace(text.str(),"x: 3.000000","x: .nan"));
    reject(replace(text.str(),"x: 3.000000","x: 0.050000"));
    reject(replace(text.str(),"type: PICKUP","type: INVALID"));
    reject(replace(text.str(),"load_timeout_s: 30.0","load_timeout_s: -1.0"));
    reject(replace(replace(text.str(),"x: 3.000000","x: 5.790000"),"y: 3.000000","y: 4.200000"));
    std::filesystem::remove(path); std::cout<<"8 registry/map checks passed\n";
  } catch(...) { std::filesystem::remove(path); throw; }
}
