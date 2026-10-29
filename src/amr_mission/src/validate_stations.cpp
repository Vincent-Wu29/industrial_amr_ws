#include "amr_mission/station_registry.hpp"
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=3 && argc!=4) { std::cerr<<"usage: validate_stations STATIONS_YAML MAP_YAML [PADDING]\n"; return 2; }
  try {
    amr::StationRegistry registry(argv[1],argv[2],argc==4?std::stod(argv[3]):.12);
    std::cout<<registry.stations().size()<<" stations validated for map "<<registry.map_id()<<'\n';
    return 0;
  } catch(const std::exception &e) { std::cerr<<e.what()<<'\n'; return 1; }
}
