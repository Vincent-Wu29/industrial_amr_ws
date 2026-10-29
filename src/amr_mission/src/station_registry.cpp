#include "amr_mission/station_registry.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <set>
#include <stdexcept>

namespace amr {
namespace {
void require(bool ok,const std::string &why) { if(!ok) throw std::runtime_error("station/map validation: "+why); }
std::string token(std::istream &in) {
  std::string s;
  while(in>>s) { if(s[0]=='#') { std::getline(in,s); continue; } return s; }
  throw std::runtime_error("truncated PGM map");
}
}
StationRegistry::StationRegistry(const std::string &stations_file,const std::string &map_file,double safety_padding) {
  require(std::isfinite(safety_padding) && safety_padding>=0 && safety_padding<=.2,"invalid footprint padding");
  const double half_length=.2+safety_padding,half_width=.15+safety_padding;
  auto root=YAML::LoadFile(stations_file), map=YAML::LoadFile(map_file);
  require(root["schema_version"].as<int>()==1,"unsupported station schema");
  require(root["frame_id"].as<std::string>()=="map","frame must be map");
  map_id_=root["map_id"].as<std::string>();
  require(!map_id_.empty() && map["map_id"] && map["map_id"].as<std::string>()==map_id_,"map_id mismatch/missing in map YAML");
  double res=map["resolution"].as<double>(), ox=map["origin"][0].as<double>(), oy=map["origin"][1].as<double>();
  require(std::isfinite(res) && res>0 && std::isfinite(ox) && std::isfinite(oy),"invalid map scale/origin");
  require(std::abs(map["origin"][2].as<double>())<1e-9,"rotated map origin unsupported");
  require(map["negate"].as<int>()==0,"negated map unsupported; export standard PGM");
  auto image=std::filesystem::path(map_file).parent_path()/map["image"].as<std::string>();
  std::ifstream in(image,std::ios::binary); require(in.good(),"cannot open map image");
  std::string format=token(in); require(format=="P5" || format=="P2","map must be PGM P5/P2");
  int width=std::stoi(token(in)),height=std::stoi(token(in)),maxval=std::stoi(token(in));
  require(width>0 && height>0 && width<=20000 && height<=20000 &&
    static_cast<int64_t>(width)*height<=25000000 && maxval==255,"invalid PGM dimensions/range");
  std::vector<unsigned char> pixels(static_cast<size_t>(width)*height);
  if(format=="P5") {
    char delim=static_cast<char>(in.get());
    if(delim=='\r' && in.peek()=='\n') in.get();
    in.read(reinterpret_cast<char *>(pixels.data()),pixels.size());
    require(static_cast<size_t>(in.gcount())==pixels.size(),"truncated PGM pixels");
  } else for(auto &p:pixels) {
    int value=std::stoi(token(in)); require(value>=0 && value<=255,"PGM sample outside range");
    p=static_cast<unsigned char>(value);
  }
  double free_threshold=map["free_thresh"].as<double>();
  require(std::isfinite(free_threshold) && free_threshold>0 && free_threshold<1,"invalid map threshold");
  auto free_cell=[&](int col,int bottom_row) {
    int row=height-1-bottom_row;
    return col>=0 && col<width && row>=0 && row<height &&
      (1.0-pixels[static_cast<size_t>(row)*width+col]/255.0)<free_threshold;
  };
  const std::set<std::string> roles{"PICKUP","DROPOFF","BUFFER","HOME"};
  require(root["stations"].IsSequence() && root["stations"].size()>0,"empty stations");
  for(auto entry:root["stations"]) {
    Station s; s.id=entry["station_id"].as<std::string>(); s.type=entry["type"].as<std::string>();
    require(MissionCore::valid_id(s.id) && !stations_.count(s.id),"invalid or duplicate station ID: "+s.id);
    require(roles.count(s.type)>0,"unknown station type");
    s.x=entry["pose"]["x"].as<double>(); s.y=entry["pose"]["y"].as<double>(); s.yaw=entry["pose"]["yaw"].as<double>();
    require(std::isfinite(s.x)&&std::isfinite(s.y)&&std::isfinite(s.yaw),"nonfinite pose");
    s.yaw=std::remainder(s.yaw,2*3.141592653589793);
    require(entry["docking_mode"].as<std::string>()=="nav_only" && !entry["approach_pose"],"only nav_only supported");
    s.load_timeout=entry["load_timeout_s"].as<double>(30); s.unload_timeout=entry["unload_timeout_s"].as<double>(30);
    require(std::isfinite(s.load_timeout)&&std::isfinite(s.unload_timeout)&&s.load_timeout>0&&s.load_timeout<=300&&s.unload_timeout>0&&s.unload_timeout<=300,"invalid operation timeout");
    // Exact rectangle/cell separating-axis test, including tiny corner overlaps.
    double c=std::cos(s.yaw),sn=std::sin(s.yaw),ac=std::abs(c),as=std::abs(sn);
    double hx=half_length*ac+half_width*as,hy=half_length*as+half_width*ac;
    require(s.x-hx>=ox && s.x+hx<ox+width*res && s.y-hy>=oy && s.y+hy<oy+height*res,
      "station footprint outside map: "+s.id);
    int min_x=static_cast<int>(std::floor((s.x-hx-ox)/res)),max_x=static_cast<int>(std::floor((s.x+hx-ox)/res));
    int min_y=static_cast<int>(std::floor((s.y-hy-oy)/res)),max_y=static_cast<int>(std::floor((s.y+hy-oy)/res));
    for(int x=min_x;x<=max_x;++x) for(int y=min_y;y<=max_y;++y) {
      double dx=ox+(x+.5)*res-s.x,dy=oy+(y+.5)*res-s.y;
      bool intersects=std::abs(dx)<=hx+res/2 && std::abs(dy)<=hy+res/2 &&
        std::abs(dx*c+dy*sn)<=half_length+res/2*(ac+as) && std::abs(-dx*sn+dy*c)<=half_width+res/2*(ac+as);
      require(!intersects || free_cell(x,y),"station footprint intersects occupied/unknown map: "+s.id);
    }
    stations_.emplace(s.id,s);
  }
}
}  // namespace amr
