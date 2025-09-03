#include "TestLoadRobotModule.h"


#include <mc_rbdyn/RobotModule.h>
#include <RBDyn/parsers/urdf.h>
#include <SpaceVecAlg/SpaceVecAlg>
// For unique_path only
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <filesystem>
namespace fs = std::filesystem;

namespace mc_rbdyn
{


std::string make_temporary_path(const std::string & prefix)
{
  auto tmp = fs::temp_directory_path();
  auto pattern = fmt::format("{}-%%%%-%%%%-%%%%-%%%%", prefix);
  // std::filesystem does not have a unique_path function in c++17
  // keep boost around for now
  auto out = tmp / bfs::unique_path(pattern).string();
  fs::create_directories(out);
  return out.string();
}

// FIXME: cylinder not supported in mc-rtc-magnum
static auto cylinderYaml = R"(
name: cylinder
origin:
  translation: [0, 0, 0]
  rotation: [0, 0, 0]
material:
  color:
    r: 1
    g: 0
    b: 0
    a: 1
geometry:
  cylinder:
    radius: 0.5
    length: 1.0
    )";
static auto sphereYaml = R"(
name: sphere
origin:
  translation: [0, 0, 0]
  rotation: [0, 0, 0]
material:
  color:
    r: 1
    g: 0
    b: 0
    a: 1
geometry:
  sphere:
    radius: 0.5
    )";

inline RobotModulePtr robotModuleFromVisual(
    const std::string & name,
    const rbd::parsers::Visual & visual,
    bool isFixed = true
    )
{
  auto pr = rbd::parsers::ParserResult{};
  pr.visual[name] = { visual };
  pr.collision[name] = { visual };

  auto rbInertia = sva::RBInertiad{};
  rbd::Body body(rbInertia, name);
  pr.mbg.addBody(body);

  pr.mb = pr.mbg.makeMultiBody(name, isFixed);
  pr.mbc = rbd::MultiBodyConfig(pr.mb);
  pr.mbc.zero(pr.mb);

  auto rmPtr = std::make_shared<RobotModule>(name, pr);
  auto & rm = *rmPtr;


  // Module is created, but we still need to export it for other tools (visualization, ...)
  auto path = make_temporary_path(name);
  auto urdf_path = (fs::path(path) / "urdf" / (name + ".urdf")).string();
  auto urdf_dir = fs::path(urdf_path).parent_path();
  if(!fs::exists(urdf_dir)) { fs::create_directories(urdf_dir); }
  std::ofstream ofs(urdf_path);
  ofs << rbd::parsers::to_urdf(pr);


  // Generate a module file so that the generated RobotModule can be re-used
  // XXX: currently devices are not saved so strictly speaking the generated yaml robot
  // won't be exactly identical to the one generated here
  {
    std::string module_yaml = fmt::format("{}/{}.yaml", path, name);
    rm.path = path;
    rm.urdf_path = urdf_path;
    rm._parameters = {"json", module_yaml};
    rm._canonicalParameters = rm._parameters;

    auto yaml = mc_rtc::ConfigurationLoader<mc_rbdyn::RobotModule>::save(rm, false, {}, rm.mb.joint(0).dof() == 0);
    yaml.save(module_yaml);
    mc_rtc::log::info("Result module for {} in: {}", name, module_yaml);
  }

  return rmPtr;
}

}

TestLoadRobotModule::TestLoadRobotModule(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  rbd::parsers::Visual visual = mc_rtc::Configuration::fromYAMLData(mc_rbdyn::sphereYaml);
  auto rmV = mc_rbdyn::robotModuleFromVisual("sphere", visual);
  this->loadRobot(rmV, "sphere");
  mc_rtc::log::success("TestLoadRobotModule init done ");
}

bool TestLoadRobotModule::run()
{
  return mc_control::fsm::Controller::run();
}

void TestLoadRobotModule::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


