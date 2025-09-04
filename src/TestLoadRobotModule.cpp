#include "TestLoadRobotModule.h"
#include "RobotModuleFactory.h"


namespace mc_rbdyn
{


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
static auto boxYaml = R"(
name: box
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
  box:
    size: [1., 0.5, 2.]
# inertia is provided in the same format as RBInertiad config
# only mass should be required for basic shapes
inertia:
  mass: 10.0
  momentum: [0., 0., 0.]
    )";


}

TestLoadRobotModule::TestLoadRobotModule(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  // rbd::parsers::Visual visual = mc_rtc::Configuration::fromYAMLData(mc_rbdyn::sphereYaml);
  // auto rmV = mc_rbdyn::robotModuleFromVisual("sphere", visual);
  // this->loadRobot(rmV, "sphere");

  auto boxConfig = mc_rtc::Configuration::fromYAMLData(mc_rbdyn::boxYaml);
  auto rmV = mc_rbdyn::robotModuleFromVisualConfig("box", boxConfig);
  this->loadRobot(rmV, "box");
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


