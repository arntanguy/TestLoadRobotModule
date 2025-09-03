#include "TestLoadRobotModule_Initial.h"

#include "../TestLoadRobotModule.h"

void TestLoadRobotModule_Initial::configure(const mc_rtc::Configuration & config)
{
}

void TestLoadRobotModule_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TestLoadRobotModule &>(ctl_);
}

bool TestLoadRobotModule_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TestLoadRobotModule &>(ctl_);
  output("OK");
  return true;
}

void TestLoadRobotModule_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<TestLoadRobotModule &>(ctl_);
}

EXPORT_SINGLE_STATE("TestLoadRobotModule_Initial", TestLoadRobotModule_Initial)
