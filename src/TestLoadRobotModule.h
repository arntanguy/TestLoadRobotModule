#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_trajectory/LinearInterpolation.h>

#include "api.h"


/* Default time updater functor */
template<typename T>
struct TimeUpdaterInc
{
  bool operator()(T & t, const T & dt, T Min, T Max) const { t += dt; return t >= Max; }
};

template<typename T>
struct TimeUpdaterDec
{
  bool operator()(T & t, const T & dt, T Min, T Max) const { t += dt; return t <= Min; }
};

template<typename Elem,
  typename T,
  typename Callback = std::function<void()>,
  typename TimeUpdater = TimeUpdaterInc<T>>
struct ElemWithInterpolation
{
  ElemWithInterpolation(const Elem & elem, T minVal, T maxVal, double dt, double duration) : elem_(elem), minVal_(minVal), maxVal_(maxVal), dt_(dt), duration_(duration) {}

  // true if interpolation is complete
  bool operator()()
  {
    // called every gui timestep
    double alpha = t_ / duration_;
    interp = mc_trajectory::LinearInterpolation<T>{}(minVal_, maxVal_, alpha);
    // t+=dt or t-=dt
    return TimeUpdater{}(t_, dt_, 0, duration_);
  }

  bool operator==(const ElemWithInterpolation & other) const noexcept { return elem_ == other.elem_; }
  bool operator==(const Elem & other) const noexcept { return elem_ == other; }
  bool operator<(const ElemWithInterpolation & other) const noexcept { return elem_ < other.elem_; }

  T interpValue() const noexcept { return interp; }

  const Elem & elem_;
  T minVal_;
  T maxVal_;
  double t_ = 0;
  double dt_ = 0;
  double duration_ = 0;
  T interp = 0;
};

// template<typename T, typename ElementActive, typename ElementInactive>
template<typename T>
struct SetGUI
{
  template<typename Callback, typename CallbackComplete>
  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> category, const std::set<T> & trackedSet, Callback callback, CallbackComplete callbackComplete)
  {
    gui_ = &gui;
    category_ = category;
    callback_ = callback;
    callbackComplete_ = callbackComplete;
    trackedSet_ = &const_cast<std::set<T>&>(trackedSet);
    for(const auto & elem : trackedSet)
    {
      addElement(elem);
    }
  }

  void removeFromGUI()
  {
    if(gui_)
    {
      gui_->removeElements(this);
    }
  }

  void update()
  {
    // compare trackedSet_ and trackedSetInGUI_
    for(const auto & elem : *trackedSet_)
    {
      if(trackedSetInGUI_.find(elem) == trackedSetInGUI_.end())
      {
        mc_rtc::log::info("Adding {} to GUI", elem);
        addElement(elem);
      }
    }

    for(auto it = trackedSetInGUI_.begin(); it != trackedSetInGUI_.end(); )
    {
      if(trackedSet_->find(*it) == trackedSet_->end())
      {
        mc_rtc::log::info("Removing {} to GUI", *it);
        gui_->removeElement(category_, *it);
        it = trackedSetInGUI_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    for(auto it = activeElements_.begin(); it != activeElements_.end(); )
    {
      callback_(it->elem_, it->interpValue());
      if((*it)())
      {
        // interpolation complete
        fmt::print("Interpolation of {} complete\n",
           it->elem_);
        callbackComplete_(it->elem_, it->interpValue());
        gui_->removeElement(category_, it->elem_ + "_action");
        it = activeElements_.erase(it); // erase returns the next valid iterator
      }
      else
      {
        ++it;
      }
    }
  }

  protected:
    void addElement(const T & elem)
    {
      trackedSetInGUI_.insert(elem);
      gui_->addElement(category_,
        mc_rtc::gui::Button(elem,
          [this, &elem]()
          {
            fmt::print("Pressed {}\n", elem);
            if(std::find(activeElements_.begin(), activeElements_.end(), elem) != activeElements_.end())
            {
              // already active
              return;
            }
            const auto & elemUpdater = activeElements_.emplace_back(elem, 0., 1., 0.005, 2.0);
              gui_->addElement(category_,
                  mc_rtc::gui::Label(elem + "_action",
                    [this, &elemUpdater]()
                    {
                      return elemUpdater.elem_ + " : " + std::to_string(elemUpdater.interpValue());
                    })
                  );
          }
          ));
    }

    std::set<T> * trackedSet_ = nullptr;
    std::set<T> trackedSetInGUI_;
    // cannot use set here as it always provides const access to its elements
    std::list<ElemWithInterpolation<T, double>> activeElements_;
    std::function<void(const T &, double)> callback_;
    std::function<void(const T &, double)> callbackComplete_;
    mc_rtc::gui::StateBuilder * gui_ = nullptr;
    std::vector<std::string> category_;
};

struct TestLoadRobotModule_DLLAPI TestLoadRobotModule : public mc_control::fsm::Controller
{
  TestLoadRobotModule(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  SetGUI<std::string> setGUI;
  std::set<std::string> trackedSet = {"box", "test"};

};
