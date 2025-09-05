#pragma once

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_trajectory/LinearInterpolation.h>

#include "api.h"

//
// /* Default time updater functor */
// template<typename T>
// struct TimeUpdaterInc
// {
//   bool operator()(T & t, const T & dt, T Min, T Max) const { t += dt; return t >= Max; }
// };
//
// template<typename T>
// struct TimeUpdaterDec
// {
//   bool operator()(T & t, const T & dt, T Min, T Max) const { t += dt; return t <= Min; }
// };
//
// template<typename Elem,
//   typename T,
//   typename Callback = std::function<void()>,
//   typename TimeUpdater = TimeUpdaterInc<T>>
// struct ElemWithInterpolation
// {
//   ElemWithInterpolation(const Elem & elem, T minVal, T maxVal, double dt, double duration) : elem_(elem), minVal_(minVal), maxVal_(maxVal), dt_(dt), duration_(duration) {}
//
//   // true if interpolation is complete
//   bool operator()()
//   {
//     // called every gui timestep
//     double alpha = t_ / duration_;
//     interp = mc_trajectory::LinearInterpolation<T>{}(minVal_, maxVal_, alpha);
//     // t+=dt or t-=dt
//     return TimeUpdater{}(t_, dt_, 0, duration_);
//   }
//
//   bool operator==(const ElemWithInterpolation & other) const noexcept { return elem_ == other.elem_; }
//   bool operator==(const Elem & other) const noexcept { return elem_ == other; }
//   bool operator<(const ElemWithInterpolation & other) const noexcept { return elem_ < other.elem_; }
//
//   T interpValue() const noexcept { return interp; }
//
//   const Elem & elem_;
//   T minVal_;
//   T maxVal_;
//   double t_ = 0;
//   double dt_ = 0;
//   double duration_ = 0;
//   T interp = 0;
// };


struct BidirectionalLinearInterpolation
{
  BidirectionalLinearInterpolation() = default;
  BidirectionalLinearInterpolation(double minVal, double maxVal, double dt, double duration) : delta_(dt/duration), minVal_(minVal), maxVal_(maxVal) {
    setValue(minVal_);
  }

  // true if interpolation is complete
  double operator()(bool direction)
  {
    // called every gui timestep
    if(direction)
    {
      interpValue_ += delta_;
    }
    else
    {
      interpValue_ -= delta_;
    }
    setValue(interpValue_);
    return interpValue_;
  }

  double value() const noexcept { return interpValue_; }
  void setValue(double val)
  {
    interpValue_ = std::clamp(val, minVal_, maxVal_);
  }

  bool complete() const noexcept { return interpValue_ <= minVal_ || interpValue_ >= maxVal_; }

  double minVal_ = 0.;
  double maxVal_ = 1.;
  double delta_ = 0;
  double interpValue_ = 0.;
};

template<typename Elem>
struct BidirectionalGUIInterpolator
{
  BidirectionalGUIInterpolator() = default;
  BidirectionalGUIInterpolator(double min, double max, double dt, double duration) : dt_(dt), interp_(min, max, dt, duration) {}
  ~BidirectionalGUIInterpolator()
  {
    if(gui_)
    {
      gui_->removeElements(this);
    }
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> category, const Elem & elem)
  {
    elem_ = &elem;
    using namespace mc_rtc::gui;
    gui_ = &gui;
    gui.addElement(this,
        category,
        ElementsStacking::Horizontal,
        Button("Add " + elem,
            [this]()
            {
              mc_rtc::log::info("Add {} pressed", *elem_);
              remove_ = false;
              add_ = true;
            }),
        NumberSlider(elem,
            [this]() { return interp_.value();; },
            [this](double val) { add_=false; remove_=false; interp_.setValue(val); },
            interp_.minVal_,
            interp_.maxVal_),
        Button("Remove " + elem,
            [this]()
            {
              mc_rtc::log::info("Remove {} pressed", *elem_);
              add_ = false;
              remove_ = true;
            })
        );
  }

  bool update()
  {
    if(add_)
    {
      interp_(true);
    }
    else if(remove_)
    {
      interp_(false);
    }
    if(interp_.complete())
    {
      add_ = false;
      remove_ = false;
      return true;
    }
    return false;
  }

  double interpValue() const noexcept { return interp_.value(); }

  bool operator==(const BidirectionalGUIInterpolator & other) const noexcept { return elem_ == other.elem_; }

  Elem & elem()
  {
    return const_cast<Elem&>(*elem_);
  }
  const Elem & elem() const
  {
    return *elem_;
  }

    const Elem * elem_;
  protected:
    mc_rtc::gui::StateBuilder * gui_ = nullptr;
    double dt_ = 0.0;
    BidirectionalLinearInterpolation interp_;
    bool add_ = false;
    bool remove_ = false;
};

template<typename T, typename GUIElement>
struct GUIContainerWatcher
{
  template<typename Callback, typename CallbackComplete>
  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> category, const std::set<T> & trackedSet, GUIElement guiElement, Callback callback, CallbackComplete callbackComplete)
  {
    gui_ = &gui;
    category_ = category;
    callback_ = callback;
    callbackComplete_ = callbackComplete;
    trackedSet_ = &const_cast<std::set<T>&>(trackedSet);
    guiElement_ = guiElement;
    update();
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
    // Check for changes in the tracked set
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
        mc_rtc::log::info("Removing {} from GUI", *it);
        activeElements_.remove_if([this, it](const BidirectionalGUIInterpolator<T> & elem) { return elem.elem() == *it; });
        it = trackedSetInGUI_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    for(auto it = activeElements_.begin(); it != activeElements_.end(); )
    {
      callback_(it->elem(), it->interpValue());
      if(it->update())
      {
        callbackComplete_(it->elem(), it->interpValue());
      }
      ++it;
    }
  }

  protected:
    void addElement(const T & elem)
    {
      trackedSetInGUI_.insert(elem);
      activeElements_.push_back(guiElement_); // emplace_back(elem, 0., 1., 0.005, 2.0);
      auto & elemUpdater = activeElements_.back();
      elemUpdater.addToGUI(*gui_, category_, elem);
      gui_->addElement(category_,
          mc_rtc::gui::Button("Remove element " + elem,
          [this,&elemUpdater]()
          {
            mc_rtc::log::info("Remove {} pressed", elemUpdater.elem());
            trackedSet_->erase(elemUpdater.elem());
          }));
    }

    std::set<T> * trackedSet_ = nullptr;
    std::set<T> trackedSetInGUI_;
    // cannot use set here as it always provides const access to its elements
    std::list<BidirectionalGUIInterpolator<T>> activeElements_;
    std::function<void(const T &, double)> callback_;
    std::function<void(const T &, double)> callbackComplete_;
    mc_rtc::gui::StateBuilder * gui_ = nullptr;
    std::vector<std::string> category_;
    GUIElement guiElement_;
};

struct TestLoadRobotModule_DLLAPI TestLoadRobotModule : public mc_control::fsm::Controller
{
  TestLoadRobotModule(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  GUIContainerWatcher<std::string, BidirectionalGUIInterpolator<std::string>> setGUI;
  std::set<std::string> trackedSet = {"box", "test"};

};
