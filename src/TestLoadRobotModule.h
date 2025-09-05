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


struct BidirectionalLinearInterpolation
{
  BidirectionalLinearInterpolation(double minVal, double maxVal, double duration) : duration_(duration) {}

  // true if interpolation is complete
  double operator()(double dt)
  {
    // called every gui timestep
    double alpha = t_ / duration_;
    interpValue_ = mc_trajectory::LinearInterpolation<double>{}(minVal_, maxVal_, alpha);
    t_ = std::clamp(t_ + dt, 0.0, duration_);
    return interpValue_;
  }

  void setValue(double val)
  {
    interpValue_ = val;
    // set t_ accordingly
    double alpha = (val - minVal_) / (maxVal_ - minVal_);
    t_ = std::clamp(alpha * duration_, 0.0, duration_);
  }

  bool complete() const noexcept { return t_ >= duration_ || t_ <= 0; }

  double minVal_ = 0.;
  double maxVal_ = 1.;
  double t_ = 0;
  double duration_ = 0;
  double interpValue_ = 0.;
};

template<typename Elem>
struct BidirectionalGUIInterpolator
{
  BidirectionalGUIInterpolator(const Elem & elem, double min, double max, double dt, double duration) : elem_(elem), dt_(dt), interp_(min, max, duration) {}
  ~BidirectionalGUIInterpolator()
  {
    if(gui_)
    {
      gui_->removeElements(this);
      mc_rtc::log::info("Removed {} from GUI", elem_);
    }
  }

  void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> category)
  {
    using namespace mc_rtc::gui;
    gui_ = &gui;
    gui.addElement(this,
        category,
        ElementsStacking::Horizontal,
        Button("Add " + elem_,
            [this]()
            {
              mc_rtc::log::info("Add {} pressed", elem_);
              remove_ = false;
              add_ = true;
            }),
        NumberSlider(elem_,
            [this]() { return interpValue_; },
            [this](double val) { setInterpValue(interpValue_ = val);
            interp_.setValue(val);
            },
            interp_.minVal_,
            interp_.maxVal_),
        // NumberInput(elem_ + "_duration",
        //     [this]() { return interpValue_; },
        //     [this](double val) { setInterpValue(interpValue_ = val);
        //     interp_.setValue(val);
        //     }),
        Button("Remove " + elem_,
            [this]()
            {
              mc_rtc::log::info("Remove {} pressed", elem_);
              add_ = false;
              remove_ = true;
            })
        );
  }

  void update()
  {
    // mc_rtc::log::info("Updating {}: {}, {}, {}", elem_, add_, remove_, interpValue_);
    if(add_)
    {
      setInterpValue(interp_(dt_));
    }
    else if(remove_)
    {
      setInterpValue(interp_(-dt_));
    }
    if(interp_.complete())
    {
      add_ = false;
      remove_ = false;
    }
  }

  double interpValue() const noexcept { return interpValue_; }

  void setInterpValue(double val)
  {
    interpValue_ = val;
  }

  bool operator==(const BidirectionalGUIInterpolator & other) const noexcept { return elem_ == other.elem_; }
  bool operator==(const Elem & other) const noexcept { return elem_ == elem_; }

  const Elem & elem_;
  protected:
    mc_rtc::gui::StateBuilder * gui_ = nullptr;
    double interpValue_ = 0.;
    double dt_ = 0.0;
    BidirectionalLinearInterpolation interp_;
    bool add_ = false;
    bool remove_ = false;
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
    // update();
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
        // gui_->removeElement(category_, *it);
        activeElements_.remove_if([this, it](const BidirectionalGUIInterpolator<T> & elem) { return elem.elem_ == *it; });
        it = trackedSetInGUI_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    for(auto it = activeElements_.begin(); it != activeElements_.end(); )
    {
      it->update();
      callback_(it->elem_, it->interpValue());
      ++it;
    }
      // if((*it)())
      // {
      //   // interpolation complete
      //   fmt::print("Interpolation of {} complete\n",
      //      it->elem_);
      //   callbackComplete_(it->elem_, it->interpValue());
      //   gui_->removeElement(category_, it->elem_ + "_action");
      //   it = activeElements_.erase(it); // erase returns the next valid iterator
      // }
      // else
      // {
      //   ++it;
      // }
    // }
  }

  protected:
    void addElement(const T & elem)
    {
      trackedSetInGUI_.insert(elem);
      auto & elemUpdater = activeElements_.emplace_back(elem, 0., 1., 0.005, 2.0);
      elemUpdater.addToGUI(*gui_, category_);
      gui_->addElement(category_,
          mc_rtc::gui::Button("Remove element " + elem,
          [this,&elemUpdater]()
          {
            mc_rtc::log::info("Remove {} pressed", elemUpdater.elem_);
            trackedSet_->erase(elemUpdater.elem_);
            // activeElements_.remove(elemUpdater);
            // trackedSetInGUI_.erase(elemUpdater.elem_);
          }));

      // gui_->addElement(category_,
      //   mc_rtc::gui::Button("Add " + elem,
      //     [this, &elem]()
      //     {
      //       fmt::print("Pressed {}\n", elem);
      //       if(std::find(activeElements_.begin(), activeElements_.end(), elem) != activeElements_.end())
      //       {
      //         // already active
      //         return;
      //       }
      //       const auto & elemUpdater = activeElements_.emplace_back(elem, 0., 1., 0.005, 2.0);
      //         gui_->addElement(category_,
      //             mc_rtc::gui::Label(elem + "_action",
      //               [this, &elemUpdater]()
      //               {
      //                 return elemUpdater.elem_ + " : " + std::to_string(elemUpdater.interpValue());
      //               })
      //             );
      //     }
      //     ));
    }

    std::set<T> * trackedSet_ = nullptr;
    std::set<T> trackedSetInGUI_;
    // cannot use set here as it always provides const access to its elements
    // std::list<ElemWithInterpolation<T, double>> activeElements_;
    std::list<BidirectionalGUIInterpolator<T>> activeElements_;
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
