#####################IDK###########REVIEW
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.hpp>
#include <robot_controllers_interface/controller.h>
#include <robot_controllers_interface/controller_manager.h>
#include <robot_controllers_interface/joint_handle.h>

namespace fetch_twist_controller {
class JointVelocityController : public robot_controllers::Controller {
  std::vector<std::string> names_;
  std::vector<robot_controllers::JointHandlePtr> joints_;
  std::vector<double> command_, limits_;
  ros::Subscriber sub_;
  ros::WallTime received_;
  ros::Time stamp_;
  std::mutex mutex_;
  double timeout_=0.2, margin_=0.05;
  bool initialized_=false;
public:
  int init(ros::NodeHandle& nh, robot_controllers::ControllerManager* manager) override {
    if (!manager) throw std::runtime_error("No Fetch controller manager");
    Controller::init(nh, manager);
    nh.param("timeout",timeout_,0.2); nh.param("position_margin",margin_,0.05);
    if (!nh.getParam("joints",names_) || names_.empty() ||
        !nh.getParam("velocity_limits",limits_) || names_.size()!=limits_.size() ||
        !std::isfinite(timeout_) || timeout_<=0 || !std::isfinite(margin_) || margin_<0)
      throw std::runtime_error("Invalid velocity plugin configuration");
    for (size_t i=0; i<names_.size(); ++i) {
      auto joint=manager->getJointHandle(names_[i]);
      if (!joint || !std::isfinite(limits_[i]) || limits_[i]<=0)
        throw std::runtime_error("Missing joint or invalid limit: "+names_[i]);
      joints_.push_back(joint);
      if (joint->getVelocityMax()>0) limits_[i]=std::min(limits_[i],joint->getVelocityMax());
    }
    command_.assign(names_.size(),0.0);
    sub_=nh.subscribe<sensor_msgs::JointState>("command",1,&JointVelocityController::command,this);
    initialized_=true; return 0;
  }
  void command(const sensor_msgs::JointState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::fill(command_.begin(),command_.end(),0.0);
    received_=ros::WallTime(); stamp_=ros::Time();
    // Require exact ordered joint names: prevents accidental wheel/torso commands.
    if (msg->name!=names_ || msg->velocity.size()!=names_.size()) return;
    double age=(ros::Time::now()-msg->header.stamp).toSec();
    if (msg->header.stamp.isZero() || age>timeout_ || age < -0.05) return;
    for (double v:msg->velocity) if (!std::isfinite(v)) return;
    command_=msg->velocity; received_=ros::WallTime::now(); stamp_=msg->header.stamp;
  }
  bool start() override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::fill(command_.begin(),command_.end(),0.0); received_=ros::WallTime();
    return initialized_;
  }
  bool stop(bool /*force*/) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::fill(command_.begin(),command_.end(),0.0); received_=ros::WallTime();
    for (auto& j:joints_) j->setVelocity(0.0,0.0);
    return true;
  }
  bool reset() override { return stop(true); }
  void update(const ros::Time& now,const ros::Duration& dt) override {
    std::lock_guard<std::mutex> lock(mutex_);
    double age=(now-stamp_).toSec();
    bool stale=received_.isZero() || (ros::WallTime::now()-received_).toSec()>timeout_
        || age>timeout_ || age < -0.05;
    std::vector<double> v=command_;
    double scale=1.0;
    for (size_t i=0;i<v.size();++i)
      if (std::abs(v[i])>limits_[i]) scale=std::min(scale,limits_[i]/std::abs(v[i]));
    for (size_t i=0;i<v.size();++i) {
      double speed=stale?0.0:v[i]*scale;
      double q=joints_[i]->getPosition();
      if (!std::isfinite(q)) speed=0.0;
      if (!joints_[i]->isContinuous()) {
        double horizon=std::max(0.001,dt.toSec());
        double lo=joints_[i]->getPositionMin()+margin_;
        double hi=joints_[i]->getPositionMax()-margin_;
        if (speed>0) speed=std::min(speed,std::max(0.0,(hi-q)/horizon));
        if (speed<0) speed=std::max(speed,std::min(0.0,(lo-q)/horizon));
      }
      joints_[i]->setVelocity(speed,0.0);
    }
  }
  std::string getType() override { return "fetch_twist_controller/JointVelocityController"; }
  std::vector<std::string> getCommandedNames() override {return names_;}
  std::vector<std::string> getClaimedNames() override {return names_;}
};
}
PLUGINLIB_EXPORT_CLASS(fetch_twist_controller::JointVelocityController,robot_controllers::Controller)
