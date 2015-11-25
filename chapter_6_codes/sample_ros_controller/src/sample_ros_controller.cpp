#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

namespace controller_ns{

class PositionController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {
    // get joint name from the parameter server
    std::string my_joint;
    if (!n.getParam("joint_name", my_joint)){
      ROS_ERROR("Could not find joint name");
      return false;
    }

    // get the joint object to use in the realtime loop
    joint_ = hw->getHandle("r_shoulder_pan_joint");  // throws on failure
    return true;
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {

//    double error = setpoint_ - joint_.getPosition();
//    joint_.setCommand(error*gain_);
  double desired_pos = init_pos_ + 15 * sin(ros::Time::now().toSec());
  double current_pos = joint_.getPosition();
  joint_.setCommand(-10 * (current_pos - desired_pos));

  }

  void starting(const ros::Time& time) {
  	init_pos_ = joint_.getPosition();
 }
  void stopping(const ros::Time& time) { }

private:
  hardware_interface::JointHandle joint_;
  static const double gain_ = 1.25;
  static const double setpoint_ = 3.00;
  double init_pos_;
};
PLUGINLIB_DECLARE_CLASS(sample_ros_controller, PositionController, controller_ns::PositionController, controller_interface::ControllerBase);
}//
