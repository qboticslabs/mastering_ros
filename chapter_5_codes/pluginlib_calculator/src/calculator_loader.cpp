
#include <boost/shared_ptr.hpp>

#include <pluginlib/class_loader.h>
#include <pluginlib_calculator/calculator_base.h>

int main(int argc, char** argv)
{


  pluginlib::ClassLoader<calculator_base::calc_functions> calc_loader("pluginlib_calculator", "calculator_base::calc_functions");

  try
  {
    boost::shared_ptr<calculator_base::calc_functions> add = calc_loader.createInstance("pluginlib_calculator/Add");
    add->get_numbers(10.0,10.0);
    double result = add->operation();

    ROS_INFO("Triangle area: %.2f", result);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }


  try
  {
    boost::shared_ptr<calculator_base::calc_functions> sub = calc_loader.createInstance("pluginlib_calculator/Sub");

    sub->get_numbers(10.0,10.0);
    double result = sub->operation();

    ROS_INFO("Substracted result: %.2f", result);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }




  try
  {
    boost::shared_ptr<calculator_base::calc_functions> mul = calc_loader.createInstance("pluginlib_calculator/Mul");
    mul->get_numbers(10.0,10.0);
    double result = mul->operation();

    ROS_INFO("Multiplied result: %.2f", result);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }




  try
  {
    boost::shared_ptr<calculator_base::calc_functions> div = calc_loader.createInstance("pluginlib_calculator/Div");
    div->get_numbers(10.0,10.0);
    double result = div->operation();

    ROS_INFO("Division result: %.2f", result);
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}
