#ifndef PLUGINLIB_CALCULATOR_CALCULTOR_BASE_H_
#define PLUGINLIB_CALCULATOR_CALCULTOR_BASE_H_

namespace calculator_base 
{
  class calc_functions
  {
    public:
      virtual void get_numbers(double number1, double number2) = 0;
      virtual double operation() = 0;
      virtual ~calc_functions(){}

    protected:
      calc_functions(){}
  };
};
#endif
