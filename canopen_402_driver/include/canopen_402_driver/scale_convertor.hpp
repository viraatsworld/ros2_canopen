#ifndef SCALE_CONVERTOR_HPP
#define SCALE_CONVERTOR_HPP

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <regex>
#include <string>

#include "muParser.h"

namespace ros2_canopen
{
class ScaleConverter
{
public:
  ScaleConverter(const std::string & input)
  {
    parser_ = std::make_shared<mu::Parser>();

    parser_->DefineConst("pi", M_PI);
    parser_->DefineConst("nan", std::numeric_limits<double>::quiet_NaN());
    parser_->DefineFun("rad2deg", rad2deg);
    parser_->DefineFun("deg2rad", deg2rad);
    parser_->DefineFun("avg", avg);
    parser_->DefineFun("norm", norm);
    parser_->DefineFun("smooth", smooth);

    std::size_t commaPosition = input.find(',');
    std::string expression;
    if (commaPosition != std::string::npos)
    {
      expression = input.substr(0, commaPosition);
    }
    else
    {
      expression = input;
    }

    extractVariables(input);
    for (auto & [key, value] : variables_)
    {
      parser_->DefineVar(key, value);
    }

    parser_->SetExpr(expression);
  }

  double eval() { return parser_->Eval(); }

private:
  std::shared_ptr<mu::Parser> parser_;
  std::map<std::string, double *> variables_;

  static double rad2deg(double rad) { return rad * 180.0 / M_PI; }
  static double deg2rad(double deg) { return deg * M_PI / 180.0; }
  static double avg(const double * val, int num)
  {
    double sum = 0;
    for (int i = 0; i < num; i++)
    {
      const double & v = val[i];
      if (std::isnan(v)) break;
      sum += v;
    }
    return sum / num;
  }
  static double norm(double val, double min, double max)
  {
    while (val >= max) val -= (max - min);
    while (val < min) val += (max - min);
    return val;
  }
  static double smooth(double val, double old_val, double alpha)
  {
    if (std::isnan(val)) return 0;
    if (std::isnan(old_val)) return val;
    return alpha * val + (1.0 - alpha) * old_val;
  }
  void extractVariables(const std::string & expression)
  {
    std::regex variableRegex("([a-zA-Z_][a-zA-Z0-9_]*)\\s*=\\s*([\\d.]+)");
    std::sregex_iterator iterator(expression.begin(), expression.end(), variableRegex);
    std::sregex_iterator endIterator;

    while (iterator != endIterator)
    {
      std::smatch match = *iterator;
      std::string variableName = match[1].str();
      double variableValue = std::stod(match[2].str());
      variables_[variableName] = new double(variableValue);
      ++iterator;
    }
  }
};
}  // namespace ros2_canopen
#endif  // SCALE_CONVERTOR_HPP
