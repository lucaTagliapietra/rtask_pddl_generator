#include "pddl_generator/NumericalFunction.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
NumericalFunction::NumericalFunction()
{
  expr_type_ = NumericalExpressionType::Function;
  function_name_ = {};
}

NumericalFunction::NumericalFunction(const std::string& t_name, const std::vector<std::string>& t_args)
{
  expr_type_ = ::NumericalExpressionType::Function;
  set(t_name, t_args);
}

NumericalFunction::NumericalFunction(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!helpers::checkXmlRpcSanity("num_fnc", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)) {
    std::cerr << "Fatal: Invalid NumericalFunction Structure, should be a Struct" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string name{};
  std::vector<std::string> args{};

  if (helpers::checkXmlRpcSanity("name", t_rpc_val["num_fnc"], XmlRpc::XmlRpcValue::TypeString, true)) {
    name = static_cast<std::string>(t_rpc_val["num_fnc"]["name"]);
  }

  if (helpers::checkXmlRpcSanity("args", t_rpc_val["num_fnc"], XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["num_fnc"]["args"].size() != 0) {
      for (int i = 0; i < t_rpc_val["num_fnc"]["args"].size(); ++i) {
        if (t_rpc_val["num_fnc"]["args"][i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          args.emplace_back(static_cast<std::string>(t_rpc_val["num_fnc"]["args"][i]));
        }
      }
    }
  }

  set(name, args);
  expr_type_ = NumericalExpressionType::Function;
}

void NumericalFunction::clear()
{
  function_name_.clear();
  args_.clear();
}

bool NumericalFunction::set(const std::string& t_name, const std::vector<std::string>& t_args)
{
  return setFunctionName(t_name) && setFunctionArgs(t_args);
}

bool NumericalFunction::setFunctionName(const std::string& t_fnc_name)
{
  if (t_fnc_name.empty()) {
    std::cerr << "Empty NumericalFunction name" << std::endl;
    return false;
  }
  function_name_ = std::move(t_fnc_name);
  return true;
}

bool NumericalFunction::setFunctionArgs(const std::vector<std::string>& t_args)
{
  // TODO: add policy to handle duplicates
  args_ = std::move(t_args);
  return true;
}

NumericalFunction& NumericalFunction::operator=(const NumericalFunction& t_other)
{
  set(t_other.getFunctionName(), t_other.getFunctionArgs());
  return *this;
}

std::string NumericalFunction::toPddl(bool, int t_pad_lv) const
{
  if (function_name_.empty()) {
    return {};
  };

  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + function_name_;
  for (const auto& a : args_) {
    out += " ?" + a;
  }
  out += ")";

  return out;
}

bool rtask::commons::pddl_generator::operator==(const NumericalFunction& t_first, const NumericalFunction& t_second)
{
  if (t_first.getFunctionName() != t_second.getFunctionName()) {
    return false;
  }

  const auto& f_args = t_first.getFunctionArgs();
  const auto& s_args = t_second.getFunctionArgs();

  if (f_args.size() != s_args.size()) {
    return false;
  }
  for (const auto& fs : f_args) {
    const auto& it = std::find_if(s_args.begin(), s_args.end(), [fs](const auto& ss) { return ss == fs; });
    if (it == s_args.end()) {
      return false;
    }
  }
  for (const auto& ss : s_args) {
    const auto& it = std::find_if(f_args.begin(), f_args.end(), [ss](const auto& fs) { return ss == fs; });
    if (it == f_args.end()) {
      return false;
    }
  }
  return true;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const NumericalFunction& t_expr)
{
  t_out << "NumericalFunction: name: " << t_expr.getFunctionName() << std::endl;
  unsigned int i = 0;
  for (const auto& a : t_expr.getFunctionArgs()) {
    (i != 0) ? t_out << std::endl : t_out << "";
    t_out << " - args[" << i << "]: " << a;
    ++i;
  }
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         const std::shared_ptr<NumericalFunction> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : NumericalFunction());
  return t_out;
}

//// TODO: This strongly depends on the implementation of the action class, check it
// bool LiteralExpression::validate(const UnordStrToLitTermMap& t_known_constants,
//                                 const UnordStrToUIntMap& t_belonging_action_args,
//                                 const std::string& t_belonging_action_name) const
//{
//  if (expr_name_.empty()) {
//    std::cerr << "VALIDATION ERROR: Empty LiteralExpression name" << std::endl;
//    return false;
//  }

//  for (const auto& arg : args_) {
//    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
//      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
//      std::cerr << "\t(In LiteralExpression **" << expr_name_ << "** of Action **" << t_belonging_action_name << "**)"
//                << std::endl;
//      return false;
//    }
//  }
//  return true;
//}
