#include "pddl_generator/LiteralExpression.h"
#include "pddl_generator/Helpers.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
LiteralExpression::LiteralExpression()
{
  expr_type_ = LogicalExpressionType::Literal;
  expr_name_ = {};
}

LiteralExpression::LiteralExpression(const std::string& t_name, const std::vector<std::string>& t_args)
{
  expr_type_ = ::LogicalExpressionType::Literal;
  set(t_name, t_args);
}

LiteralExpression::LiteralExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  std::string name{};
  std::vector<std::string> args{};

  if (helpers::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    name = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (helpers::checkXmlRpcSanity("args", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["args"].size() == 0) {
      std::cout << "Empty args vector for LiteralLogicalExpression " << name << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["args"].size(); ++i) {
        if (t_rpc_val["args"][i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          args.emplace_back(static_cast<std::string>(t_rpc_val["args"][i]));
        }
      }
    }
  }

  set(name, args);
  expr_type_ = LogicalExpressionType::Literal;
}

void LiteralExpression::clear()
{
  expr_name_.clear();
  args_.clear();
}

bool LiteralExpression::set(const std::string& t_name, const std::vector<std::string>& t_args)
{
  return setExpressionName(t_name) && setExpressionArgs(t_args);
}

bool LiteralExpression::setExpressionName(const std::string& t_expr_name)
{
  if (t_expr_name.empty()) {
    std::cerr << "Empty LiteralExpression name" << std::endl;
    return false;
  }
  expr_name_ = std::move(t_expr_name);
  return true;
}

bool LiteralExpression::setExpressionArgs(const std::vector<std::string>& t_args)
{
  // TODO: add policy to handle duplicates
  args_ = std::move(t_args);
  return true;
}

bool LiteralExpression::operator==(const LiteralExpression& t_other) const
{
  if (expr_name_ != t_other.getExpressionName()) {
    return false;
  }

  const auto& other_args = t_other.getExpressionArgs();

  if (args_.size() != other_args.size()) {
    return false;
  }
  for (unsigned int i = 0; i < args_.size(); ++i) {
    if (args_.at(i) != other_args.at(i)) {
      return false;
    }
  }
  return true;
}

LiteralExpression& LiteralExpression::operator=(const LiteralExpression& t_other)
{
  set(t_other.getExpressionName(), t_other.getExpressionArgs());
  return *this;
}

std::string LiteralExpression::toPddl(const bool) const
{
  if (expr_name_.empty()) {
    return {};
  };

  std::string out{};
  out += " (" + expr_name_;
  for (const auto& a : args_) {
    out += " ?" + a;
  }
  out += ")";
  return out;
}

// TODO: This strongly depends on the implementation of the action class, check it
bool LiteralExpression::validate(const UnordStrToLitTermMap& t_known_constants,
                                 const UnordStrToUIntMap& t_belonging_action_args,
                                 const std::string& t_belonging_action_name) const
{
  if (expr_name_.empty()) {
    std::cerr << "VALIDATION ERROR: Empty LiteralExpression name" << std::endl;
    return false;
  }

  for (const auto& arg : args_) {
    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
      std::cerr << "\t(In LiteralExpression **" << expr_name_ << "** of Action **" << t_belonging_action_name << "**)"
                << std::endl;
      return false;
    }
  }
  return true;
}