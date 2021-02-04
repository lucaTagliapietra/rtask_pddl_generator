#include "pddl_generator/LiteralExpression.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

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
  if (!helpers::checkXmlRpcSanity("expr", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)) {
    std::cerr << "Fatal: Invalid LiteralExpression Structure, should be a Struct" << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string name{};
  std::vector<std::string> args{};

  if (helpers::checkXmlRpcSanity("name", t_rpc_val["expr"], XmlRpc::XmlRpcValue::TypeString, true)) {
    name = static_cast<std::string>(t_rpc_val["expr"]["name"]);
  }

  if (helpers::checkXmlRpcSanity("args", t_rpc_val["expr"], XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["expr"]["args"].size() == 0) {
      std::cout << "Empty args vector for LiteralLogicalExpression " << name << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["expr"]["args"].size(); ++i) {
        if (t_rpc_val["expr"]["args"][i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          args.emplace_back(static_cast<std::string>(t_rpc_val["expr"]["args"][i]));
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

LiteralExpression& LiteralExpression::operator=(const LiteralExpression& t_other)
{
  set(t_other.getExpressionName(), t_other.getExpressionArgs());
  return *this;
}

std::string LiteralExpression::toPddl(bool, int t_pad_lv) const
{
  if (expr_name_.empty()) {
    return {};
  };

  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_;
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

bool rtask::commons::pddl_generator::operator==(const LiteralExpression& t_first, const LiteralExpression& t_second)
{
  if (t_first.getExpressionName() != t_second.getExpressionName()) {
    return false;
  }

  const auto& f_args = t_first.getExpressionArgs();
  const auto& s_args = t_second.getExpressionArgs();

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

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const LiteralExpression& t_expr)
{
  t_out << "LiteralExpression: name: " << t_expr.getExpressionName() << std::endl;
  unsigned int i = 0;
  for (const auto& a : t_expr.getExpressionArgs()) {
    (i != 0) ? t_out << std::endl : t_out << "";
    t_out << " - args[" << i << "]: " << a;
    ++i;
  }
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<LiteralExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : LiteralExpression());
  return t_out;
}
