#include "pddl_generator/NotExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
NotExpression::NotExpression()
{
  expr_type_ = LogicalExpressionType::Not;
  expr_name_ = "not";
}

NotExpression::NotExpression(const LogicalExpression& t_expr)
{
  expr_type_ = LogicalExpressionType::Not;
  expr_name_ = "not";
  set(std::make_shared<LogicalExpression>(t_expr));
}

NotExpression::NotExpression(LogicalExprPtr t_expr_ptr)
{
  expr_type_ = LogicalExpressionType::Not;
  expr_name_ = "not";
  set(t_expr_ptr);
}

NotExpression::NotExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  expr_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val);

  if (!expr_) {
    std::cerr << "Fatal: Invalid Boolean Expression as argument of current NotExpression" << std::endl;
    exit(EXIT_FAILURE);
  }
  expr_name_ = "not";
  expr_type_ = LogicalExpressionType::Not;
}

void NotExpression::clear()
{
  expr_.reset();
}

std::string NotExpression::toPddl(const bool t_typing) const
{
  std::string out{};
  out += " (" + expr_name_;
  out += ::helpers::logicalExprToPddl(expr_, t_typing);
  out += ")";
  return out;
}

NotExpression& NotExpression::operator=(const NotExpression& t_other)
{
  expr_.reset(t_other.getExpression().get());
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const NotExpression& t_first, const NotExpression& t_second)
{
  return helpers::operator==(*t_first.getExpression(), *t_second.getExpression());
};

//// TODO: This strongly depends on the implementation of the action class, check it
// bool LiteralLogicalExpression::validate(const UnordStrToLitTermMap& t_known_constants,
//                                        const UnordStrToUIntMap& t_belonging_action_args,
//                                        const std::string& t_belonging_action_name) const
//{
//  if (expr_name_.empty()) {
//    std::cerr << "VALIDATION ERROR: Empty LiteralLogicalExpression name" << std::endl;
//    return false;
//  }

//  for (const auto& arg : args_) {
//    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
//      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
//      std::cerr << "\t(In LiteralLogicalExpression **" << expr_name_ << "** of Action **"
//                << t_belonging_action_name << "**)" << std::endl;
//      return false;
//    }
//  }
//  return true;
//}
