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
  if (!helpers::checkXmlRpcSanity("not", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)) {
    std::cerr << "Fatal: Invalid NotExpression Structure, should be a Struct" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["not"]);

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

std::string NotExpression::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_;
  out += aligners[1] + ::helpers::logicalExprToPddl(expr_, t_typing, pad_lv);
  out += aligners[2] + ")";

  return out;
}

NotExpression& NotExpression::operator=(const NotExpression& t_other)
{
  expr_ = t_other.getExpression();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const NotExpression& t_first, const NotExpression& t_second)
{
  return helpers::operator==(*t_first.getExpression(), *t_second.getExpression());
};

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const NotExpression& t_expr)
{
  t_out << "NotExpression: name: " << t_expr.getExpressionName() << std::endl;
  t_out << t_expr.getExpression();
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<NotExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : NotExpression());
  return t_out;
}

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
