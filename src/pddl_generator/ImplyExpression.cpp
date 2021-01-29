#include "pddl_generator/ImplyExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
ImplyExpression::ImplyExpression()
{
  expr_type_ = LogicalExpressionType::Imply;
  expr_name_ = "imply";
}

ImplyExpression::ImplyExpression(const LogicalExpression& t_condition, const LogicalExpression& t_consequence)
{
  expr_type_ = LogicalExpressionType::Imply;
  expr_name_ = "imply";
  set(std::make_shared<LogicalExpression>(t_condition), std::make_shared<LogicalExpression>(t_consequence));
}

ImplyExpression::ImplyExpression(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr)
{
  expr_type_ = LogicalExpressionType::Imply;
  expr_name_ = "imply";
  set(t_condition_ptr, t_consequence_ptr);
}

ImplyExpression::ImplyExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("imply", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("condition", t_rpc_val["imply"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("consequence", t_rpc_val["imply"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid ImplyExpression Structure, should be a Struct with condition and consequence fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  condition_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["imply"]["condition"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as condition of current ImplyExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  consequence_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["imply"]["consequence"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as consequence of current ImplyExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_name_ = "imply";
  expr_type_ = LogicalExpressionType::Imply;
}

void ImplyExpression::clear()
{
  condition_.reset();
  consequence_.reset();
}

void ImplyExpression::set(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr)
{
  setCondition(t_condition_ptr);
  setConsequence(t_consequence_ptr);
}

std::string ImplyExpression::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_;
  out += aligners[1] + ::helpers::logicalExprToPddl(condition_, t_typing, pad_lv);
  out += aligners[1] + ::helpers::logicalExprToPddl(consequence_, t_typing, pad_lv);
  out += aligners[2] + ")";

  return out;
}

ImplyExpression& ImplyExpression::operator=(const ImplyExpression& t_other)
{
  condition_.reset(t_other.getCondition().get());
  consequence_.reset(t_other.getConsequence().get());
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const ImplyExpression& t_first, const ImplyExpression& t_second)
{
  return helpers::operator==(*t_first.getCondition(), *t_second.getCondition())
         && helpers::operator==(*t_first.getConsequence(), *t_second.getConsequence());
};

////// TODO: This strongly depends on the implementation of the action class, check it
//// bool LiteralLogicalExpression::validate(const UnordStrToLitTermMap& t_known_constants,
////                                        const UnordStrToUIntMap& t_belonging_action_args,
////                                        const std::string& t_belonging_action_name) const
////{
////  if (expr_name_.empty()) {
////    std::cerr << "VALIDATION ERROR: Empty LiteralLogicalExpression name" << std::endl;
////    return false;
////  }

////  for (const auto& arg : args_) {
////    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
////      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
////      std::cerr << "\t(In LiteralLogicalExpression **" << expr_name_ << "** of Action **"
////                << t_belonging_action_name << "**)" << std::endl;
////      return false;
////    }
////  }
////  return true;
////}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const ImplyExpression& t_expr)
{
  t_out << "ImplyExpression name: " << t_expr.getExpressionName() << std::endl;
  t_out << "\t - condition: " << t_expr.getCondition() << std::endl;
  t_out << "\t - consequence: " << t_expr.getConsequence();
  return t_out;
}
