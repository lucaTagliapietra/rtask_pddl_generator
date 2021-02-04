#include "pddl_generator/ExistsExpression.h"
using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
ExistsExpression::ExistsExpression()
{
  expr_type_ = LogicalExpressionType::Exists;
  expr_name_ = "exists";
}

ExistsExpression::ExistsExpression(const LiteralTerm& t_what, const LogicalExpression& t_condition)
{
  expr_type_ = LogicalExpressionType::Exists;
  expr_name_ = "exists";
  set(std::make_shared<LiteralTerm>(t_what), std::make_shared<LogicalExpression>(t_condition));
}

ExistsExpression::ExistsExpression(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr)
{
  expr_type_ = LogicalExpressionType::Exists;
  expr_name_ = "exists";
  set(t_what_ptr, t_condition_ptr);
}

ExistsExpression::ExistsExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("exists", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("what", t_rpc_val["exists"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("condition", t_rpc_val["exists"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid ExistsExpression Structure, should be a Struct with condition and consequence fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  what_ = std::make_shared<LiteralTerm>(t_rpc_val["exists"]["what"]);
  if (!what_) {
    std::cerr << "Fatal: Invalid LiteralTerm as what of current ExistsExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  condition_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["exists"]["condition"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as condition of current ExistsExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_name_ = "exists";
  expr_type_ = LogicalExpressionType::Exists;
}

void ExistsExpression::clear()
{
  what_.reset();
  condition_.reset();
}

void ExistsExpression::set(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr)
{
  setWhat(t_what_ptr);
  setCondition(t_condition_ptr);
}

std::string ExistsExpression::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_;
  out += " (" + what_->toPddl(t_typing) + ")";
  out += aligners[1] + ::helpers::logicalExprToPddl(condition_, t_typing, pad_lv);
  out += aligners[2] + ")";

  return out;
}

ExistsExpression& ExistsExpression::operator=(const ExistsExpression& t_other)
{
  what_ = t_other.getWhat();
  condition_ = t_other.getCondition();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const ExistsExpression& t_first, const ExistsExpression& t_second)
{
  return (*t_first.getWhat() == *t_second.getWhat())
         && helpers::operator==(*t_first.getCondition(), *t_second.getCondition());
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
