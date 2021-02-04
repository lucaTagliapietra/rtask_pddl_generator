#include "pddl_generator/ForAllExpression.h"
using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
ForAllExpression::ForAllExpression()
{
  expr_type_ = LogicalExpressionType::ForAll;
  expr_name_ = "forall";
}

ForAllExpression::ForAllExpression(const LiteralTerm& t_what, const LogicalExpression& t_condition)
{
  expr_type_ = LogicalExpressionType::ForAll;
  expr_name_ = "forall";
  set(std::make_shared<LiteralTerm>(t_what), std::make_shared<LogicalExpression>(t_condition));
}

ForAllExpression::ForAllExpression(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr)
{
  expr_type_ = LogicalExpressionType::ForAll;
  expr_name_ = "forall";
  set(t_what_ptr, t_condition_ptr);
}

ForAllExpression::ForAllExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("forall", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("what", t_rpc_val["forall"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("condition", t_rpc_val["forall"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid ForAllExpression Structure, should be a Struct with what and condition fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  what_ = std::make_shared<LiteralTerm>(t_rpc_val["forall"]["what"]);
  if (!what_) {
    std::cerr << "Fatal: Invalid LiteralTerm as what of current ExistsExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  condition_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["forall"]["condition"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as condition of current ExistsExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_name_ = "forall";
  expr_type_ = LogicalExpressionType::ForAll;
}

void ForAllExpression::clear()
{
  what_.reset();
  condition_.reset();
}

void ForAllExpression::set(LiteralTermPtr t_what_ptr, LogicalExprPtr t_condition_ptr)
{
  setWhat(t_what_ptr);
  setCondition(t_condition_ptr);
}

std::string ForAllExpression::toPddl(bool t_typing, int t_pad_lv) const
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

ForAllExpression& ForAllExpression::operator=(const ForAllExpression& t_other)
{
  what_ = t_other.getWhat();
  condition_ = t_other.getCondition();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const ForAllExpression& t_first, const ForAllExpression& t_second)
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
