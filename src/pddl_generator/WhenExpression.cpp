#include "pddl_generator/WhenExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
WhenExpression::WhenExpression()
{
  expr_type_ = LogicalExpressionType::When;
  expr_name_ = "when";
}

WhenExpression::WhenExpression(const LogicalExpression& t_condition, const LogicalExpression& t_consequence)
{
  expr_type_ = LogicalExpressionType::When;
  expr_name_ = "when";
  set(std::make_shared<LogicalExpression>(t_condition), std::make_shared<LogicalExpression>(t_consequence));
}

WhenExpression::WhenExpression(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr)
{
  expr_type_ = LogicalExpressionType::When;
  expr_name_ = "when";
  set(t_condition_ptr, t_consequence_ptr);
}

WhenExpression::WhenExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("when", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("condition", t_rpc_val["when"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("consequence", t_rpc_val["when"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid WhenExpression Structure, should be a Struct with condition and consequence fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  condition_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["when"]["condition"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as condition of current WhenExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  consequence_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["when"]["consequence"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as consequence of current WhenExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_name_ = "when";
  expr_type_ = LogicalExpressionType::When;
}

void WhenExpression::clear()
{
  condition_.reset();
  consequence_.reset();
}

void WhenExpression::set(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr)
{
  setCondition(t_condition_ptr);
  setConsequence(t_consequence_ptr);
}

std::string WhenExpression::toPddl(const bool t_typing) const
{
  std::string out{};
  out += " (" + expr_name_;
  out += ::helpers::logicalExprToPddl(condition_, t_typing);
  out += ::helpers::logicalExprToPddl(consequence_, t_typing);
  out += ")";
  return out;
}

WhenExpression& WhenExpression::operator=(const WhenExpression& t_other)
{
  condition_.reset(t_other.getCondition().get());
  consequence_.reset(t_other.getConsequence().get());
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const WhenExpression& t_first, const WhenExpression& t_second)
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
