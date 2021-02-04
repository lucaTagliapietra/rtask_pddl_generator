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

std::string WhenExpression::toPddl(bool t_typing, int t_pad_lv) const
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

WhenExpression& WhenExpression::operator=(const WhenExpression& t_other)
{
  condition_ = t_other.getCondition();
  consequence_ = t_other.getConsequence();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const WhenExpression& t_first, const WhenExpression& t_second)
{
  return helpers::operator==(*t_first.getCondition(), *t_second.getCondition())
         && helpers::operator==(*t_first.getConsequence(), *t_second.getConsequence());
};

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const WhenExpression& t_expr)

{
  t_out << "WhenExpression: name: " << t_expr.getExpressionName() << std::endl;
  t_out << " - condition: " << t_expr.getCondition() << std::endl;
  t_out << " - consequence: " << t_expr.getConsequence();
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<WhenExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : WhenExpression());
  return t_out;
}
