#include "pddl_generator/CompareExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
CompareExpression::CompareExpression()
{
  expr_type_ = LogicalExpressionType::Compare;
}

CompareExpression::CompareExpression(const std::string& t_op_name,
                                     const NumericalExpression& t_lhs,
                                     const NumericalExpression& t_rhs)
{
  expr_type_ = LogicalExpressionType::Compare;
  set(t_op_name, std::make_shared<NumericalExpression>(t_lhs), std::make_shared<NumericalExpression>(t_rhs));
}

CompareExpression::CompareExpression(const std::string& t_op_name, NumericalExprPtr t_lhs, NumericalExprPtr t_rhs)
{
  expr_type_ = LogicalExpressionType::Compare;
  set(t_op_name, t_lhs, t_rhs);
}

CompareExpression::CompareExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("compare", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("operator", t_rpc_val["compare"], XmlRpc::XmlRpcValue::Type::TypeString, false)
        && helpers::checkXmlRpcSanity("lhs", t_rpc_val["compare"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("rhs", t_rpc_val["compare"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid Compare Structure, should be a Struct with operator, lhs, and rhs fields" << std::endl;
    exit(EXIT_FAILURE);
  }

  auto name = static_cast<std::string>(t_rpc_val["compare"]["operator"]);

  lhs_expr_ = helpers::getNumericalExprFromXmlRpc(t_rpc_val["compare"]["lhs"]);
  if (!lhs_expr_) {
    std::cerr << "Fatal: Invalid Numerical Expression as lhs of current CompareExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  rhs_expr_ = helpers::getNumericalExprFromXmlRpc(t_rpc_val["compare"]["rhs"]);
  if (!rhs_expr_) {
    std::cerr << "Fatal: Invalid Numerical Expression as rhs of current CompareExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  setComparisonOperator(name);
  expr_type_ = LogicalExpressionType::Compare;
}

void CompareExpression::clear()
{
  expr_name_.clear();
  lhs_expr_.reset();
  rhs_expr_.reset();
}

bool CompareExpression::set(const std::string& t_op_name, NumericalExprPtr t_lhs, NumericalExprPtr t_rhs)
{
  setLhsExpression(t_lhs);
  setRhsExpression(t_rhs);
  return setComparisonOperator(t_op_name);
}

bool CompareExpression::setComparisonOperator(const std::string& t_op_name)
{
  if (t_op_name.empty()
      || (t_op_name != "<" && t_op_name != "<=" && t_op_name != "=" && t_op_name != ">" && t_op_name != ">=")) {
    std::cerr << "Empty or invalid Comparison Operator name : " << t_op_name << std::endl;
    exit(EXIT_FAILURE);
  }
  expr_name_ = std::move(t_op_name);
  return true;
}

CompareExpression CompareExpression::mirror() const
{
  std::string mirrored_expr_name = expr_name_;
  switch (mirrored_expr_name.front()) {
    case '>':
      mirrored_expr_name.front() = '<';
      break;
    case '<':
      mirrored_expr_name.front() = '>';
      break;
    default:
      break;
  }
  return {mirrored_expr_name, rhs_expr_, lhs_expr_};
}

bool CompareExpression::equals(const CompareExpression& t_other) const
{
  return (expr_name_ == t_other.getComparisonOperator() && helpers::operator==(*lhs_expr_, *t_other.lhs_expr_)
          && helpers::operator==(*rhs_expr_, *t_other.rhs_expr_));
}

std::string CompareExpression::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_;
  out += aligners[1] + ::helpers::numericalExprToPddl(lhs_expr_, t_typing, pad_lv);
  out += aligners[1] + ::helpers::numericalExprToPddl(rhs_expr_, t_typing, pad_lv);
  out += aligners[2] + ")";

  return out;
}

CompareExpression& CompareExpression::operator=(const CompareExpression& t_other)
{
  expr_name_ = t_other.getComparisonOperator();
  lhs_expr_ = t_other.getLhsExpression();
  rhs_expr_ = t_other.getRhsExpression();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const CompareExpression& t_first, const CompareExpression& t_second)
{
  return t_first.equals(t_second) || t_first.equals(t_second.mirror());
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

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const CompareExpression& t_expr)
{
  t_out << "CompareExpression: operator: " << t_expr.getComparisonOperator() << std::endl;
  t_out << " - lhs: " << t_expr.getLhsExpression() << std::endl;
  t_out << " - rhs: " << t_expr.getRhsExpression();
  return t_out;
}
std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<CompareExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : CompareExpression());
  return t_out;
}
