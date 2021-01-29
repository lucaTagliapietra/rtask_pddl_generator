#include "pddl_generator/NumericalOperator.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
NumericalOperator::NumericalOperator()
{
  expr_type_ = NumericalExpressionType::Function;
}

NumericalOperator::NumericalOperator(const std::string& t_op_name,
                                     const NumericalExpression& t_lhs,
                                     const NumericalExpression& t_rhs)
{
  expr_type_ = ::NumericalExpressionType::Function;
  set(t_op_name, std::make_shared<NumericalExpression>(t_lhs), std::make_shared<NumericalExpression>(t_rhs));
}

NumericalOperator::NumericalOperator(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("num_op", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("operator", t_rpc_val["num_op"], XmlRpc::XmlRpcValue::Type::TypeString)
        && helpers::checkXmlRpcSanity("lhs", t_rpc_val["num_op"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("rhs", t_rpc_val["num_op"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid NumericalOperator Structure, should be a Struct with oprator, lhs, and rhs fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }
  std::string operator_name = static_cast<std::string>(t_rpc_val["num_op"]["operator"]);

  lhs_expr_ = helpers::getNumericalExprFromXmlRpc(t_rpc_val["num_op"]["lhs"]);
  if (!lhs_expr_) {
    std::cerr << "Fatal: Invalid Numerical Expression as lhs of current NumericalOperator" << std::endl;
    exit(EXIT_FAILURE);
  }

  rhs_expr_ = helpers::getNumericalExprFromXmlRpc(t_rpc_val["num_op"]["rhs"]);
  if (!rhs_expr_) {
    std::cerr << "Fatal: Invalid Numerical Expression as rhs of current NumericalOperator" << std::endl;
    exit(EXIT_FAILURE);
  }
  setOperatorName(operator_name);
  expr_type_ = NumericalExpressionType::Operator;
}

void NumericalOperator::clear()
{
  operator_name_.clear();
  lhs_expr_.reset();
  rhs_expr_.reset();
}

bool NumericalOperator::set(const std::string& t_op_name, NumericalExprPtr t_lhs_ptr, NumericalExprPtr t_rhs_ptr)
{
  setLhsExpression(t_lhs_ptr);
  setRhsExpression(t_rhs_ptr);
  return setOperatorName(t_op_name);
}

bool NumericalOperator::setOperatorName(const std::string& t_op_name)
{
  if (t_op_name.empty() || (t_op_name != "+" && t_op_name != "-" && t_op_name != "*" && t_op_name != "/")) {
    std::cerr << "Empty or invalid NumericalOperator name : " << t_op_name << std::endl;
    return false;
  }
  operator_name_ = std::move(t_op_name);
  return true;
}

NumericalOperator& NumericalOperator::operator=(const NumericalOperator& t_other)
{
  set(t_other.getOperatorName(), t_other.getLhsExpression(), t_other.getRhsExpression());
  return *this;
}

std::string NumericalOperator::toPddl(bool t_typing, int t_pad_lv) const
{
  if (operator_name_.empty()) {
    return {};
  };

  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;

  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + operator_name_;
  out += aligners[1] + ::helpers::numericalExprToPddl(lhs_expr_, t_typing, pad_lv);
  out += aligners[1] + ::helpers::numericalExprToPddl(rhs_expr_, t_typing, pad_lv);
  out += aligners[2] + ")";
  return out;
}

bool rtask::commons::pddl_generator::operator==(const NumericalOperator& t_first, const NumericalOperator& t_second)
{
  if (t_first.getOperatorName() != t_second.getOperatorName()) {
    return false;
  }
  return helpers::operator==(*t_first.getLhsExpression().get(), *t_second.getLhsExpression().get())
         && helpers::operator==(*t_first.getRhsExpression().get(), *t_second.getRhsExpression().get());
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const NumericalOperator& t_expr)
{
  t_out << "NumericalOperator name: " << t_expr.getOperatorName() << std::endl;
  t_out << "\t - lhs: " << t_expr.getLhsExpression() << std::endl;
  t_out << "\t - rhs: " << t_expr.getRhsExpression();
  return t_out;
}

////// TODO: This strongly depends on the implementation of the action class, check it
//// bool LiteralExpression::validate(const UnordStrToLitTermMap& t_known_constants,
////                                 const UnordStrToUIntMap& t_belonging_action_args,
////                                 const std::string& t_belonging_action_name) const
////{
////  if (expr_name_.empty()) {
////    std::cerr << "VALIDATION ERROR: Empty LiteralExpression name" << std::endl;
////    return false;
////  }

////  for (const auto& arg : args_) {
////    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
////      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
////      std::cerr << "\t(In LiteralExpression **" << expr_name_ << "** of Action **" << t_belonging_action_name <<
///"**)" /                << std::endl; /      return false; /    } /  } /  return true;
////}
