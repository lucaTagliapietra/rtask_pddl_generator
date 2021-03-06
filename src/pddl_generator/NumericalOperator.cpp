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
    std::cerr << "Empty or invalid NumericalOperator name: " << t_op_name << std::endl;
    exit(EXIT_FAILURE);
  }
  operator_name_ = std::move(t_op_name);
  return true;
}

NumericalOperator NumericalOperator::mirror() const
{
  switch (operator_name_.front()) {
    case '+':
    case '*':
      return {operator_name_, *rhs_expr_, *lhs_expr_};
    default:
      return *this;
  }
}

bool NumericalOperator::equals(const NumericalOperator& t_other) const
{
  return (operator_name_ == t_other.operator_name_ && helpers::operator==(*lhs_expr_, *t_other.lhs_expr_)
          && helpers::operator==(*rhs_expr_, *t_other.rhs_expr_));
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
  return (t_first.equals(t_second) || t_first.equals(t_second.mirror()));
}

bool rtask::commons::pddl_generator::operator!=(const NumericalOperator& t_first, const NumericalOperator& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const NumericalOperator& t_expr)
{
  return t_out << " ## NUMERICAL OPERATOR ## " << std::endl << t_expr.toPddl();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<NumericalOperator> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : NumericalOperator());
  return t_out;
}
