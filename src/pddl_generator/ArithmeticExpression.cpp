#include "pddl_generator/ArithmeticExpression.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
ArithmeticExpression::ArithmeticExpression()
{
  expr_type_ = LogicalExpressionType::Arithmetic;
}

ArithmeticExpression::ArithmeticExpression(const std::string& t_op_name,
                                           const NumericalExpression& t_lhs,
                                           const NumericalExpression& t_rhs)
{
  expr_type_ = LogicalExpressionType::Arithmetic;
  set(t_op_name, std::make_shared<NumericalExpression>(t_lhs), std::make_shared<NumericalExpression>(t_rhs));
}

ArithmeticExpression::ArithmeticExpression(const std::string& t_op_name, NumericalExprPtr t_lhs, NumericalExprPtr t_rhs)
{
  expr_type_ = LogicalExpressionType::Arithmetic;
  set(t_op_name, t_lhs, t_rhs);
}

ArithmeticExpression::ArithmeticExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("modify", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("operation", t_rpc_val["modify"], XmlRpc::XmlRpcValue::Type::TypeString, false)
        && helpers::checkXmlRpcSanity("lhs", t_rpc_val["modify"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("rhs", t_rpc_val["modify"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid Arithmetic Structure, should be a Struct with operator, lhs, and rhs fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string name = static_cast<std::string>(t_rpc_val["modify"]["operation"]);

  lhs_expr_ = helpers::getNumericalExprFromXmlRpc(t_rpc_val["modify"]["lhs"]);
  if (!lhs_expr_) {
    std::cerr << "Fatal: Invalid Numerical Expression as lhs of current ArithmeticExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  rhs_expr_ = helpers::getNumericalExprFromXmlRpc(t_rpc_val["modify"]["rhs"]);
  if (!rhs_expr_) {
    std::cerr << "Fatal: Invalid Numerical Expression as rhs of current ArithmeticExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  setArithmeticOperation(name);
  expr_type_ = LogicalExpressionType::Arithmetic;
}

void ArithmeticExpression::clear()
{
  expr_name_.clear();
  lhs_expr_.reset();
  rhs_expr_.reset();
}

bool ArithmeticExpression::set(const std::string& t_op_name, NumericalExprPtr t_lhs, NumericalExprPtr t_rhs)
{
  setLhsExpression(t_lhs);
  setRhsExpression(t_rhs);
  return setArithmeticOperation(t_op_name);
}

bool ArithmeticExpression::setArithmeticOperation(const std::string& t_op_name)
{
  auto name = t_op_name;
  std::for_each(name.begin(), name.end(), [](char& c) { c = static_cast<char>(tolower(c)); });

  if (name.empty()
      || (name != "increase" && name != "decrease" && name != "assign" && name != "scale-up" && name != "scale-down")) {
    std::cerr << "Empty or invalid Arithmetic Operation name : " << name << std::endl;
    exit(EXIT_FAILURE);
  }
  expr_name_ = std::move(name);
  return true;
}

std::string ArithmeticExpression::toPddl(bool t_typing, int t_pad_lv) const
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

ArithmeticExpression& ArithmeticExpression::operator=(const ArithmeticExpression& t_other)
{
  expr_name_ = t_other.getArithmeticOperation();
  lhs_expr_ = t_other.getLhsExpression();
  rhs_expr_ = t_other.getRhsExpression();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const ArithmeticExpression& t_first,
                                                const ArithmeticExpression& t_second)
{
  return (t_first.getArithmeticOperation() == t_second.getArithmeticOperation()
          && helpers::operator==(*t_first.getLhsExpression(), *t_second.getLhsExpression())
          && helpers::operator==(*t_first.getRhsExpression(), *t_second.getRhsExpression()));
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

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const ArithmeticExpression& t_expr)
{
  t_out << "ArithmeticExpression \n\t - operation: " << t_expr.getArithmeticOperation() << std::endl;
  t_out << "\t - lhs: " << t_expr.getLhsExpression() << std::endl;
  t_out << "\t - rhs: " << t_expr.getRhsExpression();
  return t_out;
}
