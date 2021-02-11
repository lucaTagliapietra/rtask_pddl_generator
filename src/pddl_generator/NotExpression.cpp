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
    std::cerr << "Fatal: Invalid Logical Expression as argument of current NotExpression" << std::endl;
    exit(EXIT_FAILURE);
  }
  expr_name_ = "not";
  expr_type_ = LogicalExpressionType::Not;
}

void NotExpression::clear()
{
  expr_.reset();
}

bool NotExpression::isValid(UmapStrStr t_action_params,
                            const UmapStrStr& t_known_types,
                            const std::vector<LiteralTerm>& t_known_constants,
                            const std::vector<Predicate>& t_known_predicates,
                            const std::vector<LiteralExpression>& t_known_timeless,
                            const bool t_is_an_effect) const
{
  return helpers::isValid(
    expr_, t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
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

bool rtask::commons::pddl_generator::operator!=(const NotExpression& t_first, const NotExpression& t_second)
{
  return !(t_first == t_second);
}

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
