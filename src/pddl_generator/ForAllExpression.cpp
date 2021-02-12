#include "pddl_generator/ForAllExpression.h"

#include <algorithm>

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

bool ForAllExpression::isValid(UmapStrStr t_action_params,
                               const UmapStrStr& t_known_types,
                               const std::vector<LiteralTerm>& t_known_constants,
                               const std::vector<Predicate>& t_known_predicates,
                               const std::vector<LiteralExpression>& t_known_timeless,
                               const bool t_is_an_effect) const
{

  if (!what_->isValid(t_known_types)) {
    std::cerr << "Validation Error: invalid WHAT of current ForAllExpression" << std::endl;
    return false;
  }

  if (t_action_params.count(what_->getName()) != 0) {
    std::cerr << "Validation Error: invalid WHAT of current ForAllExpression, expression arg used" << std::endl;
    return false;
  }

  if (std::find(t_known_constants.begin(), t_known_constants.end(), *this->what_) != t_known_constants.end()) {
    std::cerr << "Validation Error: invalid WHAT of current ForAllExpression, constant used" << std::endl;
    return false;
  }

  return helpers::isValid(
    condition_, t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, false);
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

bool rtask::commons::pddl_generator::operator!=(const ForAllExpression& t_first, const ForAllExpression& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const ForAllExpression& t_expr)
{
  return t_out << " ## FOR_ALL EXPRESSION ## " << std::endl << t_expr.toPddl();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<ForAllExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : ForAllExpression());
  return t_out;
}
