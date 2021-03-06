#include "pddl_generator/ExistsExpression.h"

#include <algorithm>

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

bool ExistsExpression::isValid(UmapStrStr t_action_params,
                               const UmapStrStr& t_known_types,
                               const std::vector<LiteralTerm>& t_known_constants,
                               const std::vector<Predicate>& t_known_predicates,
                               const std::vector<LiteralExpression>& t_known_timeless,
                               const bool t_is_an_effect) const
{

  if (!what_->isValid(t_known_types)) {
    std::cerr << "Validation Error: invalid WHAT of current ExistsExpression" << std::endl;
    return false;
  }

  if (t_action_params.count(what_->getName()) != 0) {
    std::cerr << "Validation Error: invalid WHAT of current ExistsExpression, expression arg used" << std::endl;
    return false;
  }

  if (std::find(t_known_constants.begin(), t_known_constants.end(), *this->what_) != t_known_constants.end()) {
    std::cerr << "Validation Error: invalid WHAT of current ExistsExpression, constant used" << std::endl;
    return false;
  }

  UmapStrStr par;
  for (auto& ap : t_action_params) {
    par.emplace(ap.first, ap.second);
  }
  par.emplace(what_->getName(), what_->getType());

  return helpers::isValid(
    condition_, par, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, false);
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

bool rtask::commons::pddl_generator::operator!=(const ExistsExpression& t_first, const ExistsExpression& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const ExistsExpression& t_expr)
{
  return t_out << " ## EXISTS EXPRESSION ## " << std::endl << t_expr.toPddl();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<ExistsExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : ExistsExpression());
  return t_out;
}
