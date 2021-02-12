#include "pddl_generator/ImplyExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
ImplyExpression::ImplyExpression()
{
  expr_type_ = LogicalExpressionType::Imply;
  expr_name_ = "imply";
}

ImplyExpression::ImplyExpression(const LogicalExpression& t_condition, const LogicalExpression& t_consequence)
{
  expr_type_ = LogicalExpressionType::Imply;
  expr_name_ = "imply";
  set(std::make_shared<LogicalExpression>(t_condition), std::make_shared<LogicalExpression>(t_consequence));
}

ImplyExpression::ImplyExpression(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr)
{
  expr_type_ = LogicalExpressionType::Imply;
  expr_name_ = "imply";
  set(t_condition_ptr, t_consequence_ptr);
}

ImplyExpression::ImplyExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("imply", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("condition", t_rpc_val["imply"], XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("consequence", t_rpc_val["imply"], XmlRpc::XmlRpcValue::Type::TypeStruct))) {
    std::cerr << "Fatal: Invalid ImplyExpression Structure, should be a Struct with condition and consequence fields"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  condition_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["imply"]["condition"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as condition of current ImplyExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  consequence_ = helpers::getLogicalExprFromXmlRpc(t_rpc_val["imply"]["consequence"]);
  if (!condition_) {
    std::cerr << "Fatal: Invalid Boolean Expression as consequence of current ImplyExpression" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_name_ = "imply";
  expr_type_ = LogicalExpressionType::Imply;
}

void ImplyExpression::clear()
{
  condition_.reset();
  consequence_.reset();
}

void ImplyExpression::set(LogicalExprPtr t_condition_ptr, LogicalExprPtr t_consequence_ptr)
{
  setCondition(t_condition_ptr);
  setConsequence(t_consequence_ptr);
}

bool ImplyExpression::isValid(UmapStrStr t_action_params,
                              const UmapStrStr& t_known_types,
                              const std::vector<LiteralTerm>& t_known_constants,
                              const std::vector<Predicate>& t_known_predicates,
                              const std::vector<LiteralExpression>& t_known_timeless,
                              const bool t_is_an_effect) const
{
  if (!helpers::isValid(
        condition_, t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, false)) {
    std::cerr << "Validation Error: Invalid LogicalExpression as CONDITION of current ImplyExpression" << std::endl;
    return false;
  }
  if (!helpers::isValid(
        consequence_, t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, true)) {
    std::cerr << "Validation Error: Invalid LogicalExpression as CONSEQUENCE of current ImplyExpression" << std::endl;
    return false;
  }
  return true;
}

std::string ImplyExpression::toPddl(bool t_typing, int t_pad_lv) const
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

ImplyExpression& ImplyExpression::operator=(const ImplyExpression& t_other)
{
  condition_ = t_other.getCondition();
  consequence_ = t_other.getConsequence();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const ImplyExpression& t_first, const ImplyExpression& t_second)
{
  return helpers::operator==(*t_first.getCondition(), *t_second.getCondition())
         && helpers::operator==(*t_first.getConsequence(), *t_second.getConsequence());
};

bool rtask::commons::pddl_generator::operator!=(const ImplyExpression& t_first, const ImplyExpression& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const ImplyExpression& t_expr)
{
  return t_out << " ## IMPLY EXPRESSION ## " << std::endl << t_expr.toPddl();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<ImplyExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : ImplyExpression());
  return t_out;
}
