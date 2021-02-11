#include "pddl_generator/EqualsExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
EqualsExpression::EqualsExpression()
{
  expr_type_ = LogicalExpressionType::Equals;
  expr_name_ = "=";
}

EqualsExpression::EqualsExpression(const std::string& t_lhs, const std::string& t_rhs)
{
  expr_type_ = LogicalExpressionType::Equals;
  expr_name_ = "=";
  set(t_lhs, t_rhs);
}

EqualsExpression::EqualsExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!(helpers::checkXmlRpcSanity("equals", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeStruct)
        && helpers::checkXmlRpcSanity("lhs", t_rpc_val["equals"], XmlRpc::XmlRpcValue::Type::TypeString)
        && helpers::checkXmlRpcSanity("rhs", t_rpc_val["equals"], XmlRpc::XmlRpcValue::Type::TypeString))) {
    std::cerr << "Fatal: Invalid EqualsExpression Structure, should be a Struct with lhs and rhs fields" << std::endl;
    exit(EXIT_FAILURE);
  }

  auto lhs = static_cast<std::string>(t_rpc_val["equals"]["lhs"]);
  auto rhs = static_cast<std::string>(t_rpc_val["equals"]["rhs"]);

  expr_name_ = "=";
  expr_type_ = LogicalExpressionType::Equals;
  set(lhs, rhs);
}

void EqualsExpression::clear()
{
  lhs_term_name_.clear();
  rhs_term_name_.clear();
}

void EqualsExpression::set(const std::string& t_lhs, const std::string& t_rhs)
{
  setLhsTerm(t_lhs);
  setRhsTerm(t_rhs);
}

void EqualsExpression::setLhsTerm(const std::string& t_lhs)
{
  if (t_lhs.empty()) {
    std::cerr << "Empty lhs term of EqualsExpression: " << t_lhs << std::endl;
    exit(EXIT_FAILURE);
  }
  lhs_term_name_ = std::move(t_lhs);
}

void EqualsExpression::setRhsTerm(const std::string& t_rhs)
{
  if (t_rhs.empty()) {
    std::cerr << "Empty lhs term of EqualsExpression: " << t_rhs << std::endl;
    exit(EXIT_FAILURE);
  }
  rhs_term_name_ = std::move(t_rhs);
}

bool EqualsExpression::isValid(UmapStrStr t_action_params,
                               const UmapStrStr& t_known_types,
                               const std::vector<LiteralTerm>& t_known_constants,
                               const std::vector<Predicate>& t_known_predicates,
                               const std::vector<LiteralExpression>& t_known_timeless,
                               const bool t_is_an_effect) const
{
  for (const auto& kc : t_known_constants) {
    t_action_params.emplace(kc.getName(), kc.getType());
  }

  if (t_action_params.count(lhs_term_name_) == 0) {
    std::cerr << "Validation Error: unknown LHS Term of current EqualsExpression" << std::endl;
    return false;
  }

  if (t_action_params.count(rhs_term_name_) == 0) {
    std::cerr << "Validation Error: unknown RHS Term of current EqualsExpression" << std::endl;
    return false;
  }

  return true;
}

std::string EqualsExpression::toPddl(bool, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_ + " ?" + lhs_term_name_ + " ?" + rhs_term_name_ + ")";
  return out;
}

EqualsExpression& EqualsExpression::operator=(const EqualsExpression& t_other)
{
  lhs_term_name_ = t_other.getLhsTerm();
  rhs_term_name_ = t_other.getRhsTerm();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const EqualsExpression& t_first, const EqualsExpression& t_second)
{
  return ((t_first.getLhsTerm() == t_second.getLhsTerm() && t_first.getRhsTerm() == t_second.getRhsTerm())
          || (t_first.getLhsTerm() == t_second.getRhsTerm() && t_first.getRhsTerm() == t_second.getLhsTerm()));
};

bool rtask::commons::pddl_generator::operator!=(const EqualsExpression& t_first, const EqualsExpression& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const EqualsExpression& t_expr)
{
  t_out << "EqualsExpression: name: " << t_expr.getExpressionName() << std::endl;
  t_out << " - lhs: " << t_expr.getLhsTerm() << std::endl;
  t_out << " - rhs: " << t_expr.getRhsTerm();
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out,
                                                         std::shared_ptr<EqualsExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : EqualsExpression());
  return t_out;
}
