#include "pddl_generator/AndExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
AndExpression::AndExpression(const LogicalExprPtrVector& t_expr_vec)
{
  expr_type_ = LogicalExpressionType::And;
  expr_name_ = "and";
  set(t_expr_vec);
}

AndExpression::AndExpression(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (!helpers::checkXmlRpcSanity("and", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray)) {
    std::cerr << "Fatal: Invalid AndExpression Structure, should be an Array" << std::endl;
    exit(EXIT_FAILURE);
  }

  expr_name_ = "and";
  expr_type_ = LogicalExpressionType::And;

  for (int i = 0; i < t_rpc_val["and"].size(); ++i) {
    auto expr = helpers::getLogicalExprFromXmlRpc(t_rpc_val["and"][i]);
    if (!expr) {
      std::cerr << "Fatal: Invalid Logical Expression as argument [" << i << "] of current AndExpression" << std::endl;
      exit(EXIT_FAILURE);
    }
    expr_vec_.push_back(expr);
  }
}

void AndExpression::clear()
{
  expr_vec_.clear();
}

LogicalExprPtr AndExpression::getExpression(int t_idx) const
{
  if (t_idx != -1 && static_cast<unsigned int>(t_idx) < expr_vec_.size()) {
    return expr_vec_[static_cast<unsigned int>(t_idx)];
  }
  return nullptr;
}

int AndExpression::findExpression(LogicalExprPtr t_expr) const
{
  for (unsigned int i = 0; i < expr_vec_.size(); ++i) {
    if (helpers::operator==(*t_expr, *expr_vec_[i]))
      return static_cast<int>(i);
  }
  return -1;
}

bool AndExpression::hasExpression(LogicalExprPtr t_expr) const
{
  for (const auto& expr : expr_vec_) {
    if (helpers::operator==(*t_expr, *expr)) {
      return true;
    }
  }
  return false;
}

bool AndExpression::addExpression(LogicalExprPtr t_expr)
{
  if (hasExpression(t_expr)) {
    return false;
  }
  expr_vec_.push_back(t_expr);
  return true;
}

bool AndExpression::removeExpression(LogicalExprPtr t_expr)
{
  return removeExpression(findExpression(t_expr));
}

bool AndExpression::removeExpression(int t_idx)
{
  if (t_idx != -1 && static_cast<unsigned int>(t_idx) < expr_vec_.size()) {
    expr_vec_[static_cast<unsigned int>(t_idx)].reset();
    expr_vec_.erase(expr_vec_.begin() + t_idx);
    return true;
  }
  return false;
}

bool AndExpression::isValid(UmapStrStr t_action_params,
                            const UmapStrStr& t_known_types,
                            const std::vector<LiteralTerm>& t_known_constants,
                            const std::vector<Predicate>& t_known_predicates,
                            const std::vector<LiteralExpression>& t_known_timeless,
                            const bool t_is_an_effect) const
{
  for (const auto& expr : expr_vec_) {
    if (!helpers::isValid(expr,
                          t_action_params,
                          t_known_types,
                          t_known_constants,
                          t_known_predicates,
                          t_known_timeless,
                          t_is_an_effect)) {
      return false;
    }
  }
  return true;
}

std::string AndExpression::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& pad_lv = pad_aligners.first;
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + expr_name_;
  for (const auto& expr : expr_vec_) {
    out += aligners[1] + ::helpers::logicalExprToPddl(expr, t_typing, pad_lv);
  }
  out += aligners[2] + ")";

  return out;
}

AndExpression& AndExpression::operator=(const AndExpression& t_other)
{
  expr_vec_ = t_other.getExpressions();
  return *this;
}

bool rtask::commons::pddl_generator::operator==(const AndExpression& t_first, const AndExpression& t_second)
{
  const auto& f_exp = t_first.getExpressions();
  const auto& s_exp = t_second.getExpressions();

  if (f_exp.size() != s_exp.size()) {
    return false;
  }

  // Check, for every expr in first if there is one in second that satisfy the childClass operator ==
  for (const auto& expr : f_exp) {
    if (!t_second.hasExpression(expr)) {
      return false;
    }
  }

  // Check, for every expr in second if there is one in first that satisfy the childClass operator ==
  // Necessary to handle the case a,a,b == a,b,c
  for (const auto& expr : s_exp) {
    if (!t_first.hasExpression(expr)) {
      return false;
    }
  }

  return true;
};

bool rtask::commons::pddl_generator::operator!=(const AndExpression& t_first, const AndExpression& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const AndExpression& t_expr)
{
  t_out << "AndExpression name: " << t_expr.getExpressionName() << std::endl;
  unsigned int i = 0;
  for (const auto& expr : t_expr.getExpressions()) {
    (i != 0) ? t_out << std::endl : t_out << "";
    t_out << " - expr[" << i << "]: " << expr;
    ++i;
  }
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<AndExpression> t_expr_ptr)
{
  t_out << (t_expr_ptr ? *t_expr_ptr : AndExpression());
  return t_out;
}
