#include "pddl_generator/AndExpression.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
AndExpression::AndExpression()
{
  expr_type_ = LogicalExpressionType::And;
  expr_name_ = "and";
}

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
  int pos = findExpression(t_expr);
  if (pos != -1) {
    expr_vec_[static_cast<unsigned int>(pos)].reset();
    expr_vec_.erase(expr_vec_.begin() + pos);
    return true;
  }
  return false;
}

std::string AndExpression::toPddl(const bool t_typing) const
{
  std::string out{};
  out += " (" + expr_name_;
  for (const auto& expr : expr_vec_) {
    out += ::helpers::logicalExprToPddl(expr, t_typing);
  }
  out += ")";
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

//// TODO: This strongly depends on the implementation of the action class, check it
// bool LiteralLogicalExpression::validate(const UnordStrToLitTermMap& t_known_constants,
//                                        const UnordStrToUIntMap& t_belonging_action_args,
//                                        const std::string& t_belonging_action_name) const
//{
//  if (expr_name_.empty()) {
//    std::cerr << "VALIDATION ERROR: Empty LiteralLogicalExpression name" << std::endl;
//    return false;
//  }

//  for (const auto& arg : args_) {
//    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
//      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
//      std::cerr << "\t(In LiteralLogicalExpression **" << expr_name_ << "** of Action **"
//                << t_belonging_action_name << "**)" << std::endl;
//      return false;
//    }
//  }
//  return true;
//}
