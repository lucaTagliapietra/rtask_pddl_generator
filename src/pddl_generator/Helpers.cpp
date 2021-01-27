#include "pddl_generator/Helpers.h"

#include "pddl_generator/LiteralTerm.h"
#include "pddl_generator/NumericalTerm.h"

#include "pddl_generator/AndExpression.h"
#include "pddl_generator/LiteralExpression.h"
#include "pddl_generator/NotExpression.h"
#include "pddl_generator/OrExpression.h"

using namespace rtask::commons::pddl_generator;

LogicalExpressionType helpers::getLogicalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (t_rpc_val.hasMember("not")) {
    return LogicalExpressionType::Not;
  }
  else if (t_rpc_val.hasMember("expr")) {
    return LogicalExpressionType::Literal;
  }
  else if (t_rpc_val.hasMember("and")) {
    return LogicalExpressionType::And;
  }
  else if (t_rpc_val.hasMember("or")) {
    return LogicalExpressionType::Or;
  }
  else if (t_rpc_val.hasMember("compare")) {
    return LogicalExpressionType::Compare;
  }
  else if (t_rpc_val.hasMember("when")) {
    return LogicalExpressionType::When;
  }
  else if (t_rpc_val.hasMember("exists")) {
    return LogicalExpressionType::Exists;
  }
  else if (t_rpc_val.hasMember("forall")) {
    return LogicalExpressionType::ForAll;
  }
  else if (t_rpc_val.hasMember("modify")) {
    return LogicalExpressionType::Arithmetic;
  }
  else {
    return LogicalExpressionType::Invalid;
  }
}

LogicalExprPtr helpers::getLogicalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (t_rpc_val.hasMember("not")) {
    return std::make_shared<NotExpression>(t_rpc_val["not"]);
  }
  else if (t_rpc_val.hasMember("expr")) {
    return std::make_shared<LiteralExpression>(t_rpc_val["expr"]);
  }
  else if (t_rpc_val.hasMember("and")) {
    return std::make_shared<AndExpression>(t_rpc_val["and"]);
  }
  else if (t_rpc_val.hasMember("or")) {
    return std::make_shared<OrExpression>(t_rpc_val["or"]);
  }
  //  else if (t_rpc_val.hasMember("compare")) {
  //    return std::make_shared<CompareExpression>(t_rpc_val);
  //  }
  //  else if (t_rpc_val.hasMember("when")) {
  //    return std::make_shared<WhenExpression>(t_rpc_val);
  //  }
  //  else if (t_rpc_val.hasMember("exists")) {
  //    return std::make_shared<ExistsExpression>(t_rpc_val);
  //  }
  //  else if (t_rpc_val.hasMember("forall")) {
  //    return std::make_shared<ForallExpression>(t_rpc_val);
  //  }
  else {
    return nullptr;
  }
}

NumericalExpressionType helpers::getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (t_rpc_val.hasMember("num_expr")) {
    return NumericalExpressionType::LiteralNumericalExpression;
  }
  else if (t_rpc_val.hasMember("num_op")) {
    return NumericalExpressionType::ArithmeticExpression;
  }
  else if (t_rpc_val.hasMember("num_val")) {
    return NumericalExpressionType::NumericalTerm;
  }
  else {
    return NumericalExpressionType::Invalid;
  }
}

bool helpers::checkXmlRpcSanity(const std::string& t_tag,
                                XmlRpc::XmlRpcValue& t_node,
                                const XmlRpc::XmlRpcValue::Type t_type,
                                bool allow_empty_string)
{
  if (!t_node.hasMember(t_tag)) {
    std::cout << "Tag: " << t_tag << " not found" << std::endl;
    return false;
  }
  if (t_node[t_tag].getType() != t_type) {
    std::cout << "Tag: " << t_tag << ". Type different from expected" << std::endl;
    return false;
  }
  if (t_type == XmlRpc::XmlRpcValue::TypeString) {
    if (!t_node[t_tag].valid() || (!allow_empty_string && t_node[t_tag] == "")) {
      std::cout << "Tag: " << t_tag << ". Empty or not set string value not allowed" << std::endl;
      return false;
    }
  }
  else {
    if (!t_node[t_tag].valid()) {
      std::cout << "Tag: " << t_tag << ". Empty value not allowed." << std::endl;
      return false;
    }
  }

  return true;
}

XmlRpc::XmlRpcValue::Type helpers::getTagValueType(const std::string& t_tag, XmlRpc::XmlRpcValue& t_node)
{
  if (!t_node.hasMember(t_tag)) {
    std::cout << "Tag: " << t_tag << " not found" << std::endl;
    return XmlRpc::XmlRpcValue::Type::TypeInvalid;
  }
  return t_node[t_tag].getType();
}

std::any helpers::getAsChild(Term& t_parent)
{
  TermType type = t_parent.getObjectType();
  switch (type) {
    case TermType::LiteralTerm:
      return dynamic_cast<LiteralTerm&>(t_parent);
    case TermType::NumericalTerm:
      return dynamic_cast<NumericalTerm&>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(std::shared_ptr<Term> t_parent)
{
  TermType type = t_parent->getObjectType();
  switch (type) {
    case TermType::LiteralTerm:
      return std::dynamic_pointer_cast<LiteralTerm>(t_parent);
    case TermType::NumericalTerm:
      return std::dynamic_pointer_cast<NumericalTerm>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(LogicalExpression& t_parent)
{
  LogicalExpressionType type = t_parent.getExpressionType();
  switch (type) {
    case LogicalExpressionType::Literal:
      return dynamic_cast<LiteralExpression&>(t_parent);
    case LogicalExpressionType::Not:
      return dynamic_cast<NotExpression&>(t_parent);
    case LogicalExpressionType::And:
      return dynamic_cast<AndExpression&>(t_parent);
    case LogicalExpressionType::Or:
      return dynamic_cast<OrExpression&>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(LogicalExprPtr t_parent)
{
  LogicalExpressionType type = t_parent->getExpressionType();
  switch (type) {
    case LogicalExpressionType::Literal:
      return std::dynamic_pointer_cast<LiteralExpression>(t_parent);
    case LogicalExpressionType::Not:
      return std::dynamic_pointer_cast<NotExpression>(t_parent);
    case LogicalExpressionType::And:
      return std::dynamic_pointer_cast<AndExpression>(t_parent);
    case LogicalExpressionType::Or:
      return std::dynamic_pointer_cast<OrExpression>(t_parent);
    case LogicalExpressionType::When:
      return std::dynamic_pointer_cast<WhenExpression>(t_parent);
    default:
      return {};
  }
}
std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, LogicalExprPtr t_expr)
{
  switch (t_expr->getExpressionType()) {
    case LogicalExpressionType::Literal:
      t_out << *std::dynamic_pointer_cast<LiteralExpression>(t_expr);
      break;
    case LogicalExpressionType::Not:
      t_out << *std::dynamic_pointer_cast<NotExpression>(t_expr);
      break;
    case LogicalExpressionType::And:
      t_out << *std::dynamic_pointer_cast<AndExpression>(t_expr);
      break;
    case LogicalExpressionType::Or:
      t_out << *std::dynamic_pointer_cast<OrExpression>(t_expr);
      break;
    default:
      break;
  }
  return t_out;
}

bool helpers::operator==(const LogicalExpression& t_first, const LogicalExpression& t_second)
{
  if (t_first.getExpressionType() != t_second.getExpressionType()
      || t_first.getExpressionName() != t_second.getExpressionName()) {
    return false;
  }
  LogicalExpressionType type = t_first.getExpressionType();
  switch (type) {
    case LogicalExpressionType::Literal:
      return operator==(dynamic_cast<const LiteralExpression&>(t_first),
                        dynamic_cast<const LiteralExpression&>(t_second));
    case LogicalExpressionType::Not:
      return operator==(dynamic_cast<const NotExpression&>(t_first), dynamic_cast<const NotExpression&>(t_second));
    case LogicalExpressionType::And:
      return operator==(dynamic_cast<const AndExpression&>(t_first), dynamic_cast<const AndExpression&>(t_second));
    case LogicalExpressionType::Or:
      return operator==(dynamic_cast<const OrExpression&>(t_first), dynamic_cast<const OrExpression&>(t_second));
    case LogicalExpressionType::When:
      return operator==(dynamic_cast<const WhenExpression&>(t_first), dynamic_cast<const WhenExpression&>(t_second));
    default:
      return false;
  }
}

std::string helpers::logicalExprToPddl(LogicalExprPtr t_ptr, const bool t_typing)
{
  LogicalExpressionType type = t_ptr->getExpressionType();
  switch (type) {
    case LogicalExpressionType::Literal:
      return std::dynamic_pointer_cast<LiteralExpression>(t_ptr)->toPddl(t_typing);
    case LogicalExpressionType::Not:
      return std::dynamic_pointer_cast<NotExpression>(t_ptr)->toPddl(t_typing);
    case LogicalExpressionType::And:
      return std::dynamic_pointer_cast<AndExpression>(t_ptr)->toPddl(t_typing);
    case LogicalExpressionType::Or:
      return std::dynamic_pointer_cast<OrExpression>(t_ptr)->toPddl(t_typing);
    default:
      return {};
  }
}
