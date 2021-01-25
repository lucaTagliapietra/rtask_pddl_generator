#include "pddl_generator/Helpers.h"
#include "pddl_generator/NotExpression.h"

#include "pddl_generator/LiteralTerm.h"
#include "pddl_generator/NumericalTerm.h"

using namespace rtask::commons::pddl_generator;

BooleanExpressionType helpers::getBooleanExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (t_rpc_val.hasMember("not")) {
    return BooleanExpressionType::NotExpression;
  }
  else if (t_rpc_val.hasMember("expr")) {
    return BooleanExpressionType::LiteralBooleanExpression;
  }
  else if (t_rpc_val.hasMember("and")) {
    return BooleanExpressionType::AndExpression;
  }
  else if (t_rpc_val.hasMember("or")) {
    return BooleanExpressionType::OrExpression;
  }
  else if (t_rpc_val.hasMember("compare")) {
    return BooleanExpressionType::CompareExpression;
  }
  else if (t_rpc_val.hasMember("when")) {
    return BooleanExpressionType::WhenExpression;
  }
  else if (t_rpc_val.hasMember("exists")) {
    return BooleanExpressionType::ExistsExpression;
  }
  else if (t_rpc_val.hasMember("forall")) {
    return BooleanExpressionType::ForAllExpression;
  }
  else {
    return BooleanExpressionType::Invalid;
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

std::any helpers::getAsChild(BooleanExpression& t_parent)
{
  BooleanExpressionType type = t_parent.getExpressionType();
  switch (type) {
    case BooleanExpressionType::NotExpression:
      return dynamic_cast<NotExpression&>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(std::shared_ptr<BooleanExpression> t_parent)
{
  BooleanExpressionType type = t_parent->getExpressionType();
  switch (type) {
    case BooleanExpressionType::NotExpression:
      return std::dynamic_pointer_cast<NotExpression>(t_parent);
    default:
      return {};
  }
}
