#include "pddl_generator/Helpers.h"

#include "pddl_generator/LiteralTerm.h"
#include "pddl_generator/NumericalTerm.h"

#include "pddl_generator/AndExpression.h"
#include "pddl_generator/ArithmeticExpression.h"
#include "pddl_generator/CompareExpression.h"
#include "pddl_generator/EqualsExpression.h"
#include "pddl_generator/ExistsExpression.h"
#include "pddl_generator/ForAllExpression.h"
#include "pddl_generator/ImplyExpression.h"
#include "pddl_generator/LiteralExpression.h"
#include "pddl_generator/NotExpression.h"
#include "pddl_generator/OrExpression.h"
#include "pddl_generator/WhenExpression.h"

#include "pddl_generator/NumericalFunction.h"
#include "pddl_generator/NumericalOperator.h"

#include <algorithm>

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
  else if (t_rpc_val.hasMember("when")) {
    return LogicalExpressionType::When;
  }
  else if (t_rpc_val.hasMember("compare")) {
    return LogicalExpressionType::Compare;
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
  else if (t_rpc_val.hasMember("imply")) {
    return LogicalExpressionType::Imply;
  }
  else if (t_rpc_val.hasMember("equals")) {
    return LogicalExpressionType::Equals;
  }
  else {
    return LogicalExpressionType::Invalid;
  }
}

LogicalExprPtr helpers::getLogicalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  switch (getLogicalExprTypeFromXmlRpc(t_rpc_val)) {
    case LogicalExpressionType::Literal:
      return std::make_shared<LiteralExpression>(t_rpc_val);
    case LogicalExpressionType::Not:
      return std::make_shared<NotExpression>(t_rpc_val);
    case LogicalExpressionType::And:
      return std::make_shared<AndExpression>(t_rpc_val);
    case LogicalExpressionType::Or:
      return std::make_shared<OrExpression>(t_rpc_val);
    case LogicalExpressionType::When:
      return std::make_shared<WhenExpression>(t_rpc_val);
    case LogicalExpressionType::Exists:
      return std::make_shared<ExistsExpression>(t_rpc_val);
    case LogicalExpressionType::ForAll:
      return std::make_shared<ForAllExpression>(t_rpc_val);
    case LogicalExpressionType::Compare:
      return std::make_shared<CompareExpression>(t_rpc_val);
    case LogicalExpressionType::Arithmetic:
      return std::make_shared<ArithmeticExpression>(t_rpc_val);
    case LogicalExpressionType::Imply:
      return std::make_shared<ImplyExpression>(t_rpc_val);
    case LogicalExpressionType::Equals:
      return std::make_shared<EqualsExpression>(t_rpc_val);
    default:
      return nullptr;
  }
}

NumericalExpressionType helpers::getNumericalExprTypeFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (t_rpc_val.hasMember("num_fnc")) {
    return NumericalExpressionType::Function;
  }
  else if (t_rpc_val.hasMember("num_op")) {
    return NumericalExpressionType::Operator;
  }
  else if (t_rpc_val.hasMember("num_term")) {
    return NumericalExpressionType::Term;
  }
  else {
    return NumericalExpressionType::Invalid;
  }
}

NumericalExprPtr helpers::getNumericalExprFromXmlRpc(XmlRpc::XmlRpcValue& t_rpc_val)
{
  switch (getNumericalExprTypeFromXmlRpc(t_rpc_val)) {
    case NumericalExpressionType::Function:
      return std::make_shared<NumericalFunction>(t_rpc_val);
    case NumericalExpressionType::Operator:
      return std::make_shared<NumericalOperator>(t_rpc_val);
    case NumericalExpressionType::Term:
      return std::make_shared<NumericalTerm>(t_rpc_val);
    default:
      return {};
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

bool helpers::hasValidXmlRpcTag(const std::string& t_tag,
                                XmlRpc::XmlRpcValue& t_node,
                                const XmlRpc::XmlRpcValue::Type t_type)
{
  if (!t_node.hasMember(t_tag) || t_node[t_tag].getType() != t_type || !t_node[t_tag].valid()) {
    return false;
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
  switch (t_parent.getObjectType()) {
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
  switch (t_parent->getObjectType()) {
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
  switch (t_parent.getExpressionType()) {
    case LogicalExpressionType::Literal:
      return dynamic_cast<LiteralExpression&>(t_parent);
    case LogicalExpressionType::Not:
      return dynamic_cast<NotExpression&>(t_parent);
    case LogicalExpressionType::And:
      return dynamic_cast<AndExpression&>(t_parent);
    case LogicalExpressionType::Or:
      return dynamic_cast<OrExpression&>(t_parent);
    case LogicalExpressionType::When:
      return dynamic_cast<WhenExpression&>(t_parent);
    case LogicalExpressionType::Exists:
      return dynamic_cast<ExistsExpression&>(t_parent);
    case LogicalExpressionType::ForAll:
      return dynamic_cast<ForAllExpression&>(t_parent);
    case LogicalExpressionType::Compare:
      return dynamic_cast<CompareExpression&>(t_parent);
    case LogicalExpressionType::Arithmetic:
      return dynamic_cast<ArithmeticExpression&>(t_parent);
    case LogicalExpressionType::Imply:
      return dynamic_cast<ImplyExpression&>(t_parent);
    case LogicalExpressionType::Equals:
      return dynamic_cast<EqualsExpression&>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(LogicalExprPtr t_parent)
{
  switch (t_parent->getExpressionType()) {
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
    case LogicalExpressionType::Exists:
      return std::dynamic_pointer_cast<ExistsExpression>(t_parent);
    case LogicalExpressionType::ForAll:
      return std::dynamic_pointer_cast<ForAllExpression>(t_parent);
    case LogicalExpressionType::Compare:
      return std::dynamic_pointer_cast<CompareExpression>(t_parent);
    case LogicalExpressionType::Arithmetic:
      return std::dynamic_pointer_cast<ArithmeticExpression>(t_parent);
    case LogicalExpressionType::Imply:
      return std::dynamic_pointer_cast<ImplyExpression>(t_parent);
    case LogicalExpressionType::Equals:
      return std::dynamic_pointer_cast<EqualsExpression>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(NumericalExpression& t_parent)
{
  switch (t_parent.getExpressionType()) {
    case NumericalExpressionType::Function:
      return dynamic_cast<NumericalFunction&>(t_parent);
    case NumericalExpressionType::Operator:
      return dynamic_cast<NumericalOperator&>(t_parent);
    case NumericalExpressionType::Term:
      return dynamic_cast<NumericalTerm&>(t_parent);
    default:
      return {};
  }
}

std::any helpers::getAsChild(NumericalExprPtr t_parent)
{
  switch (t_parent->getExpressionType()) {
    case NumericalExpressionType::Function:
      return std::dynamic_pointer_cast<NumericalFunction>(t_parent);
    case NumericalExpressionType::Operator:
      return std::dynamic_pointer_cast<NumericalOperator>(t_parent);
    case NumericalExpressionType::Term:
      return std::dynamic_pointer_cast<NumericalTerm>(t_parent);
    default:
      return {};
  }
}

bool helpers::isValid(LogicalExprPtr t_expr_ptr,
                      const UmapStrStr& t_action_params,
                      const UmapStrStr& t_known_types,
                      const std::vector<LiteralTerm>& t_known_constants,
                      const std::vector<Predicate>& t_known_predicates,
                      const std::vector<LiteralExpression>& t_known_timeless,
                      bool t_is_an_effect)
{
  if (t_expr_ptr) {
    if (LogicalExprSupportedAs.count(t_expr_ptr->getExpressionType()) == 0
        || (t_is_an_effect && !LogicalExprSupportedAs.at(t_expr_ptr->getExpressionType()).second)
        || (!t_is_an_effect && !LogicalExprSupportedAs.at(t_expr_ptr->getExpressionType()).first)) {
      return false;
    }
    switch (t_expr_ptr->getExpressionType()) {
      case LogicalExpressionType::Literal:
        return std::dynamic_pointer_cast<LiteralExpression>(t_expr_ptr)
          ->isValid(t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless);
      case LogicalExpressionType::Not:
        return std::dynamic_pointer_cast<NotExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::And:
        return std::dynamic_pointer_cast<AndExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::Or:
        return std::dynamic_pointer_cast<OrExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::When:
        return std::dynamic_pointer_cast<WhenExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::Exists:
        return std::dynamic_pointer_cast<ExistsExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::ForAll:
        return std::dynamic_pointer_cast<ForAllExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::Compare:
        // To be implemented
        return true;
        //        return std::dynamic_pointer_cast<CompareExpression>(t_expr_ptr)
        //          ->isValid(t_action_params, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::Arithmetic:
        // To be implemented
        return true;
        //        return std::dynamic_pointer_cast<ArithmeticExpression>(t_expr_ptr)
        //          ->isValid(t_action_params, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::Imply:
        return std::dynamic_pointer_cast<ImplyExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      case LogicalExpressionType::Equals:
        return std::dynamic_pointer_cast<EqualsExpression>(t_expr_ptr)
          ->isValid(
            t_action_params, t_known_types, t_known_constants, t_known_predicates, t_known_timeless, t_is_an_effect);
      default:
        return false;
    }
  }
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, LogicalExprPtr t_expr)
{
  if (t_expr) {
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
      case LogicalExpressionType::When:
        t_out << *std::dynamic_pointer_cast<WhenExpression>(t_expr);
        break;
      case LogicalExpressionType::Exists:
        t_out << *std::dynamic_pointer_cast<ExistsExpression>(t_expr);
        break;
      case LogicalExpressionType::ForAll:
        t_out << *std::dynamic_pointer_cast<ForAllExpression>(t_expr);
        break;
      case LogicalExpressionType::Compare:
        t_out << *std::dynamic_pointer_cast<CompareExpression>(t_expr);
        break;
      case LogicalExpressionType::Arithmetic:
        t_out << *std::dynamic_pointer_cast<ArithmeticExpression>(t_expr);
        break;
      case LogicalExpressionType::Imply:
        t_out << *std::dynamic_pointer_cast<ImplyExpression>(t_expr);
        break;
      case LogicalExpressionType::Equals:
        t_out << *std::dynamic_pointer_cast<EqualsExpression>(t_expr);
        break;
      default:
        break;
    }
  }
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, NumericalExprPtr t_expr)
{
  if (t_expr) {
    switch (t_expr->getExpressionType()) {
      case NumericalExpressionType::Function:
        t_out << *std::dynamic_pointer_cast<NumericalFunction>(t_expr);
        break;
      case NumericalExpressionType::Operator:
        t_out << *std::dynamic_pointer_cast<NumericalOperator>(t_expr);
        break;
      case NumericalExpressionType::Term:
        t_out << *std::dynamic_pointer_cast<NumericalTerm>(t_expr);
        break;
      default:
        break;
    }
  }
  return t_out;
}

bool helpers::operator==(const LogicalExpression& t_first, const LogicalExpression& t_second)
{
  if (t_first.getExpressionType() != t_second.getExpressionType()
      || t_first.getExpressionName() != t_second.getExpressionName()) {
    return false;
  }
  switch (t_first.getExpressionType()) {
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
    case LogicalExpressionType::Exists:
      return operator==(dynamic_cast<const ExistsExpression&>(t_first),
                        dynamic_cast<const ExistsExpression&>(t_second));
    case LogicalExpressionType::ForAll:
      return operator==(dynamic_cast<const ForAllExpression&>(t_first),
                        dynamic_cast<const ForAllExpression&>(t_second));
    case LogicalExpressionType::Compare:
      return operator==(dynamic_cast<const CompareExpression&>(t_first),
                        dynamic_cast<const CompareExpression&>(t_second));
    case LogicalExpressionType::Arithmetic:
      return operator==(dynamic_cast<const ArithmeticExpression&>(t_first),
                        dynamic_cast<const ArithmeticExpression&>(t_second));
    case LogicalExpressionType::Imply:
      return operator==(dynamic_cast<const ImplyExpression&>(t_first), dynamic_cast<const ImplyExpression&>(t_second));
    case LogicalExpressionType::Equals:
      return operator==(dynamic_cast<const EqualsExpression&>(t_first),
                        dynamic_cast<const EqualsExpression&>(t_second));

    default:
      return false;
  }
}

bool helpers::operator==(const NumericalExpression& t_first, const NumericalExpression& t_second)
{
  if (t_first.getExpressionType() != t_second.getExpressionType()) {
    return false;
  }
  switch (t_first.getExpressionType()) {
    case NumericalExpressionType::Function:
      return operator==(dynamic_cast<const NumericalFunction&>(t_first),
                        dynamic_cast<const NumericalFunction&>(t_second));
    case NumericalExpressionType::Operator:
      return operator==(dynamic_cast<const NumericalOperator&>(t_first),
                        dynamic_cast<const NumericalOperator&>(t_second));
    case NumericalExpressionType::Term:
      return operator==(dynamic_cast<const NumericalTerm&>(t_first), dynamic_cast<const NumericalTerm&>(t_second));
    default:
      return false;
  }
}

bool helpers::operator!=(const LogicalExpression& t_first, const LogicalExpression& t_second)
{
  return !(t_first == t_second);
}

bool helpers::operator!=(const NumericalExpression& t_first, const NumericalExpression& t_second)
{
  return !(t_first == t_second);
}

std::string helpers::logicalExprToPddl(LogicalExprPtr t_ptr, bool t_typing, int t_pad_lv)
{
  LogicalExpressionType type = t_ptr->getExpressionType();
  switch (type) {
    case LogicalExpressionType::Literal:
      return std::dynamic_pointer_cast<LiteralExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Not:
      return std::dynamic_pointer_cast<NotExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::And:
      return std::dynamic_pointer_cast<AndExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Or:
      return std::dynamic_pointer_cast<OrExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::When:
      return std::dynamic_pointer_cast<WhenExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Exists:
      return std::dynamic_pointer_cast<ExistsExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::ForAll:
      return std::dynamic_pointer_cast<ForAllExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Compare:
      return std::dynamic_pointer_cast<CompareExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Arithmetic:
      return std::dynamic_pointer_cast<ArithmeticExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Imply:
      return std::dynamic_pointer_cast<ImplyExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case LogicalExpressionType::Equals:
      return std::dynamic_pointer_cast<EqualsExpression>(t_ptr)->toPddl(t_typing, t_pad_lv);
    default:
      return {};
  }
}

std::string helpers::numericalExprToPddl(NumericalExprPtr t_ptr, bool t_typing, int t_pad_lv)
{
  switch (t_ptr->getExpressionType()) {
    case NumericalExpressionType::Function:
      return std::dynamic_pointer_cast<NumericalFunction>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case NumericalExpressionType::Operator:
      return std::dynamic_pointer_cast<NumericalOperator>(t_ptr)->toPddl(t_typing, t_pad_lv);
    case NumericalExpressionType::Term:
      return std::dynamic_pointer_cast<NumericalTerm>(t_ptr)->toPddl(t_typing, t_pad_lv);
    default:
      return {};
  }
}

std::string helpers::padding(int t_n_pads)
{
  std::string ret_str{};
  while (--t_n_pads >= 0) {
    ret_str += PAD;
  }
  return ret_str;
}

std::pair<int, std::vector<std::string>> helpers::getPddlAligners(int t_pad_lv)
{
  if (t_pad_lv != -1) {
    return {t_pad_lv + 1, {padding(t_pad_lv), "\n", "\n" + padding(t_pad_lv)}};
  }
  else {
    return {t_pad_lv, {"", " ", ""}};
  }
}

void helpers::fixTypesHierarchy(UmapStrStr& t_known_types)
{
  auto th = buildTypesHierarchy(t_known_types);
  if (th.count("object") != 0) {
    return;
  }
  const auto& it = std::max_element(
    th.begin(), th.end(), [](const auto& l, const auto& r) { return l.second.size() < r.second.size(); });
  if (it->first != "object") {
    t_known_types[it->first] = "object";
  }
}

UmapStrVecStr helpers::buildTypesHierarchy(const UmapStrStr& t_known_types)
{
  auto out = aggregateByParentType(t_known_types);
  for (auto& t : out) {
    for (size_t i = 0; i < t.second.size(); ++i) {
      if (out.count(t.second.at(i)) != 0) {
        for (const auto& j : out.at(t.second.at(i)))
          t.second.push_back(j);
      }
    }
  }
  return out;
}

UmapStrVecStr helpers::aggregateByParentType(const UmapStrStr& t_known_types)
{
  UmapStrVecStr out;
  for (auto const& [key, val] : t_known_types) {
    out[val].push_back(key);
  }
  return out;
}
