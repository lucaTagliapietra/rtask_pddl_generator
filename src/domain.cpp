#include <algorithm>

#include "boost/make_shared.hpp"

#include "commons/domain.h"
#include "commons/utils.h"

rtask::commons::Domain::Domain(const std::string& t_name,
                               const std::vector<SemanticEntity>& t_semantic_entities,
                               const std::vector<Predicate>& t_predicates,
                               const std::vector<Action>& t_actions,
                               const bool t_req_typing,
                               const bool t_req_equality,
                               const bool t_req_strips)
{
  setDomain(t_name, t_semantic_entities, t_predicates, t_actions, t_req_typing, t_req_equality, t_req_strips);
}

rtask::commons::Domain::Domain(const rtask_msgs::Domain& t_msg)
{
  setFromDomainMsg(t_msg);
}
rtask::commons::Domain::Domain(const rtask_msgs::DomainConstPtr t_msg_ptr)
{
  setFromDomainMsg(t_msg_ptr);
}
rtask::commons::Domain::Domain(XmlRpc::XmlRpcValue& t_node)
{
  // setDomainFromXmlRpc(t_node);
}

bool rtask::commons::Domain::isValid() const
{
  return !m_name.empty() && !m_semantic_entities.empty() && !m_predicates.empty() && !m_actions.empty();
}

rtask_msgs::DomainPtr rtask::commons::Domain::toDomainMsg() const
{
  rtask_msgs::DomainPtr t_msg = boost::make_shared<rtask_msgs::Domain>();
  t_msg->name = m_name;
  t_msg->requires_strips = m_requirements.strips;
  t_msg->requires_typing = m_requirements.typing;
  t_msg->requires_equalty = m_requirements.equality;
  for (const auto& semantic_entity : m_semantic_entities) {
    t_msg->semantic_entities.emplace_back(semantic_entity.toMsg());
  }
  for (const auto& predicate : m_predicates) {
    t_msg->predicates.emplace_back(predicate.toMsg());
  }
  for (const auto& action : m_actions) {
    t_msg->actions.emplace_back(*action.toActionMsg());
  }

  return t_msg;
}

// std::string toPddl() const;

// bool setDomainFromXmlRpc(XmlRpc::XmlRpcValue& t_node);

bool rtask::commons::Domain::setFromDomainMsg(const rtask_msgs::Domain& t_msg)
{
  m_name = t_msg.name;
  m_requirements.strips = t_msg.requires_strips;
  m_requirements.typing = t_msg.requires_typing;
  m_requirements.equality = t_msg.requires_equalty;

  for (const auto& semantic_entity : t_msg.semantic_entities) {
    m_semantic_entities.emplace_back(semantic_entity);
  }
  for (const auto& predicate : t_msg.predicates) {
    m_predicates.emplace_back(predicate);
  }
  for (const auto& action : t_msg.actions) {
    m_actions.emplace_back(action);
  }

  return isValid();
}
bool rtask::commons::Domain::setFromDomainMsg(const rtask_msgs::DomainConstPtr& t_msg_ptr)
{
  return setFromDomainMsg(*t_msg_ptr);
}
bool rtask::commons::Domain::setDomain(const std::string& t_name,
                                       const std::vector<SemanticEntity>& t_semantic_entities,
                                       const std::vector<Predicate>& t_predicates,
                                       const std::vector<Action>& t_actions,
                                       const bool t_req_typing,
                                       const bool t_req_equality,
                                       const bool t_req_strips)
{
  m_name = t_name;
  m_requirements.strips = t_req_strips;
  m_requirements.typing = t_req_typing;
  m_requirements.equality = t_req_equality;

  for (const auto& semantic_entity : t_semantic_entities) {
    m_semantic_entities.emplace_back(semantic_entity);
  }

  for (const auto& predicate : t_predicates) {
    m_predicates.emplace_back(predicate);
  }

  for (const auto& action : t_actions) {
    m_actions.emplace_back(action);
  }

  return isValid();
}

void rtask::commons::Domain::clear()
{
  m_name.clear();
  m_semantic_entities.clear();
  m_predicates.clear();
  m_actions.clear();
  m_requirements = {};
}

// ---------------------
// Semantic Entity Level
// ---------------------

bool rtask::commons::Domain::hasSemanticEntity(const std::string& t_symbol, const std::string& t_type) const
{
  if (t_type.empty()) {

    auto equal_name = [t_symbol](const SemanticEntity& a) {
      bool eq = false;
      for (unsigned int i = 0; i < a.symbols.size(); ++i) {
        eq = a.symbols[i] == t_symbol;
      }
      return eq;
    };
    return std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name)
           != m_semantic_entities.end();
  }
  auto equal_name_type = [t_symbol, t_type](const SemanticEntity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.symbols.size(); ++i) {
      eq = a.symbols[i] == t_symbol && a.type == t_type;
    }
    return eq;
  };
  return std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name_type)
         != m_semantic_entities.end();
}

bool rtask::commons::Domain::getSemanticEntityType(const std::string& t_symbol, std::string& t_type) const
{
  if (!hasSemanticEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const SemanticEntity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.symbols.size(); ++i) {
      eq = a.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name);
  t_type = it->type;
  return true;
}

bool rtask::commons::Domain::getSemanticEntityClass(const std::string& t_symbol, std::string& t_class) const
{
  if (!hasSemanticEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const SemanticEntity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.symbols.size(); ++i) {
      eq = a.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name);
  t_class = it->semantic_class;
  return true;
}

bool rtask::commons::Domain::getSemanticEntity(const std::string& t_symbol, SemanticEntity& t_semantic_entity) const
{

  std::string t_type{};
  std::string t_class{};
  std::vector<std::string> t_properties{};

  if (!getSemanticEntityType(t_symbol, t_type) || !getSemanticEntityClass(t_symbol, t_class)) {
    return false;
  }

  t_semantic_entity = {{t_symbol}, t_type, t_class};
  return true;
}

bool rtask::commons::Domain::addSemanticEntity(const std::string& t_symbol,
                                               const std::string& t_type,
                                               const std::string& t_class)
{
  if (!hasSemanticEntity(t_symbol, t_type)) {
    if (hasSemanticEntity(t_symbol)) {
      return false;
    }
    m_semantic_entities.push_back({{t_symbol}, t_type, t_class});
  }
  return true;
}

bool rtask::commons::Domain::removeSemanticEntity(const std::string& t_symbol, const std::string& t_type)
{
  if (hasSemanticEntity(t_symbol)) {
    if (!t_type.empty() && !hasSemanticEntity(t_symbol, t_type)) {
      return false;
    }
    auto equal_name = [t_symbol](const SemanticEntity& a) {
      bool eq = false;
      for (unsigned int i = 0; i < a.symbols.size(); ++i) {
        eq = a.symbols[i] == t_symbol;
      }
      return eq;
    };
    auto const it = std::find_if(m_semantic_entities.begin(), m_semantic_entities.end(), equal_name);
    m_semantic_entities.erase(it);
  }
  return true;
}

// ---------------
// Predicate Level
// ---------------
bool rtask::commons::Domain::hasPredicate(const std::string& t_name, const std::vector<std::string>& t_args) const
{

  auto equal_name_and_args = [t_name, t_args](const Predicate& a) { return a.name == t_name && a.args == t_args; };
  return std::find_if(m_predicates.begin(), m_predicates.end(), equal_name_and_args) != m_predicates.end();
}

bool rtask::commons::Domain::hasPredicateName(const std::string& t_name) const
{

  auto equal_name = [t_name](const Predicate& a) { return a.name == t_name; };
  return std::find_if(m_predicates.begin(), m_predicates.end(), equal_name) != m_predicates.end();
}

bool rtask::commons::Domain::getPredicates(const std::string& t_name, std::vector<Predicate>& t_predicates) const
{

  std::vector<std::string> t_args;
  if (!hasPredicateName(t_name))
    return false;

  for (unsigned int i = 0; i < m_predicates.size(); ++i) {
    if (m_predicates.at(i).name == t_name) {
      t_predicates.push_back(m_predicates.at(i));
    }
  }
  return true;
}

bool rtask::commons::Domain::addPredicate(const std::string& t_name, const std::vector<std::string>& t_args)
{
  if (hasPredicate(t_name, t_args))
    return false;

  m_predicates.push_back({t_name, t_args});
  return true;
}
bool rtask::commons::Domain::removePredicate(const std::string& t_name, const std::vector<std::string>& t_args)
{
  if (!hasPredicate(t_name, t_args))
    return false;

  auto equal_name_and_args = [t_name, t_args](const Predicate& a) { return a.name == t_name && a.args == t_args; };
  auto const it = std::find_if(m_predicates.begin(), m_predicates.end(), equal_name_and_args);
  m_predicates.erase(it);

  return true;
}

// ------------
// Action Level
// ------------
