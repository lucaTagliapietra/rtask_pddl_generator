#include <algorithm>

#include "boost/make_shared.hpp"

#include "commons/taskDefinition.h"
#include "commons/utils.h"

rtask::commons::TaskDefinition::TaskDefinition(const std::string& t_name,
                                               const std::string& t_domain_name,
                                               const std::vector<Entity>& t_entities,
                                               const std::vector<Command>& t_initial_state,
                                               const std::vector<Command>& t_goal_state,
                                               const bool t_req_typing,
                                               const bool t_req_equality,
                                               const bool t_req_strips)
{
  setTaskDefinition(
    t_name, t_domain_name, t_entities, t_initial_state, t_goal_state, t_req_typing, t_req_equality, t_req_strips);
}

rtask::commons::TaskDefinition::TaskDefinition(const rtask_msgs::TaskDefinition& t_msg)
{
  setFromTaskDefinitionMsg(t_msg);
}

rtask::commons::TaskDefinition::TaskDefinition(const rtask_msgs::TaskDefinitionConstPtr t_msg_ptr)
{
  setFromTaskDefinitionMsg(t_msg_ptr);
}

rtask::commons::TaskDefinition::TaskDefinition(XmlRpc::XmlRpcValue& t_node)
{
  setTaskDefinitionFromXmlRpc(t_node);
}

bool rtask::commons::TaskDefinition::isValid() const
{
  return !m_name.empty() && !m_domain_name.empty() && !m_entities.empty() && !m_initial_state.empty()
         && !m_goal_state.empty();
}

rtask_msgs::TaskDefinitionPtr rtask::commons::TaskDefinition::toTaskDefinitionMsg() const
{
  rtask_msgs::TaskDefinitionPtr t_msg = boost::make_shared<rtask_msgs::TaskDefinition>();
  t_msg->name = m_name;
  t_msg->domain = m_domain_name;
  t_msg->requires_strips = m_requirements.strips;
  t_msg->requires_typing = m_requirements.typing;
  t_msg->requires_equalty = m_requirements.equality;
  for (const auto& entity : m_entities) {
    t_msg->entities.emplace_back(entity.toMsg());
  }
  for (const auto& c : m_initial_state) {
    t_msg->initial_state.emplace_back(c.toMsg());
  }
  for (const auto& c : m_goal_state) {
    t_msg->goal_state.emplace_back(c.toMsg());
  }

  return t_msg;
}

std::string rtask::commons::TaskDefinition::toPddl() const
{
  std::string out = "(define (problem " + m_name + ")\n";
  out += ("\t(:domain " + m_domain_name + ")\n");

  if (m_requirements.strips || m_requirements.typing || m_requirements.equality) {
    out += "\t(:requirements";
    m_requirements.typing ? out += " :typing" : "";
    m_requirements.strips ? out += " :strips" : "";
    m_requirements.equality ? out += " :equality" : "";
    out += ")\n";
  }

  out += "\t(:objects";
  for (const auto& o : m_entities) {
    for (const auto& symbol : o.semantic_entity.symbols) {
      out += (" " + symbol);
      if (m_requirements.typing) {
        out += (" - " + o.semantic_entity.type);
      }
    }
  }
  out += ")\n";

  out += "\t(:init ";
  unsigned count = 0;
  for (const auto& is : m_initial_state) {
    if (count != 0)
      out += "\n\t\t";
    out += "(";
    out += is.predicate.name;
    for (const auto& p : is.predicate.args) {
      out += " ";
      out += (p);
    }
    out += ")";
  }
  out += ")";

  out += "\n\t(:goal (and";
  count = 0;
  for (const auto& gs : m_goal_state) {
    if (count != 0)
      out += "\n\t\t";
    out += " (";
    gs.negate ? out += "not (" : "";
    out += gs.predicate.name;
    for (const auto& p : gs.predicate.args) {
      out += " ";
      out += (p);
    }
    out += ")";
    gs.negate ? out += ")" : out += "";
  }
  out += "))\n";
  return out;
}

bool rtask::commons::TaskDefinition::setTaskDefinitionFromXmlRpc(XmlRpc::XmlRpcValue& t_node)
{
  // Clear all just to be sure
  clear();

  // Name
  if (commons::utils::checkXmlRpcSanity("name", t_node, XmlRpc::XmlRpcValue::TypeString)) {
    m_name = static_cast<std::string>(t_node["name"]);
  }

  // Domain Name
  if (commons::utils::checkXmlRpcSanity("domain", t_node, XmlRpc::XmlRpcValue::TypeString)) {
    m_domain_name = static_cast<std::string>(t_node["domain"]);
  }

  // Requirements
  if (commons::utils::checkXmlRpcSanity("requirements", t_node, XmlRpc::XmlRpcValue::TypeStruct)) {
    if (commons::utils::checkXmlRpcSanity("equality", t_node["requirements"], XmlRpc::XmlRpcValue::TypeBoolean)) {
      m_requirements.equality = static_cast<bool>(t_node["requirements"]["equality"]);
    }
    if (commons::utils::checkXmlRpcSanity("requirements", t_node["requirements"], XmlRpc::XmlRpcValue::TypeBoolean)) {
      m_requirements.strips = static_cast<bool>(t_node["requirements"]["strips"]);
    }
    if (commons::utils::checkXmlRpcSanity("requirements", t_node["requirements"], XmlRpc::XmlRpcValue::TypeBoolean)) {
      m_requirements.typing = static_cast<bool>(t_node["requirements"]["typing"]);
    }
  }

  // Entities
  if (commons::utils::checkXmlRpcSanity("objects", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["objects"].size() == 0) {
      std::cout << "Empty object vector for task " << m_name << " in domain: " << m_domain_name << std::endl;
      return isValid();
    }
    for (auto& entities_same_type : t_node["objects"]) {

      std::string type = commons::NULL_TYPE;
      if (commons::utils::checkXmlRpcSanity("type", entities_same_type.second, XmlRpc::XmlRpcValue::TypeString)) {
        type = static_cast<std::string>(entities_same_type.second["type"]);
      }

      std::string semantic_class{};
      if (commons::utils::checkXmlRpcSanity("class", entities_same_type.second, XmlRpc::XmlRpcValue::TypeString)) {
        semantic_class = static_cast<std::string>(entities_same_type.second["class"]);
      }

      std::vector<std::string> symbols;
      if (commons::utils::checkXmlRpcSanity("symbols", entities_same_type.second, XmlRpc::XmlRpcValue::TypeArray)) {
        if (entities_same_type.second["symbols"].size() == 0) {
          std::cout << "Empty object symbols vector for type " << type << " for task " << m_name
                    << " in domain: " << m_domain_name << std::endl;
          return isValid();
        }
        for (auto& symbol : entities_same_type.second["symbols"]) {
          if (symbol.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(symbol.second).empty()) {
            symbols.emplace_back(static_cast<std::string>(symbol.second));
          }
        }
      }

      std::vector<std::string> properties;
      if (commons::utils::checkXmlRpcSanity("properties", entities_same_type.second, XmlRpc::XmlRpcValue::TypeArray)) {
        if (entities_same_type.second["properties"].size() == 0) {
          std::cout << "Empty object properties vector for type " << type << " for task " << m_name
                    << " in domain: " << m_domain_name << std::endl;
          return isValid();
        }
        for (auto& property : entities_same_type.second["properties"]) {
          if (property.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(property.second).empty()) {
            properties.emplace_back(static_cast<std::string>(property.second));
          }
        }
      }

      m_entities.emplace_back(Entity({symbols, type, semantic_class}, properties));
    }
  }

  // Initial State
  if (commons::utils::checkXmlRpcSanity("init", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["init"].size() == 0) {
      std::cout << "Empty initial state for task " << m_name << " in domain: " << m_domain_name << std::endl;
      return false;
    }
    for (auto& cond : t_node["init"]) {
      if (!commons::utils::checkXmlRpcSanity("cmd", cond.second, XmlRpc::XmlRpcValue::TypeString)) {
        std::cout << "Empty or invalid cmd in initial state for task " << m_name << " in domain: " << m_domain_name
                  << std::endl;
        return false;
      }
      std::string name = cond.second["cmd"];
      std::vector<std::string> args{};
      bool negate = false;
      if (commons::utils::checkXmlRpcSanity("args", cond.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : cond.second["args"]) {

          if (arg.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(arg.second).empty()) {
            args.emplace_back(static_cast<std::string>(arg.second));
          }
        }
      }

      if (cond.second.hasMember("not") && cond.second["not"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        negate = cond.second["not"];
      }
      m_initial_state.push_back({{name, args}, negate});
    }
  }

  // Goal State
  if (commons::utils::checkXmlRpcSanity("goal", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["goal"].size() == 0) {
      std::cout << "Empty goal state for task " << m_name << " in domain: " << m_domain_name << std::endl;
      return false;
    }
    for (auto& cond : t_node["goal"]) {
      if (!commons::utils::checkXmlRpcSanity("cmd", cond.second, XmlRpc::XmlRpcValue::TypeString)) {
        std::cout << "Empty or invalid cmd in goal state for task " << m_name << " in domain: " << m_domain_name
                  << std::endl;
        return false;
      }
      std::string name = cond.second["cmd"];
      std::vector<std::string> args{};
      bool negate = false;
      if (commons::utils::checkXmlRpcSanity("args", cond.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : cond.second["args"]) {

          if (arg.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(arg.second).empty()) {
            args.emplace_back(static_cast<std::string>(arg.second));
          }
        }
      }

      if (cond.second.hasMember("not") && cond.second["not"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        negate = cond.second["not"];
      }
      m_goal_state.push_back({{name, args}, negate});
    }
  }

  return isValid();
}

bool rtask::commons::TaskDefinition::setFromTaskDefinitionMsg(const rtask_msgs::TaskDefinition& t_msg)
{
  m_name = t_msg.name;
  m_domain_name = t_msg.domain;
  m_requirements.strips = t_msg.requires_strips;
  m_requirements.typing = t_msg.requires_typing;
  m_requirements.equality = t_msg.requires_equalty;

  for (const auto& entity : t_msg.entities) {
    m_entities.emplace_back(entity);
  }
  for (const auto& c : t_msg.initial_state) {
    m_initial_state.emplace_back(c);
  }
  for (const auto& c : t_msg.goal_state) {
    m_goal_state.emplace_back(c);
  }
  return isValid();
}

bool rtask::commons::TaskDefinition::setFromTaskDefinitionMsg(const rtask_msgs::TaskDefinitionConstPtr& t_msg_ptr)
{
  return setFromTaskDefinitionMsg(*t_msg_ptr);
}

bool rtask::commons::TaskDefinition::setTaskDefinition(const std::string& t_name,
                                                       const std::string& t_domain_name,
                                                       const std::vector<Entity>& t_entities,
                                                       const std::vector<Command>& t_initial_state,
                                                       const std::vector<Command>& t_goal_state,
                                                       const bool t_req_typing,
                                                       const bool t_req_equality,
                                                       const bool t_req_strips)
{
  m_name = t_name;
  m_domain_name = t_domain_name;
  m_requirements.strips = t_req_strips;
  m_requirements.typing = t_req_typing;
  m_requirements.equality = t_req_equality;

  for (const auto& entity : t_entities) {
    m_entities.emplace_back(entity);
  }
  for (const auto& c : t_initial_state) {
    m_initial_state.emplace_back(c);
  }
  for (const auto& c : t_goal_state) {
    m_goal_state.emplace_back(c);
  }
  return isValid();
}

void rtask::commons::TaskDefinition::clear()
{
  m_name.clear();
  m_domain_name.clear();
  m_entities.clear();
  m_initial_state.clear();
  m_goal_state.clear();
  m_requirements = {};
}

// ------------
// Entity Level
// ------------

bool rtask::commons::TaskDefinition::hasEntity(const std::string& t_symbol, const std::string& t_type) const
{
  if (t_type.empty()) {

    auto equal_name = [t_symbol](const Entity& a) {
      bool eq = false;
      for (unsigned int i = 0; i < a.semantic_entity.symbols.size(); ++i) {
        eq = a.semantic_entity.symbols[i] == t_symbol;
      }
      return eq;
    };
    return std::find_if(m_entities.begin(), m_entities.end(), equal_name) != m_entities.end();
  }
  auto equal_name_type = [t_symbol, t_type](const Entity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.semantic_entity.symbols.size(); ++i) {
      eq = a.semantic_entity.symbols[i] == t_symbol && a.semantic_entity.type == t_type;
    }
    return eq;
  };
  return std::find_if(m_entities.begin(), m_entities.end(), equal_name_type) != m_entities.end();
}

bool rtask::commons::TaskDefinition::getEntityType(const std::string& t_symbol, std::string& t_type) const
{
  if (!hasEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const Entity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.semantic_entity.symbols.size(); ++i) {
      eq = a.semantic_entity.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_entities.begin(), m_entities.end(), equal_name);
  t_type = it->semantic_entity.type;
  return true;
}

bool rtask::commons::TaskDefinition::getEntityClass(const std::string& t_symbol, std::string& t_class) const
{
  if (!hasEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const Entity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.semantic_entity.symbols.size(); ++i) {
      eq = a.semantic_entity.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_entities.begin(), m_entities.end(), equal_name);
  t_class = it->semantic_entity.semantic_class;
  return true;
}

bool rtask::commons::TaskDefinition::getEntityProperties(const std::string& t_symbol,
                                                         std::vector<std::string>& t_properties) const
{
  if (!hasEntity(t_symbol)) {
    return false;
  }
  auto equal_name = [t_symbol](const Entity& a) {
    bool eq = false;
    for (unsigned int i = 0; i < a.semantic_entity.symbols.size(); ++i) {
      eq = a.semantic_entity.symbols[i] == t_symbol;
    }
    return eq;
  };
  auto const it = std::find_if(m_entities.begin(), m_entities.end(), equal_name);
  t_properties = it->properties;
  return true;
}

bool rtask::commons::TaskDefinition::getEntity(const std::string& t_symbol, Entity& t_entity) const
{

  std::string t_type{};
  std::string t_class{};
  std::vector<std::string> t_properties{};

  if (!getEntityType(t_symbol, t_type) || !getEntityClass(t_symbol, t_class)
      || !getEntityProperties(t_symbol, t_properties)) {
    return false;
  }

  t_entity = {{{t_symbol}, t_type, t_class}, t_properties};
  return true;
}

bool rtask::commons::TaskDefinition::addEntity(const std::string& t_symbol,
                                               const std::string& t_type,
                                               const std::string& t_class,
                                               const std::vector<std::string>& t_properties)
{
  if (!hasEntity(t_symbol, t_type)) {
    if (hasEntity(t_symbol)) {
      return false;
    }
    m_entities.push_back({{{t_symbol}, t_type, t_class}, t_properties});
  }
  return true;
}

bool rtask::commons::TaskDefinition::removeEntity(const std::string& t_symbol, const std::string& t_type)
{
  if (hasEntity(t_symbol)) {
    if (!t_type.empty() && !hasEntity(t_symbol, t_type)) {
      return false;
    }
    auto equal_name = [t_symbol](const Entity& a) {
      bool eq = false;
      for (unsigned int i = 0; i < a.semantic_entity.symbols.size(); ++i) {
        eq = a.semantic_entity.symbols[i] == t_symbol;
      }
      return eq;
    };
    auto const it = std::find_if(m_entities.begin(), m_entities.end(), equal_name);
    m_entities.erase(it);
  }
  return true;
}

// -------------
// Private Level
// -------------
bool rtask::commons::TaskDefinition::hasCondition(const Command& t_cmd, unsigned int t_on) const
{
  auto equal = [t_cmd](const Command& c) { return c == t_cmd; };
  if (t_on == INITIAL_STATE) {
    return std::find_if(m_initial_state.begin(), m_initial_state.end(), equal) != m_initial_state.end();
  }
  if (t_on == GOAL_STATE) {
    return std::find_if(m_goal_state.begin(), m_goal_state.end(), equal) != m_goal_state.end();
  }
  return false;
}

bool rtask::commons::TaskDefinition::addCondition(const Command& t_cmd, unsigned int t_on)
{
  auto equal = [t_cmd](const Command& c) { return c == t_cmd; };
  if (t_on == INITIAL_STATE) {
    const auto it = std::find_if(m_initial_state.begin(), m_initial_state.end(), equal);
    if (it == m_initial_state.end()) {
      m_initial_state.emplace_back(t_cmd);
    }
    return true;
  }
  if (t_on == GOAL_STATE) {
    const auto it = std::find_if(m_goal_state.begin(), m_goal_state.end(), equal);
    if (it == m_goal_state.end()) {
      m_goal_state.emplace_back(t_cmd);
    }
    return true;
  }
  return false;
}

bool rtask::commons::TaskDefinition::removeCondition(const Command& t_cmd, unsigned int t_on)
{
  auto equal = [t_cmd](const Command& c) { return c == t_cmd; };
  if (t_on == INITIAL_STATE) {
    const auto it = std::find_if(m_initial_state.begin(), m_initial_state.end(), equal);
    if (it != m_initial_state.end()) {
      m_initial_state.erase(it);
    }
    return true;
  }
  if (t_on == GOAL_STATE) {
    const auto it = std::find_if(m_goal_state.begin(), m_goal_state.end(), equal);
    if (it != m_goal_state.end()) {
      m_goal_state.erase(it);
    }
    return true;
  }
  return false;
}
