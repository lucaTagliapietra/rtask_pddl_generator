#include <algorithm>

#include "boost/make_shared.hpp"

#include "commons/taskDefinition.h"
#include "commons/utils.h"

rtask::commons::TaskDefinition::TaskDefinition(const std::string& t_name,
                                               const std::string& t_domain_name,
                                               const std::vector<Parameter>& t_objects,
                                               const std::vector<Command>& t_initial_state,
                                               const std::vector<Command>& t_goal_state,
                                               const bool t_req_typing,
                                               const bool t_req_equality,
                                               const bool t_req_strips)
{
  setTaskDefinition(
    t_name, t_domain_name, t_objects, t_initial_state, t_goal_state, t_req_typing, t_req_equality, t_req_strips);
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
  return !m_name.empty() && !m_domain_name.empty() && !m_objects.empty() && !m_initial_state.empty()
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
  for (const auto& obj : m_objects) {
    t_msg->objs.emplace_back(obj.toMsg());
  }
  for (const auto& c : m_initial_state) {
    t_msg->initial_state.emplace_back(c.toMsg());
  }
  for (const auto& c : m_goal_state) {
    t_msg->goal_state.emplace_back(c.toMsg());
  }
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
  for (const auto& o : m_objects) {
    out += (" " + o.name);
    if (m_requirements.typing) {
      out += (" - " + o.type);
    }
  }
  out += ")\n";

  out += "\t(:init ";
  unsigned count = 0;
  for (const auto& is : m_initial_state) {
    if (count != 0)
      out += "\n\t\t";
    out += "(";
    out += is.predicate.cmd;
    for (const auto& p : is.predicate.args) {
      out += " ";
      out += (p.name);
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
    out += gs.predicate.cmd;
    for (const auto& p : gs.predicate.args) {
      out += " ";
      out += (p.name);
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

  // Objects
  if (commons::utils::checkXmlRpcSanity("objects", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["objects"].size() == 0) {
      std::cout << "Empty object vector for task " << m_name << " in domain: " << m_domain_name << std::endl;
      return isValid();
    }
    for (auto& objs_same_type : t_node["objects"]) {
      std::string type = commons::NULL_TYPE;

      if (commons::utils::checkXmlRpcSanity("type", objs_same_type.second, XmlRpc::XmlRpcValue::TypeString)) {
        type = static_cast<std::string>(objs_same_type.second["type"]);
      }

      if (commons::utils::checkXmlRpcSanity("names", objs_same_type.second, XmlRpc::XmlRpcValue::TypeArray)) {
        if (objs_same_type.second["names"].size() == 0) {
          std::cout << "Empty object name vector for type " << type << " for task " << m_name
                    << " in domain: " << m_domain_name << std::endl;
          return isValid();
        }
        for (auto& obj : objs_same_type.second["names"]) {
          if (obj.second.getType() == XmlRpc::XmlRpcValue::TypeString
              && !static_cast<std::string>(obj.second).empty()) {
            m_objects.emplace_back(static_cast<std::string>(obj.second), type);
          }
        }
      }
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
      std::string cmd = cond.second["cmd"];
      std::vector<Parameter> args{};
      bool negate = false;
      if (commons::utils::checkXmlRpcSanity("args", cond.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : cond.second["args"]) {
          std::string arg_name = static_cast<std::string>(arg.second);
          std::string arg_type{};
          if (getObjectType(arg_name, arg_type)) {
            args.emplace_back(arg_name, arg_type);
          }
        }
      }

      if (cond.second.hasMember("not") && cond.second["not"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        negate = cond.second["not"];
      }
      m_initial_state.push_back({{cmd, args}, negate});
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
      std::string cmd = cond.second["cmd"];
      std::vector<Parameter> args{};
      bool negate = false;
      if (commons::utils::checkXmlRpcSanity("args", cond.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : cond.second["args"]) {
          std::string arg_name = static_cast<std::string>(arg.second);
          std::string arg_type{};
          if (getObjectType(arg_name, arg_type)) {
            args.emplace_back(arg_name, arg_type);
          }
        }
      }

      if (cond.second.hasMember("not") && cond.second["not"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        negate = cond.second["not"];
      }
      m_goal_state.push_back({{cmd, args}, negate});
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

  for (const auto& obj : t_msg.objs) {
    m_objects.emplace_back(obj);
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
                                                       const std::vector<Parameter>& t_objects,
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

  for (const auto& obj : t_objects) {
    m_objects.emplace_back(obj);
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
  m_objects.clear();
  m_initial_state.clear();
  m_goal_state.clear();
  m_requirements = {};
}

// ------------
// Object Level
// ------------
bool rtask::commons::TaskDefinition::hasObject(const std::string& t_name, const std::string& t_type) const
{
  if (t_type.empty()) {
    auto equal_name = [t_name](const Parameter& a) { return a.name == t_name; };
    return std::find_if(m_objects.begin(), m_objects.end(), equal_name) != m_objects.end();
  }
  auto equal_name_type = [t_name, t_type](const Parameter& a) { return (a.name == t_name) && a.type == t_type; };
  return std::find_if(m_objects.begin(), m_objects.end(), equal_name_type) != m_objects.end();
}

bool rtask::commons::TaskDefinition::getObjectType(const std::string& t_name, std::string& t_type) const
{
  if (!hasObject(t_name)) {
    return false;
  }
  auto equal_name = [t_name](const Parameter& a) { return a.name == t_name; };
  auto const it = std::find_if(m_objects.begin(), m_objects.end(), equal_name);
  t_type = it->type;
  return true;
}

bool rtask::commons::TaskDefinition::addObject(const std::string& t_name, const std::string& t_type)
{
  if (!hasObject(t_name, t_type)) {
    if (hasObject(t_name)) {
      return false;
    }
    m_objects.emplace_back(t_name, t_type);
  }
  return true;
}

bool rtask::commons::TaskDefinition::removeObject(const std::string& t_name, const std::string& t_type)
{
  if (hasObject(t_name)) {
    if (!t_type.empty() && !hasObject(t_name, t_type)) {
      return false;
    }
    auto equal_name = [t_name](const Parameter& a) { return a.name == t_name; };
    auto const it = std::find_if(m_objects.begin(), m_objects.end(), equal_name);
    m_objects.erase(it);
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
