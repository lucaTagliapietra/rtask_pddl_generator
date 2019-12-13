#include <algorithm>

#include "boost/make_shared.hpp"

#include "commons/action.h"
#include "commons/utils.h"

rtask::commons::Action::Action(const std::string t_name,
                               const std::vector<Parameter>& t_params,
                               const std::vector<Command>& t_preconditions,
                               const std::vector<Command>& t_effects)
{
  setAction(t_name, t_params, t_preconditions, t_effects);
}

rtask::commons::Action::Action(const rtask_msgs::Action& t_msg)
{
  setFromActionMsg(t_msg);
}

rtask::commons::Action::Action(const rtask_msgs::ActionConstPtr t_msg_ptr)
{
  setFromActionMsg(t_msg_ptr);
}

rtask::commons::Action::Action(XmlRpc::XmlRpcValue& t_node, bool t_typing, bool t_equality, bool t_strips)
{
  setActionFromXmlRpc(t_node, t_typing, t_equality, t_strips);
}

bool rtask::commons::Action::isValid() const
{
  return !m_name.empty() && !m_params.empty() && !m_preconditions.empty() && !m_effects.empty();
}

rtask_msgs::ActionPtr rtask::commons::Action::toActionMsg() const
{
  rtask_msgs::ActionPtr t_agent = boost::make_shared<rtask_msgs::Action>();
  t_agent->name = m_name;
  for (const auto& param : m_params) {
    t_agent->params.emplace_back(param.toMsg());
  }
  for (const auto& cmd : m_preconditions) {
    t_agent->preconditions.emplace_back(cmd.toMsg());
  }
  for (const auto& cmd : m_effects) {
    t_agent->preconditions.emplace_back(cmd.toMsg());
  }
}

std::string rtask::commons::Action::toPddl(bool t_typing, bool t_equality, bool t_strips) const
{
  std::string out = "(:action " + m_name + "\n";
  out += "\t:parameters (";
  unsigned int count = 0;
  for (const auto& p : m_params) {
    count == 0 ? out += "?" : out += " ?";
    out += (p.name + " ");
    if (t_typing) {
      out += ("- " + p.type);
    }
  }
  out += ")\n";
  out += "\t:precondition (and";
  count = 0;
  for (const auto& pr : m_preconditions) {
    if (count != 0)
      out += "\n\t\t";
    out += " (";
    pr.negate ? out += "not (" : "";
    out += pr.predicate.cmd;
    for (const auto& p : pr.predicate.args) {
      out += " ?";
      out += (p.name);
    }
    out += ")";
    pr.negate ? out += ")" : out += "";
  }
  out += ")";
  out += "\n\t:effect (and";
  count = 0;
  for (const auto& pr : m_effects) {
    if (count != 0)
      out += "\n\t\t";
    out += " (";
    pr.negate ? out += "not (" : "";
    out += pr.predicate.cmd;
    for (const auto& p : pr.predicate.args) {
      out += " ?";
      out += (p.name);
    }
    out += ")";
    pr.negate ? out += ")" : out += "";
  }
  out += "))\n";
  return out;
}

bool rtask::commons::Action::setActionFromXmlRpc(XmlRpc::XmlRpcValue& t_node,
                                                 bool t_typing,
                                                 bool t_equality,
                                                 bool t_strips)
{
  // Clear all just to be sure
  clear();

  // Name
  if (commons::utils::checkXmlRpcSanity("name", t_node, XmlRpc::XmlRpcValue::TypeString)) {
    m_name = static_cast<std::string>(t_node["name"]);
  }

  // Parameters
  if (commons::utils::checkXmlRpcSanity("params", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["params"].size() == 0) {
      std::cout << "Empty parameters vector for action " << m_name << std::endl;
      return isValid();
    }
    for (auto& param : t_node["params"]) {
      if (!commons::utils::checkXmlRpcSanity("name", param.second, XmlRpc::XmlRpcValue::TypeString)) {
        std::cout << "Empty name for parameter not allowed in action" << m_name << std::endl;
        return isValid();
      }
      std::string name = static_cast<std::string>(param.second["name"]);
      std::string type = commons::NULL_TYPE;

      if (t_typing) {
        if (!utils::checkXmlRpcSanity("type", param.second, XmlRpc::XmlRpcValue::TypeString)) {
          std::cout << "Empty or invalid type for parameter " << name
                    << " not allowed when typing is required. Action: " << m_name << std::endl;
          continue;
        }
        type = static_cast<std::string>(param.second["type"]);
      }
      m_params.emplace_back(name, type);
    }
  }

  // Preconditions
  if (commons::utils::checkXmlRpcSanity("preconditions", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["preconditions"].size() == 0) {
      std::cout << "Empty preconditions vector for action " << m_name << std::endl;
      return false;
    }
    for (auto& prec : t_node["preconditions"]) {
      if (!commons::utils::checkXmlRpcSanity("cmd", prec.second, XmlRpc::XmlRpcValue::TypeString)) {
        std::cout << "Empty or invalid cmd in precondition for action " << m_name << std::endl;
        return false;
      }
      std::string cmd = prec.second["cmd"];
      std::vector<Parameter> args{};
      bool negate = false;
      if (commons::utils::checkXmlRpcSanity("args", prec.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : prec.second["args"]) {
          std::string arg_name = static_cast<std::string>(arg.second);
          std::string arg_type{};
          if (getParameterType(arg_name, arg_type)) {
            args.emplace_back(arg_name, arg_type);
          }
        }
      }

      if (prec.second.hasMember("not") && prec.second["not"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        negate = prec.second["not"];
      }
      m_preconditions.push_back({{cmd, args}, negate});
    }
  }

  // Effects
  if (commons::utils::checkXmlRpcSanity("effects", t_node, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_node["effects"].size() == 0) {
      std::cout << "Empty effects vector for action " << m_name << std::endl;
      return false;
    }
    for (auto& eff : t_node["effects"]) {
      if (!commons::utils::checkXmlRpcSanity("cmd", eff.second, XmlRpc::XmlRpcValue::TypeString)) {
        std::cout << "Empty or invalid cmd in effects for action " << m_name << std::endl;
        return false;
      }
      std::string cmd = eff.second["cmd"];
      std::vector<Parameter> args{};
      bool negate = false;
      if (commons::utils::checkXmlRpcSanity("args", eff.second, XmlRpc::XmlRpcValue::TypeArray)) {
        for (auto& arg : eff.second["args"]) {
          std::string arg_name = static_cast<std::string>(arg.second);
          std::string arg_type{};
          if (getParameterType(arg_name, arg_type)) {
            args.emplace_back(arg_name, arg_type);
          }
        }
      }

      if (eff.second.hasMember("not") && eff.second["not"].getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
        negate = eff.second["not"];
      }
      m_effects.push_back({{cmd, args}, negate});
    }
  }
  return isValid();
}

bool rtask::commons::Action::setFromActionMsg(const rtask_msgs::Action& t_msg)
{
  m_name = t_msg.name;
  for (const auto& param : t_msg.params) {
    m_params.emplace_back(param);
  }
  for (const auto& cmd : t_msg.preconditions) {
    m_preconditions.emplace_back(cmd);
  }
  for (const auto& cmd : t_msg.effects) {
    m_effects.emplace_back(cmd);
  }
  return isValid();
}

bool rtask::commons::Action::setFromActionMsg(const rtask_msgs::ActionConstPtr& t_msg_ptr)
{
  return setFromActionMsg(*t_msg_ptr);
}

bool rtask::commons::Action::setAction(const std::string t_name,
                                       const std::vector<Parameter>& t_params,
                                       const std::vector<Command>& t_preconditions,
                                       const std::vector<Command>& t_effects)
{
  m_name = t_name;
  for (const auto& param : t_params) {
    m_params.emplace_back(param);
  }
  for (const auto& cmd : t_preconditions) {
    m_preconditions.emplace_back(cmd);
  }
  for (const auto& cmd : t_effects) {
    m_effects.emplace_back(cmd);
  }
  return isValid();
}

void rtask::commons::Action::clear()
{
  m_name.clear();
  m_params.clear();
  m_preconditions.clear();
  m_effects.clear();
}

// ---------------
// Parameter Level
// ---------------
bool rtask::commons::Action::hasParameter(const std::string& t_name, const std::string& t_type) const
{
  if (t_type.empty()) {
    auto equal_name = [t_name](const Parameter& a) { return a.name == t_name; };
    return std::find_if(m_params.begin(), m_params.end(), equal_name) != m_params.end();
  }
  auto equal_name_type = [t_name, t_type](const Parameter& a) { return (a.name == t_name) && a.type == t_type; };
  return std::find_if(m_params.begin(), m_params.end(), equal_name_type) != m_params.end();
}

bool rtask::commons::Action::getParameterType(const std::string& t_name, std::string& t_type) const
{
  if (!hasParameter(t_name)) {
    return false;
  }
  auto equal_name = [t_name](const Parameter& a) { return a.name == t_name; };
  auto const it = std::find_if(m_params.begin(), m_params.end(), equal_name);
  t_type = it->type;
  return true;
}

bool rtask::commons::Action::addParameter(const std::string& t_name, const std::string& t_type)
{
  if (!hasParameter(t_name, t_type)) {
    if (hasParameter(t_name)) {
      return false;
    }
    m_params.emplace_back(t_name, t_type);
  }
  return true;
}

bool rtask::commons::Action::removeParameter(const std::string& t_name, const std::string& t_type)
{
  if (hasParameter(t_name)) {
    if (!t_type.empty() && !hasParameter(t_name, t_type)) {
      return false;
    }
    auto equal_name = [t_name](const Parameter& a) { return a.name == t_name; };
    auto const it = std::find_if(m_params.begin(), m_params.end(), equal_name);
    m_params.erase(it);
  }
  return true;
}

// ---------------
// Private Section
// ---------------

bool rtask::commons::Action::hasCommand(const Command& t_cmd, unsigned int t_on) const
{
  auto equal = [t_cmd](const Command& c) { return c == t_cmd; };
  if (t_on == PRECONDITION) {
    return std::find_if(m_preconditions.begin(), m_preconditions.end(), equal) != m_preconditions.end();
  }
  if (t_on == EFFECT) {
    return std::find_if(m_effects.begin(), m_effects.end(), equal) != m_effects.end();
  }
  return false;
}
bool rtask::commons::Action::addCommand(const Command& t_cmd, unsigned int t_on)
{
  auto equal = [t_cmd](const Command& c) { return c == t_cmd; };
  if (t_on == PRECONDITION) {
    const auto it = std::find_if(m_preconditions.begin(), m_preconditions.end(), equal);
    if (it == m_preconditions.end()) {
      m_preconditions.emplace_back(t_cmd);
    }
    return true;
  }
  if (t_on == EFFECT) {
    const auto it = std::find_if(m_effects.begin(), m_effects.end(), equal);
    if (it == m_effects.end()) {
      m_effects.emplace_back(t_cmd);
    }
    return true;
  }
  return false;
}

bool rtask::commons::Action::removeCommand(const Command& t_cmd, unsigned int t_on)
{
  auto equal = [t_cmd](const Command& c) { return c == t_cmd; };
  if (t_on == PRECONDITION) {
    const auto it = std::find_if(m_preconditions.begin(), m_preconditions.end(), equal);
    if (it != m_preconditions.end()) {
      m_preconditions.erase(it);
    }
    return true;
  }
  if (t_on == EFFECT) {
    const auto it = std::find_if(m_effects.begin(), m_effects.end(), equal);
    if (it != m_effects.end()) {
      m_effects.erase(it);
    }
    return true;
  }
  return false;
}
