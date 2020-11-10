#include "commons/agentGroup.h"

#include "commons/utils.h"
// ------------
// CONSTRUCTORS
// ------------

rtask::commons::AgentGroup::AgentGroup(const std::string& t_name,
                                       const std::string& t_description,
                                       const std::vector<Agent> t_agents)
{
  set(t_name, t_description, t_agents);
}

rtask::commons::AgentGroup::AgentGroup(const rtask_msgs::AgentGroup& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::AgentGroup::AgentGroup(const rtask_msgs::AgentGroupConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::AgentGroup::AgentGroup(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("description", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    description_ = static_cast<std::string>(t_rpc_val["description"]);
  }

  if (commons::utils::checkXmlRpcSanity("agents", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["agents"].size() == 0) {
      std::cout << "Empty agents vector for AgentGroup " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["agents"].size(); ++i) {
        Agent a(t_rpc_val["agents"][i]);
        if (a.isValid()) {
          agents_.push_back(a);
        }
      }
    }
  }

  updValidity();
}

// -----------------
// Agent Group Level
// -----------------

rtask_msgs::AgentGroup rtask::commons::AgentGroup::toMsg() const
{
  rtask_msgs::AgentGroup msg;
  msg.name = name_;
  msg.description = description_;
  for (const auto& a : agents_) {
    msg.agents.emplace_back(a.toMsg());
  }
  return msg;
}

void rtask::commons::AgentGroup::clear()
{
  name_.clear();
  description_.clear();
  agents_.clear();
}
void rtask::commons::AgentGroup::set(const std::string& t_name,
                                     const std::string& t_description,
                                     const std::vector<Agent>& t_agents)
{
  name_ = t_name;
  description_ = t_description;
  agents_ = t_agents;
  updValidity();
}

std::vector<std::string> rtask::commons::AgentGroup::getAgentList() const
{
  std::vector<std::string> out;
  std::for_each(agents_.begin(), agents_.end(), [&out](const auto& a) { out.emplace_back(a.getName()); });
  return out;
}

// -----------
// Agent Level
// -----------

bool rtask::commons::AgentGroup::hasAgent(const std::string& t_name) const
{
  auto it = std::find_if(agents_.begin(), agents_.end(), [t_name](auto& a) { return a.getName() == t_name; });
  return it != agents_.end();
}

bool rtask::commons::AgentGroup::deleteAgent(const std::string& t_name)
{
  auto it = std::find_if(agents_.begin(), agents_.end(), [t_name](auto& a) { return a.getName() == t_name; });
  if (it == agents_.end()) {
    return false;
  }

  agents_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::AgentGroup::isAgentValid(const std::string& t_name) const
{
  auto it = std::find_if(agents_.begin(), agents_.end(), [t_name](auto& a) { return a.getName() == t_name; });
  if (it == agents_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Agent> rtask::commons::AgentGroup::getAgent(const std::string& t_name) const
{
  auto it = std::find_if(agents_.begin(), agents_.end(), [t_name](auto& a) { return a.getName() == t_name; });
  if (it == agents_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::AgentGroup::setAgent(const std::string& t_name,
                                          const std::string& t_descr,
                                          const commons::AgentStatus& t_status)
{
  auto it = std::find_if(agents_.begin(), agents_.end(), [t_name](auto& a) { return a.getName() == t_name; });

  if (it == agents_.end()) {
    agents_.emplace_back(t_name, t_descr, t_status);
    updValidity();
    return;
  }
  it->set(t_name, t_descr, t_status);
  updValidity();
}

void rtask::commons::AgentGroup::setAgent(const std::string& t_name, const Agent& t_val)
{
  auto it = std::find_if(agents_.begin(), agents_.end(), [t_name](auto& a) { return a.getName() == t_name; });

  if (it == agents_.end()) {
    agents_.emplace_back(t_name, t_val.getDescription(), t_val.getStatus());
    updValidity();
    return;
  }
  it->set(t_name, t_val.getDescription(), t_val.getStatus());
  updValidity();
}

void rtask::commons::AgentGroup::fromMsg(const rtask_msgs::AgentGroup& t_msg)
{
  name_ = t_msg.name;
  description_ = t_msg.description;
  for (const auto& a : t_msg.agents) {
    agents_.emplace_back(a);
  }

  updValidity();
}

bool rtask::commons::AgentGroup::operator==(const rtask::commons::AgentGroup& t_agent_group) const
{
  if (!(name_ == t_agent_group.getName()))
    return false;

  auto other_agents = t_agent_group.getAgents();
  for (const auto& a : agents_) {
    auto it = std::find_if(other_agents.begin(), other_agents.end(), [a](auto& oa) { return a == oa; });
    if (it == other_agents.end()) {
      return false;
    }
  }
  return true;
}

rtask::commons::AgentGroup& rtask::commons::AgentGroup::operator=(const rtask::commons::AgentGroup& t_agent_group)
{
  name_ = t_agent_group.getName();
  description_ = t_agent_group.getDescription();
  agents_ = t_agent_group.getAgents();
  valid_ = t_agent_group.isValid();
  return *this;
}

void rtask::commons::AgentGroup::updValidity()
{
  valid_ = !name_.empty();
  std::for_each(agents_.begin(), agents_.end(), [this](const auto& a) { valid_ &= a.isValid(); });
}
