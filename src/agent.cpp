#include "commons/agent.h"

#include "commons/capability.h"
#include "commons/property.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Agent::Agent(const std::string& t_name, const AgentStatus& t_status, const std::string& t_description)
{
  set(t_name, t_status, t_description);
}
rtask::commons::Agent::Agent(const rtask_msgs::Agent& t_msg)
{
  fromMsg(t_msg);
}
rtask::commons::Agent::Agent(const rtask_msgs::AgentConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}
rtask::commons::Agent::Agent(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("description", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    description_ = static_cast<std::string>(t_rpc_val["description"]);
  }

  if (commons::utils::checkXmlRpcSanity("status", t_rpc_val, XmlRpc::XmlRpcValue::TypeInt)) {
    status_ = static_cast<AgentStatus>(static_cast<int>(t_rpc_val["status"]));
  }
  updValidity();
}

rtask_msgs::Agent rtask::commons::Agent::toMsg() const
{
  rtask_msgs::Agent msg;
  msg.name = name_;
  msg.description = description_;
  msg.status = static_cast<uint8_t>(status_);
  return msg;
}

void rtask::commons::Agent::fromMsg(const rtask_msgs::Agent& t_msg)
{
  name_ = t_msg.name;
  description_ = t_msg.description;
  status_ = static_cast<AgentStatus>(t_msg.status);
  updValidity();
}

void rtask::commons::Agent::clear()
{
  name_.clear();
  description_.clear();
  status_ = AgentStatus::UNKNOWN;
  valid_ = false;
}
void rtask::commons::Agent::set(const std::string& t_name,
                                const AgentStatus& t_status,
                                const std::string& t_description)
{
  name_ = t_name;
  status_ = t_status;
  description_ = t_description;
  updValidity();

  valid_ = !t_name.empty();
}

bool rtask::commons::Agent::operator==(const rtask::commons::Agent& t_agent) const
{
  return name_ == t_agent.getName() && status_ == t_agent.getStatus();
}

rtask::commons::Agent& rtask::commons::Agent::operator=(const rtask::commons::Agent& t_agent)
{
  name_ = t_agent.getName();
  description_ = t_agent.getDescription();
  status_ = t_agent.getStatus();
  valid_ = t_agent.isValid();
  return *this;
}

void rtask::commons::Agent::updValidity()
{
  valid_ = !name_.empty() && status_ != AgentStatus::UNKNOWN;
};
