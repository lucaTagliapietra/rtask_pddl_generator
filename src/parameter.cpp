#include "commons/parameter.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Parameter::Parameter(const std::string& t_name, const std::string& t_type)
{
  set(t_name, t_type);
}

rtask::commons::Parameter::Parameter(const rtask_msgs::ParameterConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Parameter::Parameter(const rtask_msgs::Parameter& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Parameter::Parameter(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString), true) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (commons::utils::checkXmlRpcSanity("type", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    type_ = static_cast<std::string>(t_rpc_val["type"]);
  }

  valid_ = !name_.empty();
}
void rtask::commons::Parameter::fromMsg(const rtask_msgs::Parameter& t_msg)
{
  name_ = t_msg.name;
  type_ = t_msg.type;

  valid_ = !name_.empty();
}

rtask_msgs::Parameter rtask::commons::Parameter::toMsg() const
{
  rtask_msgs::Parameter msg;
  msg.name = name_;
  msg.type = type_;
  return msg;
}

void rtask::commons::Parameter::set(const std::string& t_name, const std::string& t_type)
{
  name_ = t_name;
  type_ = t_type;
  valid_ = !name_.empty();
}

void rtask::commons::Parameter::setType(const std::string& t_type)
{
  type_ = t_type;
  valid_ = !name_.empty();
}

std::string rtask::commons::Parameter::toPddl(const bool t_typing) const
{
  std::string out{};
  if (valid_) {
    out += "?" + name_;
    if (t_typing) {
      out += " - " + type_;
    }
  }
  return out;
}

bool rtask::commons::Parameter::isEquivalent(const Parameter& t_other, const bool t_typing) const
{
  if (!t_typing) {
    return true;
  }
  // Same validity status and same type
  return type_ == t_other.getType();
}

bool rtask::commons::Parameter::isEqual(const Parameter& t_other, const bool t_typing) const
{
  return isEquivalent(t_other, t_typing) && valid_ == t_other.isValid() && name_ == t_other.getName();
}

// bool rtask::commons::Parameter::operator==(const Parameter& t_parameter) const
// {
//   return isEquivalent(t_parameter) && name_ == t_parameter.getName();
// }

rtask::commons::Parameter& rtask::commons::Parameter::operator=(const Parameter& t_parameter)
{
  name_ = t_parameter.getName();
  valid_ = t_parameter.isValid();
  type_ = t_parameter.getType();
  return *this;
}
