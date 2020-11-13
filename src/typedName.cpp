#include "commons/typedName.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::TypedName::TypedName(const std::string& t_name, const std::string& t_type)
{
  set(t_name, t_type);
}

rtask::commons::TypedName::TypedName(const rtask_msgs::ParameterConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::TypedName::TypedName(const rtask_msgs::Parameter& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::TypedName::TypedName(XmlRpc::XmlRpcValue& t_rpc_val)
{
  std::string name = {};
  std::string type_name = {};

  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString), true) {
    name = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (commons::utils::checkXmlRpcSanity("type", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    type_name = static_cast<std::string>(t_rpc_val["type"]);
  }

  set(name, type_name);
}

void rtask::commons::TypedName::fromMsg(const rtask_msgs::Parameter& t_msg)
{
  set(t_msg.name, t_msg.type);
}

rtask_msgs::Parameter rtask::commons::TypedName::toMsg() const
{
  rtask_msgs::Parameter msg;
  msg.name = name_;
  msg.type = type_name_;
  return msg;
}

void rtask::commons::TypedName::set(const std::string& t_name, const std::string& t_type_name)
{
  if (t_name.empty()) {
    std::cerr << "Empty TypedName object name" << std::endl;
    return;
  }
  std::string type = std::move(t_type_name);
  if (type.empty()) {
    std::cout << "Empty TypedName object Type not allowed, setting to default **object** type" << std::endl;
    type = {"object"};
  }

  name_ = std::move(t_name);
  type_name_ = std::move(type);
}

std::string rtask::commons::TypedName::toPddl(const bool t_typing) const
{
  if (name_.empty()) {
    return {};
  };

  std::string out = "?" + name_;
  if (t_typing) {
    out += " - " + type_name_;
  }
  return out;
}

bool rtask::commons::TypedName::validate(const UnorderedTypedNameMap& t_known_types) const
{
  if (!t_known_types.count(type_name_)) {
    std::cerr << "VALIDATION ERROR: Unknown Type **" << type_name_ << "**" << std::endl;
    return false;
  }
  return true;
}

bool rtask::commons::TypedName::operator==(const TypedName& t_other) const
{
  return name_ == t_other.getName() && type_name_ == t_other.getTypeName();
}

rtask::commons::TypedName& rtask::commons::TypedName::operator=(const TypedName& t_other)
{
  set(t_other.getName(), t_other.getTypeName());
  return *this;
}
