#include "commons/typedTerm.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::TypedTerm::TypedTerm(const std::string& t_name, const std::string& t_type)
{
  set(t_name, t_type);
}

rtask::commons::TypedTerm::TypedTerm(XmlRpc::XmlRpcValue& t_rpc_val)
{
  std::string name = {};
  std::string type = {};

  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString), true) {
    name = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (commons::utils::checkXmlRpcSanity("type", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    type = static_cast<std::string>(t_rpc_val["type"]);
  }

  set(name, type);
}

void rtask::commons::TypedTerm::set(const std::string& t_name, const std::string& t_type)
{
  if (t_name.empty()) {
    std::cerr << "Empty TypedName object name" << std::endl;
    return;
  }
  std::string type = std::move(t_type);
  if (type.empty()) {
    std::cout << "Empty TypedName object Type not allowed, setting to default **object** type" << std::endl;
    type = {"object"};
  }

  name_ = std::move(t_name);
  type_ = std::move(type);
}

std::string rtask::commons::TypedTerm::toPddl(const bool t_typing) const
{
  if (name_.empty()) {
    return {};
  };

  std::string out = "?" + name_;
  if (t_typing) {
    out += " - " + type_;
  }
  return out;
}

bool rtask::commons::TypedTerm::validate(const UnorderedNameTypedTermMap& t_known_types) const
{
  if (name_.empty()) {
    std::cerr << "VALIDATION ERROR: Empty TypedName name" << std::endl;
    return false;
  }

  if (!t_known_types.count(type_)) {
    std::cerr << "VALIDATION ERROR: Unknown Type **" << type_ << "**" << std::endl;
    return false;
  }
  return true;
}

bool rtask::commons::TypedTerm::operator==(const TypedTerm& t_other) const
{
  return name_ == t_other.getName() && type_ == t_other.getType();
}

rtask::commons::TypedTerm& rtask::commons::TypedTerm::operator=(const TypedTerm& t_other)
{
  set(t_other.getName(), t_other.getType());
  return *this;
}
