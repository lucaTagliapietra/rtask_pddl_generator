#include "pddl_generator/LiteralTerm.h"
#include "pddl_generator/Helpers.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
LiteralTerm::LiteralTerm()
{
  obj_type_ = TermType::LiteralTerm;
}

LiteralTerm::LiteralTerm(const std::string& t_name, const std::string& t_type)
{
  set(t_name, t_type);
  obj_type_ = TermType::LiteralTerm;
}

LiteralTerm::LiteralTerm(XmlRpc::XmlRpcValue& t_rpc_val)
{

  std::string name, type = {};

  if (helpers::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    name = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (helpers::checkXmlRpcSanity("type", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    type = static_cast<std::string>(t_rpc_val["type"]);
  }

  set(name, type);
  obj_type_ = TermType::LiteralTerm;
}

std::string LiteralTerm::toPddl(const bool t_typing) const
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

bool LiteralTerm::validate(const UnordStrToLitTermMap& t_known_types) const
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

bool LiteralTerm::operator==(const LiteralTerm& t_other) const
{
  return name_ == t_other.getName() && type_ == t_other.getType();
}

LiteralTerm& LiteralTerm::operator=(const LiteralTerm& t_other)
{
  set(t_other.getName(), t_other.getType());
  return *this;
}

void LiteralTerm::set(const std::string& t_name, const std::string& t_type)
{
  if (t_name.empty()) {
    std::cerr << "Empty TypedName object name" << std::endl;
    return;
  }
  std::string type = std::move(t_type);
  if (type.empty()) {
    std::cout << "Empty TypedName object Type not allowed, setting to default **object** type" << std::endl;
    type = "object";
  }

  name_ = std::move(t_name);
  type_ = std::move(type);
}
