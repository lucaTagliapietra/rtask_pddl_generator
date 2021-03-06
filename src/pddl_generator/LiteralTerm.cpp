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

LiteralTerm::LiteralTerm(const std::string& t_name, const std::string& t_type, const bool t_is_a_const_term)
{
  set(t_name, t_type, t_is_a_const_term);
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

std::string LiteralTerm::toPddl(const bool t_typing, int) const
{
  if (name_.empty()) {
    return {};
  };

  std::string out = is_a_constant_term_ ? "" : "?";
  out += name_;
  if (t_typing) {
    out += " - " + type_;
  }
  return out;
}

bool LiteralTerm::isValid(const UmapStrStr& t_known_types) const
{
  return (!name_.empty() && !type_.empty() && t_known_types.count(type_) != 0);
}

bool LiteralTerm::isEquivalentTo(const LiteralTerm& t_other) const
{
  return type_ == t_other.getType();
}

LiteralTerm& LiteralTerm::operator=(const LiteralTerm& t_other)
{
  set(t_other.getName(), t_other.getType());
  return *this;
}

void LiteralTerm::set(const std::string& t_name, const std::string& t_type, const bool t_is_a_const_term)
{
  if (t_name.empty()) {
    std::cerr << "Empty LiteralTerm object name" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string type = std::move(t_type);
  if (type.empty()) {
    std::cout << "Empty LiteralTerm object Type not allowed, setting to default **object** type" << std::endl;
    type = "object";
  }

  name_ = std::move(t_name);
  type_ = std::move(type);
  is_a_constant_term_ = t_is_a_const_term;
}

bool rtask::commons::pddl_generator::operator==(const LiteralTerm& t_first, const LiteralTerm& t_second)
{
  return t_first.getName() == t_second.getName() && t_first.getType() == t_second.getType();
}

bool rtask::commons::pddl_generator::operator!=(const LiteralTerm& t_first, const LiteralTerm& t_second)
{
  return !(t_first == t_second);
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const LiteralTerm& t_lt)
{
  return t_out << " ## LITERAL TERM ## " << std::endl << t_lt.toPddl();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<LiteralTerm> t_lt_ptr)
{
  t_out << (t_lt_ptr ? *t_lt_ptr : LiteralTerm());
  return t_out;
}
