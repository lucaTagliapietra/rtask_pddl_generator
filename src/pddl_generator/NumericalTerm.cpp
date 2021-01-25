#include "pddl_generator/NumericalTerm.h"
#include "pddl_generator/Helpers.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
NumericalTerm::NumericalTerm()
{
  obj_type_ = TermType::NumericalTerm;
}

NumericalTerm::NumericalTerm(const double& t_value)
{
  set(t_value);
}

NumericalTerm::NumericalTerm(XmlRpc::XmlRpcValue& t_rpc_val)
{
  double value{};

  if (helpers::checkXmlRpcSanity("value", t_rpc_val, XmlRpc::XmlRpcValue::TypeDouble, true)) {
    value = static_cast<double>(t_rpc_val["name"]);
  }

  set(value);
}

bool NumericalTerm::operator==(const NumericalTerm& t_other) const
{
  return value_ == t_other.getValue();
}

NumericalTerm& NumericalTerm::operator=(const NumericalTerm& t_other)
{
  set(t_other.getValue());
  return *this;
}

std::string NumericalTerm::toPddl(const bool) const
{
  return std::to_string(value_);
}

std::any NumericalTerm::getAsChild(Term& t_parent) const
{
  return {dynamic_cast<NumericalTerm&>(t_parent)};
}

std::any NumericalTerm::getAsChild(std::shared_ptr<Term> t_parent) const
{
  return {std::dynamic_pointer_cast<NumericalTerm>(t_parent)};
}
