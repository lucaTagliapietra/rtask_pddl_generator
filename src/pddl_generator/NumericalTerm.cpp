#include "pddl_generator/NumericalTerm.h"
#include "pddl_generator/Helpers.h"

using namespace rtask::commons::pddl_generator;

// ------------
// CONSTRUCTORS
// ------------
NumericalTerm::NumericalTerm()
{
  obj_type_ = TermType::NumericalTerm;
  expr_type_ = NumericalExpressionType::Term;
}

NumericalTerm::NumericalTerm(const double& t_value)
{
  set(t_value);
  obj_type_ = TermType::NumericalTerm;
  expr_type_ = NumericalExpressionType::Term;
}

NumericalTerm::NumericalTerm(const int& t_value)
{
  set(t_value);
  obj_type_ = TermType::NumericalTerm;
  expr_type_ = NumericalExpressionType::Term;
}

NumericalTerm::NumericalTerm(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (helpers::checkXmlRpcSanity("num_term", t_rpc_val, XmlRpc::XmlRpcValue::TypeInt, true)) {
    val_ = static_cast<int>(t_rpc_val["num_term"]);
  }
  else if (helpers::checkXmlRpcSanity("num_term", t_rpc_val, XmlRpc::XmlRpcValue::TypeDouble, true)) {
    val_ = static_cast<double>(t_rpc_val["num_term"]);
  }
  else {
    std::cerr << "Fatal: Invalid Numerical Value, should be a Double or Int" << std::endl;
    exit(EXIT_FAILURE);
  }
  obj_type_ = TermType::NumericalTerm;
  expr_type_ = NumericalExpressionType::Term;
}

void NumericalTerm::set(const double& t_value)
{
  val_ = std::move(t_value);
}
void NumericalTerm::set(const int& t_value)
{
  val_ = std::move(t_value);
}

NumericalTerm& NumericalTerm::operator=(const NumericalTerm& t_other)
{
  val_ = t_other.getValue();
  return *this;
}

std::string NumericalTerm::toPddl(bool, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  std::string w = std::visit([](auto&& arg) -> std::string { return std::to_string(arg); }, val_);
  return pad_aligners.second.at(0) + w;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const NumericalTerm& t_nt)
{
  auto t = t_nt.getValue();
  std::string w = std::visit([](auto&& arg) -> std::string { return std::to_string(arg); }, t);
  t_out << "num_term: " << w;
  return t_out;
}

bool rtask::commons::pddl_generator::operator==(const NumericalTerm& t_first, const NumericalTerm& t_second)
{
  return t_first.getValue() == t_second.getValue();
}
