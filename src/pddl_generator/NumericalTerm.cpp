#include "pddl_generator/NumericalTerm.h"
#include "pddl_generator/Helpers.h"

#include <math.h>

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
  const auto tag_type = helpers::getTagValueType("num_term", t_rpc_val);
  if (tag_type == XmlRpc::XmlRpcValue::TypeInt) {
    val_ = static_cast<int>(t_rpc_val["num_term"]);
  }
  else if (tag_type == XmlRpc::XmlRpcValue::TypeDouble) {
    val_ = static_cast<double>(t_rpc_val["num_term"]);
  }
  else {
    std::cerr << "Fatal: Invalid Numerical Value, should be Int or Double" << std::endl;
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

bool rtask::commons::pddl_generator::operator==(const NumericalTerm& t_first, const NumericalTerm& t_second)
{
  return t_first.getValue() == t_second.getValue();
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const NumericalTerm& t_nt)
{
  auto t = t_nt.getValue();
  std::string w = std::visit([](auto&& arg) -> std::string { return std::to_string(arg); }, t);
  t_out << "NumericalTerm: " << w;
  return t_out;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, std::shared_ptr<NumericalTerm> t_nt_ptr)
{
  t_nt_ptr = nullptr;
  t_out << (t_nt_ptr ? *t_nt_ptr : NumericalTerm());
  return t_out;
}
