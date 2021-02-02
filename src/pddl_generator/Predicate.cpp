#include "pddl_generator/Predicate.h"
#include "pddl_generator/Helpers.h"

#include <algorithm>

using namespace rtask::commons::pddl_generator;

Predicate::Predicate(const std::string& t_name, const LiteralTermVector& t_params)
{
  set(t_name, t_params);
}

Predicate::Predicate(XmlRpc::XmlRpcValue& t_rpc_val)
{

  if (!(t_rpc_val.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct
        && helpers::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeString, false)
        && helpers::checkXmlRpcSanity("params", t_rpc_val, XmlRpc::XmlRpcValue::Type::TypeArray))) {
    std::cerr << "Fatal: Invalid Predicate Structure, should be a Struct with name and params fields" << std::endl;
    exit(EXIT_FAILURE);
  }

  name_ = static_cast<std::string>(t_rpc_val["name"]);

  if (t_rpc_val["params"].size() == 0) {
    std::cout << "Empty params vector for Predicate " << name_ << std::endl;
  }
  else {
    for (int i = 0; i < t_rpc_val["params"].size(); ++i) {
      params_.emplace_back(t_rpc_val["params"][i]);
    }
  }
}

void Predicate::clear()
{
  name_.clear();
  params_.clear();
}

bool Predicate::set(const std::string& t_name, const LiteralTermVector& t_params)
{
  setParameters(t_params);
  return setName(t_name);
}

bool Predicate::setName(const std::string& t_name)
{
  if (t_name.empty()) {
    std::cerr << "Invalid empty name for current Predicate" << std::endl;
    return false;
  }
  name_ = std::move(t_name);
  return true;
}

void Predicate::setParameters(const LiteralTermVector& t_params)
{
  params_ = std::move(t_params);
}

int Predicate::getParameterIndex(const std::string& t_name) const
{
  auto it = std::find_if(params_.begin(), params_.end(), [t_name](const auto& p) { return p.getName() == t_name; });
  if (it != params_.end()) {
    return static_cast<int>(params_.begin() - it);
  }
  else {
    return -1;
  }
}

int Predicate::getParameterIndex(const LiteralTerm& t_param) const
{
  auto it = std::find_if(params_.begin(), params_.end(), [t_param](const auto& p) { return p == t_param; });
  if (it != params_.end()) {
    return static_cast<int>(params_.begin() - it);
  }
  else {
    return -1;
  }
}

bool Predicate::hasParameter(const std::string& t_name) const
{
  return getParameterIndex(t_name) != -1;
}

bool Predicate::hasParameter(const LiteralTerm& t_param) const
{
  return getParameterIndex(t_param) != -1;
}

ConstLiteralTermPtr Predicate::getParameter(int t_idx) const
{
  if (t_idx < 0 || t_idx > getNumParameters()) {
    return nullptr;
  }
  else {
    return std::make_shared<const LiteralTerm>(params_[static_cast<size_t>(t_idx)]);
  }
}

ConstLiteralTermPtr Predicate::getParameter(const std::string& t_name) const
{
  return getParameter(getParameterIndex(t_name));
}

std::string Predicate::getParameterType(int t_idx) const
{
  if (t_idx < 0 || t_idx > getNumParameters()) {
    return {};
  }
  else {
    return params_[static_cast<size_t>(t_idx)].getType();
  }
}

std::string Predicate::getParameterType(const std::string& t_name) const
{
  return getParameterType(getParameterIndex(t_name));
}

std::shared_ptr<const LiteralTermVector> Predicate::getParameters() const
{
  return std::make_shared<const LiteralTermVector>(params_);
}

Predicate& Predicate::operator=(const Predicate& t_other)
{
  name_ = t_other.getName();
  params_ = const_cast<LiteralTermVector&>(*t_other.getParameters());
  return *this;
}

std::string Predicate::toPddl(bool t_typing, int t_pad_lv) const
{
  auto pad_aligners = helpers::getPddlAligners(t_pad_lv);
  const auto& aligners = pad_aligners.second;

  std::string out = aligners[0] + "(" + name_;
  for (const auto& arg : params_) {
    out += " " + arg.toPddl(t_typing);
  }
  out += ")";
  return out;
}

bool rtask::commons::pddl_generator::operator==(const Predicate& t_first, const Predicate& t_second)
{
  if (t_first.getName() != t_second.getName() || t_first.getNumParameters() != t_second.getNumParameters()) {
    return false;
  }

  auto f_args = t_first.getParameters();
  auto s_args = t_second.getParameters();

  for (unsigned int ix = 0; ix < f_args->size(); ++ix) {

    if (f_args->at(ix).getType() != s_args->at(ix).getType())
      return false;
  }
  return true;
}

std::ostream& rtask::commons::pddl_generator::operator<<(std::ostream& t_out, const Predicate& t_pred)
{
  t_out << "Predicate: " << t_pred.getName() << std::endl;
  unsigned int i = 0;
  for (const auto& param : *t_pred.getParameters()) {
    t_out << " - param[" << i++ << "] : " << param << std::endl;
  }
  return t_out;
}
