#include "commons/predicate.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Predicate::Predicate(const std::string& t_name, const std::vector<Parameter>& t_params)
{
  set(t_name, t_params);
}

rtask::commons::Predicate::Predicate(const rtask_msgs::PredicateConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Predicate::Predicate(const rtask_msgs::Predicate& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Predicate::Predicate(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString), true) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (commons::utils::checkXmlRpcSanity("params", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["params"].size() == 0) {
      std::cout << "Empty params vector for predicate " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["params"].size(); ++i) {
        Parameter p(t_rpc_val["params"][i]);
        if (p.isValid()) {
          parameters_.push_back(p);
        }
      }
    }
  }

  valid_ = !name_.empty();
}

void rtask::commons::Predicate::set(const std::string& t_name, const std::vector<Parameter>& t_params)
{
  if (t_name.empty()) {
    valid_ = false;
    return;
  }

  name_ = t_name;
  valid_ = true;

  for (const auto& p : t_params) {
    if (p.isValid()) {
      parameters_.push_back(p);
    }
    else {
      valid_ = false;
    }
  }
}

void rtask::commons::Predicate::fromMsg(const rtask_msgs::Predicate& t_msg)
{
  if (t_msg.name.empty()) {
    valid_ = false;
    return;
  }
  name_ = t_msg.name;
  valid_ = true;
  for (const auto& p_msg : t_msg.params) {
    auto param = Parameter(p_msg);
    if (param.isValid()) {
      parameters_.push_back(param);
    }
    else {
      valid_ = false;
    }
  }
}

void rtask::commons::Predicate::updValidity()
{
  valid_ = !name_.empty();
  std::for_each(parameters_.begin(), parameters_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
}

rtask_msgs::Predicate rtask::commons::Predicate::toMsg() const
{
  rtask_msgs::Predicate msg;
  msg.name = name_;
  for (const auto& p : parameters_) {
    msg.params.push_back(p.toMsg());
  }
  return msg;
}

void rtask::commons::Predicate::clear()
{
  name_.clear();
  parameters_.clear();
  valid_ = false;
}

std::vector<std::string> rtask::commons::Predicate::getParameterList() const
{
  std::vector<std::string> out;
  std::for_each(parameters_.begin(), parameters_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

std::string rtask::commons::Predicate::toPddl(const bool t_typing) const
{
  std::string out{};
  if (valid_) {
    out += "(" + name_;
    for (const auto& p : parameters_) {
      out += " " + p.toPddl(t_typing);
    }
    out += ")";
  }
  return out;
}

bool rtask::commons::Predicate::hasParameter(const std::string& t_name) const
{
  auto it = std::find_if(parameters_.begin(), parameters_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != parameters_.end();
}

bool rtask::commons::Predicate::deleteParameter(const std::string& t_name)
{
  auto it = std::find_if(parameters_.begin(), parameters_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == parameters_.end()) {
    return false;
  }

  parameters_.erase(it);
  updValidity();
  return true;
}

std::pair<bool, rtask::commons::Parameter> rtask::commons::Predicate::getParameter(const std::string& t_name) const
{
  auto it = std::find_if(parameters_.begin(), parameters_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == parameters_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::Predicate::setParameter(const std::string& t_name, const std::string& t_type)
{
  auto it = std::find_if(parameters_.begin(), parameters_.end(), [t_name](auto& p) { return p.getName() == t_name; });

  if (it == parameters_.end()) {
    parameters_.emplace_back(t_name, t_type);
  }
  it->setType(t_type);
  updValidity();
}

bool rtask::commons::Predicate::isParameterValid(const std::string& t_name) const
{
  auto it = std::find_if(parameters_.begin(), parameters_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == parameters_.end()) {
    return false;
  }
  return it->isValid();
}

bool rtask::commons::Predicate::operator==(const rtask::commons::Predicate& t_predicate) const
{
  if (!(name_ == t_predicate.getName() && valid_ == t_predicate.isValid())) {
    return false;
  }

  auto other_params = t_predicate.getParameters();
  for (const auto& p : parameters_) {
    auto it = std::find_if(other_params.begin(), other_params.end(), [p](auto& op) { return p == op; });
    if (it == other_params.end()) {
      return false;
    }
  }
  return true;
}

rtask::commons::Predicate& rtask::commons::Predicate::operator=(const rtask::commons::Predicate& t_predicate)
{
  name_ = t_predicate.getName();
  valid_ = t_predicate.isValid();
  parameters_ = t_predicate.getParameters();
  return *this;
}
