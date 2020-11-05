#include "commons/condition.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Condition::Condition(const std::string& t_name,
                                     const std::vector<std::string>& t_args,
                                     const bool t_negated)
{
  set(t_name, t_args, t_negated);
}

rtask::commons::Condition::Condition(const rtask_msgs::ConditionConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Condition::Condition(const rtask_msgs::Condition& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Condition::Condition(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("args", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["args"].size() == 0) {
      std::cout << "Empty args vector for condition " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["args"].size(); ++i) {
        if (t_rpc_val["args"][i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          args_.emplace_back(static_cast<std::string>(t_rpc_val["args"][i]));
        }
      }
    }
  }
  if (commons::utils::checkXmlRpcSanity("not", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    negated_ = static_cast<bool>(t_rpc_val["not"]);
  }

  valid_ = !name_.empty();
}

void rtask::commons::Condition::set(const std::string& t_name,
                                    const std::vector<std::string>& t_args,
                                    const bool t_negated)
{
  if (t_name.empty()) {
    valid_ = false;
    return;
  }
  name_ = t_name;
  negated_ = t_negated;
  valid_ = true;

  for (const auto& a : t_args) {
    args_.push_back(a);
  }
}

void rtask::commons::Condition::fromMsg(const rtask_msgs::Condition& t_msg)
{
  if (t_msg.name.empty()) {
    valid_ = false;
    return;
  }

  name_ = t_msg.name;
  negated_ = t_msg.negate;

  for (const auto& a : t_msg.args) {
    if (!a.empty()) {
      args_.push_back(a);
    }
  }
  valid_ = true;
}

rtask_msgs::Condition rtask::commons::Condition::toMsg() const
{
  rtask_msgs::Condition msg;
  msg.name = name_;
  msg.negate = negated_;

  for (const auto& a : args_) {
    msg.args.push_back(a);
  }
  return msg;
}

void rtask::commons::Condition::clear()
{
  name_.clear();
  args_.clear();
  negated_ = false;
  valid_ = false;
}

std::string rtask::commons::Condition::toPddl(const bool t_typing) const
{
  std::string out{};
  if (valid_) {
    if (negated_) {
      out += "(not ";
    }
    out += "(" + name_;
    for (const auto& a : args_) {
      out += " ?" + a;
    }
    out += ")";
    if (negated_) {
      out += ")";
    }
  }
  return out;
};

bool rtask::commons::Condition::operator==(const rtask::commons::Condition& t_condition) const
{
  if (!(name_ == t_condition.getName() && valid_ == t_condition.isValid() && negated_ == t_condition.getNegated())) {
    return false;
  }

  auto other_args = t_condition.getArgs();
  if (args_.size() != other_args.size()) {
    return false;
  }
  for (unsigned int i = 0; i < args_.size(); ++i) {
    if (args_.at(i) != other_args.at(i)) {
      return false;
    }
  }
  return true;
}

rtask::commons::Condition& rtask::commons::Condition::operator=(const rtask::commons::Condition& t_condition)
{
  name_ = t_condition.getName();
  valid_ = t_condition.isValid();
  negated_ = t_condition.getNegated();
  args_ = t_condition.getArgs();
  return *this;
}
