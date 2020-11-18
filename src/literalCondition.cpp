#include "commons/literalCondition.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::LiteralCondition::LiteralCondition(const std::string& t_name,
                                                   const std::vector<std::string>& t_args,
                                                   const bool t_negated)
{
  set(t_name, t_args, t_negated);
}

rtask::commons::LiteralCondition::LiteralCondition(const rtask_msgs::ConditionConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::LiteralCondition::LiteralCondition(const rtask_msgs::Condition& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::LiteralCondition::LiteralCondition(XmlRpc::XmlRpcValue& t_rpc_val)
{
  std::string name{};
  StringVector args{};
  bool negated{false};

  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    name = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("args", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["args"].size() == 0) {
      std::cout << "Empty args vector for condition " << name << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["args"].size(); ++i) {
        if (t_rpc_val["args"][i].getType() == XmlRpc::XmlRpcValue::TypeString) {
          args.emplace_back(static_cast<std::string>(t_rpc_val["args"][i]));
        }
      }
    }
  }

  if (commons::utils::checkXmlRpcSanity("not", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    negated = static_cast<bool>(t_rpc_val["not"]);
  }

  set(name, args, negated);
}

void rtask::commons::LiteralCondition::set(const std::string& t_name, const StringVector& t_args, const bool t_negated)
{
  if (t_name.empty()) {
    std::cerr << "Empty LiteralCondition name" << std::endl;
    return;
  }

  name_ = std::move(t_name);
  args_ = std::move(t_args);
  negated_ = t_negated;
}

void rtask::commons::LiteralCondition::fromMsg(const rtask_msgs::Condition& t_msg)
{
  set(t_msg.name, t_msg.args, t_msg.negate);
}

rtask_msgs::Condition rtask::commons::LiteralCondition::toMsg() const
{
  rtask_msgs::Condition msg;
  msg.name = name_;
  msg.negate = negated_;

  for (const auto& a : args_) {
    msg.args.push_back(a);
  }
  return msg;
}

void rtask::commons::LiteralCondition::clear()
{
  name_.clear();
  args_.clear();
  negated_ = false;
}

std::string rtask::commons::LiteralCondition::toPddl(const bool t_typing) const
{
  if (name_.empty()) {
    return {};
  };

  std::string out{};
  if (negated_) {
    out += "(not "; // Open negate
  }
  out += "(" + name_; // Open args list
  for (const auto& a : args_) {
    out += " ?" + a;
  }
  out += ")"; // Close args list
  if (negated_) {
    out += ")"; // Close negate
  }
  return out;
}

bool rtask::commons::LiteralCondition::validate(const UnorderedTypedNameMap& t_known_constants,
                                                const UnorderedStringToUIntMap& t_belonging_action_args,
                                                const std::string& t_belonging_action_name) const
{
  if (name_.empty()) {
    std::cerr << "VALIDATION ERROR: Empty predicate name" << std::endl;
    return false;
  }

  for (const auto& arg : args_) {
    if (!t_belonging_action_args.count(arg) && !t_known_constants.count(arg)) {
      std::cerr << "VALIDATION ERROR: Unknown Arg **" << arg << "**" << std::endl;
      std::cerr << "\t(In LiteralCondition **" << name_ << "** of Action **" << t_belonging_action_name << "**)"
                << std::endl;
      return false;
    }
  }
  return true;
}

bool rtask::commons::LiteralCondition::operator==(const rtask::commons::LiteralCondition& t_other) const
{
  if (!(name_ == t_other.getName() && negated_ == t_other.getNegated())) {
    return false;
  }

  const auto& other_args = t_other.getArguments();
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

rtask::commons::LiteralCondition&
rtask::commons::LiteralCondition::operator=(const rtask::commons::LiteralCondition& t_other)
{
  set(t_other.getName(), t_other.getArguments(), t_other.getNegated());
  return *this;
}
