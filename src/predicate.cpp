#include "commons/predicate.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Predicate::Predicate(const std::string& t_name, const std::vector<TypedName>& t_args)
{
  set(t_name, t_args);
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
  std::string name = {};
  std::vector<TypedName> args = {};

  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString), true) {
    name = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (commons::utils::checkXmlRpcSanity("params", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["params"].size() == 0) {
      std::cout << "Empty params vector for predicate " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["params"].size(); ++i) {
        args.emplace_back(t_rpc_val["params"][i]);
      }
    }
  }
  set(name, args);
}

rtask_msgs::Predicate rtask::commons::Predicate::toMsg() const
{
  rtask_msgs::Predicate msg;
  msg.name = name_;
  for (const auto& arg : args_) {
    msg.params.push_back(arg.toMsg());
  }
  return msg;
}

void rtask::commons::Predicate::clear()
{
  name_.clear();
  args_.clear();
}

void rtask::commons::Predicate::set(const std::string& t_name, const std::vector<TypedName>& t_args)
{
  if (t_name.empty()) {
    std::cerr << "Empty Predicate name" << std::endl;
    return;
  }

  name_ = std::move(t_name);
  args_ = std::move(t_args);
}

std::vector<std::string> rtask::commons::Predicate::getArgumentNames() const
{
  std::vector<std::string> out;
  std::for_each(args_.begin(), args_.end(), [&out](const auto& tn) { out.emplace_back(tn.getName()); });
  return out;
}

std::vector<std::string> rtask::commons::Predicate::getArgumentTypeNames() const
{
  std::vector<std::string> out;
  std::for_each(args_.begin(), args_.end(), [&out](const auto& tn) { out.emplace_back(tn.getTypeName()); });
  return out;
}

std::string rtask::commons::Predicate::toPddl(const bool t_typing) const
{
  if (name_.empty()) {
    return {};
  };

  std::string out{};
  out += "(" + name_;
  for (const auto& arg : args_) {
    out += " " + arg.toPddl(t_typing);
  }
  out += ")";
  return out;
}

bool rtask::commons::Predicate::validate(const UnorderedTypedNameMap& t_known_types) const
{
  if (name_.empty()) {
    std::cerr << "VALIDATION ERROR: Empty predicate name" << std::endl;
    return false;
  }

  for (const auto& arg : args_) {
    if (!arg.validate(t_known_types)) {
      std::cerr << "\t(In definition of Predicate **" << name_ << "**" << std::endl;
      return false;
    }
  }

  return true;
}

bool rtask::commons::Predicate::operator==(const rtask::commons::Predicate& t_other) const
{
  if (name_ != t_other.getName()) {
    return false;
  }
  const auto& other_args = t_other.getArguments();
  if (other_args.size() != args_.size()) {
    return false;
  }
  else {
    for (unsigned int i = 0; i < args_.size(); ++i) {
      if (!(args_.at(i) == other_args.at(i))) {
        return false;
      }
    }
  }
  return true;
}

rtask::commons::Predicate& rtask::commons::Predicate::operator=(const rtask::commons::Predicate& t_other)
{
  set(t_other.getName(), t_other.getArguments());
  return *this;
}
