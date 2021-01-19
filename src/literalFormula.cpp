#include "commons/literalFormula.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::LiteralFormula::LiteralFormula(const std::string& t_name, const std::vector<std::string>& t_args)
{
  set(t_name, t_args);
}

rtask::commons::LiteralFormula::LiteralFormula(XmlRpc::XmlRpcValue& t_rpc_val)
{
  std::string name{};
  StringVector args{};

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

  set(name, args);
}

void rtask::commons::LiteralFormula::set(const std::string& t_name, const StringVector& t_args)
{
  if (t_name.empty()) {
    std::cerr << "Empty LiteralFormula name" << std::endl;
    return;
  }

  name_ = std::move(t_name);
  args_ = std::move(t_args);
}

void rtask::commons::LiteralFormula::clear()
{
  name_.clear();
  args_.clear();
}

std::string rtask::commons::LiteralFormula::toPddl(const bool t_typing) const
{
  if (name_.empty()) {
    return {};
  };

  std::string out{};
  out += "(" + name_; // Open args list
  for (const auto& a : args_) {
    out += " ?" + a;
  }
  out += ")"; // Close args list
  return out;
}

bool rtask::commons::LiteralFormula::validate(const UnorderedNameTypedTermMap& t_known_constants,
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

bool rtask::commons::LiteralFormula::operator==(const rtask::commons::LiteralFormula& t_other) const
{
  if (name_ != t_other.getName()) {
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

rtask::commons::LiteralFormula& rtask::commons::LiteralFormula::operator=(const rtask::commons::LiteralFormula& t_other)
{
  set(t_other.getName(), t_other.getArguments());
  return *this;
}
