#include "commons/capability.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Capability::Capability(const std::string& t_name, const std::vector<Property>& t_props)
{
  set(t_name, t_props);
}

rtask::commons::Capability::Capability(const rtask_msgs::Capability& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Capability::Capability(const rtask_msgs::CapabilityConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Capability::Capability(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
    if (commons::utils::checkXmlRpcSanity("properties", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
      if (t_rpc_val["properties"].size() == 0) {
        std::cout << "Empty properties vector for capability " << name_ << std::endl;
      }
      else {
        for (int i = 0; i < t_rpc_val["properties"].size(); ++i) {
          Property p(t_rpc_val["properties"][i]);
          if (p.isValid()) {
            properties_.push_back(p);
          }
        }
      }
    }
  }
  updValidity();
}

void rtask::commons::Capability::set(const std::string& t_name, const std::vector<Property>& t_props)
{
  if (t_name.empty()) {
    valid_ = false;
    return;
  }

  name_ = t_name;
  valid_ = true;

  for (const auto& p : t_props) {
    if (p.isValid()) {
      properties_.push_back(p);
    }
    else {
      valid_ = false;
    }
  }
}

void rtask::commons::Capability::fromMsg(const rtask_msgs::Capability& t_msg)
{
  if (t_msg.name.empty()) {
    valid_ = false;
    return;
  }
  name_ = t_msg.name;
  valid_ = true;
  for (auto& p_msg : t_msg.properties) {
    auto prop = Property(p_msg);
    if (prop.isValid()) {
      properties_.push_back(prop);
    }
    else {
      valid_ = false;
    }
  }
}

rtask_msgs::Capability rtask::commons::Capability::toMsg() const
{
  rtask_msgs::Capability msg;
  msg.name = name_;
  for (const auto& p : properties_) {
    msg.properties.push_back(p.toMsg());
  }
  return msg;
}

void rtask::commons::Capability::updValidity()
{
  valid_ = !name_.empty();
  std::for_each(properties_.begin(), properties_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
}

void rtask::commons::Capability::clear()
{
  valid_ = false;
  properties_.clear();
  name_.clear();
}

std::vector<std::string> rtask::commons::Capability::getPropertyList() const
{
  std::vector<std::string> out;
  std::for_each(properties_.begin(), properties_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

bool rtask::commons::Capability::hasProperty(const std::string& t_name) const
{
  auto it = std::find_if(properties_.begin(), properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != properties_.end();
}

bool rtask::commons::Capability::deleteProperty(const std::string& t_name)
{
  auto it = std::find_if(properties_.begin(), properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == properties_.end()) {
    return false;
  }

  properties_.erase(it);
  updValidity();
  return true;
}

std::pair<bool, rtask::commons::Property> rtask::commons::Capability::getProperty(const std::string& t_name) const
{
  auto it = std::find_if(properties_.begin(), properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == properties_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::Capability::setProperty(const std::string& t_name, const rtask::commons::PropertyVariant& t_val)
{
  auto it = std::find_if(properties_.begin(), properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });

  if (it == properties_.end()) {
    properties_.emplace_back(t_name, t_val);
  }
  it->setValue(t_val);
  updValidity();
}

bool rtask::commons::Capability::isPropertyValid(const std::string& t_name) const
{
  auto it = std::find_if(properties_.begin(), properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == properties_.end()) {
    return false;
  }
  return it->isValid();
}

bool rtask::commons::Capability::operator==(const rtask::commons::Capability& t_capability) const
{
  if (!(name_ == t_capability.getName() && valid_ == t_capability.isValid())) {
    return false;
  }

  auto other_props = t_capability.getProperties();
  for (const auto& p : properties_) {
    auto it = std::find_if(other_props.begin(), other_props.end(), [p](auto& op) { return p == op; });
    if (it == other_props.end()) {
      return false;
    }
  }

  return true;
}

rtask::commons::Capability& rtask::commons::Capability::operator=(const rtask::commons::Capability& t_capability)
{
  name_ = t_capability.getName();
  valid_ = t_capability.isValid();
  properties_ = t_capability.getProperties();
  return *this;
}
