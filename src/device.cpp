#include "commons/device.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Device::Device(const std::string& t_name,
                               const DeviceClass& t_class,
                               const std::string& t_subclass,
                               const std::vector<Property> t_unique_props,
                               const std::vector<Property> t_extra_properties,
                               const std::vector<Capability> t_capabilities)
{
  set(t_name, t_class, t_subclass, t_unique_props, t_extra_properties, t_capabilities);
}

rtask::commons::Device::Device(const rtask_msgs::Device& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::Device::Device(const rtask_msgs::DeviceConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::Device::Device(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }
  if (commons::utils::checkXmlRpcSanity("class", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    std::string class_name = static_cast<std::string>(t_rpc_val["class"]);
    if (class_name == "Robot") {
      class_ = DeviceClass::ROBOT;
    }
    else if (class_name == "Gripper") {
      class_ = DeviceClass::GRIPPER;
    }
    else if (class_name == "Sensor") {
      class_ = DeviceClass::SENSOR;
    }
    else if (class_name == "Tool") {
      class_ = DeviceClass::TOOL;
    }
    else {
      class_ = DeviceClass::INVALID;
    }
  }
  if (commons::utils::checkXmlRpcSanity("subclass", t_rpc_val, XmlRpc::XmlRpcValue::TypeString, true)) {
    subclass_ = static_cast<std::string>(t_rpc_val["subclass"]);
  }
  if (commons::utils::checkXmlRpcSanity("unique_properties", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["unique_properties"].size() == 0) {
      std::cout << "Empty unique_properties vector for Device " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["unique_properties"].size(); ++i) {
        Property p(t_rpc_val["unique_properties"][i]);
        if (p.isValid()) {
          unique_properties_.push_back(p);
        }
      }
    }
  }
  if (commons::utils::checkXmlRpcSanity("extra_properties", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["extra_properties"].size() == 0) {
      std::cout << "Empty extra_properties vector for Device " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["extra_properties"].size(); ++i) {
        Property p(t_rpc_val["extra_properties"][i]);
        if (p.isValid()) {
          extra_properties_.push_back(p);
        }
      }
    }
  }
  if (commons::utils::checkXmlRpcSanity("capabilities", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["capabilities"].size() == 0) {
      std::cout << "Empty capabilities vector for Device " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["capabilities"].size(); ++i) {
        Capability cap(t_rpc_val["capabilities"][i]);
        if (cap.isValid()) {
          capabilities_.push_back(cap);
        }
      }
    }
  }
  updValidity();
}

rtask_msgs::Device rtask::commons::Device::toMsg() const
{
  rtask_msgs::Device msg;
  msg.name = name_;
  msg.class_value = static_cast<int8_t>(class_);
  msg.subclass_name = subclass_;
  for (const auto& up : unique_properties_) {
    msg.unique_properties.emplace_back(up.toMsg());
  }

  for (const auto& ep : extra_properties_) {
    msg.extra_properties.emplace_back(ep.toMsg());
  }
  for (const auto& cap : capabilities_) {
    msg.capabilities.emplace_back(cap.toMsg());
  }
  return msg;
}

void rtask::commons::Device::fromMsg(const rtask_msgs::Device& t_msg)
{
  name_ = t_msg.name;
  class_ = static_cast<DeviceClass>(t_msg.class_value);
  subclass_ = t_msg.subclass_name;
  for (const auto& up : t_msg.unique_properties) {
    unique_properties_.emplace_back(up);
  }

  for (const auto& ep : t_msg.extra_properties) {
    extra_properties_.emplace_back(ep);
  }
  for (const auto& cap : t_msg.capabilities) {
    capabilities_.emplace_back(cap);
  }
  updValidity();
}

void rtask::commons::Device::clear()
{
  name_.clear();
  class_ = DeviceClass::INVALID;
  subclass_.clear();
  unique_properties_.clear();
  extra_properties_.clear();
  capabilities_.clear();
}

void rtask::commons::Device::set(const std::string& t_name,
                                 const DeviceClass& t_class,
                                 const std::string& t_subclass,
                                 const std::vector<Property> t_unique_props,
                                 const std::vector<Property> t_extra_properties,
                                 const std::vector<Capability> t_capabilities)
{
  name_ = t_name;
  class_ = t_class;
  subclass_ = t_subclass;
  unique_properties_ = t_unique_props;
  extra_properties_ = t_extra_properties;
  capabilities_ = t_capabilities;
  updValidity();
}

std::vector<std::string> rtask::commons::Device::getUniquePropertyList() const
{
  std::vector<std::string> out;
  std::for_each(
    unique_properties_.begin(), unique_properties_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

std::vector<std::string> rtask::commons::Device::getExtraPropertyList() const
{
  std::vector<std::string> out;
  std::for_each(
    extra_properties_.begin(), extra_properties_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

std::vector<std::string> rtask::commons::Device::getCapabilityList() const
{
  std::vector<std::string> out;
  std::for_each(capabilities_.begin(), capabilities_.end(), [&out](const auto& c) { out.emplace_back(c.getName()); });
  return out;
}

bool rtask::commons::Device::hasUniqueProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    unique_properties_.begin(), unique_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != unique_properties_.end();
}

bool rtask::commons::Device::deleteUniqueProperty(const std::string& t_name)
{
  auto it = std::find_if(
    unique_properties_.begin(), unique_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == unique_properties_.end()) {
    return false;
  }

  unique_properties_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::Device::isUniquePropertyValid(const std::string& t_name) const
{
  auto it = std::find_if(
    unique_properties_.begin(), unique_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == unique_properties_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Property> rtask::commons::Device::getUniqueProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    unique_properties_.begin(), unique_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == unique_properties_.end()) {
    return {false, {}};
  }
  return {true, *it};
}
void rtask::commons::Device::setUniqueProperty(const std::string& t_name, const PropertyVariant& t_val)
{
  auto it = std::find_if(
    unique_properties_.begin(), unique_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });

  if (it == unique_properties_.end()) {
    unique_properties_.emplace_back(t_name, t_val);
    updValidity();
    return;
  }
  it->setValue(t_val);
  updValidity();
}

bool rtask::commons::Device::hasExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != extra_properties_.end();
}

bool rtask::commons::Device::deleteExtraProperty(const std::string& t_name)
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return false;
  }

  extra_properties_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::Device::isExtraPropertyValid(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Property> rtask::commons::Device::getExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return {false, {}};
  }
  return {true, *it};
}
void rtask::commons::Device::setExtraProperty(const std::string& t_name, const PropertyVariant& t_val)
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });

  if (it == extra_properties_.end()) {
    extra_properties_.emplace_back(t_name, t_val);
    updValidity();
    return;
  }
  it->setValue(t_val);
  updValidity();
}

bool rtask::commons::Device::hasCapability(const std::string& t_name) const
{
  auto it =
    std::find_if(capabilities_.begin(), capabilities_.end(), [t_name](auto& c) { return c.getName() == t_name; });
  return it != capabilities_.end();
}

bool rtask::commons::Device::deleteCapability(const std::string& t_name)
{
  auto it =
    std::find_if(capabilities_.begin(), capabilities_.end(), [t_name](auto& c) { return c.getName() == t_name; });
  if (it == capabilities_.end()) {
    return false;
  }

  capabilities_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::Device::isCapabilityValid(const std::string& t_name) const
{
  auto it =
    std::find_if(capabilities_.begin(), capabilities_.end(), [t_name](auto& c) { return c.getName() == t_name; });
  if (it == capabilities_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Capability> rtask::commons::Device::getCapability(const std::string& t_name) const
{
  auto it =
    std::find_if(capabilities_.begin(), capabilities_.end(), [t_name](auto& c) { return c.getName() == t_name; });
  if (it == capabilities_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::Device::setCapability(const std::string& t_name, const std::vector<Property>& t_val)
{
  auto it =
    std::find_if(capabilities_.begin(), capabilities_.end(), [t_name](auto& c) { return c.getName() == t_name; });

  if (it == capabilities_.end()) {
    capabilities_.emplace_back(t_name, t_val);
    updValidity();
    return;
  }
  it->set(t_name, t_val);
  updValidity();
}

bool rtask::commons::Device::operator==(const rtask::commons::Device& t_dev) const
{
  if (!(name_ == t_dev.getName() && valid_ == t_dev.isValid() && class_ == t_dev.getClass()
        && subclass_ == t_dev.getSubclass())) {
    return false;
  }

  auto other_unique_props = t_dev.getUniqueProperties();
  for (const auto& up : unique_properties_) {
    auto it = std::find_if(other_unique_props.begin(), other_unique_props.end(), [up](auto& oup) { return up == oup; });
    if (it == other_unique_props.end()) {
      return false;
    }
  }

  auto other_capabilities = t_dev.getCapabilities();
  for (const auto& c : capabilities_) {
    auto it = std::find_if(other_capabilities.begin(), other_capabilities.end(), [c](auto& oc) { return c == oc; });
    if (it == other_capabilities.end()) {
      return false;
    }
  }
  return true;
}

rtask::commons::Device& rtask::commons::Device::operator=(const rtask::commons::Device& t_dev)
{
  name_ = t_dev.getName();
  class_ = t_dev.getClass();
  subclass_ = t_dev.getSubclass();
  valid_ = t_dev.isValid();
  unique_properties_ = t_dev.getUniqueProperties();
  extra_properties_ = t_dev.getExtraProperties();
  capabilities_ = t_dev.getCapabilities();
  return *this;
}

void rtask::commons::Device::updValidity()
{
  valid_ = !name_.empty() && !(class_ == DeviceClass::INVALID);
  std::for_each(unique_properties_.begin(), unique_properties_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
  std::for_each(extra_properties_.begin(), extra_properties_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
  std::for_each(capabilities_.begin(), capabilities_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
}
