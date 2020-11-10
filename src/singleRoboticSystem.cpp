#include "commons/singleRoboticSystem.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
rtask::commons::SingleRoboticSystem::SingleRoboticSystem(const std::string& t_name,
                                                         const std::vector<Device> t_devices,
                                                         const std::vector<Property> t_extra_properties)
{
  set(t_name, t_devices, t_extra_properties);
}

rtask::commons::SingleRoboticSystem::SingleRoboticSystem(const rtask_msgs::SingleRoboticSystem& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::SingleRoboticSystem::SingleRoboticSystem(const rtask_msgs::SingleRoboticSystemConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::SingleRoboticSystem::SingleRoboticSystem(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("devices", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["devices"].size() == 0) {
      std::cout << "Empty devices vector for SingleRoboticSystem " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["devices"].size(); ++i) {
        Device d(t_rpc_val["devices"][i]);
        if (d.isValid()) {
          devices_.push_back(d);
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
  updValidity();
}

rtask_msgs::SingleRoboticSystem rtask::commons::SingleRoboticSystem::toMsg() const
{
  rtask_msgs::SingleRoboticSystem msg;
  msg.name = name_;
  for (const auto& d : devices_) {
    msg.devices.emplace_back(d.toMsg());
  }

  for (const auto& ep : extra_properties_) {
    msg.extra_properties.emplace_back(ep.toMsg());
  }
  return msg;
}

void rtask::commons::SingleRoboticSystem::fromMsg(const rtask_msgs::SingleRoboticSystem& t_msg)
{
  name_ = t_msg.name;
  for (const auto& d : t_msg.devices) {
    devices_.emplace_back(d);
  }
  for (const auto& ep : t_msg.extra_properties) {
    extra_properties_.emplace_back(ep);
  }

  updValidity();
}

void rtask::commons::SingleRoboticSystem::clear()
{
  name_.clear();
  devices_.clear();
  extra_properties_.clear();
}

void rtask::commons::SingleRoboticSystem::set(const std::string& t_name,
                                              const std::vector<Device>& t_devices,
                                              const std::vector<Property> t_extra_properties)
{
  name_ = t_name;
  devices_ = t_devices;
  extra_properties_ = t_extra_properties;
  updValidity();
}

std::vector<std::string> rtask::commons::SingleRoboticSystem::getExtraPropertyList() const
{
  std::vector<std::string> out;
  std::for_each(
    extra_properties_.begin(), extra_properties_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

std::vector<std::string> rtask::commons::SingleRoboticSystem::getDeviceList() const
{
  std::vector<std::string> out;
  std::for_each(devices_.begin(), devices_.end(), [&out](const auto& c) { out.emplace_back(c.getName()); });
  return out;
}

bool rtask::commons::SingleRoboticSystem::hasExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != extra_properties_.end();
}

bool rtask::commons::SingleRoboticSystem::deleteExtraProperty(const std::string& t_name)
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

bool rtask::commons::SingleRoboticSystem::isExtraPropertyValid(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Property>
rtask::commons::SingleRoboticSystem::getExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::SingleRoboticSystem::setExtraProperty(const std::string& t_name, const PropertyVariant& t_val)
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

bool rtask::commons::SingleRoboticSystem::hasDevice(const std::string& t_name) const
{
  auto it = std::find_if(devices_.begin(), devices_.end(), [t_name](auto& d) { return d.getName() == t_name; });
  return it != devices_.end();
}

bool rtask::commons::SingleRoboticSystem::deleteDevice(const std::string& t_name)
{
  auto it = std::find_if(devices_.begin(), devices_.end(), [t_name](auto& d) { return d.getName() == t_name; });
  if (it == devices_.end()) {
    return false;
  }

  devices_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::SingleRoboticSystem::isDeviceValid(const std::string& t_name) const
{
  auto it = std::find_if(devices_.begin(), devices_.end(), [t_name](auto& d) { return d.getName() == t_name; });
  if (it == devices_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Device> rtask::commons::SingleRoboticSystem::getDevice(const std::string& t_name) const
{
  auto it = std::find_if(devices_.begin(), devices_.end(), [t_name](auto& d) { return d.getName() == t_name; });
  if (it == devices_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::SingleRoboticSystem::setDevice(const std::string& t_name,
                                                    const DeviceClass& t_dev_class,
                                                    const std::string& t_dev_subclass,
                                                    const std::vector<Property> t_dev_unique_props,
                                                    const std::vector<Property> t_dev_extra_properties,
                                                    const std::vector<Capability> t_dev_capabilities)
{
  auto it = std::find_if(devices_.begin(), devices_.end(), [t_name](auto& d) { return d.getName() == t_name; });

  if (it == devices_.end()) {
    devices_.emplace_back(
      t_name, t_dev_class, t_dev_subclass, t_dev_unique_props, t_dev_extra_properties, t_dev_capabilities);
    updValidity();
    return;
  }
  it->set(t_name, t_dev_class, t_dev_subclass, t_dev_unique_props, t_dev_extra_properties, t_dev_capabilities);
  updValidity();
}

void rtask::commons::SingleRoboticSystem::setDevice(const std::string& t_name, const Device& t_dev)
{
  auto it = std::find_if(devices_.begin(), devices_.end(), [t_name](auto& d) { return d.getName() == t_name; });

  if (it == devices_.end()) {
    devices_.emplace_back(t_dev);
    updValidity();
    return;
  }
  it->set(t_name,
          t_dev.getClass(),
          t_dev.getSubclass(),
          t_dev.getUniqueProperties(),
          t_dev.getExtraProperties(),
          t_dev.getCapabilities());
  updValidity();
}

bool rtask::commons::SingleRoboticSystem::operator==(const rtask::commons::SingleRoboticSystem& t_srs) const
{
  if (!(name_ == t_srs.getName() && valid_ == t_srs.isValid())) {
    return false;
  }

  auto other_devices = t_srs.getDevices();
  for (const auto& d : devices_) {
    auto it = std::find_if(other_devices.begin(), other_devices.end(), [d](auto& od) { return d == od; });
    if (it == other_devices.end()) {
      return false;
    }
  }
  return true;
}

rtask::commons::SingleRoboticSystem& rtask::commons::SingleRoboticSystem::
operator=(const rtask::commons::SingleRoboticSystem& t_srs)
{
  name_ = t_srs.getName();
  valid_ = t_srs.isValid();
  extra_properties_ = t_srs.getExtraProperties();
  devices_ = t_srs.getDevices();
  return *this;
}

void rtask::commons::SingleRoboticSystem::updValidity()
{
  valid_ = !name_.empty();
  std::for_each(extra_properties_.begin(), extra_properties_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
  std::for_each(devices_.begin(), devices_.end(), [this](const auto& d) { valid_ &= d.isValid(); });
}
