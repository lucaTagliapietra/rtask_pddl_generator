#include "commons/collectiveRoboticSystem.h"
#include "commons/utils.h"

// ------------
// CONSTRUCTORS
rtask::commons::CollectiveRoboticSystem::CollectiveRoboticSystem(const std::string& t_name,
                                                                 const std::vector<SingleRoboticSystem> t_srs,
                                                                 const std::vector<Property> t_extra_properties)
{
  set(t_name, t_srs, t_extra_properties);
}

rtask::commons::CollectiveRoboticSystem::CollectiveRoboticSystem(const rtask_msgs::CollectiveRoboticSystem& t_msg)
{
  fromMsg(t_msg);
}

rtask::commons::CollectiveRoboticSystem::CollectiveRoboticSystem(
  const rtask_msgs::CollectiveRoboticSystemConstPtr t_msg_ptr)
{
  fromMsg(*t_msg_ptr);
}

rtask::commons::CollectiveRoboticSystem::CollectiveRoboticSystem(XmlRpc::XmlRpcValue& t_rpc_val)
{
  if (commons::utils::checkXmlRpcSanity("name", t_rpc_val, XmlRpc::XmlRpcValue::TypeString)) {
    name_ = static_cast<std::string>(t_rpc_val["name"]);
  }

  if (commons::utils::checkXmlRpcSanity("single_robotic_systems", t_rpc_val, XmlRpc::XmlRpcValue::TypeArray)) {
    if (t_rpc_val["single_robotic_systems"].size() == 0) {
      std::cout << "Empty single_robotic_systems vector for CollectiveRoboticSystem " << name_ << std::endl;
    }
    else {
      for (int i = 0; i < t_rpc_val["single_robotic_systems"].size(); ++i) {
        SingleRoboticSystem srs(t_rpc_val["single_robotic_systems"][i]);
        if (srs.isValid()) {
          single_robotic_systems_.push_back(srs);
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

rtask_msgs::CollectiveRoboticSystem rtask::commons::CollectiveRoboticSystem::toMsg() const
{
  rtask_msgs::CollectiveRoboticSystem msg;
  msg.name = name_;
  for (const auto& srs : single_robotic_systems_) {
    msg.single_robotic_systems.emplace_back(srs.toMsg());
  }

  for (const auto& ep : extra_properties_) {
    msg.extra_properties.emplace_back(ep.toMsg());
  }
  return msg;
}

void rtask::commons::CollectiveRoboticSystem::fromMsg(const rtask_msgs::CollectiveRoboticSystem& t_msg)
{
  name_ = t_msg.name;
  for (const auto& srs : t_msg.single_robotic_systems) {
    single_robotic_systems_.emplace_back(srs);
  }
  for (const auto& ep : t_msg.extra_properties) {
    extra_properties_.emplace_back(ep);
  }

  updValidity();
}

void rtask::commons::CollectiveRoboticSystem::clear()
{
  name_.clear();
  single_robotic_systems_.clear();
  extra_properties_.clear();
}

void rtask::commons::CollectiveRoboticSystem::set(const std::string& t_name,
                                                  const std::vector<SingleRoboticSystem>& t_srs,
                                                  const std::vector<Property>& t_extra_properties)
{
  name_ = t_name;
  single_robotic_systems_ = t_srs;
  extra_properties_ = t_extra_properties;
  updValidity();
}

std::vector<std::string> rtask::commons::CollectiveRoboticSystem::getExtraPropertyList() const
{
  std::vector<std::string> out;
  std::for_each(
    extra_properties_.begin(), extra_properties_.end(), [&out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

std::vector<std::string> rtask::commons::CollectiveRoboticSystem::getSingleRoboticSystemList() const
{
  std::vector<std::string> out;
  std::for_each(single_robotic_systems_.begin(), single_robotic_systems_.end(), [&out](const auto& srs) {
    out.emplace_back(srs.getName());
  });
  return out;
}

bool rtask::commons::CollectiveRoboticSystem::hasExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  return it != extra_properties_.end();
}

bool rtask::commons::CollectiveRoboticSystem::deleteExtraProperty(const std::string& t_name)
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

bool rtask::commons::CollectiveRoboticSystem::isExtraPropertyValid(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::Property>
rtask::commons::CollectiveRoboticSystem::getExtraProperty(const std::string& t_name) const
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });
  if (it == extra_properties_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::CollectiveRoboticSystem::setExtraProperty(const std::string& t_name, const PropertyVariant& t_val)
{
  auto it = std::find_if(
    extra_properties_.begin(), extra_properties_.end(), [t_name](auto& p) { return p.getName() == t_name; });

  if (it == extra_properties_.end()) {
    extra_properties_.emplace_back(t_name, t_val);
  }
  it->setValue(t_val);
  updValidity();
}

bool rtask::commons::CollectiveRoboticSystem::hasSingleRoboticSystem(const std::string& t_name) const
{
  auto it = std::find_if(single_robotic_systems_.begin(), single_robotic_systems_.end(), [t_name](auto& srs) {
    return srs.getName() == t_name;
  });
  return it != single_robotic_systems_.end();
}

bool rtask::commons::CollectiveRoboticSystem::deleteSingleRoboticSystem(const std::string& t_name)
{
  auto it = std::find_if(single_robotic_systems_.begin(), single_robotic_systems_.end(), [t_name](auto& srs) {
    return srs.getName() == t_name;
  });
  if (it == single_robotic_systems_.end()) {
    return false;
  }

  single_robotic_systems_.erase(it);
  updValidity();
  return true;
}

bool rtask::commons::CollectiveRoboticSystem::isSingleRoboticSystemValid(const std::string& t_name) const
{
  auto it = std::find_if(single_robotic_systems_.begin(), single_robotic_systems_.end(), [t_name](auto& srs) {
    return srs.getName() == t_name;
  });
  if (it == single_robotic_systems_.end()) {
    return false;
  }
  return it->isValid();
}

std::pair<bool, rtask::commons::SingleRoboticSystem>
rtask::commons::CollectiveRoboticSystem::getSingleRoboticSystem(const std::string& t_name) const
{
  auto it = std::find_if(single_robotic_systems_.begin(), single_robotic_systems_.end(), [t_name](auto& srs) {
    return srs.getName() == t_name;
  });
  if (it == single_robotic_systems_.end()) {
    return {false, {}};
  }
  return {true, *it};
}

void rtask::commons::CollectiveRoboticSystem::setSingleRoboticSystem(const std::string& t_name,
                                                                     const std::vector<Device> t_devices,
                                                                     const std::vector<Property> t_extra_properties)
{
  auto it = std::find_if(single_robotic_systems_.begin(), single_robotic_systems_.end(), [t_name](auto& d) {
    return d.getName() == t_name;
  });

  if (it == single_robotic_systems_.end()) {
    single_robotic_systems_.emplace_back(t_name, t_devices, t_extra_properties);
  }
  it->set(t_name, t_devices, t_extra_properties);
  updValidity();
}

void rtask::commons::CollectiveRoboticSystem::setSingleRoboticSystem(const std::string& t_name,
                                                                     const SingleRoboticSystem& t_srs)
{
  auto it = std::find_if(single_robotic_systems_.begin(), single_robotic_systems_.end(), [t_name](auto& d) {
    return d.getName() == t_name;
  });

  if (it == single_robotic_systems_.end()) {
    single_robotic_systems_.emplace_back(t_srs);
  }
  it->set(t_name, t_srs.getDevices(), t_srs.getExtraProperties());
  updValidity();
}

bool rtask::commons::CollectiveRoboticSystem::operator==(const rtask::commons::CollectiveRoboticSystem& t_crs) const
{
  if (!(name_ == t_crs.getName() && valid_ == t_crs.isValid())) {
    return false;
  }

  auto other_srss = t_crs.getSingleRoboticSystems();
  for (const auto& srs : single_robotic_systems_) {
    auto it = std::find_if(other_srss.begin(), other_srss.end(), [srs](auto& osrs) { return srs == osrs; });
    if (it == other_srss.end()) {
      return false;
    }
  }
  return true;
}

rtask::commons::CollectiveRoboticSystem&
rtask::commons::CollectiveRoboticSystem::operator=(const rtask::commons::CollectiveRoboticSystem& t_crs)
{
  name_ = t_crs.getName();
  valid_ = t_crs.isValid();
  extra_properties_ = t_crs.getExtraProperties();
  single_robotic_systems_ = t_crs.getSingleRoboticSystems();
  return *this;
}

void rtask::commons::CollectiveRoboticSystem::updValidity()
{
  valid_ = !name_.empty();
  std::for_each(extra_properties_.begin(), extra_properties_.end(), [this](const auto& p) { valid_ &= p.isValid(); });
  std::for_each(single_robotic_systems_.begin(), single_robotic_systems_.end(), [this](const auto& srs) {
    valid_ &= srs.isValid();
  });
}
