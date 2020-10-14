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

void rtask::commons::Capability::set(const std::string t_name, const std::vector<Property>& t_props)
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

void rtask::commons::Capability::updValidity()
{
  valid_ = !name_.empty();
  valid_ = true;
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
  std::for_each(properties_.begin(), properties_.end(), [out](const auto& p) { out.emplace_back(p.getName()); });
  return out;
}

// rtask::commons::Capability::Capability(XmlRpc::XmlRpcValue& t_value)
//{
//  setCapacityFromXmlRpc(t_value);
//}
//// ----------------
// PUBLIC FUNCTIONS
// ----------------

// --------------
// Capacity Level
// --------------
// rtask_msgs::CapacityPtr rtask::commons::Capacity::toCapacityMsg() const
//{
//  rtask_msgs::CapacityPtr capacity = boost::make_shared<rtask_msgs::Capacity>();
//  capacity->capability = m_capability;
//  for (auto& p : m_properties) {
//    capacity->properties.push_back(*(p.second.toPropertyMsg()));
//  }
//  return capacity;
//}

// bool rtask::commons::Capacity::setCapacityFromXmlRpc(XmlRpc::XmlRpcValue& t_value)
//{
//  if (commons::utils::checkXmlRpcSanity("capability", t_value, XmlRpc::XmlRpcValue::TypeString)) {
//    m_capability = static_cast<std::string>(t_value["capability"]);
//  }
//  if (commons::utils::checkXmlRpcSanity("properties", t_value, XmlRpc::XmlRpcValue::TypeArray)) {
//    if (t_value["properties"].size() == 0) {
//      std::cout << "Empty property vector for capability " << m_capability << std::endl;
//      return updValidity();
//    }
//    for (int i = 0; i < t_value["properties"].size(); ++i) {
//      Property tmp(t_value["properties"][i]);
//      if (!addProperty(tmp)) {
//        std::cout << "Malformed or already present property " << tmp.getName() << std::endl;
//        return updValidity();
//      }
//    }
//  }
//  return updValidity();
//}

// void rtask::commons::Capacity::setFromCapacityMsg(const rtask_msgs::Capacity& t_msg)
//{

//  if (!t_msg.capability.empty()) {
//    clear();
//    m_capability = t_msg.capability;
//    for (auto& p : t_msg.properties) {
//      m_properties[p.name] = {p};
//    }
//  }
//  updValidity();
//}

// void rtask::commons::Capacity::setFromCapacityMsg(const rtask_msgs::CapacityConstPtr t_msg_ptr)
//{
//  setFromCapacityMsg(*t_msg_ptr);
//}

// void rtask::commons::Capacity::setCapacity(const std::string& t_capability, const std::vector<Property>&
// t_properties)
//{
//  clear();
//  if (!t_capability.empty()) {
//    m_capability = t_capability;
//  }
//  if (!t_properties.empty()) {
//    setProperties(t_properties);
//  }
//  updValidity();
//}

// std::string rtask::commons::Capacity::getCapabilityName() const
//{
//  return m_capability;
//}

// void rtask::commons::Capacity::clear()
//{
//  m_capability = "";
//  m_valid = false;
//  m_properties.clear();
//}

// void rtask::commons::Capacity::getProperties(std::vector<rtask::commons::Property>& t_properties) const
//{
//  t_properties.reserve(m_properties.size());
//  for (auto it = m_properties.begin(); it != m_properties.end(); ++it) {
//    t_properties.emplace_back(it->second);
//  }
//}

//// --------------
//// Property Level
//// --------------
// bool rtask::commons::Capacity::hasProperty(const std::string& t_property_name) const
//{
//  if (m_properties.count(t_property_name) != 1) {
//    return false;
//  }
//  return true;
//}

// bool rtask::commons::Capacity::getProperty(const std::string& t_property_name, Property& t_property) const
//{
//  if (!hasProperty(t_property_name)) {
//    return false;
//  }

//  t_property = {m_properties.at(t_property_name)};
//  return true;
//}

// bool rtask::commons::Capacity::addProperty(const Property& t_property)
//{
//  if (hasProperty(t_property.getName()) || !t_property.isValid()) {
//    return false;
//  }
//  m_properties[t_property.getName()] = {t_property};
//  updValidity();
//  return true;
//}

// void rtask::commons::Capacity::setProperty(const Property& t_property)
//{
//  if (t_property.isValid()) {
//    m_properties[t_property.getName()] = {t_property};
//  }
//  updValidity();
//}

// bool rtask::commons::Capacity::removeProperty(const std::string& t_property_name)
//{
//  if (!hasProperty(t_property_name)) {
//    return false;
//  }
//  m_properties.erase(t_property_name);
//  updValidity();
//  return true;
//}

//// ---------
//// Operators
//// ---------

// bool rtask::commons::Capacity::operator==(const Capacity& t_capacity) const
//{

//  auto pred = [](const std::pair<std::string, Property>& a, const std::pair<std::string, Property>& b) {
//    return a.second.operator==(b.second);
//  };

//  return ((m_capability == t_capacity.getCapabilityName()) && (m_properties.size() ==
//  t_capacity.m_properties.size())
//          && std::equal(m_properties.begin(), m_properties.end(), t_capacity.m_properties.begin(), pred));
//}

//// -----------------
//// PRIVATE FUNCTIONS
//// -----------------
// void rtask::commons::Capacity::setCapabilityName(const std::string& t_capability)
//{
//  if (!t_capability.empty() && !m_properties.empty()) {
//    m_capability = t_capability;
//    m_valid = true;
//  }
//}

// void rtask::commons::Capacity::setProperties(const std::vector<Property>& t_properties)
//{
//  m_properties.clear();
//  for (auto& p : t_properties) {
//    m_properties[p.getName()] = p;
//  }
//  updValidity();
//}

// void rtask::commons::Capacity::clearProperties()
//{
//  m_properties.clear();
//}

// bool rtask::commons::Capacity::updValidity()
//{
//  m_valid = false;
//  if (!m_capability.empty() && !m_properties.empty()) {
//    m_valid = true;
//    for (const auto& p : m_properties) {
//      m_valid &= p.second.isValid();
//    }
//  }
//  return m_valid;
//}
