#include "commons/capacity.h"
#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Capacity::Capacity(const std::string t_capability, const std::vector<Property>& t_properties)
{
  setCapacity(t_capability, t_properties);
}

rtask::commons::Capacity::Capacity(const rtask_msgs::Capacity& t_msg)
{
  setFromCapacityMsg(t_msg);
}

rtask::commons::Capacity::Capacity(const rtask_msgs::CapacityConstPtr t_msg_ptr)
{
  setFromCapacityMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

// --------------
// Capacity Level
// --------------
rtask_msgs::CapacityPtr rtask::commons::Capacity::toCapacityMsg() const
{
  rtask_msgs::CapacityPtr capacity = boost::make_shared<rtask_msgs::Capacity>();
  capacity->capability = m_capability;
  for (auto& p : m_properties) {
    capacity->properties.push_back(*(p.second.toPropertyMsg()));
  }
  return capacity;
}

void rtask::commons::Capacity::setFromCapacityMsg(const rtask_msgs::Capacity& t_msg)
{
  m_capability = t_msg.capability;
  clearProperties();
  for (auto& p : t_msg.properties) {
    m_properties[p.name] = {p};
  }
}

void rtask::commons::Capacity::setFromCapacityMsg(const rtask_msgs::CapacityConstPtr t_msg_ptr)
{
  setFromCapacityMsg(*t_msg_ptr);
}

void rtask::commons::Capacity::setCapacity(const std::string& t_capability, const std::vector<Property>& t_properties)
{
  m_capability = t_capability;
  setProperties(t_properties);
}

std::string rtask::commons::Capacity::getCapabilityName() const
{
  return m_capability;
}

void rtask::commons::Capacity::clear()
{
  m_capability = "";
  m_properties.clear();
}

void rtask::commons::Capacity::getProperties(std::vector<rtask::commons::Property>& t_properties) const
{
  t_properties.reserve(m_properties.size());
  for (auto it = m_properties.begin(); it != m_properties.end(); ++it) {
    t_properties.emplace_back(it->second);
  }
}

// --------------
// Property Level
// --------------
bool rtask::commons::Capacity::hasProperty(const std::string& t_property_name)
{
  if (m_properties.count(t_property_name) != 1) {
    return false;
  }
  return true;
}

bool rtask::commons::Capacity::getProperty(const std::string& t_property_name, Property& t_property)
{
  if (!hasProperty(t_property_name)) {
    return false;
  }

  t_property = {m_properties[t_property_name]};
  return true;
}

bool rtask::commons::Capacity::addProperty(const Property& t_property)
{
  const std::string property_name = t_property.getName();
  if (hasProperty(t_property.getName())) {
    return false;
  }
  m_properties[property_name] = {t_property};
  return true;
}

void rtask::commons::Capacity::setProperty(const Property& t_property)
{
  m_properties[t_property.getName()] = {t_property};
}

bool rtask::commons::Capacity::removeProperty(const std::string& t_property_name)
{
  if (!hasProperty(t_property_name)) {
    return false;
  }
  m_properties.erase(t_property_name);
  return true;
}

// -----------------
// PRIVATE FUNCTIONS
// -----------------
void rtask::commons::Capacity::setCapabilityName(const std::string& t_capability)
{
  m_capability = t_capability;
}

void rtask::commons::Capacity::setProperties(const std::vector<Property>& t_properties)
{
  m_properties.clear();
  for (auto& p : t_properties) {
    m_properties[p.getName()] = p;
  }
}

void rtask::commons::Capacity::clearProperties()
{
  m_properties.clear();
}
