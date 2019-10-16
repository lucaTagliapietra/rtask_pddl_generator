#include "commons/component.h"

#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Component::Component(const unsigned int t_id,
                                     const std::string& t_name,
                                     const std::string& t_type,
                                     const std::string& t_ref_frame,
                                     const std::vector<Capacity> t_capacities,
                                     const std::string& t_description,
                                     const std::string& t_urdf_link,
                                     const std::string& t_moveit_group_name,
                                     const std::string& t_parent_link)
{
  setComponent(
    t_id, t_name, t_type, t_ref_frame, t_capacities, t_description, t_urdf_link, t_moveit_group_name, t_parent_link);
}

rtask::commons::Component::Component(const rtask_msgs::Component& t_msg)
{
  setFromComponentMsg(t_msg);
}

rtask::commons::Component::Component(const rtask_msgs::ComponentConstPtr t_msg_ptr)
{
  setFromComponentMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

// ---------------
// Component level
// ---------------

rtask_msgs::ComponentPtr rtask::commons::Component::toComponentMsg() const
{
  rtask_msgs::ComponentPtr component = boost::make_shared<rtask_msgs::Component>();
  component->id = m_params.id;
  component->name = m_params.name;
  component->type = m_params.type;
  component->description = m_params.description;
  component->urdf_link = m_params.urdf_link;
  component->moveit_group = m_params.moveit_group_name;
  component->reference_frame = m_params.reference_frame;
  component->parent_link = m_params.parent_link;
  for (auto& c : m_params.capacities) {
    component->capacities.push_back(*(c.second.toCapacityMsg()));
  }
  return component;
}

void rtask::commons::Component::setFromComponentMsg(const rtask_msgs::Component& t_msg)
{
  m_params.id = t_msg.id;
  m_params.name = t_msg.name;
  m_params.type = t_msg.type;
  m_params.description = t_msg.description;
  m_params.urdf_link = t_msg.urdf_link;
  m_params.moveit_group_name = t_msg.moveit_group;
  m_params.reference_frame = t_msg.reference_frame;
  m_params.parent_link = t_msg.parent_link;
  clearCapacities();
  for (auto& c : t_msg.capacities) {
    m_params.capacities[c.capability] = {c};
  }
}

void rtask::commons::Component::setFromComponentMsg(const rtask_msgs::ComponentConstPtr t_msg_ptr)
{
  setFromComponentMsg(*t_msg_ptr);
}

void rtask::commons::Component::setComponent(const unsigned int t_id,
                                             const std::string& t_name,
                                             const std::string& t_type,
                                             const std::string& t_ref_frame,
                                             const std::vector<rtask::commons::Capacity> t_capacities,
                                             const std::string& t_description,
                                             const std::string& t_urdf_link,
                                             const std::string& t_moveit_group_name,
                                             const std::string& t_parent_link)
{
  m_params.id = t_id;
  m_params.name = t_name;
  m_params.type = t_type;
  m_params.description = t_description;
  m_params.urdf_link = t_urdf_link;
  m_params.moveit_group_name = t_moveit_group_name;
  m_params.reference_frame = t_ref_frame;
  m_params.parent_link = t_parent_link;
  if (!t_capacities.empty()) {
    setCapacities(t_capacities);
  }
}

void rtask::commons::Component::clear()
{
  m_params = {};
}

void rtask::commons::Component::getCapacities(std::vector<rtask::commons::Capacity>& t_capacities) const
{
  t_capacities.reserve(m_params.capacities.size());
  for (auto it = m_params.capacities.begin(); it != m_params.capacities.end(); ++it) {
    t_capacities.emplace_back(it->second);
  }
}

void rtask::commons::Component::getCapabilities(std::vector<std::string>& t_capacity_names) const
{
  t_capacity_names.reserve(m_params.capacities.size());
  for (auto it = m_params.capacities.begin(); it != m_params.capacities.end(); ++it) {
    t_capacity_names.emplace_back(it->first);
  }
}

// ---------------
// Component level
// ---------------

bool rtask::commons::Component::hasCapacity(const std::string& t_capacity_name)
{
  if (m_params.capacities.count(t_capacity_name) != 1) {
    return false;
  }
  return true;
}

bool rtask::commons::Component::removeCapacity(const std::string& t_capacity_name)
{
  if (!hasCapacity(t_capacity_name)) {
    return false;
  }
  m_params.capacities.erase(t_capacity_name);
  return true;
}

bool rtask::commons::Component::getCapacity(const std::string& t_capacity_name, rtask::commons::Capacity& t_capacity)
{
  if (!hasCapacity(t_capacity_name)) {
    return false;
  }
  t_capacity = {m_params.capacities[t_capacity_name]};
  return true;
}

bool rtask::commons::Component::getCapacityProperties(const std::string& t_capacity_name,
                                                      std::vector<rtask::commons::Property>& t_props)
{
  if (!hasCapacity(t_capacity_name)) {
    return false;
  }
  m_params.capacities[t_capacity_name].getProperties(t_props);
  return true;
}

bool rtask::commons::Component::addCapacity(const rtask::commons::Capacity& t_capacity)
{
  if (!hasCapacity(t_capacity.getCapabilityName())) {
    return false;
  }
  m_params.capacities[t_capacity.getCapabilityName()] = {t_capacity};
  return true;
}

void rtask::commons::Component::setCapacity(const rtask::commons::Capacity& t_capacity)
{
  m_params.capacities[t_capacity.getCapabilityName()] = {t_capacity};
}

// --------------
// Property level
// --------------

bool rtask::commons::Component::hasCapacityProperty(const std::string& t_capacity_name, const std::string& t_prop_name)
{
  if (!hasCapacity(t_capacity_name)) {
    return false;
  }
  if (!m_params.capacities[t_capacity_name].hasProperty(t_prop_name)) {
    return false;
  }
  return true;
}
bool rtask::commons::Component::removeCapacityProperty(const std::string& t_capacity_name,
                                                       const std::string& t_prop_name)
{
  if (!hasCapacity(t_capacity_name)) {
    return false;
  }
  return m_params.capacities[t_capacity_name].removeProperty(t_prop_name);
}
bool rtask::commons::Component::getCapacityProperty(const std::string& t_capacity_name,
                                                    const std::string& t_prop_name,
                                                    rtask::commons::Property& t_prop)
{
  if (!hasCapacity(t_capacity_name)) {
    return false;
  }
  return m_params.capacities[t_capacity_name].getProperty(t_prop_name, t_prop);
}
bool rtask::commons::Component::addCapacityProperty(const std::string& t_capacity_name,
                                                    const rtask::commons::Property& t_prop)
{
  if (hasCapacityProperty(t_capacity_name, t_prop.getName())) {
    return false;
  }

  if (!hasCapacity(t_capacity_name)) {
    m_params.capacities[t_capacity_name] = {t_capacity_name};
  }
  return m_params.capacities[t_capacity_name].addProperty(t_prop);
}

void rtask::commons::Component::setCapacityProperty(const std::string& t_capacity_name,
                                                    const rtask::commons::Property& t_prop)
{
  if (!hasCapacity(t_capacity_name)) {
    m_params.capacities[t_capacity_name] = {t_capacity_name};
  }
  m_params.capacities[t_capacity_name].setProperty(t_prop);
}

// -----------------
// PRIVATE FUNCTIONS
// -----------------

void rtask::commons::Component::clearCapacities()
{
  m_params.capacities.clear();
}

void rtask::commons::Component::setCapacities(const std::vector<Capacity>& t_capacities)
{
  clearCapacities();
  for (auto& cap : t_capacities) {
    m_params.capacities[cap.getCapabilityName()] = {cap};
  }
}
