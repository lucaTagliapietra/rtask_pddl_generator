#include "commons/agent.h"

#include "commons/capacity.h"
#include "commons/component.h"
#include "commons/property.h"

#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------
rtask::commons::Agent::Agent(const unsigned int t_id,
                             const std::string& t_name,
                             const std::vector<Component> t_comps,
                             const std::string t_description,
                             const std::string t_urdf_link,
                             const std::string t_base_link)
{
  setAgent(t_id, t_name, t_comps, t_description, t_urdf_link, t_base_link);
}

rtask::commons::Agent::Agent(const rtask_msgs::Agent& t_msg)
{
  setFromAgentMsg(t_msg);
}

rtask::commons::Agent::Agent(const rtask_msgs::AgentConstPtr& t_msg_ptr)
{
  setFromAgentMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

// -----------
// Agent Level
// -----------

rtask_msgs::AgentPtr rtask::commons::Agent::toAgentMsg() const
{
  rtask_msgs::AgentPtr t_agent = boost::make_shared<rtask_msgs::Agent>();

  t_agent->header.frame_id = m_params.base_link;

  t_agent->id = m_params.id;
  t_agent->name = m_params.name;
  t_agent->description = m_params.description;
  t_agent->urdf_link = m_params.urdf_link;
  t_agent->components.reserve(m_params.components.size());
  for (const auto& c : m_params.components) {
    t_agent->components.push_back(*(c.second.toComponentMsg()));
  }
  return t_agent;
}

void rtask::commons::Agent::setFromAgentMsg(const rtask_msgs::Agent& t_msg)
{
  m_params.base_link = t_msg.header.frame_id;
  m_params.id = t_msg.id;
  m_params.name = t_msg.name;
  m_params.urdf_link = t_msg.urdf_link;
  m_params.description = t_msg.description;
  clearComponents();
  for (const auto& c : t_msg.components) {
    m_params.components[c.name] = {c};
    updCapabilityMap(m_params.components[c.name]);
  }
}

void rtask::commons::Agent::setFromAgentMsg(const rtask_msgs::AgentConstPtr t_msg_ptr)
{
  setFromAgentMsg(*t_msg_ptr);
}

void rtask::commons::Agent::setAgent(const unsigned int t_id,
                                     const std::string& t_name,
                                     const std::vector<Component> t_comps,
                                     const std::string t_description,
                                     const std::string t_urdf_link,
                                     const std::string t_base_link)
{
  m_params.id = t_id;
  m_params.name = t_name;
  m_params.description = t_description;
  m_params.urdf_link = t_urdf_link;
  m_params.base_link = t_base_link;

  clearComponents();
  if (!t_comps.empty()) {
    for (const auto& c : t_comps) {
      m_params.components[c.getName()] = {c};
      updCapabilityMap(m_params.components[c.getName()]);
    }
  }
}

void rtask::commons::Agent::clear()
{
  m_params = {};
  m_capability_to_component.clear();
}

void rtask::commons::Agent::getComponents(std::vector<Component>& t_components) const
{
  t_components.reserve(m_params.components.size());
  for (auto it = m_params.components.begin(); it != m_params.components.end(); ++it) {
    t_components.emplace_back(it->second);
  }
}

void rtask::commons::Agent::getComponentNames(std::vector<std::string>& t_component_names) const
{
  t_component_names.reserve(m_params.components.size());
  for (auto it = m_params.components.begin(); it != m_params.components.end(); ++it) {
    t_component_names.emplace_back(it->first);
  }
}

void rtask::commons::Agent::getCapabilities(std::vector<std::string>& t_capabilities) const
{
  t_capabilities.reserve(m_capability_to_component.size());
  for (auto it = m_capability_to_component.begin(); it != m_capability_to_component.end(); ++it) {
    t_capabilities.emplace_back(it->first);
  }
}

bool rtask::commons::Agent::hasCapability(const std::string& t_capability) const
{
  if (m_capability_to_component.count(t_capability) != 1) {
    return false;
  }
  return true;
}

// ---------------
// Component Level
// ---------------
bool rtask::commons::Agent::hasComponent(const std::string& t_component_name) const
{
  if (m_params.components.count(t_component_name) != 1) {
    return false;
  }
  return true;
}

bool rtask::commons::Agent::removeComponent(const std::string& t_component_name)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  m_params.components.erase(t_component_name);
  updCapabilityMap();
  return true;
}

bool rtask::commons::Agent::getComponent(const std::string& t_component_name, commons::Component& t_component) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_component = {m_params.components.at(t_component_name)};
  return true;
}

bool rtask::commons::Agent::addComponent(const commons::Component& t_component)
{
  if (hasComponent(t_component.getName())) {
    return false;
  }
  setComponent(t_component);
  return true;
}

void rtask::commons::Agent::setComponent(const commons::Component& t_component)
{
  m_params.components[t_component.getName()] = {t_component};
  updCapabilityMap(t_component);
}

bool rtask::commons::Agent::getComponentId(const std::string& t_component_name, unsigned int& t_id) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_id = m_params.components.at(t_component_name).getId();
  return true;
}
bool rtask::commons::Agent::getComponentType(const std::string& t_component_name, std::string& t_type) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_type = m_params.components.at(t_component_name).getType();
  return true;
}
bool rtask::commons::Agent::getComponentDescription(const std::string& t_component_name,
                                                    std::string& t_description) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_description = m_params.components.at(t_component_name).getDescription();
  return true;
}
bool rtask::commons::Agent::getComponentUrdfLink(const std::string& t_component_name, std::string& t_urdf_link) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_urdf_link = m_params.components.at(t_component_name).getUrdfLink();
  return true;
}
bool rtask::commons::Agent::getComponentMoveitGroupName(const std::string& t_component_name,
                                                        std::string& t_moveit_group) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_moveit_group = m_params.components.at(t_component_name).getMoveitGroupName();
  return true;
}
bool rtask::commons::Agent::getComponentReferenceFrame(const std::string& t_component_name,
                                                       std::string& t_ref_frame) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_ref_frame = m_params.components.at(t_component_name).getReferenceFrame();
  return true;
}
bool rtask::commons::Agent::getComponentParentLink(const std::string& t_component_name,
                                                   std::string& t_parent_link) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  t_parent_link = m_params.components.at(t_component_name).getParentLink();
  return true;
}
bool rtask::commons::Agent::getComponentCapacities(const std::string& t_component_name,
                                                   std::vector<Capacity>& t_capacities) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  m_params.components.at(t_component_name).getCapacities(t_capacities);
  return true;
}
bool rtask::commons::Agent::getComponentCapabilities(const std::string& t_component_name,
                                                     std::vector<std::string>& t_capacity_names) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  m_params.components.at(t_component_name).getCapabilities(t_capacity_names);
  return true;
}

// --------------
// Capacity Level
// --------------
bool rtask::commons::Agent::hasComponentCapacity(const std::string& t_component_name,
                                                 const std::string& t_capacity_name) const
{
  if (!hasComponent(t_component_name) || !m_params.components.at(t_component_name).hasCapacity(t_capacity_name)) {
    return false;
  }
  return true;
}

bool rtask::commons::Agent::removeComponentCapacity(const std::string& t_component_name,
                                                    const std::string& t_capacity_name)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  if (!m_params.components.at(t_component_name).removeCapacity(t_capacity_name)) {
    return false;
  }
  updCapabilityMap();
  return true;
}
bool rtask::commons::Agent::getComponentCapacity(const std::string& t_component_name,
                                                 const std::string& t_capacity_name,
                                                 commons::Capacity& t_capacity) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  return m_params.components.at(t_component_name).getCapacity(t_capacity_name, t_capacity);
}
bool rtask::commons::Agent::getComponentCapacityProperties(const std::string& t_component_name,
                                                           const std::string& t_capacity_name,
                                                           std::vector<commons::Property>& t_props) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  return m_params.components.at(t_component_name).getCapacityProperties(t_capacity_name, t_props);
}

bool rtask::commons::Agent::addComponentCapacity(const std::string& t_component_name, const Capacity& t_capacity)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  return m_params.components.at(t_component_name).addCapacity(t_capacity);
}

bool rtask::commons::Agent::setComponentCapacity(const std::string& t_component_name, const Capacity& t_capacity)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  m_params.components.at(t_component_name).setCapacity(t_capacity);
  return true;
}

// --------------
// Property level
// --------------
bool rtask::commons::Agent::hasComponentCapacityProperty(const std::string& t_component_name,
                                                         const std::string& t_capacity_name,
                                                         const std::string& t_prop_name) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  return m_params.components.at(t_component_name).hasCapacityProperty(t_capacity_name, t_prop_name);
}
bool rtask::commons::Agent::removeComponentCapacityProperty(const std::string& t_component_name,
                                                            const std::string& t_capacity_name,
                                                            const std::string& t_prop_name)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  return m_params.components.at(t_component_name).removeCapacityProperty(t_capacity_name, t_prop_name);
}

bool rtask::commons::Agent::getComponentCapacityProperty(const std::string& t_component_name,
                                                         const std::string& t_capacity_name,
                                                         const std::string& t_prop_name,
                                                         Property& t_prop) const
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  return m_params.components.at(t_component_name).getCapacityProperty(t_capacity_name, t_prop_name, t_prop);
}

bool rtask::commons::Agent::addComponentCapacityProperty(const std::string& t_component_name,
                                                         const std::string& t_capacity_name,
                                                         const Property& t_prop)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }
  bool ok = m_params.components.at(t_component_name).addCapacityProperty(t_capacity_name, t_prop);
  return ok;
}

bool rtask::commons::Agent::setComponentCapacityProperty(const std::string& t_component_name,
                                                         const std::string& t_capacity_name,
                                                         const Property& t_prop)
{
  if (!hasComponent(t_component_name)) {
    return false;
  }

  m_params.components.at(t_component_name).setCapacityProperty(t_capacity_name, t_prop);
  return true;
}

// -----------------
// PRIVATE FUNCTIONS
// -----------------

void rtask::commons::Agent::updCapabilityMap()
{
  for (auto& p : m_capability_to_component) {
    for (auto it = p.second.begin(); it != p.second.end(); ++it) {
      if (*it == p.first) {
        p.second.erase(it);
      }
    }
  }
}

void rtask::commons::Agent::updCapabilityMap(const commons::Component& t_comp)
{
  std::vector<std::string> capabilities;
  m_params.components[t_comp.getName()].getCapabilities(capabilities);
  for (const auto& cap : capabilities) {
    if (m_capability_to_component.count(cap) != 1) {
      m_capability_to_component[cap] = {};
    };
    m_capability_to_component[cap].push_back(t_comp.getName());
  }
}

void rtask::commons::Agent::clearComponents()
{
  m_params.components.clear();
  m_capability_to_component.clear();
}
