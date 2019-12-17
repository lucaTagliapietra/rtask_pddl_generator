#include "commons/task.h"

#include "boost/make_shared.hpp"

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Task::Task(const unsigned int t_id,
                           const std::string t_name,
                           const std::string t_ref_frame,
                           const std::vector<Capacity> t_requirements,
                           const std::string t_description,
                           const ros::Duration t_timeout,
                           const Status t_status)
{
  setTask(t_id, t_name, t_ref_frame, t_requirements, t_description, t_timeout, t_status);
}

rtask::commons::Task::Task(const rtask_msgs::Task& t_msg)
{
  setFromTaskMsg(t_msg);
}

rtask::commons::Task::Task(const rtask_msgs::TaskConstPtr t_msg_ptr)
{
  setFromTaskMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

// ----------
// Task Level
// ----------

rtask_msgs::TaskPtr rtask::commons::Task::toTaskMsg() const
{
  rtask_msgs::TaskPtr t_msg_ptr = boost::make_shared<rtask_msgs::Task>();
  t_msg_ptr->id = m_params.id;
  t_msg_ptr->name = m_params.name;
  t_msg_ptr->header.frame_id = m_params.ref_frame;
  t_msg_ptr->status = *(m_params.status.toStatusMsg());
  t_msg_ptr->timeout = m_params.timeout;
  t_msg_ptr->description = m_params.description;
  for (auto& r : m_params.requirements) {
    t_msg_ptr->requirements.push_back(*(r.second.toCapacityMsg()));
  }
  return t_msg_ptr;
}

void rtask::commons::Task::setFromTaskMsg(const rtask_msgs::Task& t_msg)
{
  m_params.id = t_msg.id;
  m_params.name = t_msg.name;
  m_params.ref_frame = t_msg.header.frame_id;
  m_params.status.setFromStatusMsg(t_msg.status);
  m_params.timeout = t_msg.timeout;
  m_params.description = t_msg.description;
  for (auto& r : t_msg.requirements) {
    m_params.requirements[r.capability] = {};
    m_params.requirements[r.capability].setFromCapacityMsg(r);
  }
}

void rtask::commons::Task::setFromTaskMsg(const rtask_msgs::TaskConstPtr t_msg_ptr)
{
  setFromTaskMsg(*t_msg_ptr);
}

void rtask::commons::Task::setTask(const unsigned int t_id,
                                   const std::string t_name,
                                   const std::string t_ref_frame,
                                   const std::vector<Capacity> t_requirements,
                                   const std::string t_description,
                                   const ros::Duration t_timeout,
                                   const Status t_status)
{
  m_params.id = t_id;
  m_params.name = t_name;
  m_params.ref_frame = t_ref_frame;
  setStatus(t_status);
  m_params.timeout = t_timeout;
  m_params.description = t_description;
  for (auto& r : t_requirements) {
    m_params.requirements[r.getCapabilityName()] = {r};
  }
}

void rtask::commons::Task::setTask(const Task& t_task)
{
  m_params.id = t_task.getId();
  m_params.name = t_task.getName();
  m_params.ref_frame = t_task.getReferenceFrame();
  t_task.getStatus(m_params.status);
  m_params.timeout = t_task.getTimeout();
  m_params.description = t_task.getDescription();
  std::vector<rtask::commons::Capacity> tmp;
  t_task.getRequirements(tmp);
  for (auto& r : tmp) {
    m_params.requirements[r.getCapabilityName()] = {r};
  }
}

void rtask::commons::Task::setStatus(const Status t_status)
{
  m_params.status.setStatus(t_status.getStatus(), t_status.getDescription());
}

void rtask::commons::Task::clear()
{
  m_params.clear();
}

void rtask::commons::Task::getStatus(Status& t_status) const
{
  t_status.setStatus(m_params.status.getStatus(), m_params.status.getDescription());
}

void rtask::commons::Task::getRequiredCapabilities(std::vector<std::string>& t_req_capabilities) const
{
  t_req_capabilities.reserve(m_params.requirements.size());
  for (auto it = m_params.requirements.begin(); it != m_params.requirements.end(); ++it) {
    t_req_capabilities.emplace_back(it->first);
  }
}

void rtask::commons::Task::getRequirements(std::vector<Capacity>& t_reqs) const
{
  t_reqs.reserve(m_params.requirements.size());
  for (auto it = m_params.requirements.begin(); it != m_params.requirements.end(); ++it) {
    t_reqs.emplace_back(it->second);
  }
}

// ------------
// Status Level
// ------------
void rtask::commons::Task::setStatusValue(const State t_state)
{
  m_params.status.setStatus(t_state, m_params.status.getDescription());
}

void rtask::commons::Task::setStatusDescription(const std::string& t_descr)
{
  m_params.status.setStatus(m_params.status.getStatus(), t_descr);
}

// --------------
// Capacity Level
// --------------
bool rtask::commons::Task::requiresCapacity(const std::string& t_capacity_name) const
{
  if (m_params.requirements.count(t_capacity_name) != 1) {
    return false;
  }
  return true;
}

bool rtask::commons::Task::removeRequiredCapacity(const std::string& t_capacity_name)
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  m_params.requirements.erase(t_capacity_name);
  return true;
}

bool rtask::commons::Task::getRequiredCapacity(const std::string& t_capacity_name, Capacity& t_capacity) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  t_capacity = {m_params.requirements.at(t_capacity_name)};
  return true;
}

bool rtask::commons::Task::getRequiredCapacityProperties(const std::string& t_capacity_name,
                                                         std::vector<Property>& t_props) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  m_params.requirements.at(t_capacity_name).getProperties(t_props);
  return true;
}

bool rtask::commons::Task::addRequiredCapacity(const Capacity& t_capacity)
{
  if (requiresCapacity(t_capacity.getCapabilityName())) {
    return false;
  }
  setRequiredCapacity(t_capacity);
  return true;
}

void rtask::commons::Task::setRequiredCapacity(const Capacity& t_capacity)
{
  m_params.requirements[t_capacity.getCapabilityName()] = {t_capacity};
}

// --------------
// Property Level
// --------------
bool rtask::commons::Task::requiresCapacityProperty(const std::string& t_capacity_name,
                                                    const std::string& t_prop_name) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  if (!m_params.requirements.at(t_capacity_name).hasProperty(t_prop_name)) {
    return false;
  }
  return true;
}

bool rtask::commons::Task::removeRequiredCapacityProperty(const std::string& t_capacity_name,
                                                          const std::string& t_prop_name)
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  return m_params.requirements[t_capacity_name].removeProperty(t_prop_name);
}

bool rtask::commons::Task::getRequiredCapacityProperty(const std::string& t_capacity_name,
                                                       const std::string& t_prop_name,
                                                       Property& t_prop) const
{
  if (!requiresCapacity(t_capacity_name)) {
    return false;
  }
  return m_params.requirements.at(t_capacity_name).getProperty(t_prop_name, t_prop);
}

bool rtask::commons::Task::addRequiredCapacityProperty(const std::string& t_capacity_name, const Property& t_prop)
{
  if (requiresCapacityProperty(t_capacity_name, t_prop.getName())) {
    return false;
  }

  if (!requiresCapacity(t_capacity_name)) {
    m_params.requirements[t_capacity_name] = {t_capacity_name};
  }
  return m_params.requirements[t_capacity_name].addProperty(t_prop);
}

void rtask::commons::Task::setRequiredCapacityProperty(const std::string& t_capacity_name, const Property& t_prop)
{
  {
    if (!requiresCapacity(t_capacity_name)) {
      m_params.requirements[t_capacity_name] = {t_capacity_name};
    }
    m_params.requirements[t_capacity_name].setProperty(t_prop);
  }
}

// -----------------
// PRIVATE FUNCTIONS
// -----------------
