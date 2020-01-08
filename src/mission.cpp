#include "commons/mission.h"

#include <boost/make_shared.hpp>

// ------------
// CONSTRUCTORS
// ------------

rtask::commons::Mission::Mission(const unsigned int t_id,
                                 const std::string t_name,
                                 const std::string t_ref_frame,
                                 const std::vector<Task> t_tasks,
                                 const std::string t_description,
                                 const ros::Duration t_timeout,
                                 const Status t_status)
{
  setMission(t_id, t_name, t_ref_frame, t_tasks, t_description, t_timeout, t_status);
}

rtask::commons::Mission::Mission(const rtask_msgs::Mission& t_msg)
{
  setFromMissionMsg(t_msg);
}

rtask::commons::Mission::Mission(const rtask_msgs::MissionConstPtr t_msg_ptr)
{
  setFromMissionMsg(t_msg_ptr);
}

// ----------------
// PUBLIC FUNCTIONS
// ----------------

// -------------
// Mission Level
// -------------
rtask_msgs::MissionPtr rtask::commons::Mission::toMissionMsg() const
{
  rtask_msgs::MissionPtr t_msg_ptr = boost::make_shared<rtask_msgs::Mission>();
  t_msg_ptr->id = m_params.id;
  t_msg_ptr->name = m_params.name;
  t_msg_ptr->header.frame_id = m_params.ref_frame;
  t_msg_ptr->description = m_params.description;
  t_msg_ptr->status = *(m_params.status.toStatusMsg());
  t_msg_ptr->timeout = m_params.timeout;
  for (auto& t : m_params.tasks) {
    t_msg_ptr->tasks.push_back(*(t.second.toTaskMsg()));
  }
  return t_msg_ptr;
}

void rtask::commons::Mission::setFromMissionMsg(const rtask_msgs::Mission& t_msg)
{
  m_params.id = t_msg.id;
  m_params.name = t_msg.name;
  m_params.ref_frame = t_msg.header.frame_id;
  m_params.description = t_msg.description;
  m_params.timeout = t_msg.timeout;
  m_params.status.setFromStatusMsg(t_msg.status);
  for (auto& t : t_msg.tasks) {
    m_params.tasks[t.name].setFromTaskMsg(t);
  }
}

void rtask::commons::Mission::setFromMissionMsg(const rtask_msgs::MissionConstPtr t_msg_ptr)
{
  setFromMissionMsg(*t_msg_ptr);
}

void rtask::commons::Mission::setMission(const unsigned int t_id,
                                         const std::string t_name,
                                         const std::string t_ref_frame,
                                         const std::vector<Task> t_tasks,
                                         const std::string t_description,
                                         const ros::Duration t_timeout,
                                         const Status t_status)
{
  m_params.id = t_id;
  m_params.name = t_name;
  m_params.ref_frame = t_ref_frame;
  m_params.description = t_description;
  m_params.timeout = t_timeout;
  m_params.status.setStatus(t_status.getStatus(), t_status.getDescription());
  for (const auto& t : t_tasks) {
    m_params.tasks[t.getName()].setTask(t);
  }
}

void rtask::commons::Mission::setStatus(const Status t_status)
{
  m_params.status.setStatus(t_status.getStatus(), t_status.getDescription());
}

void rtask::commons::Mission::clear()
{
  m_params.clear();
}

void rtask::commons::Mission::getStatus(Status& t_status) const
{
  t_status.setStatus(t_status.getStatus(), t_status.getDescription());
}

void rtask::commons::Mission::getTaskNames(std::vector<std::string>& t_task_names) const
{
  t_task_names.reserve(m_params.tasks.size());
  for (auto& t : m_params.tasks) {
    t_task_names.emplace_back(t.first);
  }
}

void rtask::commons::Mission::getTasks(std::vector<Task>& t_tasks) const
{
  t_tasks.reserve(m_params.tasks.size());
  for (auto& t : m_params.tasks) {
    t_tasks.emplace_back(t.second);
  }
}

// ------------
// Status Level
// ------------

void rtask::commons::Mission::setStatusValue(const State t_state)
{
  m_params.status.setStatus(t_state);
}

void rtask::commons::Mission::setStatusDescription(std::string& t_descr)
{
  m_params.status.setDescription(t_descr);
}

// ----------
// Task Level
// ----------
bool rtask::commons::Mission::hasTask(const std::string& t_task_name) const
{
  if (m_params.tasks.count(t_task_name) != 1) {
    return false;
  }
  return true;
}

bool rtask::commons::Mission::removeTask(const std::string& t_task_name)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.erase(t_task_name);
  return true;
}

bool rtask::commons::Mission::getTask(const std::string& t_task_name, Task& t_task) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  t_task.setTask(m_params.tasks.at(t_task_name));
  return true;
}

bool rtask::commons::Mission::addTask(const Task& t_task)
{
  if (hasTask(t_task.getName())) {
    return false;
  }

  setTask(t_task);
  return true;
}

void rtask::commons::Mission::setTask(const Task& t_task)
{
  m_params.tasks[t_task.getName()].setTask(t_task);
}

bool rtask::commons::Mission::getTaskId(const std::string& t_task_name, unsigned int& t_id) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  t_id = m_params.tasks.at(t_task_name).getId();
  return true;
}

bool rtask::commons::Mission::getTaskReferenceFrame(const std::string& t_task_name, std::string& t_ref_frame) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  t_ref_frame = m_params.tasks.at(t_task_name).getReferenceFrame();
  return true;
}

bool rtask::commons::Mission::getTaskDescription(const std::string& t_task_name, std::string& t_descr) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  t_descr = m_params.tasks.at(t_task_name).getDescription();
  return true;
}

bool rtask::commons::Mission::getTaskTimeout(const std::string& t_task_name, ros::Duration& t_timeout) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  t_timeout = m_params.tasks.at(t_task_name).getTimeout();
  return true;
}

bool rtask::commons::Mission::getTaskType(const std::string& t_task_name, std::string& t_type) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  t_type = m_params.tasks.at(t_task_name).getType();
  return true;
}

bool rtask::commons::Mission::getTaskStatus(const std::string& t_task_name, Status& t_status) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).getStatus(t_status);
  return true;
}

bool rtask::commons::Mission::getTaskRequiredCapabilities(const std::string& t_task_name,
                                                          std::vector<std::string>& t_req_capabilities) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).getRequiredCapabilities(t_req_capabilities);
  return true;
}

bool rtask::commons::Mission::getTaskRequirements(const std::string& t_task_name, std::vector<Capacity> t_reqs) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).getRequirements(t_reqs);
  return true;
}

bool rtask::commons::Mission::getTaskDefinition(const std::string& t_task_name, TaskDefinition& t_task_definition) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).getTaskDefinition(t_task_definition);
  return true;
}

bool rtask::commons::Mission::setTaskReferenceFrame(const std::string& t_task_name, const std::string& t_ref_frame)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setReferenceFrame(t_ref_frame);
  return true;
}

bool rtask::commons::Mission::setTaskStatus(const std::string& t_task_name, const Status& t_status)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setStatus(t_status);
  return true;
}

bool rtask::commons::Mission::setTaskStatusValue(const std::string& t_task_name, const State& t_state)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setStatusValue(t_state);
  return true;
}

bool rtask::commons::Mission::setTaskStatusDescription(const std::string& t_task_name, const std::string& t_status_decr)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setStatusDescription(t_status_decr);
  return true;
}

bool rtask::commons::Mission::setTaskDescription(const std::string& t_task_name, const std::string& t_task_descr)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setDescription(t_task_descr);
  return true;
}

bool rtask::commons::Mission::setTaskTimeout(const std::string& t_task_name, const ros::Duration& t_timeout)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setTimeout(t_timeout);
  return true;
}

bool rtask::commons::Mission::setTaskType(const std::string& t_task_name, std::string& t_type)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setType(t_type);
  return true;
}

bool rtask::commons::Mission::setTaskDefinition(const std::string& t_task_name, TaskDefinition& t_task_definition)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setTaskDefinition(t_task_definition);
  return true;
}

// --------------
// Capacity Level
// --------------

bool rtask::commons::Mission::requiresTaskCapacity(const std::string& t_task_name,
                                                   const std::string& t_capacity_name) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).requiresCapacity(t_capacity_name);
}

bool rtask::commons::Mission::removeTaskRequiredCapacity(const std::string& t_task_name,
                                                         const std::string& t_capacity_name)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).removeRequiredCapacity(t_capacity_name);
}

bool rtask::commons::Mission::getRequiredTaskCapacity(const std::string& t_task_name,
                                                      const std::string& t_capacity_name,
                                                      Capacity& t_capacity) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).getRequiredCapacity(t_capacity_name, t_capacity);
}

bool rtask::commons::Mission::getRequiredTaskCapacityProperties(const std::string& t_task_name,
                                                                const std::string& t_capacity_name,
                                                                std::vector<Property>& t_props) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).getRequiredCapacityProperties(t_capacity_name, t_props);
}

bool rtask::commons::Mission::addRequiredTaskCapacity(const std::string& t_task_name, const Capacity& t_capacity)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).addRequiredCapacity(t_capacity);
}

bool rtask::commons::Mission::setRequiredTaskCapacity(const std::string& t_task_name, const Capacity& t_capacity)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setRequiredCapacity(t_capacity);
  return true;
}

// --------------
// Property level
// --------------

bool rtask::commons::Mission::requiresTaskCapacityProperty(const std::string& t_task_name,
                                                           const std::string& t_capacity_name,
                                                           const std::string& t_prop_name) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).requiresCapacityProperty(t_capacity_name, t_prop_name);
}

bool rtask::commons::Mission::removeRequiredTaskCapacityProperty(const std::string& t_task_name,
                                                                 const std::string& t_capacity_name,
                                                                 const std::string& t_prop_name)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).removeRequiredCapacityProperty(t_capacity_name, t_prop_name);
}
bool rtask::commons::Mission::getRequiredTaskCapacityProperty(const std::string& t_task_name,
                                                              const std::string& t_capacity_name,
                                                              const std::string& t_prop_name,
                                                              Property& t_prop) const
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).getRequiredCapacityProperty(t_capacity_name, t_prop_name, t_prop);
}

bool rtask::commons::Mission::addRequiredTaskCapacityProperty(const std::string& t_task_name,
                                                              const std::string& t_capacity_name,
                                                              const Property& t_prop)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  return m_params.tasks.at(t_task_name).addRequiredCapacityProperty(t_capacity_name, t_prop);
}

bool rtask::commons::Mission::setRequiredTaskCapacityProperty(const std::string& t_task_name,
                                                              const std::string& t_capacity_name,
                                                              const Property& t_prop)
{
  if (!hasTask(t_task_name)) {
    return false;
  }
  m_params.tasks.at(t_task_name).setRequiredCapacityProperty(t_capacity_name, t_prop);
  return true;
}
